#!/usr/bin/env python3
"""
Pi-side Solis-only poller.

- Polls Solis inverter as Modbus slave 1 (FC04 input registers).
- Writes Solis metrics to InfluxDB.
- Adds Solis ToU low-battery protection logic.
- Syncs the Solis inverter clock from the Pi clock whenever ToU registers are written.

ESP32 BMS slave 5 telemetry is handled by the separate read_esp_slave5.py script
on a dedicated second USB-RS485 adapter (CH340, /dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0).

Battery protection logic:

NORMAL / UNKNOWN:
  If gridPower < 0 W (importing) AND Solis SOC < 16% AND batteryDirectionFlag == 1 (discharging):
    Write Solis ToU:
      sync Solis clock to Pi time
      RUN with grid-charge allowed
      charge current 0.1A
      charge window now-2min -> now+12h
    state = BATTERY_OFF

BATTERY_OFF:
  If pvTotalPowerW > 100W AND gridPower > 0W (exporting):
    Write Solis ToU:
      sync Solis clock to Pi time
      STOP with grid-charge allowed
      charge current 10.0A
      charge window 00:00 -> 00:00
    state = NORMAL

BATTERY_OFF refresh:
  If still grid importing, SOC < 16%, and battery discharging, and the current 12h
  OFF window has less than 2 hours remaining, refresh the OFF window.

No persistent state is used. On script restart, live grid/SOC/battery conditions decide
whether to write a fresh OFF window or release with STOP.
"""

import time
from datetime import datetime, timedelta
from typing import Dict, Optional

import requests
import serial


# ----------------------------------------------------------------------
# Config
# ----------------------------------------------------------------------

PORT = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A5010V5B-if00-port0"
BAUDRATE = 9600

INFLUX_URL = "http://localhost:8086/write?db=power"
HOST_TAG = "server01"

SOLIS_SOURCE_TAG = "solis_master"

SOLIS_SLAVE_ID = 0x01

FUNC_READ_INPUT_REGS = 0x04
FUNC_READ_HOLDING_REGS = 0x03
FUNC_WRITE_SINGLE_REG = 0x06

# Solis input-register block
SOLIS_START_DOC_REG = 33050
SOLIS_END_DOC_REG = 33142
SOLIS_REG_COUNT = SOLIS_END_DOC_REG - SOLIS_START_DOC_REG + 1  # 93

# Pacing between consecutive FC06 (write single register) calls to Solis.
# Solis does not reliably echo each write at 0.10 s; 0.50 s gives it time to
# process and de-assert its own RS485 driver before the next request.
SOLIS_WRITE_DELAY_S = 0.5

# Pause applied after any ToU / clock write sequence before resuming normal
# polling.  Gives Solis time to complete internal register processing.
SOLIS_POST_WRITE_RECOVERY_S = 2.0

POLL_INTERVAL_S = 2.0
REPLY_TIMEOUT_S = 2.0
WRITE_REPLY_TIMEOUT_S = 4.0
READ_IDLE_TIMEOUT_S = 0.2
BUFFER_MAX = 8192

# ----------------------------------------------------------------------
# Solis ToU / battery-off control config
# ----------------------------------------------------------------------

STATE_UNKNOWN = "UNKNOWN"
STATE_NORMAL = "NORMAL"
STATE_BATTERY_OFF = "BATTERY_OFF"

BATTERY_OFF_EXIT_PV_W = 100.0
BATTERY_OFF_EXIT_GRID_EXPORT_W = 0.0
BATTERY_OFF_ENTER_SOC = 17
BATTERY_OFF_ENTER_GRID_IMPORT_W = 0.0  # enter when grid < this (negative = importing)

# batteryDirectionFlag value that indicates the battery is discharging
BATTERY_DIRECTION_DISCHARGING = 1

BATTERY_OFF_START_OFFSET_MINUTES = -2  # negative = start is in the past so window is immediately active
BATTERY_OFF_WINDOW_HOURS = 12
BATTERY_OFF_REFRESH_WHEN_REMAINING_S = 2 * 60 * 60

# Avoid retry-spamming failed writes every 2 seconds.
TOU_MIN_WRITE_INTERVAL_S = 60

# Confirmed Solis holding-register mapping for this Pi script:
# The script uses doc_reg - 1 as the raw Modbus address.
REG_TOU_RUN_STOP = 43111       # bitfield: 33=STOP+grid-charge, 35=RUN+grid-charge
REG_TOU_CHARGE_CURRENT = 43142 # x10 A, e.g. 17 = 1.7A
REG_TOU_START_HOUR = 43144
REG_TOU_START_MINUTE = 43145
REG_TOU_END_HOUR = 43146
REG_TOU_END_MINUTE = 43147

# Confirmed Solis clock holding-register mapping for this Pi script:
# 43001=year as two digits, 43002=month, 43003=day,
# 43004=hour, 43005=minute, 43006=second.
REG_CLOCK_YEAR = 43001
REG_CLOCK_MONTH = 43002
REG_CLOCK_DAY = 43003
REG_CLOCK_HOUR = 43004
REG_CLOCK_MINUTE = 43005
REG_CLOCK_SECOND = 43006

TOU_STOP = 33  # self-use + allow grid charge; ToU stopped
TOU_RUN = 35   # self-use + ToU run + allow grid charge

TOU_CURRENT_0_1A = 1
TOU_CURRENT_10_0A = 100


# ----------------------------------------------------------------------
# Modbus helpers
# ----------------------------------------------------------------------

def modbus_crc(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def crc_ok(frame: bytes) -> bool:
    if len(frame) < 4:
        return False
    rx_crc = frame[-2] | (frame[-1] << 8)
    return rx_crc == modbus_crc(frame[:-2])


def s16(v: int) -> int:
    return v - 65536 if v >= 32768 else v


def s32_from_regs(high: int, low: int) -> int:
    v = ((high & 0xFFFF) << 16) | (low & 0xFFFF)
    return v - 0x100000000 if v & 0x80000000 else v


def build_request(slave_id: int, func: int, start_reg: int, count: int) -> bytes:
    # Solis documented register -> raw Modbus address is doc_reg - 1.
    raw_addr = start_reg - 1
    frame = bytes([
        slave_id,
        func,
        (raw_addr >> 8) & 0xFF,
        raw_addr & 0xFF,
        (count >> 8) & 0xFF,
        count & 0xFF,
    ])
    crc = modbus_crc(frame)
    return frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def build_write_single_request(slave_id: int, doc_reg: int, value: int) -> bytes:
    # Solis documented register -> raw Modbus address is doc_reg - 1.
    raw_addr = doc_reg - 1
    frame = bytes([
        slave_id,
        FUNC_WRITE_SINGLE_REG,
        (raw_addr >> 8) & 0xFF,
        raw_addr & 0xFF,
        (value >> 8) & 0xFF,
        value & 0xFF,
    ])
    crc = modbus_crc(frame)
    return frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def extract_frame_from_buffer(buf: bytearray, slave_id: int, func: int, expected_regs: int) -> Optional[bytes]:
    expected_len = 3 + (expected_regs * 2) + 2
    if len(buf) < expected_len:
        return None

    for i in range(0, len(buf) - expected_len + 1):
        if buf[i] != slave_id or buf[i + 1] != func:
            continue
        candidate = bytes(buf[i:i + expected_len])
        if candidate[2] != expected_regs * 2:
            continue
        if crc_ok(candidate):
            del buf[:i + expected_len]
            return candidate

    if len(buf) > BUFFER_MAX:
        del buf[:-expected_len]
    return None


def parse_modbus_response(frame: bytes, start_reg: int, expected_regs: int) -> Optional[Dict[int, int]]:
    if len(frame) != 3 + (expected_regs * 2) + 2:
        return None
    if not crc_ok(frame):
        return None

    data = frame[3:-2]
    regs: Dict[int, int] = {}
    for i in range(expected_regs):
        regs[start_reg + i] = (data[i * 2] << 8) | data[i * 2 + 1]
    return regs


def flush_serial_input(ser: serial.Serial) -> None:
    while ser.in_waiting:
        ser.read(ser.in_waiting)


# ----------------------------------------------------------------------
# Influx
# ----------------------------------------------------------------------

def write_line(measurement: str, value, source_tag: str):
    if value is None:
        return
    line = f"{measurement},host={HOST_TAG},source={source_tag} value={value}"
    try:
        requests.post(INFLUX_URL, data=line.encode("utf-8"), timeout=2)
    except Exception as e:
        print(f"[Influx] write failed: {measurement}: {e}")


# ----------------------------------------------------------------------
# Solis decode / metrics
# ----------------------------------------------------------------------

def decode_solis(regs: Dict[int, int]) -> dict:
    pv1_v = round(regs.get(33050, 0) / 10.0, 1)
    pv1_i = round(regs.get(33051, 0) / 10.0, 1)
    pv2_v = round(regs.get(33052, 0) / 10.0, 1)
    pv2_i = round(regs.get(33053, 0) / 10.0, 1)

    grid_v = round(regs.get(33074, 0) / 10.0, 1)
    grid_f = round(regs.get(33095, 0) / 100.0, 2)
    grid_power = s16(regs.get(33132, 0))

    # Published Solis register pair is likely 33079-33080 for inverter
    # active AC power. This script sends raw_addr = doc_reg - 1 and labels
    # parsed registers from the requested start_reg, so use parsed keys
    # 33080-33081 here. Temporary raw metrics below help confirm mapping.
    # Positive signed value means AC power coming out of the Solis.
    # Negative signed value means AC power going into the Solis
    # (e.g. AC/grid-assisted charging).
    inverter_active_power = s32_from_regs(
        regs.get(33080, 0),
        regs.get(33081, 0),
    )
    solis_ac_output_power = max(inverter_active_power, 0)
    solis_ac_input_power = max(-inverter_active_power, 0)

    battery_current = round(s16(regs.get(33135, 0)) / 10.0, 1)
    battery_direction_flag = regs.get(33136, 0)
    battery_direction = "discharging" if battery_direction_flag == 1 else "charging"
    battery_soc = regs.get(33140, 0)
    battery_voltage = round(regs.get(33142, 0) / 100.0, 2)

    pv1_power = round(pv1_v * pv1_i, 1)
    pv2_power = round(pv2_v * pv2_i, 1)
    pv_total_power = round(pv1_power + pv2_power, 1)

    battery_power = round(battery_voltage * battery_current, 1)
    if battery_direction_flag == 1:
        battery_power = -battery_power

    return {
        "pv1Voltage": pv1_v,
        "pv1Current": pv1_i,
        "pv2Voltage": pv2_v,
        "pv2Current": pv2_i,
        "gridVoltage": grid_v,
        "gridFrequency": grid_f,
        "gridPower": grid_power,
        "solisInverterActivePowerW": inverter_active_power,
        "solisAcOutputPowerW": solis_ac_output_power,
        "solisAcInputPowerW": solis_ac_input_power,
        "batterySoc": battery_soc,
        "batteryVoltage": battery_voltage,
        "batteryCurrent": battery_current,
        "batteryDirectionFlag": battery_direction_flag,
        "batteryDirection": battery_direction,
        "batteryPowerW": battery_power,
        "pv1PowerW": pv1_power,
        "pv2PowerW": pv2_power,
        "pvTotalPowerW": pv_total_power,
        "gridFrequencyRaw": regs.get(33095, 0),
        "reg33079Raw": regs.get(33079, 0),
        "reg33080Raw": regs.get(33080, 0),
        "reg33081Raw": regs.get(33081, 0),
    }


def write_solis_metrics(s: dict, poll_count: int, read_errors: int, controller_state: str):
    write_line("solis_pv1_voltage", s["pv1Voltage"], SOLIS_SOURCE_TAG)
    write_line("solis_pv1_current", s["pv1Current"], SOLIS_SOURCE_TAG)
    write_line("solis_pv2_voltage", s["pv2Voltage"], SOLIS_SOURCE_TAG)
    write_line("solis_pv2_current", s["pv2Current"], SOLIS_SOURCE_TAG)
    write_line("solis_grid_voltage", s["gridVoltage"], SOLIS_SOURCE_TAG)
    write_line("solis_grid_frequency", s["gridFrequency"], SOLIS_SOURCE_TAG)
    write_line("solis_grid_power", s["gridPower"], SOLIS_SOURCE_TAG)
    write_line("solis_inverter_active_power", s["solisInverterActivePowerW"], SOLIS_SOURCE_TAG)
    write_line("solis_ac_output_power", s["solisAcOutputPowerW"], SOLIS_SOURCE_TAG)
    write_line("solis_ac_input_power", s["solisAcInputPowerW"], SOLIS_SOURCE_TAG)
    write_line("solis_battery_soc", s["batterySoc"], SOLIS_SOURCE_TAG)
    write_line("solis_battery_voltage", s["batteryVoltage"], SOLIS_SOURCE_TAG)
    write_line("solis_battery_current", s["batteryCurrent"], SOLIS_SOURCE_TAG)
    write_line("solis_battery_direction_flag", s["batteryDirectionFlag"], SOLIS_SOURCE_TAG)
    write_line("solis_battery_power", s["batteryPowerW"], SOLIS_SOURCE_TAG)
    write_line("solis_pv1_power", s["pv1PowerW"], SOLIS_SOURCE_TAG)
    write_line("solis_pv2_power", s["pv2PowerW"], SOLIS_SOURCE_TAG)
    write_line("solis_pv_total_power", s["pvTotalPowerW"], SOLIS_SOURCE_TAG)
    write_line("solis_poll_count", poll_count, SOLIS_SOURCE_TAG)
    write_line("solis_read_errors", read_errors, SOLIS_SOURCE_TAG)
    write_line("solis_reg_33095_raw", s["gridFrequencyRaw"], SOLIS_SOURCE_TAG)
    write_line("solis_reg_33079_raw", s["reg33079Raw"], SOLIS_SOURCE_TAG)
    write_line("solis_reg_33080_raw", s["reg33080Raw"], SOLIS_SOURCE_TAG)
    write_line("solis_reg_33081_raw", s["reg33081Raw"], SOLIS_SOURCE_TAG)

    # Controller state as numeric metric:
    # UNKNOWN=0, NORMAL=1, BATTERY_OFF=2
    state_num = {
        STATE_UNKNOWN: 0,
        STATE_NORMAL: 1,
        STATE_BATTERY_OFF: 2,
    }.get(controller_state, 0)
    write_line("solis_battery_off_controller_state", state_num, SOLIS_SOURCE_TAG)


# ----------------------------------------------------------------------
# Solis ToU / clock write helpers
# ----------------------------------------------------------------------

def read_write_response(ser: serial.Serial, expected_func: int, expected_len: int) -> Optional[bytes]:
    deadline = time.monotonic() + WRITE_REPLY_TIMEOUT_S
    buf = bytearray()

    while time.monotonic() < deadline:
        chunk = ser.read(256)
        if chunk:
            buf.extend(chunk)

            # Normal response.
            if len(buf) >= expected_len:
                for i in range(0, len(buf) - expected_len + 1):
                    candidate = bytes(buf[i:i + expected_len])
                    if (
                        candidate[0] == SOLIS_SLAVE_ID
                        and candidate[1] == expected_func
                        and crc_ok(candidate)
                    ):
                        return candidate

            # Exception response.
            if len(buf) >= 5:
                for i in range(0, len(buf) - 5 + 1):
                    candidate = bytes(buf[i:i + 5])
                    if (
                        candidate[0] == SOLIS_SLAVE_ID
                        and candidate[1] == (expected_func | 0x80)
                        and crc_ok(candidate)
                    ):
                        return candidate
        else:
            time.sleep(0.01)

    if buf:
        print(f"[tou] RX timeout with partial: {buf.hex(' ')}")
    else:
        print("[tou] RX timeout/no response")
    return None


def write_solis_holding_reg(ser: serial.Serial, doc_reg: int, value: int) -> bool:
    req = build_write_single_request(SOLIS_SLAVE_ID, doc_reg, value)

    flush_serial_input(ser)

    print(f"[tou] WRITE {doc_reg} = {value} / 0x{value:04X}")
    print(f"[tou] TX: {req.hex(' ')}")

    ser.write(req)
    ser.flush()

    resp = read_write_response(ser, FUNC_WRITE_SINGLE_REG, 8)
    if resp is None:
        print("[tou] FAIL: no response")
        return False

    print(f"[tou] RX: {resp.hex(' ')}")

    if len(resp) == 5 and resp[1] == (FUNC_WRITE_SINGLE_REG | 0x80):
        print(f"[tou] FAIL: Modbus exception code=0x{resp[2]:02X}")
        return False

    if len(resp) != 8:
        print(f"[tou] FAIL: unexpected response length {len(resp)}")
        return False

    if resp != req:
        print("[tou] FAIL: FC06 response did not exactly echo request")
        return False

    print("[tou] OK")
    return True


def sync_solis_clock_to_pi(ser: serial.Serial) -> bool:
    """Write the Solis RTC holding registers from the Pi's local time."""
    now = datetime.now()
    values = [
        (REG_CLOCK_YEAR, now.year % 100),
        (REG_CLOCK_MONTH, now.month),
        (REG_CLOCK_DAY, now.day),
        (REG_CLOCK_HOUR, now.hour),
        (REG_CLOCK_MINUTE, now.minute),
        (REG_CLOCK_SECOND, now.second),
    ]

    print(
        f"[tou] Syncing Solis clock to Pi time: "
        f"{now.strftime('%Y-%m-%d %H:%M:%S')}"
    )

    all_ok = True
    for reg, value in values:
        ok = write_solis_holding_reg(ser, reg, value)
        all_ok = all_ok and ok
        time.sleep(SOLIS_WRITE_DELAY_S)

    if all_ok:
        print("[tou] Solis clock sync OK")
    else:
        print("[tou] WARN: Solis clock sync had one or more failed writes")

    return all_ok


def write_tou_registers(
    ser: serial.Serial,
    run_stop: int,
    current_x10: int,
    start_hour: int,
    start_minute: int,
    end_hour: int,
    end_minute: int,
) -> bool:
    """
    Sync the Solis clock, then write the six confirmed ToU registers.
    """
    clock_ok = sync_solis_clock_to_pi(ser)

    writes_config = [
        (REG_TOU_CHARGE_CURRENT, current_x10),
        (REG_TOU_START_HOUR, start_hour),
        (REG_TOU_START_MINUTE, start_minute),
        (REG_TOU_END_HOUR, end_hour),
        (REG_TOU_END_MINUTE, end_minute),
    ]

    writes_run_stop = [(REG_TOU_RUN_STOP, run_stop)]
    writes = writes_config + writes_run_stop

    all_ok = clock_ok
    for reg, value in writes:
        ok = write_solis_holding_reg(ser, reg, value)
        all_ok = all_ok and ok
        time.sleep(SOLIS_WRITE_DELAY_S)

    return all_ok


def calc_battery_off_window():
    now = datetime.now()
    start = now + timedelta(minutes=BATTERY_OFF_START_OFFSET_MINUTES)
    end = now + timedelta(hours=BATTERY_OFF_WINDOW_HOURS)

    return {
        "start_hour": start.hour,
        "start_minute": start.minute,
        "end_hour": end.hour,
        "end_minute": end.minute,
        "end_timestamp": end.timestamp(),
        "start_text": start.strftime("%H:%M"),
        "end_text": end.strftime("%H:%M"),
    }


def enter_battery_off(ser: serial.Serial, controller: dict, reason: str) -> bool:
    window = calc_battery_off_window()

    print("")
    print("=" * 80)
    print(f"[tou] ENTER/REFRESH BATTERY_OFF: {reason}")
    print(
        f"[tou] Target: RUN+grid-charge, 0.1A, "
        f"{window['start_text']} -> {window['end_text']}"
    )
    print("=" * 80)

    controller["last_tou_write_ts"] = time.time()

    ok = write_tou_registers(
        ser=ser,
        run_stop=TOU_RUN,
        current_x10=TOU_CURRENT_0_1A,
        start_hour=window["start_hour"],
        start_minute=window["start_minute"],
        end_hour=window["end_hour"],
        end_minute=window["end_minute"],
    )

    if ok:
        controller["state"] = STATE_BATTERY_OFF
        controller["battery_off_until_ts"] = window["end_timestamp"]
        print("[tou] BATTERY_OFF active")
    else:
        print("[tou] BATTERY_OFF write failed; state unchanged")

    # Allow Solis time to process the register writes and release the RS485 bus
    # before normal polling resumes.
    print(f"[tou] post-write recovery pause {SOLIS_POST_WRITE_RECOVERY_S}s")
    time.sleep(SOLIS_POST_WRITE_RECOVERY_S)
    flush_serial_input(ser)

    return ok


def exit_battery_off(ser: serial.Serial, controller: dict, reason: str) -> bool:
    print("")
    print("=" * 80)
    print(f"[tou] EXIT BATTERY_OFF: {reason}")
    print("[tou] Target: STOP+grid-charge, 10.0A, 00:00 -> 00:00")
    print("=" * 80)

    controller["last_tou_write_ts"] = time.time()

    ok = write_tou_registers(
        ser=ser,
        run_stop=TOU_STOP,
        current_x10=TOU_CURRENT_10_0A,
        start_hour=0,
        start_minute=0,
        end_hour=0,
        end_minute=0,
    )

    if ok:
        controller["state"] = STATE_NORMAL
        controller["battery_off_until_ts"] = None
        print("[tou] NORMAL active")
    else:
        print("[tou] STOP/release write failed; state unchanged")

    # Allow Solis time to process the register writes and release the RS485 bus
    # before normal polling resumes.
    print(f"[tou] post-write recovery pause {SOLIS_POST_WRITE_RECOVERY_S}s")
    time.sleep(SOLIS_POST_WRITE_RECOVERY_S)
    flush_serial_input(ser)

    return ok


def can_attempt_tou_write(controller: dict) -> bool:
    last = controller.get("last_tou_write_ts", 0)
    return (time.time() - last) >= TOU_MIN_WRITE_INTERVAL_S


def manage_battery_off_controller(ser: serial.Serial, s: dict, controller: dict) -> None:
    pv = float(s.get("pvTotalPowerW", 0.0))
    grid = float(s.get("gridPower", 0.0))
    soc = int(s.get("batterySoc", 0))
    battery_direction_flag = int(s.get("batteryDirectionFlag", 0))
    state = controller.get("state", STATE_UNKNOWN)

    entry_ready = (
        grid < BATTERY_OFF_ENTER_GRID_IMPORT_W
        and soc < BATTERY_OFF_ENTER_SOC
        and battery_direction_flag == BATTERY_DIRECTION_DISCHARGING
    )
    pv_recovered = pv > BATTERY_OFF_EXIT_PV_W
    grid_exporting = grid > BATTERY_OFF_EXIT_GRID_EXPORT_W
    exit_ready = pv_recovered and grid_exporting

    print(
        f"[tou] state={state} pv={pv:.1f}W grid={grid:.1f}W soc={soc}% "
        f"batt_dir_flag={battery_direction_flag} entry_ready={entry_ready} "
        f"pv_recovered={pv_recovered} grid_exporting={grid_exporting} exit_ready={exit_ready}"
    )

    # Startup recovery.
    if state == STATE_UNKNOWN:
        if entry_ready:
            if can_attempt_tou_write(controller):
                enter_battery_off(
                    ser,
                    controller,
                    reason=f"startup/unknown: grid {grid:.1f}W importing, SOC {soc}% < {BATTERY_OFF_ENTER_SOC}%, battery discharging",
                )
            return

        # On startup we do not know whether BATTERY_OFF is actually active on the
        # Solis, so we must NOT fire the exit/STOP write burst just because PV has
        # recovered.  Doing so causes a large FC06 write sequence that can upset
        # Solis comms (as observed in the log: write burst -> repeated read failures).
        # Instead, assume NORMAL and let the steady-state logic handle things cleanly
        # on the next poll cycles.
        controller["state"] = STATE_NORMAL
        print(
            "[tou] startup/unknown: entry conditions not met (grid not importing or SOC not low or battery not discharging); "
            "assuming NORMAL without ToU writes"
        )
        return

    # Normal -> Battery-off.
    if state == STATE_NORMAL:
        if entry_ready:
            if can_attempt_tou_write(controller):
                enter_battery_off(
                    ser,
                    controller,
                    reason=f"grid {grid:.1f}W importing, SOC {soc}% < {BATTERY_OFF_ENTER_SOC}%, battery discharging",
                )
        return

    # Battery-off -> Normal, or refresh rolling 12h window.
    if state == STATE_BATTERY_OFF:
        if exit_ready:
            if can_attempt_tou_write(controller):
                exit_battery_off(
                    ser,
                    controller,
                    reason=(
                        f"PV {pv:.1f}W > {BATTERY_OFF_EXIT_PV_W}W "
                        f"AND grid {grid:.1f}W > {BATTERY_OFF_EXIT_GRID_EXPORT_W}W (exporting)"
                    ),
                )
            return

        off_until = controller.get("battery_off_until_ts")
        if off_until is None:
            remaining_s = 0
        else:
            remaining_s = off_until - time.time()

        if entry_ready and remaining_s < BATTERY_OFF_REFRESH_WHEN_REMAINING_S:
            if can_attempt_tou_write(controller):
                enter_battery_off(
                    ser,
                    controller,
                    reason=(
                        f"refresh rolling OFF window, remaining={remaining_s / 3600.0:.2f}h, "
                        f"grid {grid:.1f}W importing, SOC {soc}%, battery discharging"
                    ),
                )
        return


# ----------------------------------------------------------------------
# Polling helpers
# ----------------------------------------------------------------------

def poll_modbus(
    ser: serial.Serial,
    slave_id: int,
    func: int,
    start_reg: int,
    count: int,
    label: str,
    debug: bool = False,
) -> Optional[Dict[int, int]]:
    request = build_request(slave_id, func, start_reg, count)

    flush_serial_input(ser)

    if debug:
        print(f"[{label}] send: slave={slave_id} func=0x{func:02X} start={start_reg} count={count}")
        print(f"[{label}] tx_bytes={request.hex()}")

    ser.write(request)
    ser.flush()

    start = time.monotonic()
    buf = bytearray()

    while (time.monotonic() - start) < REPLY_TIMEOUT_S:
        chunk = ser.read(512)
        if chunk:
            buf.extend(chunk)
            if debug:
                print(f"[{label}] rx_chunk={len(chunk)} bytes, buf={len(buf)} bytes")
            frame = extract_frame_from_buffer(buf, slave_id, func, count)
            if frame is not None:
                if debug:
                    print(f"[{label}] frame_found len={len(frame)} crc_ok={crc_ok(frame)}")
                regs = parse_modbus_response(frame, start_reg, count)
                if debug and regs is not None:
                    print(f"[{label}] parsed_regs={len(regs)}")
                return regs
        else:
            time.sleep(0.01)

    if debug:
        print(f"[{label}] timeout waiting for response")
    return None


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------

def main():
    print(f"Starting Solis-only poller on {PORT} @ {BAUDRATE}")
    print(f"Solis: slave {SOLIS_SLAVE_ID}, FC04, regs {SOLIS_START_DOC_REG}-{SOLIS_END_DOC_REG}")
    print(f"Influx URL: {INFLUX_URL}")
    print("")
    print("Timing / reliability constants:")
    print(f"  POLL_INTERVAL_S              = {POLL_INTERVAL_S}s")
    print(f"  REPLY_TIMEOUT_S              = {REPLY_TIMEOUT_S}s")
    print(f"  WRITE_REPLY_TIMEOUT_S        = {WRITE_REPLY_TIMEOUT_S}s")
    print(f"  SOLIS_WRITE_DELAY_S          = {SOLIS_WRITE_DELAY_S}s  (inter-write pacing for FC06)")
    print(f"  SOLIS_POST_WRITE_RECOVERY_S  = {SOLIS_POST_WRITE_RECOVERY_S}s  (pause after ToU write batch)")
    print("")
    print("Battery-off controller:")
    print(f"  ENTER: grid < {BATTERY_OFF_ENTER_GRID_IMPORT_W}W (importing) AND SOC < {BATTERY_OFF_ENTER_SOC}% AND battery discharging (flag={BATTERY_DIRECTION_DISCHARGING})")
    print(f"  EXIT : PV > {BATTERY_OFF_EXIT_PV_W}W AND grid > +{BATTERY_OFF_EXIT_GRID_EXPORT_W}W (exporting)")
    print(f"  OFF  : sync clock, RUN+grid-charge, 0.1A, now{BATTERY_OFF_START_OFFSET_MINUTES:+d}min -> now+{BATTERY_OFF_WINDOW_HOURS}h")
    print("  RELEASE: sync clock, STOP+grid-charge, 10.0A, 00:00 -> 00:00")
    print("  State persistence: disabled")
    print("  Startup: if entry conditions not met, assume NORMAL without writing ToU registers")
    print("")

    poll_count = 0
    read_errors = 0

    controller = {
        "state": STATE_UNKNOWN,
        "battery_off_until_ts": None,
        "last_tou_write_ts": 0,
    }

    with serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=READ_IDLE_TIMEOUT_S,
        write_timeout=1,
    ) as ser:
        while True:
            loop_start = time.monotonic()

            flush_serial_input(ser)

            # Poll Solis.
            solis_regs = poll_modbus(
                ser,
                SOLIS_SLAVE_ID,
                FUNC_READ_INPUT_REGS,
                SOLIS_START_DOC_REG,
                SOLIS_REG_COUNT,
                label="solis",
                debug=False,
            )

            if solis_regs is not None:
                poll_count += 1
                s = decode_solis(solis_regs)

                # Run controller immediately after a valid Solis poll.
                manage_battery_off_controller(ser, s, controller)

                write_solis_metrics(
                    s,
                    poll_count,
                    read_errors,
                    controller_state=controller.get("state", STATE_UNKNOWN),
                )
            else:
                print("[solis] FAIL: no response")
                read_errors += 1

            elapsed = time.monotonic() - loop_start
            sleep_for = POLL_INTERVAL_S - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)


if __name__ == "__main__":
    main()
    
