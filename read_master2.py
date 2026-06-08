#!/usr/bin/env python3
"""
Pi-side master poller.

- Polls Solis inverter as slave 1.
- Polls ESP32 battery bridge as slave 5.
- Writes Solis metrics to InfluxDB.
- Writes battery slot telemetry to InfluxDB as battery_1_*, battery_2_*, battery_3_*.
- Adds Solis ToU low-battery protection logic.
- Syncs the Solis inverter clock from the Pi clock whenever ToU registers are written.

Battery protection logic:

NORMAL / UNKNOWN:
  If pvTotalPowerW < 80W and Solis SOC < 16%:
    Write Solis ToU:
      sync Solis clock to Pi time
      RUN with grid-charge allowed
      charge current 0.0A
      charge window now+2min -> now+12h
    state = BATTERY_OFF

BATTERY_OFF:
  If pvTotalPowerW > 110W:
    Write Solis ToU:
      sync Solis clock to Pi time
      STOP with grid-charge allowed
      charge current 10.0A
      charge window 00:00 -> 00:00
    state = NORMAL

BATTERY_OFF refresh:
  If still pvTotalPowerW < 80W and SOC < 16%, and the current 12h
  OFF window has less than 2 hours remaining, refresh the OFF window.

No persistent state is used. On script restart, live PV/SOC conditions decide
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

PORT = "/dev/ttyUSB0"
BAUDRATE = 9600

INFLUX_URL = "http://localhost:8086/write?db=power"
HOST_TAG = "server01"

SOLIS_SOURCE_TAG = "solis_master"
BATTERY_SOURCE_TAG = "battery_master"

SOLIS_SLAVE_ID = 0x01
BATTERY_SLAVE_ID = 0x05

FUNC_READ_INPUT_REGS = 0x04
FUNC_READ_HOLDING_REGS = 0x03
FUNC_WRITE_SINGLE_REG = 0x06

# Solis input-register block
SOLIS_START_DOC_REG = 33050
SOLIS_END_DOC_REG = 33142
SOLIS_REG_COUNT = SOLIS_END_DOC_REG - SOLIS_START_DOC_REG + 1  # 93

# ESP32 battery bridge block
BATTERY_START_REG = 0
BATTERY_REG_COUNT = 80
BATTERY_BLOCK_SIZE = 24
BATTERY_COUNT = 3

POLL_INTERVAL_S = 2.0
REPLY_TIMEOUT_S = 0.30
WRITE_REPLY_TIMEOUT_S = 0.80
READ_IDLE_TIMEOUT_S = 0.05
BUFFER_MAX = 8192

# ----------------------------------------------------------------------
# Solis ToU / battery-off control config
# ----------------------------------------------------------------------

STATE_UNKNOWN = "UNKNOWN"
STATE_NORMAL = "NORMAL"
STATE_BATTERY_OFF = "BATTERY_OFF"

BATTERY_OFF_ENTER_PV_W = 80.0
BATTERY_OFF_EXIT_PV_W = 110.0
BATTERY_OFF_ENTER_SOC = 15

BATTERY_OFF_START_DELAY_MINUTES = 2
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

TOU_CURRENT_0_0A = 0
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


def build_request(slave_id: int, func: int, start_reg: int, count: int) -> bytes:
    # For Solis, start_reg is documented register and frame uses raw_addr=start-1.
    # For ESP32 slave 5, registers are 0-based and raw_addr is same as start_reg.
    raw_addr = start_reg if slave_id == BATTERY_SLAVE_ID else (start_reg - 1)
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
    }


def write_solis_metrics(s: dict, poll_count: int, read_errors: int, controller_state: str):
    write_line("solis_pv1_voltage", s["pv1Voltage"], SOLIS_SOURCE_TAG)
    write_line("solis_pv1_current", s["pv1Current"], SOLIS_SOURCE_TAG)
    write_line("solis_pv2_voltage", s["pv2Voltage"], SOLIS_SOURCE_TAG)
    write_line("solis_pv2_current", s["pv2Current"], SOLIS_SOURCE_TAG)
    write_line("solis_grid_voltage", s["gridVoltage"], SOLIS_SOURCE_TAG)
    write_line("solis_grid_frequency", s["gridFrequency"], SOLIS_SOURCE_TAG)
    write_line("solis_grid_power", s["gridPower"], SOLIS_SOURCE_TAG)
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
        time.sleep(0.10)

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
    run_stop_last: bool,
) -> bool:
    """
    Sync the Solis clock, then write the six confirmed ToU registers.

    run_stop_last=True:
      Write config first, then RUN/STOP. Used when entering RUN so stale
      settings are not briefly enabled.

    run_stop_last=False:
      Write RUN/STOP first, then config. Used when exiting BATTERY_OFF so
      STOP happens immediately.
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

    writes = writes_config + writes_run_stop if run_stop_last else writes_run_stop + writes_config

    all_ok = clock_ok
    for reg, value in writes:
        ok = write_solis_holding_reg(ser, reg, value)
        all_ok = all_ok and ok
        time.sleep(0.10)

    return all_ok


def calc_battery_off_window():
    now = datetime.now()
    start = now + timedelta(minutes=BATTERY_OFF_START_DELAY_MINUTES)
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
        f"[tou] Target: RUN+grid-charge, 0.0A, "
        f"{window['start_text']} -> {window['end_text']}"
    )
    print("=" * 80)

    controller["last_tou_write_ts"] = time.time()

    ok = write_tou_registers(
        ser=ser,
        run_stop=TOU_RUN,
        current_x10=TOU_CURRENT_0_0A,
        start_hour=window["start_hour"],
        start_minute=window["start_minute"],
        end_hour=window["end_hour"],
        end_minute=window["end_minute"],
        run_stop_last=True,
    )

    if ok:
        controller["state"] = STATE_BATTERY_OFF
        controller["battery_off_until_ts"] = window["end_timestamp"]
        print("[tou] BATTERY_OFF active")
    else:
        print("[tou] BATTERY_OFF write failed; state unchanged")

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
        run_stop_last=False,
    )

    if ok:
        controller["state"] = STATE_NORMAL
        controller["battery_off_until_ts"] = None
        print("[tou] NORMAL active")
    else:
        print("[tou] STOP/release write failed; state unchanged")

    return ok


def can_attempt_tou_write(controller: dict) -> bool:
    last = controller.get("last_tou_write_ts", 0)
    return (time.time() - last) >= TOU_MIN_WRITE_INTERVAL_S


def manage_battery_off_controller(ser: serial.Serial, s: dict, controller: dict) -> None:
    pv = float(s.get("pvTotalPowerW", 0.0))
    soc = int(s.get("batterySoc", 0))
    state = controller.get("state", STATE_UNKNOWN)

    low_pv_low_soc = pv < BATTERY_OFF_ENTER_PV_W and soc < BATTERY_OFF_ENTER_SOC
    pv_recovered = pv > BATTERY_OFF_EXIT_PV_W

    print(
        f"[tou] state={state} pv={pv:.1f}W soc={soc}% "
        f"low_pv_low_soc={low_pv_low_soc} pv_recovered={pv_recovered}"
    )

    # Startup recovery.
    if state == STATE_UNKNOWN:
        if low_pv_low_soc:
            if can_attempt_tou_write(controller):
                enter_battery_off(
                    ser,
                    controller,
                    reason=f"startup/unknown and PV {pv:.1f}W < {BATTERY_OFF_ENTER_PV_W}W, SOC {soc}% < {BATTERY_OFF_ENTER_SOC}%",
                )
            return

        if pv_recovered:
            if can_attempt_tou_write(controller):
                exit_battery_off(
                    ser,
                    controller,
                    reason=f"startup/unknown and PV {pv:.1f}W > {BATTERY_OFF_EXIT_PV_W}W",
                )
            return

        # No strong condition either way; treat as normal runtime, but do not write.
        controller["state"] = STATE_NORMAL
        return

    # Normal -> Battery-off.
    if state == STATE_NORMAL:
        if low_pv_low_soc:
            if can_attempt_tou_write(controller):
                enter_battery_off(
                    ser,
                    controller,
                    reason=f"PV {pv:.1f}W < {BATTERY_OFF_ENTER_PV_W}W, SOC {soc}% < {BATTERY_OFF_ENTER_SOC}%",
                )
        return

    # Battery-off -> Normal, or refresh rolling 12h window.
    if state == STATE_BATTERY_OFF:
        if pv_recovered:
            if can_attempt_tou_write(controller):
                exit_battery_off(
                    ser,
                    controller,
                    reason=f"PV {pv:.1f}W > {BATTERY_OFF_EXIT_PV_W}W",
                )
            return

        off_until = controller.get("battery_off_until_ts")
        if off_until is None:
            remaining_s = 0
        else:
            remaining_s = off_until - time.time()

        if low_pv_low_soc and remaining_s < BATTERY_OFF_REFRESH_WHEN_REMAINING_S:
            if can_attempt_tou_write(controller):
                enter_battery_off(
                    ser,
                    controller,
                    reason=(
                        f"refresh rolling OFF window, remaining={remaining_s / 3600.0:.2f}h, "
                        f"PV {pv:.1f}W, SOC {soc}%"
                    ),
                )
        return


# ----------------------------------------------------------------------
# Battery bridge decode / write
# ----------------------------------------------------------------------

def parse_battery_block(regs: Dict[int, int], battery_index: int) -> dict:
    base = battery_index * BATTERY_BLOCK_SIZE + 8

    enabled = regs.get(base + 0, 0) == 1
    has_data = regs.get(base + 1, 0) == 1
    voltage = regs.get(base + 2, 0) / 100.0
    current = s16(regs.get(base + 3, 0)) / 100.0
    soc = regs.get(base + 4, 0)
    charge_mos = regs.get(base + 5, 0) == 1
    discharge_mos = regs.get(base + 6, 0) == 1
    temp_raw = s16(regs.get(base + 7, 0))
    temperature = None if temp_raw == 0 else (temp_raw / 10.0)
    cell_count = regs.get(base + 8, 0)

    cells = []
    for i in range(14):
        if i < cell_count:
            cells.append(regs.get(base + 9 + i, 0))

    return {
        "enabled": enabled,
        "hasData": has_data,
        "voltage": voltage,
        "current": current,
        "soc": soc,
        "chargeMos": charge_mos,
        "dischargeMos": discharge_mos,
        "temperature": temperature,
        "cellCount": cell_count,
        "cells": cells,
    }


def write_battery_metrics(bat: dict, battery_name: str):
    meas = f"battery_{battery_name}"
    write_line(f"{meas}_enabled", 1 if bat["enabled"] else 0, BATTERY_SOURCE_TAG)
    write_line(f"{meas}_has_data", 1 if bat["hasData"] else 0, BATTERY_SOURCE_TAG)
    write_line(f"{meas}_voltage", round(bat["voltage"], 2), BATTERY_SOURCE_TAG)
    write_line(f"{meas}_current", round(bat["current"], 2), BATTERY_SOURCE_TAG)
    write_line(f"{meas}_soc", bat["soc"], BATTERY_SOURCE_TAG)
    write_line(f"{meas}_charge_mos", 1 if bat["chargeMos"] else 0, BATTERY_SOURCE_TAG)
    write_line(f"{meas}_discharge_mos", 1 if bat["dischargeMos"] else 0, BATTERY_SOURCE_TAG)
    write_line(f"{meas}_temperature", bat["temperature"], BATTERY_SOURCE_TAG)
    write_line(f"{meas}_cell_count", bat["cellCount"], BATTERY_SOURCE_TAG)

    for idx, mv in enumerate(bat["cells"], start=1):
        write_line(f"{meas}_cell_{idx}_mv", mv, BATTERY_SOURCE_TAG)


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
    print(f"Starting master poller on {PORT} @ {BAUDRATE}")
    print(f"Solis: slave {SOLIS_SLAVE_ID}, FC04, regs {SOLIS_START_DOC_REG}-{SOLIS_END_DOC_REG}")
    print(f"Battery bridge: slave {BATTERY_SLAVE_ID}, FC03, regs {BATTERY_START_REG}-{BATTERY_START_REG + BATTERY_REG_COUNT - 1}")
    print("")
    print("Battery-off controller:")
    print(f"  ENTER: PV < {BATTERY_OFF_ENTER_PV_W}W and SOC < {BATTERY_OFF_ENTER_SOC}%")
    print(f"  EXIT : PV > {BATTERY_OFF_EXIT_PV_W}W")
    print(f"  OFF  : sync clock, RUN+grid-charge, 0.0A, now+{BATTERY_OFF_START_DELAY_MINUTES}min -> now+{BATTERY_OFF_WINDOW_HOURS}h")
    print("  RELEASE: sync clock, STOP+grid-charge, 10.0A, 00:00 -> 00:00")
    print("  State persistence: disabled")
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

            # Poll ESP32 battery bridge.
            battery_regs = poll_modbus(
                ser,
                BATTERY_SLAVE_ID,
                FUNC_READ_HOLDING_REGS,
                BATTERY_START_REG,
                BATTERY_REG_COUNT,
                label="battery5",
                debug=True,
            )

            if battery_regs is None:
                print("[battery5] FAIL: no response from slave 5")
                read_errors += 1
            else:
                print(f"[battery5] OK: got {len(battery_regs)} registers")

                for idx in range(BATTERY_COUNT):
                    bat = parse_battery_block(battery_regs, idx)
                    print(
                        f"[battery5] slot={idx + 1} enabled={bat['enabled']} "
                        f"hasData={bat['hasData']} V={bat['voltage']:.2f} "
                        f"I={bat['current']:.2f} SOC={bat['soc']} "
                        f"chg={bat['chargeMos']} dischg={bat['dischargeMos']} "
                        f"temp={bat['temperature']} cells={bat['cellCount']}"
                    )
                    write_battery_metrics(bat, str(idx + 1))

            elapsed = time.monotonic() - loop_start
            sleep_for = POLL_INTERVAL_S - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)


if __name__ == "__main__":
    main()
