#!/usr/bin/env python3
"""
Solis RS485 master poller, version 6.

This version restores the legacy register mapping used by the ESP32 code
(BLE_BMS_MARK2.ino) so Grafana/Influx remain compatible.

Legacy fields restored:
- 33050 PV1 voltage
- 33051 PV1 current
- 33052 PV2 voltage
- 33053 PV2 current
- 33132 grid power
- 33135 battery current
- 33136 battery direction flag
- 33140 battery SoC
- 33142 battery voltage

Derived values:
- PV1 power
- PV2 power
- PV total power
- battery power (from battery voltage/current + direction)

Still included:
- full raw register dump for all 93 registers
- tentative/unknown registers for future analysis
- changed-since-last-poll register summary
"""

import json
import time
from typing import Dict, Optional, Tuple

import requests
import serial

# ----------------------------------------------------------------------
# Config
# ----------------------------------------------------------------------

PORT = "/dev/ttyUSB0"
BAUDRATE = 9600

INFLUX_URL = "http://localhost:8086/write?db=power"
HOST_TAG = "server01"
SOURCE_TAG = "solis_master"

SLAVE_ID = 0x01
FUNC_READ_INPUT_REGS = 0x04

START_DOC_REG = 33050
END_DOC_REG = 33142
REG_COUNT = END_DOC_REG - START_DOC_REG + 1  # 93

POLL_INTERVAL_S = 5.0
REPLY_TIMEOUT_S = 0.30
READ_IDLE_TIMEOUT_S = 0.05
BUFFER_MAX = 8192

# ----------------------------------------------------------------------
# Register metadata
# ----------------------------------------------------------------------

SAFE_REGISTERS = {
    33050: ("pv1Voltage", 10.0, False, "V", "DC Voltage 1 / PV1 voltage"),
    33051: ("pv1Current", 10.0, False, "A", "DC Current 1 / PV1 current"),
    33052: ("pv2Voltage", 10.0, False, "V", "DC Voltage 2 / PV2 voltage"),
    33053: ("pv2Current", 10.0, False, "A", "DC Current 2 / PV2 current"),
    33132: ("gridPower", 1.0, True, "W", "grid power"),
    33135: ("batteryCurrent", 10.0, True, "A", "battery current"),
    33136: ("batteryDirectionFlag", 1.0, False, "", "0 charge / 1 discharge"),
    33140: ("batterySoc", 1.0, False, "%", "battery state of charge"),
    33142: ("batteryVoltage", 100.0, False, "V", "battery voltage"),
}

TENTATIVE_REGISTERS = {
    33059: ("reg33059", 1.0, False, "", "observed non-zero value; meaning unclear"),
    33072: ("dcBusHalfVoltage", 10.0, False, "V", "DC bus half voltage"),
    33080: ("reg33080", 1.0, False, "", "unresolved"),
    33081: ("reg33081", 1.0, False, "", "unresolved"),
    33085: ("reg33085", 1.0, False, "", "unresolved"),
    33095: ("gridFrequencyRaw", 1.0, False, "", "raw field currently left tentative"),
    33129: ("meterCurrent", 10.0, True, "A", "meter current"),
    33130: ("meterActivePowerHi", 1.0, False, "", "meter active power high word"),
    33131: ("meterActivePowerLo", 1.0, False, "", "meter active power low word"),
    33133: ("reg33133", 1.0, False, "", "unresolved"),
    33134: ("reg33134", 1.0, False, "", "unresolved"),
    33137: ("backupAcVoltagePhaseA", 10.0, False, "V", "backup AC voltage phase A"),
    33141: ("reg33141", 1.0, False, "", "unresolved"),
}

REGISTER_META = {}
REGISTER_META.update(SAFE_REGISTERS)
REGISTER_META.update(TENTATIVE_REGISTERS)

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


def build_read_request() -> bytes:
    raw_addr = START_DOC_REG - 1
    frame = bytes([
        SLAVE_ID,
        FUNC_READ_INPUT_REGS,
        (raw_addr >> 8) & 0xFF,
        raw_addr & 0xFF,
        (REG_COUNT >> 8) & 0xFF,
        REG_COUNT & 0xFF,
    ])
    crc = modbus_crc(frame)
    return frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def extract_frame_from_buffer(buf: bytearray) -> Optional[bytes]:
    expected_len = 3 + (REG_COUNT * 2) + 2
    if len(buf) < expected_len:
        return None

    for i in range(0, len(buf) - expected_len + 1):
        if buf[i] != SLAVE_ID or buf[i + 1] != FUNC_READ_INPUT_REGS:
            continue
        candidate = bytes(buf[i:i + expected_len])
        if candidate[2] != REG_COUNT * 2:
            continue
        if crc_ok(candidate):
            del buf[:i + expected_len]
            return candidate

    if len(buf) > BUFFER_MAX:
        del buf[:-expected_len]
    return None


def parse_modbus_response(frame: bytes) -> Optional[Dict[int, int]]:
    if len(frame) != 3 + (REG_COUNT * 2) + 2:
        return None
    if frame[0] != SLAVE_ID or frame[1] != FUNC_READ_INPUT_REGS:
        return None
    if frame[2] != REG_COUNT * 2:
        return None
    if not crc_ok(frame):
        return None

    data = frame[3:-2]
    regs: Dict[int, int] = {}
    for i in range(REG_COUNT):
        reg_addr = START_DOC_REG + i
        raw = (data[i * 2] << 8) | data[i * 2 + 1]
        regs[reg_addr] = raw
    return regs


# ----------------------------------------------------------------------
# Decoding helpers
# ----------------------------------------------------------------------

def s16(v: int) -> int:
    return v - 65536 if v >= 32768 else v


def decode_meta(reg: int, raw: int) -> Tuple[str, float, str, str]:
    meta = REGISTER_META.get(reg)
    if meta is None:
        return (f"reg{reg}", float(raw), "", "unknown")

    label, divisor, signed_value, unit, note = meta
    value = s16(raw) if signed_value else raw

    if divisor == 0:
        scaled = float(value)
    elif divisor == 1.0 and not signed_value:
        scaled = float(value)
    else:
        scaled = float(value) / divisor

    return (label, scaled, unit, note)


def build_payload(regs: Dict[int, int], uptime_ms: int, poll_count: int, read_errors: int) -> dict:
    payload = {
        "uptimeMs": uptime_ms,
        "pollCount": poll_count,
        "readErrors": read_errors,
        "slaveId": SLAVE_ID,
        "function": FUNC_READ_INPUT_REGS,
        "startDocReg": START_DOC_REG,
        "endDocReg": END_DOC_REG,
        "registerCount": REG_COUNT,
    }

    # Legacy dashboard values from the ESP32 mapping.
    pv1_v_raw = regs.get(33050, 0)
    pv1_i_raw = regs.get(33051, 0)
    pv2_v_raw = regs.get(33052, 0)
    pv2_i_raw = regs.get(33053, 0)

    payload["pv1Voltage"] = round(pv1_v_raw / 10.0, 1)
    payload["pv1Current"] = round(pv1_i_raw / 10.0, 1)
    payload["pv2Voltage"] = round(pv2_v_raw / 10.0, 1)
    payload["pv2Current"] = round(pv2_i_raw / 10.0, 1)

    payload["gridVoltage"] = round(regs.get(33074, 0) / 10.0, 1)
    payload["gridPower"] = s16(regs.get(33132, 0))
    payload["batteryCurrent"] = round(s16(regs.get(33135, 0)) / 10.0, 1)
    payload["batteryDirectionFlag"] = regs.get(33136, 0)
    payload["batteryDirection"] = "discharging" if payload["batteryDirectionFlag"] == 1 else "charging"
    payload["batterySoc"] = regs.get(33140, 0)
    payload["batteryVoltage"] = round(regs.get(33142, 0) / 100.0, 2)

    payload["pv1PowerW"] = round(payload["pv1Voltage"] * payload["pv1Current"], 1)
    payload["pv2PowerW"] = round(payload["pv2Voltage"] * payload["pv2Current"], 1)
    payload["pvTotalPowerW"] = round(payload["pv1PowerW"] + payload["pv2PowerW"], 1)
    payload["batteryPowerW"] = round(payload["batteryVoltage"] * payload["batteryCurrent"], 1)
    if payload["batteryDirectionFlag"] == 1:
        payload["batteryPowerW"] = -payload["batteryPowerW"]

    # Keep a few additional values around as raw diagnostics.
    payload["gridFrequencyRaw"] = regs.get(33095, 0)

    # Full raw register dump.
    dump = []
    for reg in range(START_DOC_REG, END_DOC_REG + 1):
        raw = regs.get(reg, 0)
        label, scaled, unit, note = decode_meta(reg, raw)
        dump.append({
            "reg": reg,
            "label": label,
            "raw": raw,
            "signed": s16(raw),
            "scaled": scaled,
            "unit": unit,
            "note": note,
            "known": reg in REGISTER_META,
            "safe": reg in SAFE_REGISTERS,
        })
    payload["registers"] = dump

    return payload


# ----------------------------------------------------------------------
# Influx
# ----------------------------------------------------------------------

def write_line(measurement: str, value):
    if value is None:
        return
    line = f"{measurement},host={HOST_TAG},source={SOURCE_TAG} value={value}"
    print(line)
    try:
        requests.post(INFLUX_URL, data=line.encode("utf-8"), timeout=2)
    except Exception as e:
        print(f"Influx write failed: {e}")


def write_metrics(payload: dict):
    write_line("solis_pv1_voltage", payload["pv1Voltage"])
    write_line("solis_pv1_current", payload["pv1Current"])
    write_line("solis_pv2_voltage", payload["pv2Voltage"])
    write_line("solis_pv2_current", payload["pv2Current"])
    write_line("solis_grid_voltage", payload["gridVoltage"])
    write_line("solis_grid_power", payload["gridPower"])
    write_line("solis_battery_soc", payload["batterySoc"])
    write_line("solis_battery_voltage", payload["batteryVoltage"])
    write_line("solis_battery_current", payload["batteryCurrent"])
    write_line("solis_battery_direction_flag", payload["batteryDirectionFlag"])
    write_line("solis_battery_power", payload["batteryPowerW"])
    write_line("solis_pv1_power", payload["pv1PowerW"])
    write_line("solis_pv2_power", payload["pv2PowerW"])
    write_line("solis_pv_total_power", payload["pvTotalPowerW"])
    write_line("solis_poll_count", payload["pollCount"])
    write_line("solis_read_errors", payload["readErrors"])

    # Diagnostics
    write_line("solis_reg_33095_raw", payload["gridFrequencyRaw"])


# ----------------------------------------------------------------------
# Logging
# ----------------------------------------------------------------------

def diff_registers(prev: Optional[Dict[int, int]], curr: Dict[int, int]):
    if prev is None:
        return
    changes = []
    for reg in range(START_DOC_REG, END_DOC_REG + 1):
        old = prev.get(reg, None)
        new = curr.get(reg, None)
        if old != new:
            changes.append((reg, old, new))
    if not changes:
        print("changed: none")
        return
    print("changed:")
    for reg, old, new in changes:
        if old is None:
            print(f"  {reg}: <none> -> {new}")
        else:
            print(f"  {reg}: {old} -> {new}")


def print_summary(payload: dict, prev_regs: Optional[Dict[int, int]], curr_regs: Dict[int, int]):
    print("\n=== SOLIS MASTER DUMP V6 ===")
    print(
        f"uptimeMs={payload['uptimeMs']} pollCount={payload['pollCount']} "
        f"readErrors={payload['readErrors']} registers={payload['registerCount']}"
    )
    print(
        f"pv1={payload['pv1Voltage']:.1f}V {payload['pv1Current']:.1f}A  "
        f"pv2={payload['pv2Voltage']:.1f}V {payload['pv2Current']:.1f}A  "
        f"grid={payload['gridVoltage']:.1f}V  "
        f"gridP={payload['gridPower']}W  "
        f"batt={payload['batteryVoltage']:.2f}V {payload['batteryCurrent']:.1f}A "
        f"soc={payload['batterySoc']}% dir={payload['batteryDirection']}"
    )
    diff_registers(prev_regs, curr_regs)

    for item in payload["registers"]:
        flag = "safe" if item["safe"] else "tent"
        print(
            f"{item['reg']} {item['label']:<28} "
            f"raw={item['raw']:<6} signed={item['signed']:<6} "
            f"scaled={item['scaled']:.3f}{item['unit']:<2}  "
            f"[{flag}]  # {item['note']}"
        )
    print("=============================\n")


# ----------------------------------------------------------------------
# Polling
# ----------------------------------------------------------------------

def poll_once(ser: serial.Serial, poll_count: int, read_errors: int) -> Tuple[Optional[dict], Optional[Dict[int, int]], int, int]:
    request = build_read_request()

    while ser.in_waiting:
        ser.read(ser.in_waiting)

    ser.write(request)
    ser.flush()

    start = time.monotonic()
    buf = bytearray()

    while (time.monotonic() - start) < REPLY_TIMEOUT_S:
        chunk = ser.read(512)
        if chunk:
            buf.extend(chunk)
            frame = extract_frame_from_buffer(buf)
            if frame is not None:
                regs = parse_modbus_response(frame)
                if regs is None:
                    read_errors += 1
                    return None, None, poll_count, read_errors
                poll_count += 1
                uptime_ms = int(time.monotonic() * 1000)
                payload = build_payload(regs, uptime_ms, poll_count, read_errors)
                return payload, regs, poll_count, read_errors
        else:
            time.sleep(0.01)

    read_errors += 1
    return None, None, poll_count, read_errors


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------

def main():
    print(f"Starting Solis master poller V6 on {PORT} @ {BAUDRATE}")
    print(f"Polling slave {SLAVE_ID}, function 0x{FUNC_READ_INPUT_REGS:02X}, regs {START_DOC_REG}-{END_DOC_REG}")

    poll_count = 0
    read_errors = 0
    prev_regs: Optional[Dict[int, int]] = None

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

            payload, curr_regs, poll_count, read_errors = poll_once(ser, poll_count, read_errors)
            if payload is None or curr_regs is None:
                print(f"[WARN] poll failed (errors={read_errors})")
            else:
                print_summary(payload, prev_regs, curr_regs)
                write_metrics(payload)
                print(json.dumps(payload, separators=(",", ":")))
                prev_regs = curr_regs

            elapsed = time.monotonic() - loop_start
            sleep_for = POLL_INTERVAL_S - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)


if __name__ == "__main__":
    main()
