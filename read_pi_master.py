#!/usr/bin/env python3
"""
Pi-side master poller.

Behavior:
- Poll Solis inverter as Modbus slave 1 exactly like the current V6 script.
- Poll ESP32 battery bridge as Modbus slave 5.
- Write Solis metrics to InfluxDB as before.
- Write each battery slot from the ESP32 slave into InfluxDB under batteryX measurements.

ESP32 slave register map (FC03 holding registers, slave id 5):
- 0..7   : aggregate block
- 8..31  : battery 0
- 32..55 : battery 1
- 56..79 : battery 2

Each battery block:
- enabled
- hasData
- voltage ×100
- current ×100 signed
- soc
- chargeMos
- dischargeMos
- temperature ×10 signed
- cellCount
- 14 cell millivolt values
- reserved
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

SolisSourceTag = "solis_master"
BatterySourceTag = "battery_master"

SOLIS_SLAVE_ID = 0x01
BATTERY_SLAVE_ID = 0x05

FUNC_READ_INPUT_REGS = 0x04
FUNC_READ_HOLDING_REGS = 0x03

# Solis block (existing behavior)
SOLIS_START_DOC_REG = 33050
SOLIS_END_DOC_REG = 33142
SOLIS_REG_COUNT = SOLIS_END_DOC_REG - SOLIS_START_DOC_REG + 1  # 93

# Battery bridge block
BATTERY_START_REG = 0
BATTERY_REG_COUNT = 80
BATTERY_BLOCK_SIZE = 24
BATTERY_COUNT = 3

POLL_INTERVAL_S = 5.0
REPLY_TIMEOUT_S = 0.30
READ_IDLE_TIMEOUT_S = 0.05
BUFFER_MAX = 8192

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


def build_request(slave_id: int, func: int, start_reg: int, count: int) -> bytes:
    # Modbus uses 0-based raw address in the frame.
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
        reg_addr = start_reg + i
        raw = (data[i * 2] << 8) | data[i * 2 + 1]
        regs[reg_addr] = raw
    return regs


def s16(v: int) -> int:
    return v - 65536 if v >= 32768 else v


# ----------------------------------------------------------------------
# Influx helper
# ----------------------------------------------------------------------

def write_line(measurement: str, value, source_tag: str):
    if value is None:
        return
    line = f"{measurement},host={HOST_TAG},source={source_tag} value={value}"
    print(line)
    try:
        requests.post(INFLUX_URL, data=line.encode("utf-8"), timeout=2)
    except Exception as e:
        print(f"Influx write failed: {e}")


# ----------------------------------------------------------------------
# Solis decode
# ----------------------------------------------------------------------

def decode_solis(regs: Dict[int, int]) -> dict:
    pv1_v_raw = regs.get(33050, 0)
    pv1_i_raw = regs.get(33051, 0)
    pv2_v_raw = regs.get(33052, 0)
    pv2_i_raw = regs.get(33053, 0)

    pv1_v = round(pv1_v_raw / 10.0, 1)
    pv1_i = round(pv1_i_raw / 10.0, 1)
    pv2_v = round(pv2_v_raw / 10.0, 1)
    pv2_i = round(pv2_i_raw / 10.0, 1)

    grid_v = round(regs.get(33074, 0) / 10.0, 1)
    grid_f = round(regs.get(33095, 0) / 100.0, 2)

    # Existing legacy behavior: grid power from 33132
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
        "rawGridFrequency": regs.get(33095, 0),
    }


def write_solis_metrics(s: dict, poll_count: int, read_errors: int):
    write_line("solis_pv1_voltage", s["pv1Voltage"], SolisSourceTag)
    write_line("solis_pv1_current", s["pv1Current"], SolisSourceTag)
    write_line("solis_pv2_voltage", s["pv2Voltage"], SolisSourceTag)
    write_line("solis_pv2_current", s["pv2Current"], SolisSourceTag)
    write_line("solis_grid_voltage", s["gridVoltage"], SolisSourceTag)
    write_line("solis_grid_frequency", s["gridFrequency"], SolisSourceTag)
    write_line("solis_grid_power", s["gridPower"], SolisSourceTag)
    write_line("solis_battery_soc", s["batterySoc"], SolisSourceTag)
    write_line("solis_battery_voltage", s["batteryVoltage"], SolisSourceTag)
    write_line("solis_battery_current", s["batteryCurrent"], SolisSourceTag)
    write_line("solis_battery_direction_flag", s["batteryDirectionFlag"], SolisSourceTag)
    write_line("solis_battery_power", s["batteryPowerW"], SolisSourceTag)
    write_line("solis_pv1_power", s["pv1PowerW"], SolisSourceTag)
    write_line("solis_pv2_power", s["pv2PowerW"], SolisSourceTag)
    write_line("solis_pv_total_power", s["pvTotalPowerW"], SolisSourceTag)
    write_line("solis_poll_count", poll_count, SolisSourceTag)
    write_line("solis_read_errors", read_errors, SolisSourceTag)
    write_line("solis_reg_33095_raw", s["rawGridFrequency"], SolisSourceTag)


# ----------------------------------------------------------------------
# Battery bridge decode
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
    temperature_raw = s16(regs.get(base + 7, 0))
    temperature = temperature_raw / 10.0 if temperature_raw != 0 else None
    cell_count = regs.get(base + 8, 0)

    cells = []
    for i in range(14):
        cell_val = regs.get(base + 9 + i, 0)
        if i < cell_count:
            cells.append(cell_val)

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

    write_line(f"{meas}_enabled", 1 if bat["enabled"] else 0, BatterySourceTag)
    write_line(f"{meas}_has_data", 1 if bat["hasData"] else 0, BatterySourceTag)
    write_line(f"{meas}_voltage", round(bat["voltage"], 2), BatterySourceTag)
    write_line(f"{meas}_current", round(bat["current"], 2), BatterySourceTag)
    write_line(f"{meas}_soc", bat["soc"], BatterySourceTag)
    write_line(f"{meas}_charge_mos", 1 if bat["chargeMos"] else 0, BatterySourceTag)
    write_line(f"{meas}_discharge_mos", 1 if bat["dischargeMos"] else 0, BatterySourceTag)
    write_line(f"{meas}_temperature", bat["temperature"], BatterySourceTag)
    write_line(f"{meas}_cell_count", bat["cellCount"], BatterySourceTag)

    for idx, mv in enumerate(bat["cells"], start=1):
        write_line(f"{meas}_cell_{idx}_mv", mv, BatterySourceTag)


# ----------------------------------------------------------------------
# Polling helpers
# ----------------------------------------------------------------------

def poll_modbus(ser: serial.Serial, slave_id: int, func: int, start_reg: int, count: int) -> Optional[Dict[int, int]]:
    request = build_request(slave_id, func, start_reg, count)

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
            frame = extract_frame_from_buffer(buf, slave_id, func, count)
            if frame is not None:
                return parse_modbus_response(frame, start_reg, count)
        else:
            time.sleep(0.01)

    return None


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------

def main():
    print(f"Starting master poller on {PORT} @ {BAUDRATE}")
    print(f"Solis: slave {SOLIS_SLAVE_ID}, FC04, regs {SOLIS_START_DOC_REG}-{SOLIS_END_DOC_REG}")
    print(f"Battery bridge: slave {BATTERY_SLAVE_ID}, FC03, regs {BATTERY_START_REG}-{BATTERY_START_REG + BATTERY_REG_COUNT - 1}")

    poll_count = 0
    read_errors = 0

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

            # 1) Poll Solis exactly as before
            solis_regs = poll_modbus(ser, SOLIS_SLAVE_ID, FUNC_READ_INPUT_REGS, SOLIS_START_DOC_REG, SOLIS_REG_COUNT)
            if solis_regs is None:
                read_errors += 1
                print(f"[WARN] Solis poll failed (errors={read_errors})")
            else:
                poll_count += 1
                s = decode_solis(solis_regs)
                print("\n=== SOLIS MASTER DUMP V6 ===")
                print(
                    f"pollCount={poll_count} readErrors={read_errors} "
                    f"pv1={s['pv1Voltage']:.1f}V {s['pv1Current']:.1f}A  "
                    f"pv2={s['pv2Voltage']:.1f}V {s['pv2Current']:.1f}A  "
                    f"grid={s['gridVoltage']:.1f}V  "
                    f"gridP={s['gridPower']}W  "
                    f"batt={s['batteryVoltage']:.2f}V {s['batteryCurrent']:.1f}A "
                    f"soc={s['batterySoc']}% dir={s['batteryDirection']}"
                )
                write_solis_metrics(s, poll_count, read_errors)
                print(json.dumps({"solis": s}, separators=(",", ":")))

            # 2) Poll ESP32 battery bridge slave 5
            battery_regs = poll_modbus(ser, BATTERY_SLAVE_ID, FUNC_READ_HOLDING_REGS, BATTERY_START_REG, BATTERY_REG_COUNT)
            if battery_regs is None:
                print("[WARN] Battery bridge poll failed")
            else:
                batteries = []
                for idx in range(BATTERY_COUNT):
                    bat = parse_battery_block(battery_regs, idx)
                    batteries.append(bat)

                    battery_name = f"{idx + 1}"
                    write_battery_metrics(bat, battery_name)

                print(json.dumps({"batteryBridge": batteries}, separators=(",", ":")))

            elapsed = time.monotonic() - loop_start
            sleep_for = POLL_INTERVAL_S - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)


if __name__ == "__main__":
    main()
