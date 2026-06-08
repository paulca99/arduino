#!/usr/bin/env python3
"""
Pi-side master poller.

- Polls Solis inverter as slave 1 (existing behavior).
- Polls ESP32 battery bridge as slave 5.
- Writes Solis metrics as before.
- Writes battery slot telemetry to Influx as battery_1_*, battery_2_*, battery_3_*.
- Adds focused debug for slave 5 polling so we can identify failures.
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

SOLIS_SOURCE_TAG = "solis_master"
BATTERY_SOURCE_TAG = "battery_master"

SOLIS_SLAVE_ID = 0x01
BATTERY_SLAVE_ID = 0x05

FUNC_READ_INPUT_REGS = 0x04
FUNC_READ_HOLDING_REGS = 0x03

# Solis block
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
    # For Solis (existing V6 behavior) start_reg is doc reg and the frame uses raw_addr=start-1.
    # For ESP32 slave 5, registers are 0-based and raw_addr is the same as start_reg.
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
        regs[start_reg + i] = (data[i * 2] << 8) | data[i * 2 + 1]
    return regs


def s16(v: int) -> int:
    return v - 65536 if v >= 32768 else v


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
# Solis decode / write
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


def write_solis_metrics(s: dict, poll_count: int, read_errors: int):
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
    debug: bool = False
) -> Optional[Dict[int, int]]:
    request = build_request(slave_id, func, start_reg, count)

    while ser.in_waiting:
        ser.read(ser.in_waiting)

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

            # Keep Solis working, but don't spam logs.
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
                write_solis_metrics(s, poll_count, read_errors)

            # Focused debug for the slave 5 battery poll.
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

