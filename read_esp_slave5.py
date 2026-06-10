#!/usr/bin/env python3
"""
Read ESP32 BLE_BMS_MARK2 Modbus slave 5 only.

- Polls ESP32 battery bridge as Modbus slave 5.
- Reads holding registers 0..79 using FC03.
- Writes battery slot telemetry to InfluxDB as:
  - battery_1_*
  - battery_2_*
  - battery_3_*

This is the battery/ESP32-only part of read_master2.py, intended for use
when the ESP32 BMS bridge is on its own dedicated USB-RS485 adapter.
"""

import time
from typing import Dict, Optional

import requests
import serial


# ----------------------------------------------------------------------
# Config
# ----------------------------------------------------------------------

PORT = "/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0"
BAUDRATE = 9600

INFLUX_URL = "http://localhost:8086/write?db=power"
HOST_TAG = "server01"
BATTERY_SOURCE_TAG = "battery_master"

BATTERY_SLAVE_ID = 0x05

FUNC_READ_HOLDING_REGS = 0x03

BATTERY_START_REG = 0
BATTERY_REG_COUNT = 80
BATTERY_BLOCK_SIZE = 24
BATTERY_COUNT = 3

POLL_INTERVAL_S = 4.0
REPLY_TIMEOUT_S = 2.0
READ_IDLE_TIMEOUT_S = 0.2
BUFFER_MAX = 8192

DEBUG_MODBUS = True


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
    # ESP32 slave 5 uses 0-based holding-register addresses.
    raw_addr = start_reg
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


def extract_frame_from_buffer(
    buf: bytearray,
    slave_id: int,
    func: int,
    expected_regs: int,
) -> Optional[bytes]:
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


def parse_modbus_response(
    frame: bytes,
    start_reg: int,
    expected_regs: int,
) -> Optional[Dict[int, int]]:
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

def write_line(measurement: str, value, source_tag: str) -> None:
    if value is None:
        return

    line = f"{measurement},host={HOST_TAG},source={source_tag} value={value}"

    try:
        requests.post(INFLUX_URL, data=line.encode("utf-8"), timeout=2)
    except Exception as e:
        print(f"[Influx] write failed: {measurement}: {e}")


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


def write_battery_metrics(bat: dict, battery_name: str) -> None:
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


def write_aggregate_metrics(regs: Dict[int, int]) -> None:
    """
    Optional aggregate block from BLE_BMS_MARK2 registers 0..7.

    These are additional convenience metrics; they do not replace the existing
    battery_1/2/3 metrics that read_master2.py already produced.
    """
    agg_valid = regs.get(0, 0)
    agg_contributing = regs.get(1, 0)
    agg_voltage = regs.get(2, 0) / 100.0
    agg_current = s16(regs.get(3, 0)) / 100.0
    agg_soc = regs.get(4, 0)
    agg_temp_raw = s16(regs.get(5, 0))
    agg_temp = None if agg_temp_raw == 0 else agg_temp_raw / 10.0
    agg_charge_allowed = regs.get(6, 0)
    agg_discharge_allowed = regs.get(7, 0)

    write_line("battery_aggregate_valid", agg_valid, BATTERY_SOURCE_TAG)
    write_line("battery_aggregate_contributing_count", agg_contributing, BATTERY_SOURCE_TAG)
    write_line("battery_aggregate_voltage", round(agg_voltage, 2), BATTERY_SOURCE_TAG)
    write_line("battery_aggregate_current", round(agg_current, 2), BATTERY_SOURCE_TAG)
    write_line("battery_aggregate_soc", agg_soc, BATTERY_SOURCE_TAG)
    write_line("battery_aggregate_temperature", agg_temp, BATTERY_SOURCE_TAG)
    write_line("battery_aggregate_charge_allowed", agg_charge_allowed, BATTERY_SOURCE_TAG)
    write_line("battery_aggregate_discharge_allowed", agg_discharge_allowed, BATTERY_SOURCE_TAG)


# ----------------------------------------------------------------------
# Polling
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

def main() -> None:
    print(f"Starting ESP32/BMS slave-5 poller on {PORT} @ {BAUDRATE}")
    print(f"Battery bridge: slave {BATTERY_SLAVE_ID}, FC03, regs {BATTERY_START_REG}-{BATTERY_START_REG + BATTERY_REG_COUNT - 1}")
    print(f"Influx URL: {INFLUX_URL}")
    print(f"Host tag: {HOST_TAG}")
    print(f"Source tag: {BATTERY_SOURCE_TAG}")
    print("")

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

            battery_regs = poll_modbus(
                ser,
                BATTERY_SLAVE_ID,
                FUNC_READ_HOLDING_REGS,
                BATTERY_START_REG,
                BATTERY_REG_COUNT,
                label="battery5",
                debug=DEBUG_MODBUS,
            )

            if battery_regs is None:
                read_errors += 1
                print("[battery5] FAIL: no response from slave 5")
                write_line("battery_bridge_read_errors", read_errors, BATTERY_SOURCE_TAG)
            else:
                poll_count += 1
                print(f"[battery5] OK: got {len(battery_regs)} registers")

                write_line("battery_bridge_poll_count", poll_count, BATTERY_SOURCE_TAG)
                write_line("battery_bridge_read_errors", read_errors, BATTERY_SOURCE_TAG)
                write_aggregate_metrics(battery_regs)

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
    