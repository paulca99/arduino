#!/usr/bin/env python3

import serial
import time
import json
import requests
from typing import Optional

# ----------------------------------------------------------------------
# Config
# ----------------------------------------------------------------------
PORT = "/dev/ttyUSB0"
BAUDRATE = 9600

INFLUX_URL = "http://localhost:8086/write?db=power"
HOST_TAG = "server01"
SOURCE_TAG = "solis_sniff"

# Solis block read details from BLE_BMS/BLE_BMS.ino
SOLIS_SLAVE_ID = 0x01
SOLIS_FUNC = 0x04
SOLIS_START_DOC_REG = 33050
SOLIS_END_DOC_REG = 33142
SOLIS_REG_COUNT = SOLIS_END_DOC_REG - SOLIS_START_DOC_REG + 1   # 93
SOLIS_RESPONSE_LEN = 3 + (SOLIS_REG_COUNT * 2) + 2              # 191 bytes

# Registers used by existing read_solis.sh and BLE_BMS JSON
REGISTER_LIST = [
    33050, 33051, 33052, 33053, 33059, 33072, 33074,
    33080, 33081, 33085, 33095, 33129, 33130, 33131,
    33132, 33134, 33135, 33136, 33137, 33140, 33142
]

READ_TIMEOUT_S = 0.01
IDLE_FLUSH_S = 0.05

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

def is_target_response(frame: bytes) -> bool:
    return (
        len(frame) == SOLIS_RESPONSE_LEN and
        frame[0] == SOLIS_SLAVE_ID and
        frame[1] == SOLIS_FUNC and
        frame[2] == SOLIS_REG_COUNT * 2 and
        crc_ok(frame)
    )

# ----------------------------------------------------------------------
# Frame extraction
# ----------------------------------------------------------------------
def try_extract_target_frame(buf: bytearray) -> Optional[bytes]:
    """
    Find and remove one valid Solis FC04 block response from the buffer.
    """
    min_len = SOLIS_RESPONSE_LEN
    if len(buf) < min_len:
        return None

    i = 0
    while i <= len(buf) - min_len:
        if buf[i] == SOLIS_SLAVE_ID and buf[i + 1] == SOLIS_FUNC and buf[i + 2] == SOLIS_REG_COUNT * 2:
            candidate = bytes(buf[i:i + min_len])
            if crc_ok(candidate):
                del buf[:i + min_len]
                return candidate
        i += 1

    # Prevent unbounded growth if no valid frame is found
    if len(buf) > 4096:
        del buf[:-min_len]
    return None

# ----------------------------------------------------------------------
# Solis decoding
# ----------------------------------------------------------------------
def frame_to_registers(frame: bytes) -> dict:
    data = frame[3:-2]  # strip slave, func, bytecount, crc
    regs = {}
    for i in range(0, len(data), 2):
        reg = SOLIS_START_DOC_REG + (i // 2)
        raw = (data[i] << 8) | data[i + 1]
        regs[reg] = raw
    return regs

def build_json_from_registers(regs: dict, uptime_ms: int, last_success_ms: int, poll_count: int, read_errors: int, lock_timeouts: int):
    payload = {
        "uptimeMs": uptime_ms,
        "lastPollMs": last_success_ms,
        "lastSuccessMs": last_success_ms,
        "pollCount": poll_count,
        "readErrors": read_errors,
        "lockTimeouts": lock_timeouts,
    }

    batt_dir_raw = regs.get(33136, 0)
    payload["batteryDirection"] = "discharging" if batt_dir_raw == 1 else "charging"

    pv1_v = regs.get(33050, 0) / 10.0
    pv1_i = regs.get(33051, 0) / 10.0
    pv2_v = regs.get(33052, 0) / 10.0
    pv2_i = regs.get(33053, 0) / 10.0
    batt_v = regs.get(33142, 0) / 100.0
    batt_i = regs.get(33135, 0) / 10.0

    batt_p = batt_v * batt_i
    if batt_dir_raw == 1:
        batt_p = -batt_p

    payload["batteryPowerW"] = round(batt_p, 1)
    payload["pv1PowerW"] = round(pv1_v * pv1_i, 1)
    payload["pv2PowerW"] = round(pv2_v * pv2_i, 1)
    payload["pvTotalPowerW"] = round((pv1_v * pv1_i) + (pv2_v * pv2_i), 1)

    for reg in REGISTER_LIST:
        raw = regs.get(reg, 0)
        payload[str(reg)] = {
            "valid": True,
            "raw": raw,
            "signed": s16(raw),
        }

    return payload

# ----------------------------------------------------------------------
# Influx writing
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

def write_metrics_from_json(payload: dict):
    pv1_v_raw = payload["33050"]["raw"]
    pv1_i_raw = payload["33051"]["raw"]
    pv2_v_raw = payload["33052"]["raw"]
    pv2_i_raw = payload["33053"]["raw"]
    grid_v_raw = payload["33074"]["raw"]
    grid_f_raw = payload["33095"]["raw"]
    grid_p_signed = payload["33132"]["signed"]
    batt_soc_raw = payload["33140"]["raw"]
    batt_v_raw = payload["33142"]["raw"]
    batt_i_signed = payload["33135"]["signed"]
    batt_dir_raw = payload["33136"]["raw"]

    poll_count = payload["pollCount"]
    read_errors = payload["readErrors"]
    lock_timeouts = payload["lockTimeouts"]

    pv1_v = round(pv1_v_raw / 10.0, 1)
    pv1_i = round(pv1_i_raw / 10.0, 1)
    pv2_v = round(pv2_v_raw / 10.0, 1)
    pv2_i = round(pv2_i_raw / 10.0, 1)
    grid_v = round(grid_v_raw / 10.0, 1)
    grid_f = round(grid_f_raw / 100.0, 2)
    batt_soc = batt_soc_raw
    batt_v = round(batt_v_raw / 100.0, 2)
    batt_i = round(batt_i_signed / 10.0, 1)

    pv1_p = round((pv1_v_raw / 10.0) * (pv1_i_raw / 10.0), 1)
    pv2_p = round((pv2_v_raw / 10.0) * (pv2_i_raw / 10.0), 1)
    pv_total_p = round(pv1_p + pv2_p, 1)
    batt_p = round((batt_v_raw / 100.0) * (batt_i_signed / 10.0), 1)

    write_line("solis_pv1_voltage", pv1_v)
    write_line("solis_pv1_current", pv1_i)
    write_line("solis_pv2_voltage", pv2_v)
    write_line("solis_pv2_current", pv2_i)
    write_line("solis_grid_voltage", grid_v)
    write_line("solis_grid_frequency", grid_f)
    write_line("solis_grid_power", grid_p_signed)
    write_line("solis_battery_soc", batt_soc)
    write_line("solis_battery_voltage", batt_v)
    write_line("solis_battery_current", batt_i)
    write_line("solis_battery_direction_flag", batt_dir_raw)
    write_line("solis_pv1_power", pv1_p)
    write_line("solis_pv2_power", pv2_p)
    write_line("solis_pv_total_power", pv_total_p)
    write_line("solis_battery_power", batt_p)
    write_line("solis_poll_count", poll_count)
    write_line("solis_read_errors", read_errors)
    write_line("solis_lock_timeouts", lock_timeouts)

# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------
def main():
    print(f"Sniffing Solis FC04 block responses on {PORT} @ {BAUDRATE}")
    start_time = time.monotonic()
    poll_count = 0
    read_errors = 0
    lock_timeouts = 0
    last_success_ms = 0

    with serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=READ_TIMEOUT_S,
    ) as ser:
        buf = bytearray()
        last_rx = None

        while True:
            data = ser.read(512)
            now = time.monotonic()

            if data:
                buf.extend(data)
                last_rx = now

                while True:
                    frame = try_extract_target_frame(buf)
                    if frame is None:
                        break

                    poll_count += 1
                    uptime_ms = int((now - start_time) * 1000)
                    last_success_ms = uptime_ms

                    regs = frame_to_registers(frame)
                    payload = build_json_from_registers(
                        regs=regs,
                        uptime_ms=uptime_ms,
                        last_success_ms=last_success_ms,
                        poll_count=poll_count,
                        read_errors=read_errors,
                        lock_timeouts=lock_timeouts,
                    )

                    print(json.dumps(payload, separators=(",", ":")))
                    write_metrics_from_json(payload)

            else:
                if buf and last_rx is not None and (now - last_rx) > IDLE_FLUSH_S:
                    # stale junk; count as read noise and flush
                    read_errors += 1
                    buf.clear()
                    last_rx = None

if __name__ == "__main__":
    main()
