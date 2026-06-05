#!/usr/bin/env python3
"""
Active Solis RS485 master poller for Raspberry Pi.

This differs from read_solis_python.py (passive sniffer): it actively sends the
same FC04 block-read request shape used in BLE_BMS_MARK2 and polls every 5s.
"""

import json
import time
from datetime import datetime, timezone
from typing import Dict, Optional

import requests
import serial

# ----------------------------------------------------------------------
# Config
# ----------------------------------------------------------------------
PORT = "/dev/ttyUSB0"
BAUDRATE = 9600
POLL_INTERVAL_S = 5.0
READ_TIMEOUT_S = 0.05
MODBUS_REPLY_TIMEOUT_S = 0.30

INFLUX_URL = "http://localhost:8086/write?db=power"
HOST_TAG = "server01"
SOURCE_TAG = "solis_master"

# Mirror BLE_BMS_MARK2 block read shape
SOLIS_SLAVE_ID = 0x01
SOLIS_FUNC = 0x04
SOLIS_START_DOC_REG = 33050
SOLIS_REG_COUNT = 93
SOLIS_RESPONSE_LEN = 3 + (SOLIS_REG_COUNT * 2) + 2  # 191


# Register metadata based on the Solis PDF mapping notes in memory/history.
# Unknown/resolved-later fields are still emitted as raw values.
# Note: the active poll block starts at 33050, so 33049 (DC Voltage 1) is not
# included in this fixed 93-register read shape.
KNOWN_REGS = {
    33050: {"label": "dc_current_1", "scale": 0.1, "signed": False, "unit": "A"},
    33051: {"label": "dc_voltage_2", "scale": 0.1, "signed": False, "unit": "V"},
    33052: {"label": "dc_current_2", "scale": 0.1, "signed": False, "unit": "A"},
    33053: {"label": "dc_voltage_3", "scale": 0.1, "signed": False, "unit": "V"},
    33054: {"label": "dc_current_3", "scale": 0.1, "signed": False, "unit": "A"},
    33059: {"label": "reserved_33059", "scale": None, "signed": False, "unit": None},
    33072: {"label": "dc_bus_half_voltage", "scale": None, "signed": False, "unit": None},
    33074: {"label": "bc_line_voltage", "scale": 0.1, "signed": False, "unit": "V"},
    33094: {"label": "grid_frequency", "scale": 0.01, "signed": False, "unit": "Hz"},
    33095: {"label": "inverter_current_status", "scale": None, "signed": False, "unit": None},
    33129: {"label": "meter_current", "scale": None, "signed": False, "unit": None},
    33132: {"label": "storage_control_switching_value", "scale": None, "signed": False, "unit": None},
    33133: {"label": "battery_voltage", "scale": 0.1, "signed": False, "unit": "V"},
    33134: {"label": "battery_current", "scale": 0.1, "signed": True, "unit": "A"},
    33135: {
        "label": "battery_current_direction",
        "scale": None,
        "signed": False,
        "unit": None,
        "enum_map": {0: "charging", 1: "discharging"},
    },
    33136: {"label": "llc_bus_voltage", "scale": None, "signed": False, "unit": None},
    33137: {"label": "backup_ac_voltage_phase_a", "scale": 0.1, "signed": False, "unit": "V"},
    33139: {"label": "battery_soc", "scale": 1.0, "signed": False, "unit": "%"},
    33140: {"label": "battery_soh", "scale": 1.0, "signed": False, "unit": "%"},
    33141: {"label": "battery_voltage_bms", "scale": 0.01, "signed": False, "unit": "V"},
    33142: {"label": "battery_current_bms", "scale": 0.1, "signed": True, "unit": "A"},
}


def modbus_crc(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def s16(v: int) -> int:
    return v - 65536 if v >= 32768 else v


def s32_from_u16(hi: int, lo: int) -> int:
    v = ((hi & 0xFFFF) << 16) | (lo & 0xFFFF)
    return v - 4294967296 if v >= 2147483648 else v


def build_fc04_request(slave: int, start_doc_reg: int, count: int) -> bytes:
    raw_addr = start_doc_reg - 1  # mirrors BLE_BMS_MARK2 behavior
    frame = bytes([
        slave,
        SOLIS_FUNC,
        (raw_addr >> 8) & 0xFF,
        raw_addr & 0xFF,
        (count >> 8) & 0xFF,
        count & 0xFF,
    ])
    crc = modbus_crc(frame)
    return frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def read_fc04_block(ser: serial.Serial) -> Optional[bytes]:
    request = build_fc04_request(SOLIS_SLAVE_ID, SOLIS_START_DOC_REG, SOLIS_REG_COUNT)
    ser.reset_input_buffer()
    ser.write(request)
    ser.flush()

    buf = bytearray()
    deadline = time.monotonic() + MODBUS_REPLY_TIMEOUT_S
    while time.monotonic() < deadline:
        chunk = ser.read(SOLIS_RESPONSE_LEN - len(buf))
        if chunk:
            buf.extend(chunk)
            if len(buf) >= SOLIS_RESPONSE_LEN:
                break

    if len(buf) != SOLIS_RESPONSE_LEN:
        return None

    frame = bytes(buf)
    if frame[0] != SOLIS_SLAVE_ID or frame[1] != SOLIS_FUNC:
        return None
    if frame[2] != SOLIS_REG_COUNT * 2:
        return None

    rx_crc = frame[-2] | (frame[-1] << 8)
    if rx_crc != modbus_crc(frame[:-2]):
        return None

    return frame


def frame_to_regs(frame: bytes) -> Dict[int, int]:
    data = frame[3:-2]
    regs: Dict[int, int] = {}
    for idx in range(SOLIS_REG_COUNT):
        reg = SOLIS_START_DOC_REG + idx
        offset = idx * 2
        regs[reg] = (data[offset] << 8) | data[offset + 1]
    return regs


def decode_registers(regs: Dict[int, int]) -> Dict[str, Dict[str, object]]:
    decoded: Dict[str, Dict[str, object]] = {}

    for reg in range(SOLIS_START_DOC_REG, SOLIS_START_DOC_REG + SOLIS_REG_COUNT):
        raw = regs.get(reg, 0)
        entry: Dict[str, object] = {
            "raw": raw,
            "signed": s16(raw),
        }

        meta = KNOWN_REGS.get(reg)
        if meta:
            entry["label"] = meta["label"]
            if meta.get("scale") is not None:
                base = s16(raw) if meta.get("signed") else raw
                value = base * float(meta["scale"])
                entry["value"] = round(value, 3)
                if meta.get("unit"):
                    entry["unit"] = meta["unit"]

            enum_map = meta.get("enum_map")
            if enum_map:
                entry["enum"] = enum_map.get(raw, f"unknown_{raw}")

        decoded[str(reg)] = entry

    # Known S32 pair from PDF notes.
    if 33130 in regs and 33131 in regs:
        decoded["33130_33131"] = {
            "label": "meter_active_power",
            "raw_hi": regs[33130],
            "raw_lo": regs[33131],
            "value": s32_from_u16(regs[33130], regs[33131]),
            "unit": "W",
        }

    return decoded


def write_line(measurement: str, value: object) -> None:
    if value is None:
        return
    line = f"{measurement},host={HOST_TAG},source={SOURCE_TAG} value={value}"
    print(line)
    try:
        requests.post(INFLUX_URL, data=line.encode("utf-8"), timeout=2)
    except Exception as exc:
        print(f"Influx write failed: {exc}")


def legacy_metrics(regs: Dict[int, int], poll_count: int, read_errors: int, lock_timeouts: int) -> Dict[str, object]:
    # Keep existing measurement names/shapes for Grafana compatibility.
    # Where the PDF mapping is known and unambiguous, use those corrected registers.
    # Where historical names don't map cleanly to the current block, keep old behavior.
    # Keep historical metric names while using voltage/current typed registers.
    # With this fixed 33050..33142 block, doc register 33049 is not present.
    # So these are the first two available V/I pairs in the returned block.
    legacy_pv1_v_raw = regs.get(33051, 0)  # doc: dc_voltage_2
    legacy_pv1_i_raw = regs.get(33050, 0)  # doc: dc_current_1
    legacy_pv2_v_raw = regs.get(33053, 0)  # doc: dc_voltage_3
    legacy_pv2_i_raw = regs.get(33052, 0)  # doc: dc_current_2
    grid_v_raw = regs.get(33074, 0)
    grid_f_raw = regs.get(33094, 0)
    grid_p_signed = s32_from_u16(regs.get(33130, 0), regs.get(33131, 0))
    batt_soc_raw = regs.get(33139, 0)
    batt_v_raw = regs.get(33133, 0)
    batt_i_signed = s16(regs.get(33134, 0))
    batt_dir_raw = regs.get(33135, 0)

    legacy_pv1_v = round(legacy_pv1_v_raw / 10.0, 1)
    legacy_pv1_i = round(legacy_pv1_i_raw / 10.0, 1)
    legacy_pv2_v = round(legacy_pv2_v_raw / 10.0, 1)
    legacy_pv2_i = round(legacy_pv2_i_raw / 10.0, 1)
    grid_v = round(grid_v_raw / 10.0, 1)
    grid_f = round(grid_f_raw / 100.0, 2)
    batt_soc = batt_soc_raw
    batt_v = round(batt_v_raw / 10.0, 2)
    batt_i = round(batt_i_signed / 10.0, 1)

    pv1_p = round(legacy_pv1_v * legacy_pv1_i, 1)
    pv2_p = round(legacy_pv2_v * legacy_pv2_i, 1)
    pv_total_p = round(pv1_p + pv2_p, 1)
    batt_p = round((batt_v_raw / 10.0) * (batt_i_signed / 10.0), 1)

    return {
        "solis_pv1_voltage": legacy_pv1_v,
        "solis_pv1_current": legacy_pv1_i,
        "solis_pv2_voltage": legacy_pv2_v,
        "solis_pv2_current": legacy_pv2_i,
        "solis_grid_voltage": grid_v,
        "solis_grid_frequency": grid_f,
        "solis_grid_power": grid_p_signed,
        "solis_battery_soc": batt_soc,
        "solis_battery_voltage": batt_v,
        "solis_battery_current": batt_i,
        "solis_battery_direction_flag": batt_dir_raw,
        "solis_pv1_power": pv1_p,
        "solis_pv2_power": pv2_p,
        "solis_pv_total_power": pv_total_p,
        "solis_battery_power": batt_p,
        "solis_poll_count": poll_count,
        "solis_read_errors": read_errors,
        "solis_lock_timeouts": lock_timeouts,
    }


def print_poll_logs(poll_count: int, regs: Dict[int, int], decoded: Dict[str, Dict[str, object]]) -> None:
    ts = datetime.now(timezone.utc).isoformat()
    raw_dump = {str(reg): regs[reg] for reg in sorted(regs.keys())}
    print(json.dumps({"ts": ts, "poll": poll_count, "type": "raw_registers", "data": raw_dump}, separators=(",", ":")))
    print(json.dumps({"ts": ts, "poll": poll_count, "type": "decoded_registers", "data": decoded}, separators=(",", ":")))

    # Fast human summary for terminal tailing.
    meter_power = decoded.get("33130_33131", {}).get("value")
    batt_v = decoded.get("33133", {}).get("value")
    batt_i = decoded.get("33134", {}).get("value")
    batt_soc = decoded.get("33139", {}).get("value")
    print(
        f"[{ts}] poll={poll_count} meterP={meter_power}W battV={batt_v}V battI={batt_i}A soc={batt_soc}%"
    )


def main() -> None:
    print(
        f"Active Solis FC04 polling on {PORT} @ {BAUDRATE}"
        f" (slave={SOLIS_SLAVE_ID}, start={SOLIS_START_DOC_REG}, count={SOLIS_REG_COUNT}, every={POLL_INTERVAL_S}s)"
    )

    poll_count = 0
    read_errors = 0
    lock_timeouts = 0  # Kept for measurement compatibility with existing dashboards.

    with serial.Serial(
        port=PORT,
        baudrate=BAUDRATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=READ_TIMEOUT_S,
    ) as ser:
        while True:
            loop_start = time.monotonic()
            frame = read_fc04_block(ser)
            poll_count += 1

            if frame is None:
                read_errors += 1
                print(f"Poll {poll_count}: read failed (timeout/header/crc)")
            else:
                regs = frame_to_regs(frame)
                decoded = decode_registers(regs)
                print_poll_logs(poll_count, regs, decoded)

                metrics = legacy_metrics(regs, poll_count, read_errors, lock_timeouts)
                for measurement, value in metrics.items():
                    write_line(measurement, value)

            elapsed = time.monotonic() - loop_start
            sleep_s = POLL_INTERVAL_S - elapsed
            if sleep_s > 0:
                time.sleep(sleep_s)


if __name__ == "__main__":
    main()
