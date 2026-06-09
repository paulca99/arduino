#!/usr/bin/env python3

import argparse
import os
import re
import time
from typing import Any, Dict

import requests

DEFAULT_INFLUX_URL = "http://localhost:8086/write?db=power"
DEFAULT_HOST_TAG = "server01"
DEFAULT_SOURCE_TAG = "ac_bms_monitor"
DEFAULT_AC_BMS_URL = "http://ac-battery-monitor.local/api/bms"
DEFAULT_INTERVAL_S = 2.0

INFLUX_URL = DEFAULT_INFLUX_URL
HOST_TAG = DEFAULT_HOST_TAG
SOURCE_TAG = DEFAULT_SOURCE_TAG


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Poll AC ESP32 BMS monitor and write Influx metrics")
    parser.add_argument("--url", default=None, help="AC monitor endpoint URL (default env AC_BMS_URL or built-in)")
    parser.add_argument("--interval", type=float, default=None, help="Polling interval seconds (default env AC_BMS_INTERVAL or 2.0)")
    parser.add_argument("--influx-url", default=None, help="Influx write URL (default env AC_BMS_INFLUX_URL or built-in)")
    parser.add_argument("--host-tag", default=None, help="Influx host tag (default env AC_BMS_HOST_TAG or built-in)")
    parser.add_argument("--source-tag", default=None, help="Influx source tag (default env AC_BMS_SOURCE_TAG or built-in)")
    return parser.parse_args()


def as_float(v: Any, default: float = 0.0) -> float:
    try:
        return float(v)
    except Exception:
        return default


def as_int(v: Any, default: int = 0) -> int:
    try:
        return int(v)
    except Exception:
        return default


def write_line(measurement: str, value: Any, source_tag: str = SOURCE_TAG) -> None:
    if value is None:
        return
    line = f"{measurement},host={HOST_TAG},source={source_tag} value={value}"
    try:
        requests.post(INFLUX_URL, data=line.encode("utf-8"), timeout=2)
    except Exception as e:
        print(f"[Influx] write failed: {measurement}: {e}")


def safe_name(name: str) -> str:
    cleaned = re.sub(r"[^a-zA-Z0-9]+", "_", (name or "battery").strip().lower()).strip("_")
    return cleaned or "battery"


def write_aggregate_metrics(payload: Dict[str, Any]) -> None:
    aggregate = payload.get("aggregate", {}) if isinstance(payload.get("aggregate"), dict) else {}

    write_line("ac_bms_connected_count", as_int(payload.get("connected_count", 0)))
    write_line("ac_bms_battery_count", as_int(payload.get("battery_count", 0)))
    write_line("ac_bms_valid", 1 if aggregate.get("valid") else 0)
    write_line("ac_bms_voltage", as_float(aggregate.get("voltage_v", 0.0)))
    write_line("ac_bms_current", as_float(aggregate.get("current_a", 0.0)))
    write_line("ac_bms_power", as_float(aggregate.get("power_w", 0.0)))
    write_line("ac_bms_charge_power", as_float(aggregate.get("charge_power_w", 0.0)))
    write_line("ac_bms_discharge_power", as_float(aggregate.get("discharge_power_w", 0.0)))
    write_line("ac_bms_temperature", as_float(aggregate.get("temperature_c", 0.0)))


def write_battery_metrics(payload: Dict[str, Any]) -> None:
    batteries = payload.get("batteries", [])
    if not isinstance(batteries, list):
        return

    for bat in batteries:
        if not isinstance(bat, dict):
            continue

        base = f"ac_bms_{safe_name(str(bat.get('name', 'battery')))}"

        write_line(f"{base}_connected", 1 if bat.get("connected") else 0)
        write_line(f"{base}_has_data", 1 if bat.get("has_data") else 0)
        write_line(f"{base}_voltage", as_float(bat.get("voltage_v", 0.0)))
        write_line(f"{base}_current", as_float(bat.get("current_a", 0.0)))
        write_line(f"{base}_power", as_float(bat.get("power_w", 0.0)))
        write_line(f"{base}_soc", as_int(bat.get("soc", 0)))
        write_line(f"{base}_temperature", as_float(bat.get("temperature_c", 0.0)))
        write_line(f"{base}_charge_mos", 1 if bat.get("charge_mos") else 0)
        write_line(f"{base}_discharge_mos", 1 if bat.get("discharge_mos") else 0)
        write_line(f"{base}_cell_count", as_int(bat.get("cell_count", 0)))
        write_line(f"{base}_min_cell_mv", as_int(bat.get("min_cell_mv", 0)))
        write_line(f"{base}_max_cell_mv", as_int(bat.get("max_cell_mv", 0)))
        write_line(f"{base}_cell_delta_mv", as_int(bat.get("cell_delta_mv", 0)))
        write_line(f"{base}_ok_reads", as_int(bat.get("ok_reads", 0)))
        write_line(f"{base}_failed_reads", as_int(bat.get("failed_reads", 0)))
        write_line(f"{base}_disconnect_count", as_int(bat.get("disconnect_count", 0)))
        write_line(f"{base}_last_good_ms_ago", as_int(bat.get("last_good_ms_ago", 0)))


def main() -> None:
    global INFLUX_URL, HOST_TAG, SOURCE_TAG
    args = parse_args()

    url = args.url or os.getenv("AC_BMS_URL") or DEFAULT_AC_BMS_URL
    interval = args.interval
    if interval is None:
        interval = as_float(os.getenv("AC_BMS_INTERVAL"), DEFAULT_INTERVAL_S)
    if interval <= 0:
        interval = DEFAULT_INTERVAL_S

    INFLUX_URL = args.influx_url or os.getenv("AC_BMS_INFLUX_URL") or DEFAULT_INFLUX_URL
    HOST_TAG = args.host_tag or os.getenv("AC_BMS_HOST_TAG") or DEFAULT_HOST_TAG
    SOURCE_TAG = args.source_tag or os.getenv("AC_BMS_SOURCE_TAG") or DEFAULT_SOURCE_TAG

    print(f"[ac-bms] URL={url} interval={interval:.2f}s influx={INFLUX_URL} host={HOST_TAG} source={SOURCE_TAG}")
    error_count = 0

    while True:
        cycle_start = time.monotonic()
        try:
            resp = requests.get(url, timeout=5)
            resp.raise_for_status()
            payload = resp.json()
            if not isinstance(payload, dict):
                raise ValueError("JSON root is not an object")

            write_aggregate_metrics(payload)
            write_battery_metrics(payload)
            write_line("ac_bms_error_count", error_count)
            write_line("ac_bms_last_poll_ok", 1)
        except Exception as e:
            error_count += 1
            print(f"[ac-bms] poll failed ({error_count}): {e}")
            write_line("ac_bms_error_count", error_count)
            write_line("ac_bms_last_poll_ok", 0)

        elapsed = time.monotonic() - cycle_start
        sleep_s = max(0.0, interval - elapsed)
        time.sleep(sleep_s)


if __name__ == "__main__":
    main()
