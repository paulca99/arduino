#!/usr/bin/env python3
"""
Poll the fence / second solar inverter XML endpoint and write live data to InfluxDB.

Endpoint example:
  http://tl-wr802n/real_time_data.xml

Expected XML shape:

<real_time_data>
  <state>Normal</state>
  <v-grid>243.0</v-grid>
  <i-grid>2.69</i-grid>
  <f-grid>49.91</f-grid>
  <p-ac>672</p-ac>
  <temp>30.1</temp>
  <e-today>1.0</e-today>
  <t-today>0.7</t-today>
  <e-total>2400.0</e-total>
  <CO2>2392.80</CO2>
  <t-total>4283.4</t-total>
  <v-pv1>174.8</v-pv1>
  <i-pv1>8.11</i-pv1>
  <v-pv2>26.8</v-pv2>
  <i-pv2>0.00</i-pv2>
  <v-bus>370.8</v-bus>
</real_time_data>
"""

import socket
import time
import xml.etree.ElementTree as ET
from typing import Any, Dict, Optional

import requests


# ----------------------------------------------------------------------
# Config
# ----------------------------------------------------------------------

FENCE_SOLAR_URL = "http://tl-wr802n/real_time_data.xml"

INFLUX_URL = "http://localhost:8086/write?db=power"

HOST_TAG = "server01"
SOURCE_TAG = "fence_solar"

POLL_INTERVAL_S = 10.0

# Keep this short because the Wi-Fi link may be flaky.
HTTP_TIMEOUT_S = 1.5

# Avoid spamming logs if the little Wi-Fi bridge drops out.
ERROR_LOG_INTERVAL_S = 60.0


# ----------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------

def escape_tag_value(value: str) -> str:
    """Escape InfluxDB line-protocol tag values."""
    return (
        str(value)
        .replace("\\", "\\\\")
        .replace(" ", "\\ ")
        .replace(",", "\\,")
        .replace("=", "\\=")
    )


def write_line(
    measurement: str,
    value: Any,
    source_tag: str = SOURCE_TAG,
    extra_tags: Optional[Dict[str, str]] = None,
) -> None:
    """
    Write one numeric field called 'value' to InfluxDB.

    Example line:
      fence_solar_ac_power,host=server01,source=fence_solar value=672
    """
    if value is None:
        return

    try:
        # Ensure booleans are written as 0/1 numeric values.
        if isinstance(value, bool):
            value = 1 if value else 0

        # Only write numeric-like values.
        if isinstance(value, str):
            value = float(value)

        tags = {
            "host": HOST_TAG,
            "source": source_tag,
        }

        if extra_tags:
            tags.update(extra_tags)

        tag_text = ",".join(
            f"{escape_tag_value(k)}={escape_tag_value(v)}"
            for k, v in tags.items()
        )

        line = f"{measurement},{tag_text} value={value}"

        requests.post(INFLUX_URL, data=line.encode("utf-8"), timeout=2)

    except Exception as e:
        print(f"[Influx] write failed: {measurement}={value}: {e}")


def parse_float(root: ET.Element, tag_name: str, default: Optional[float] = None) -> Optional[float]:
    el = root.find(tag_name)
    if el is None or el.text is None:
        return default

    text = el.text.strip()
    if text == "":
        return default

    try:
        return float(text)
    except ValueError:
        return default


def parse_text(root: ET.Element, tag_name: str, default: str = "") -> str:
    el = root.find(tag_name)
    if el is None or el.text is None:
        return default
    return el.text.strip()


def read_fence_solar() -> Dict[str, Any]:
    """
    Fetch and parse the inverter XML.

    Returns:
      {
        "ok": True,
        "state": "Normal",
        "ac_power": 672.0,
        ...
      }

    or:
      {
        "ok": False,
        "error": "...",
      }
    """
    try:
        response = requests.get(FENCE_SOLAR_URL, timeout=HTTP_TIMEOUT_S)
        response.raise_for_status()

        root = ET.fromstring(response.text)

        data = {
            "ok": True,
            "state": parse_text(root, "state", "Unknown"),

            # AC/grid side
            "grid_voltage": parse_float(root, "v-grid"),
            "grid_current": parse_float(root, "i-grid"),
            "grid_frequency": parse_float(root, "f-grid"),
            "ac_power": parse_float(root, "p-ac"),

            # Inverter internal / totals
            "temperature": parse_float(root, "temp"),
            "energy_today": parse_float(root, "e-today"),
            "runtime_today": parse_float(root, "t-today"),
            "energy_total": parse_float(root, "e-total"),
            "co2": parse_float(root, "CO2"),
            "runtime_total": parse_float(root, "t-total"),

            # PV/DC side
            "pv1_voltage": parse_float(root, "v-pv1"),
            "pv1_current": parse_float(root, "i-pv1"),
            "pv2_voltage": parse_float(root, "v-pv2"),
            "pv2_current": parse_float(root, "i-pv2"),
            "bus_voltage": parse_float(root, "v-bus"),
        }

        # Derived DC PV powers, useful for checking against AC output.
        pv1_v = data.get("pv1_voltage")
        pv1_i = data.get("pv1_current")
        pv2_v = data.get("pv2_voltage")
        pv2_i = data.get("pv2_current")

        data["pv1_power"] = round(pv1_v * pv1_i, 1) if pv1_v is not None and pv1_i is not None else None
        data["pv2_power"] = round(pv2_v * pv2_i, 1) if pv2_v is not None and pv2_i is not None else None

        if data["pv1_power"] is not None or data["pv2_power"] is not None:
            data["pv_total_power"] = round((data["pv1_power"] or 0) + (data["pv2_power"] or 0), 1)
        else:
            data["pv_total_power"] = None

        return data

    except Exception as e:
        return {
            "ok": False,
            "error": str(e),
        }


def write_fence_solar_metrics(
    data: Dict[str, Any],
    read_errors: int,
    last_success_age_s: Optional[float],
) -> None:
    """Write parsed fence solar data to InfluxDB."""
    ok = bool(data.get("ok", False))

    state = str(data.get("state", "Unknown")) if ok else "Offline"

    extra_tags = {
        "state": state,
    }

    write_line("fence_solar_online", 1 if ok else 0, extra_tags=extra_tags)
    write_line("fence_solar_read_errors", read_errors, extra_tags=extra_tags)

    if last_success_age_s is not None:
        write_line("fence_solar_last_success_age_s", round(last_success_age_s, 1), extra_tags=extra_tags)

    if not ok:
        return

    # AC/grid side
    write_line("fence_solar_ac_power", data.get("ac_power"), extra_tags=extra_tags)
    write_line("fence_solar_grid_voltage", data.get("grid_voltage"), extra_tags=extra_tags)
    write_line("fence_solar_grid_current", data.get("grid_current"), extra_tags=extra_tags)
    write_line("fence_solar_grid_frequency", data.get("grid_frequency"), extra_tags=extra_tags)

    # Inverter internal / totals
    write_line("fence_solar_temperature", data.get("temperature"), extra_tags=extra_tags)
    write_line("fence_solar_energy_today", data.get("energy_today"), extra_tags=extra_tags)
    write_line("fence_solar_runtime_today", data.get("runtime_today"), extra_tags=extra_tags)
    write_line("fence_solar_energy_total", data.get("energy_total"), extra_tags=extra_tags)
    write_line("fence_solar_co2", data.get("co2"), extra_tags=extra_tags)
    write_line("fence_solar_runtime_total", data.get("runtime_total"), extra_tags=extra_tags)

    # PV/DC side
    write_line("fence_solar_pv1_voltage", data.get("pv1_voltage"), extra_tags=extra_tags)
    write_line("fence_solar_pv1_current", data.get("pv1_current"), extra_tags=extra_tags)
    write_line("fence_solar_pv1_power", data.get("pv1_power"), extra_tags=extra_tags)

    write_line("fence_solar_pv2_voltage", data.get("pv2_voltage"), extra_tags=extra_tags)
    write_line("fence_solar_pv2_current", data.get("pv2_current"), extra_tags=extra_tags)
    write_line("fence_solar_pv2_power", data.get("pv2_power"), extra_tags=extra_tags)

    write_line("fence_solar_pv_total_power", data.get("pv_total_power"), extra_tags=extra_tags)
    write_line("fence_solar_bus_voltage", data.get("bus_voltage"), extra_tags=extra_tags)


# ----------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------

def main() -> None:
    print("Starting fence solar poller")
    print(f"URL: {FENCE_SOLAR_URL}")
    print(f"Influx: {INFLUX_URL}")
    print(f"Host tag: {HOST_TAG}")
    print(f"Source tag: {SOURCE_TAG}")
    print(f"Poll interval: {POLL_INTERVAL_S}s")
    print(f"HTTP timeout: {HTTP_TIMEOUT_S}s")
    print("")

    read_errors = 0
    last_success_ts: Optional[float] = None
    last_error_log_ts = 0.0

    session_hostname = socket.gethostname()
    print(f"Running on host: {session_hostname}")
    print("")

    while True:
        loop_start = time.monotonic()

        data = read_fence_solar()
        now = time.monotonic()

        if data.get("ok"):
            last_success_ts = now

            ac_power = data.get("ac_power")
            state = data.get("state")
            pv_total = data.get("pv_total_power")
            e_today = data.get("energy_today")
            e_total = data.get("energy_total")

            print(
                f"[fence] OK state={state} "
                f"ac={ac_power}W pv_total={pv_total}W "
                f"today={e_today}kWh total={e_total}kWh"
            )

        else:
            read_errors += 1

            # Log the error only occasionally, because the Wi-Fi bridge may be flaky.
            if now - last_error_log_ts >= ERROR_LOG_INTERVAL_S:
                print(f"[fence] FAIL read_errors={read_errors}: {data.get('error')}")
                last_error_log_ts = now

        if last_success_ts is None:
            last_success_age_s = None
        else:
            last_success_age_s = now - last_success_ts

        write_fence_solar_metrics(
            data=data,
            read_errors=read_errors,
            last_success_age_s=last_success_age_s,
        )

        elapsed = time.monotonic() - loop_start
        sleep_for = POLL_INTERVAL_S - elapsed
        if sleep_for > 0:
            time.sleep(sleep_for)


if __name__ == "__main__":
    main()
