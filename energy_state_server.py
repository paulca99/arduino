#!/usr/bin/env python3
"""
Tiny HTTP endpoint for ESP32 charger / GTI / charge-permission logic.

Endpoint:
  http://192.168.1.218/energy-state
  http://raspberrypi1/energy-state

Example response:
  solis_battery_power=-3576.0
  solis_battery_soc=77
  solis_grid_power=-355.0
  solis_pv_total_power=0.0
  ac_bms_discharge_power=0.0
  ac_charger_allowed=0
  ok=1

Sign conventions:
  solis_battery_power > 0      = Solis/DC battery charging
  solis_battery_power < 0      = Solis/DC battery discharging

  solis_grid_power < 0         = grid import
  solis_grid_power > 0         = grid export

  ac_bms_discharge_power > 0   = AC-side battery discharging

Charge permission:
  ac_charger_allowed = 1 only if Solis PV total power is above 100W.
  This prevents the AC battery charger from charging using power that
  originated from the Solis/DC battery when PV is zero or very low.
"""

from http.server import BaseHTTPRequestHandler, HTTPServer
from urllib.parse import urlencode, urlparse
import json
import urllib.request


INFLUX_QUERY_URL = "http://localhost:8086/query"
DB = "power"
HOST = "server01"
PORT = 80
LOOKBACK = "30s"

PV_CHARGE_ALLOW_W = 100.0


def influx_last(measurement, source=None):
    where = f"WHERE time > now() - {LOOKBACK} AND \"host\" = '{HOST}'"
    if source:
        where += f" AND \"source\" = '{source}'"

    q = f'SELECT LAST("value") FROM "{measurement}" {where}'
    url = f"{INFLUX_QUERY_URL}?{urlencode({'db': DB, 'q': q})}"

    try:
        with urllib.request.urlopen(url, timeout=1.5) as response:
            payload = json.loads(response.read().decode("utf-8"))

        return float(payload["results"][0]["series"][0]["values"][0][1])

    except Exception:
        return None


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        path = urlparse(self.path).path

        if path not in ("/", "/energy-state"):
            self.send_response(404)
            self.end_headers()
            return

        solis_battery_power = influx_last("solis_battery_power", "solis_master")
        solis_battery_soc = influx_last("solis_battery_soc", "solis_master")
        solis_grid_power = influx_last("solis_grid_power", "solis_master")
        solis_pv_total_power = influx_last("solis_pv_total_power", "solis_master")
        ac_bms_discharge_power = influx_last("ac_bms_discharge_power", "ac_bms_monitor")

        ok = 1

        if solis_battery_power is None:
            solis_battery_power = 0.0
            ok = 0

        if solis_battery_soc is None:
            solis_battery_soc = 0.0
            ok = 0

        if solis_grid_power is None:
            solis_grid_power = 0.0
            ok = 0

        if solis_pv_total_power is None:
            solis_pv_total_power = 0.0
            ok = 0

        if ac_bms_discharge_power is None:
            ac_bms_discharge_power = 0.0
            ok = 0

        # For charger permission, fail safe:
        # if endpoint data is incomplete, do NOT allow AC charging.
        ac_charger_allowed = 1 if ok == 1 and solis_pv_total_power > PV_CHARGE_ALLOW_W else 0

        body = (
            f"solis_battery_power={solis_battery_power:.1f}\n"
            f"solis_battery_soc={int(round(solis_battery_soc))}\n"
            f"solis_grid_power={solis_grid_power:.1f}\n"
            f"solis_pv_total_power={solis_pv_total_power:.1f}\n"
            f"ac_bms_discharge_power={ac_bms_discharge_power:.1f}\n"
            f"ac_charger_allowed={ac_charger_allowed}\n"
            f"ok={ok}\n"
        )

        body_bytes = body.encode("utf-8")

        self.send_response(200)
        self.send_header("Content-Type", "text/plain")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(body_bytes)))
        self.end_headers()
        self.wfile.write(body_bytes)

    def log_message(self, fmt, *args):
        return


def main():
    server = HTTPServer(("0.0.0.0", PORT), Handler)
    print(f"energy_state_server listening on http://0.0.0.0:{PORT}/energy-state")
    print(f"AC charger allowed only when solis_pv_total_power > {PV_CHARGE_ALLOW_W:.1f}W")
    server.serve_forever()


if __name__ == "__main__":
    main()
    