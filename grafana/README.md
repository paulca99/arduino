# Grafana Dashboards

This directory contains Grafana dashboard JSON files for the house energy monitoring system.

## Dashboards

| File | Title | Purpose |
|---|---|---|
| `energy-dash.json` | Solis Energy Flow | Live power flows, energy totals, AC battery monitor |
| `fit-integrity.json` | FIT Integrity | FIT path accounting: PV generated vs AC output |
| `power-overview.json` | Power Overview | High-level live power overview |

---

## House Electrical Topology

```
Solis PV panels
      │
      ▼
 Solis Inverter ──── solis_battery_power (+charge / -discharge)
      │                  2 × Growatt GBLI5001 (9.75 kWh nominal)
      │ solis_inverter_active_power  (+output / -input)
      ├─── solis_ac_output_power   (unsigned, always ≥ 0)
      ├─── solis_ac_input_power    (unsigned, always ≥ 0)
      │
      │── FIT / generation meter ── Grid  (solis_grid_power: +export / -import)
      │
AC bus ─────────────────────────────────── House load
      │
      ├── Fence solar  fence_solar_ac_power  (non-FIT eligible)
      │
      └── AC-side battery system
              ac_bms_charge_power / ac_bms_discharge_power
              ac_bms_power  (+charge / -discharge)
```

---

## Sign Conventions

| Measurement | Positive | Negative |
|---|---|---|
| `solis_grid_power` | Grid export (selling) | Grid import (buying) |
| `solis_inverter_active_power` | Solis AC output | AC into Solis (import) |
| `solis_battery_power` | Solis DC battery charging | Solis DC battery discharging |
| `ac_bms_power` | AC-side batteries charging | AC-side batteries discharging |

---

## Energy Integration — Why `fill(0)` Was Wrong

### The bug

Previous energy queries used a nested subquery pattern like:

```sql
SELECT INTEGRAL("p", 1h) AS "Battery Discharge Wh"
FROM (
  SELECT MEAN("value") AS "p"
  FROM "solis_battery_power"
  WHERE $timeFilter AND "value" < 0
  GROUP BY time(2s) fill(0)
)
```

The `GROUP BY time(2s) fill(0)` inserted a synthetic **zero-power sample** for every 2-second bucket that contained no real measurement. Because the Solis poller runs approximately every 5 seconds, roughly 60% of 2-second buckets were filled with zero. This crushed the integral by approximately **50–60%**.

### The proof

For the window `2026-06-09 18:01:21 → 2026-06-10 05:55:01` (Solis battery SoC fell from 99% to 15%):

| Method | DC Battery Discharge Wh |
|---|---|
| Old dashboard bucket (`fill(0)`) | **3,761 Wh** |
| Raw InfluxDB `INTEGRAL("value", 1h)` | **8,273 Wh** |
| Mean-power cross-check (avg × hours) | **~8,685 Wh** |

The raw integral is consistent with the mean-power estimate. The dashboard was undercounting by more than **2×**.

With the corrected 8,273 Wh discharge for an 84% SoC drop, the implied full usable capacity is:

```
8,273 Wh / 84% ≈ 9,848 Wh
```

That is consistent with 2 × Growatt GBLI5001 (≈ 9,750 Wh nominal) — the batteries are not degraded, the query was wrong.

### The fix

All energy-total queries now use direct `INTEGRAL("value", 1h)` on the raw measurement:

```sql
-- DC Battery Discharge Wh (correct)
SELECT -1 * INTEGRAL("value", 1h) AS "DC Battery Discharge Wh"
FROM "solis_battery_power"
WHERE $timeFilter AND "host" = '$host'
  AND "source" =~ /^${source:regex}$/
  AND "value" < 0

-- Solis AC Output Wh (correct)
SELECT INTEGRAL("value", 1h) AS "Solis AC Output Wh"
FROM "solis_ac_output_power"
WHERE $timeFilter AND "host" = '$host'
  AND "source" =~ /^${source:regex}$/
```

If time-bucketing is genuinely needed (e.g. for a timeseries chart, not a total), use `fill(previous)` rather than `fill(0)` — `fill(previous)` holds the last real value across gaps, which is accurate for a slowly-changing power signal.

---

## Energy Panel Unit Convention

Energy-total stat and bargauge panels intentionally use **`"unit": "none"`** (raw number display) rather than `"unit": "watth"`.

Grafana's `watth` (Wh) unit applies SI prefix auto-scaling: a value of 8273 would be displayed as "8.27 kWh", losing precision and making it harder to compare to physical meter readings.

By using `none`, the raw Wh integer is displayed directly. The unit `Wh` is included in each panel's title or alias instead.

Panels that are **not** changed:
- Live power panels keep `watt`.
- Battery SoC and FIT Ratio keep `percent`.
- Voltage, current, temperature, time panels keep their appropriate units.

---

## FIT Accounting

The key question the `fit-integrity.json` dashboard answers is:

```
Solis AC Output Wh  ≤  Solis PV Generated Wh  ?
```

If AC output exceeds PV generated, the Solis must have drawn energy from a non-PV source (DC battery, AC input) and exported it — this counts against FIT eligibility.

Derived metrics:

| Metric | Formula |
|---|---|
| FIT Headroom Wh | `Solis PV Generated Wh − Solis AC Output Wh` |
| Potential Non-Solis FIT Wh | `max(Solis AC Output Wh − Solis PV Generated Wh, 0)` |
| FIT Ratio % | `Solis AC Output Wh / Solis PV Generated Wh × 100` |

In Grafana 11.4, these derived FIT stat panels are implemented with panel transformations over the raw Influx integral queries rather than `__expr__` reduce/math chains, because the expression path was still triggering `500 Internal Server Error` for these range-total calculations.

**Important:** These ratios are only meaningful over a time window where start and end Solis battery SoC are equal (or accounted for). Overnight windows will naturally show AC output > PV generated because the battery was discharging.
