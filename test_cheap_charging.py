"""
Tests for cheap-charging support in read_master2.py.

Feature contract:

- read_ac_bms_power_w() queries InfluxDB for the latest value of ac_bms_power.
- If local time is >= 00:00 and < 06:00 AND ac_bms_power > 600W:
    enter CHEAP_CHARGING state (from NORMAL or UNKNOWN).
- CHEAP_CHARGING uses the same Solis ToU battery-off method:
    RUN + grid-charge
    0.5A kick
    now-2min -> now+12h window
    later 0.0A hold via existing hold mechanism
- While in CHEAP_CHARGING, Solis polling/metric writing continues normally.
- Exit CHEAP_CHARGING back to NORMAL when ac_bms_power < 600W.
- CHEAP_CHARGING numeric controller-state metric == 3.
"""

import importlib
from datetime import datetime as real_datetime

import pytest

m = importlib.import_module("read_master2")

STATE_CHEAP_CHARGING = m.STATE_CHEAP_CHARGING


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

class FakeSerial:
    def __init__(self):
        self.in_waiting = 0

    def read(self, *_args, **_kwargs):
        return b""

    def write(self, *_args, **_kwargs):
        return None

    def flush(self):
        return None


class FakeDateTime(real_datetime):
    fixed_now = real_datetime(2026, 1, 1, 1, 30, 0)

    @classmethod
    def now(cls, tz=None):
        if tz is not None:
            return cls.fixed_now.replace(tzinfo=tz)
        return cls.fixed_now


def _set_fake_hour(monkeypatch, hour, minute=30):
    FakeDateTime.fixed_now = real_datetime(2026, 1, 1, hour, minute, 0)
    monkeypatch.setattr(m, "datetime", FakeDateTime)


# ---------------------------------------------------------------------------
# Fixtures
# ---------------------------------------------------------------------------

@pytest.fixture
def controller():
    return {
        "state": m.STATE_NORMAL,
        "battery_off_until_ts": None,
        "last_tou_write_ts": 0,
        "battery_off_hold_ts": None,
    }


@pytest.fixture
def solis_sample():
    """A Solis sample dict that will NOT trigger battery-off conditions."""
    return {
        "pv1Voltage": 100.0,
        "pv1Current": 1.0,
        "pv2Voltage": 100.0,
        "pv2Current": 1.0,
        "gridVoltage": 240.0,
        "gridFrequency": 50.0,
        "gridPower": 100.0,          # exporting -> no grid-import battery-off
        "solisInverterActivePowerW": 200,
        "solisAcOutputPowerW": 200,
        "solisAcInputPowerW": 0,
        "batterySoc": 50,            # well above all thresholds
        "batteryVoltage": 52.0,
        "batteryCurrent": 0.0,
        "batteryDirectionFlag": 0,   # not discharging
        "batteryDirection": "charging",
        "batteryPowerW": 0.0,
        "pv1PowerW": 100.0,
        "pv2PowerW": 100.0,
        "pvTotalPowerW": 200.0,
        "gridFrequencyRaw": 5000,
        "reg33079Raw": 0,
        "reg33080Raw": 0,
        "reg33081Raw": 200,
    }


# ---------------------------------------------------------------------------
# 1. InfluxDB query helper
# ---------------------------------------------------------------------------

def test_read_ac_bms_power_queries_influx_last_value(monkeypatch):
    """read_ac_bms_power_w() must query the correct InfluxDB endpoint and return
    the float value from the 'last' column of the ac_bms_power series."""
    captured = {}

    class FakeResponse:
        def raise_for_status(self):
            return None

        def json(self):
            return {
                "results": [
                    {
                        "series": [
                            {
                                "name": "ac_bms_power",
                                "columns": ["time", "last"],
                                "values": [["2026-01-01T01:30:00Z", 650.0]],
                            }
                        ]
                    }
                ]
            }

    def fake_get(url, params=None, timeout=None):
        captured["url"] = url
        captured["params"] = params
        captured["timeout"] = timeout
        return FakeResponse()

    monkeypatch.setattr(m.requests, "get", fake_get)

    result = m.read_ac_bms_power_w()

    assert result == 650.0
    assert "/query" in captured["url"]
    assert captured["params"]["db"] == "power"
    assert captured["params"]["q"] == 'SELECT last("value") FROM "ac_bms_power"'
    assert captured["timeout"] <= 2


def test_read_ac_bms_power_returns_none_on_http_error(monkeypatch):
    """read_ac_bms_power_w() must return None and not raise when the HTTP call fails."""
    def fake_get(*_args, **_kwargs):
        raise IOError("connection refused")

    monkeypatch.setattr(m.requests, "get", fake_get)
    assert m.read_ac_bms_power_w() is None


def test_read_ac_bms_power_returns_none_on_missing_series(monkeypatch):
    """read_ac_bms_power_w() must return None when the series key is absent."""

    class FakeResponse:
        def raise_for_status(self):
            return None

        def json(self):
            return {"results": [{}]}

    monkeypatch.setattr(m.requests, "get", lambda *a, **k: FakeResponse())
    assert m.read_ac_bms_power_w() is None


# ---------------------------------------------------------------------------
# 2. Entering cheap charging
# ---------------------------------------------------------------------------

def test_enters_cheap_charging_between_midnight_and_6am_when_power_over_600(
    monkeypatch, controller, solis_sample
):
    """NORMAL -> CHEAP_CHARGING when inside 00:00–06:00 and ac_bms_power > 600W."""
    _set_fake_hour(monkeypatch, 1)
    monkeypatch.setattr(m, "read_ac_bms_power_w", lambda: 650.0)

    writes = []

    def fake_write_tou_registers(*args, **kwargs):
        writes.append(kwargs)
        return True

    monkeypatch.setattr(m, "write_tou_registers", fake_write_tou_registers)
    monkeypatch.setattr(m, "flush_serial_input", lambda _ser: None)

    m.manage_battery_off_controller(FakeSerial(), solis_sample, controller)

    assert controller["state"] == STATE_CHEAP_CHARGING
    assert controller["battery_off_until_ts"] is not None
    assert controller["battery_off_hold_ts"] is not None
    assert len(writes) == 1
    assert writes[0]["run_stop"] == m.TOU_RUN
    assert writes[0]["current_x10"] == m.TOU_CURRENT_0_5A


def test_enters_cheap_charging_from_unknown_state(monkeypatch, solis_sample):
    """UNKNOWN -> CHEAP_CHARGING when in cheap window and power above threshold."""
    controller = {
        "state": m.STATE_UNKNOWN,
        "battery_off_until_ts": None,
        "last_tou_write_ts": 0,
        "battery_off_hold_ts": None,
    }
    _set_fake_hour(monkeypatch, 2)
    monkeypatch.setattr(m, "read_ac_bms_power_w", lambda: 700.0)

    writes = []

    def fake_write_tou_registers(*args, **kwargs):
        writes.append(kwargs)
        return True

    monkeypatch.setattr(m, "write_tou_registers", fake_write_tou_registers)
    monkeypatch.setattr(m, "flush_serial_input", lambda _ser: None)

    m.manage_battery_off_controller(FakeSerial(), solis_sample, controller)

    assert controller["state"] == STATE_CHEAP_CHARGING
    assert len(writes) == 1


# ---------------------------------------------------------------------------
# 3. Not entering outside the cheap window
# ---------------------------------------------------------------------------

def test_does_not_enter_cheap_charging_outside_midnight_to_6am(
    monkeypatch, controller, solis_sample
):
    """No cheap-charging entry outside the 00:00–06:00 window."""
    _set_fake_hour(monkeypatch, 7)
    monkeypatch.setattr(m, "read_ac_bms_power_w", lambda: 650.0)

    def fail_if_called(*_args, **_kwargs):
        raise AssertionError("ToU write must not happen outside cheap-charging window")

    monkeypatch.setattr(m, "write_tou_registers", fail_if_called)

    m.manage_battery_off_controller(FakeSerial(), solis_sample, controller)

    assert controller["state"] == m.STATE_NORMAL


def test_does_not_enter_cheap_charging_at_6am_exactly(
    monkeypatch, controller, solis_sample
):
    """06:00 is exclusive – no entry at exactly 06:00."""
    _set_fake_hour(monkeypatch, 6, minute=0)
    monkeypatch.setattr(m, "read_ac_bms_power_w", lambda: 650.0)

    def fail_if_called(*_args, **_kwargs):
        raise AssertionError("ToU write must not happen at 06:00 (exclusive)")

    monkeypatch.setattr(m, "write_tou_registers", fail_if_called)

    m.manage_battery_off_controller(FakeSerial(), solis_sample, controller)

    assert controller["state"] == m.STATE_NORMAL


# ---------------------------------------------------------------------------
# 4. Not entering when power is 600W or below
# ---------------------------------------------------------------------------

def test_does_not_enter_cheap_charging_when_power_is_exactly_600(
    monkeypatch, controller, solis_sample
):
    """Threshold is strict: 600.0W must NOT trigger entry (> 600W required)."""
    _set_fake_hour(monkeypatch, 2)
    monkeypatch.setattr(m, "read_ac_bms_power_w", lambda: 600.0)

    def fail_if_called(*_args, **_kwargs):
        raise AssertionError("ToU write must not happen when ac_bms_power == 600W")

    monkeypatch.setattr(m, "write_tou_registers", fail_if_called)

    m.manage_battery_off_controller(FakeSerial(), solis_sample, controller)

    assert controller["state"] == m.STATE_NORMAL


def test_does_not_enter_cheap_charging_when_power_is_below_600(
    monkeypatch, controller, solis_sample
):
    """Power below 600W must not trigger cheap-charging entry."""
    _set_fake_hour(monkeypatch, 3)
    monkeypatch.setattr(m, "read_ac_bms_power_w", lambda: 500.0)

    def fail_if_called(*_args, **_kwargs):
        raise AssertionError("ToU write must not happen when ac_bms_power < 600W")

    monkeypatch.setattr(m, "write_tou_registers", fail_if_called)

    m.manage_battery_off_controller(FakeSerial(), solis_sample, controller)

    assert controller["state"] == m.STATE_NORMAL


# ---------------------------------------------------------------------------
# 5. Exiting cheap charging
# ---------------------------------------------------------------------------

def test_exits_cheap_charging_to_normal_when_power_under_600(
    monkeypatch, solis_sample
):
    """CHEAP_CHARGING -> NORMAL when ac_bms_power < 600W."""
    controller = {
        "state": STATE_CHEAP_CHARGING,
        "battery_off_until_ts": FakeDateTime.fixed_now.timestamp() + 3600,
        "last_tou_write_ts": 0,
        "battery_off_hold_ts": FakeDateTime.fixed_now.timestamp() + 30,
    }
    _set_fake_hour(monkeypatch, 3)
    monkeypatch.setattr(m, "read_ac_bms_power_w", lambda: 599.0)

    writes = []

    def fake_write_tou_registers(*args, **kwargs):
        writes.append(kwargs)
        return True

    monkeypatch.setattr(m, "write_tou_registers", fake_write_tou_registers)
    monkeypatch.setattr(m, "flush_serial_input", lambda _ser: None)

    m.manage_battery_off_controller(FakeSerial(), solis_sample, controller)

    assert controller["state"] == m.STATE_NORMAL
    assert controller["battery_off_until_ts"] is None
    assert controller["battery_off_hold_ts"] is None
    assert len(writes) == 1
    assert writes[0]["run_stop"] == m.TOU_STOP
    assert writes[0]["current_x10"] == m.TOU_CURRENT_10_0A
    assert writes[0]["start_hour"] == 0
    assert writes[0]["start_minute"] == 0
    assert writes[0]["end_hour"] == 0
    assert writes[0]["end_minute"] == 0


def test_exits_cheap_charging_when_ac_bms_power_unavailable(
    monkeypatch, solis_sample
):
    """CHEAP_CHARGING -> NORMAL when read_ac_bms_power_w() returns None."""
    controller = {
        "state": STATE_CHEAP_CHARGING,
        "battery_off_until_ts": FakeDateTime.fixed_now.timestamp() + 3600,
        "last_tou_write_ts": 0,
        "battery_off_hold_ts": None,
    }
    _set_fake_hour(monkeypatch, 2)
    monkeypatch.setattr(m, "read_ac_bms_power_w", lambda: None)

    writes = []

    def fake_write_tou_registers(*args, **kwargs):
        writes.append(kwargs)
        return True

    monkeypatch.setattr(m, "write_tou_registers", fake_write_tou_registers)
    monkeypatch.setattr(m, "flush_serial_input", lambda _ser: None)

    m.manage_battery_off_controller(FakeSerial(), solis_sample, controller)

    assert controller["state"] == m.STATE_NORMAL
    assert len(writes) == 1


# ---------------------------------------------------------------------------
# 6. CHEAP_CHARGING numeric metric == 3
# ---------------------------------------------------------------------------

def test_cheap_charging_controller_state_metric_is_3(monkeypatch, solis_sample):
    """write_solis_metrics() must emit solis_battery_off_controller_state = 3
    when controller_state is CHEAP_CHARGING."""
    written = []

    def fake_write_line(measurement, value, source_tag):
        written.append((measurement, value, source_tag))

    monkeypatch.setattr(m, "write_line", fake_write_line)

    m.write_solis_metrics(
        solis_sample,
        poll_count=1,
        read_errors=0,
        controller_state=STATE_CHEAP_CHARGING,
    )

    state_values = [
        value
        for name, value, _ in written
        if name == "solis_battery_off_controller_state"
    ]
    assert state_values == [3]


def test_cheap_charging_writes_all_expected_solis_metrics(monkeypatch, solis_sample):
    """All expected Solis measurements are written in CHEAP_CHARGING state."""
    written = []

    def fake_write_line(measurement, value, source_tag):
        written.append((measurement, value, source_tag))

    monkeypatch.setattr(m, "write_line", fake_write_line)

    m.write_solis_metrics(
        solis_sample,
        poll_count=123,
        read_errors=0,
        controller_state=STATE_CHEAP_CHARGING,
    )

    measurement_names = {name for name, _value, _source in written}

    expected = {
        "solis_pv1_voltage",
        "solis_pv1_current",
        "solis_pv2_voltage",
        "solis_pv2_current",
        "solis_grid_voltage",
        "solis_grid_frequency",
        "solis_grid_power",
        "solis_inverter_active_power",
        "solis_ac_output_power",
        "solis_ac_input_power",
        "solis_battery_soc",
        "solis_battery_voltage",
        "solis_battery_current",
        "solis_battery_direction_flag",
        "solis_battery_power",
        "solis_pv1_power",
        "solis_pv2_power",
        "solis_pv_total_power",
        "solis_poll_count",
        "solis_read_errors",
        "solis_battery_off_controller_state",
    }

    assert expected.issubset(measurement_names)


# ---------------------------------------------------------------------------
# 7. apply_battery_off_hold applies to CHEAP_CHARGING
# ---------------------------------------------------------------------------

def test_apply_battery_off_hold_runs_for_cheap_charging(monkeypatch):
    """apply_battery_off_hold() must write the 0.0A hold when state is
    CHEAP_CHARGING and the hold timestamp has elapsed."""
    import time as _time

    controller = {
        "state": STATE_CHEAP_CHARGING,
        "battery_off_hold_ts": _time.time() - 1,  # already elapsed
    }

    writes = []

    def fake_write_holding(ser, reg, value):
        writes.append((reg, value))
        return True

    monkeypatch.setattr(m, "write_solis_holding_reg", fake_write_holding)
    monkeypatch.setattr(m, "flush_serial_input", lambda _ser: None)

    m.apply_battery_off_hold(FakeSerial(), controller)

    assert len(writes) == 1
    assert writes[0] == (m.REG_TOU_CHARGE_CURRENT, m.TOU_CURRENT_0_0A)
    assert controller["battery_off_hold_ts"] is None


def test_apply_battery_off_hold_discards_when_state_is_normal(monkeypatch):
    """apply_battery_off_hold() must discard the hold if state is no longer
    BATTERY_OFF or CHEAP_CHARGING."""
    import time as _time

    controller = {
        "state": m.STATE_NORMAL,
        "battery_off_hold_ts": _time.time() - 1,
    }

    def fail_if_called(*_args, **_kwargs):
        raise AssertionError("write_solis_holding_reg must not be called")

    monkeypatch.setattr(m, "write_solis_holding_reg", fail_if_called)

    m.apply_battery_off_hold(FakeSerial(), controller)

    assert controller["battery_off_hold_ts"] is None
