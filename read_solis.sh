#!/bin/bash

ESP32_URL="http://esp32-9D244C/api/solis"
INFLUX_URL="http://localhost:8086/write?db=power"
HOST_TAG="server01"
SOURCE_TAG="solis"

write_line() {
    local measurement="$1"
    local value="$2"

    if [ -z "$value" ] || [ "$value" = "null" ]; then
        return
    fi

    local line="${measurement},host=${HOST_TAG},source=${SOURCE_TAG} value=${value}"
    echo "$line"
    curl -s -XPOST "$INFLUX_URL" --data-binary "$line" >/dev/null
}

while :
do
    json=$(curl -s --max-time 2 "$ESP32_URL")

    if [ -z "$json" ]; then
        echo "No JSON returned from $ESP32_URL"
        sleep 1
        continue
    fi

    # Raw register values from ESP32 JSON
    pv1_v_raw=$(echo "$json" | jq -r '."33050".raw')
    pv1_i_raw=$(echo "$json" | jq -r '."33051".raw')
    pv2_v_raw=$(echo "$json" | jq -r '."33052".raw')
    pv2_i_raw=$(echo "$json" | jq -r '."33053".raw')
    grid_v_raw=$(echo "$json" | jq -r '."33074".raw')
    grid_f_raw=$(echo "$json" | jq -r '."33095".raw')
    grid_p_signed=$(echo "$json" | jq -r '."33132".signed')
    batt_soc_raw=$(echo "$json" | jq -r '."33140".raw')
    batt_v_raw=$(echo "$json" | jq -r '."33142".raw')
    batt_i_signed=$(echo "$json" | jq -r '."33135".signed')
    batt_dir_raw=$(echo "$json" | jq -r '."33136".raw')

    poll_count=$(echo "$json" | jq -r '.pollCount')
    read_errors=$(echo "$json" | jq -r '.readErrors')
    lock_timeouts=$(echo "$json" | jq -r '.lockTimeouts')

    # Scaled known-good values
    pv1_v=$(awk "BEGIN { printf \"%.1f\", $pv1_v_raw / 10.0 }")
    pv1_i=$(awk "BEGIN { printf \"%.1f\", $pv1_i_raw / 10.0 }")
    pv2_v=$(awk "BEGIN { printf \"%.1f\", $pv2_v_raw / 10.0 }")
    pv2_i=$(awk "BEGIN { printf \"%.1f\", $pv2_i_raw / 10.0 }")
    grid_v=$(awk "BEGIN { printf \"%.1f\", $grid_v_raw / 10.0 }")
    grid_f=$(awk "BEGIN { printf \"%.2f\", $grid_f_raw / 100.0 }")
    batt_soc="$batt_soc_raw"
    batt_v=$(awk "BEGIN { printf \"%.2f\", $batt_v_raw / 100.0 }")
    batt_i=$(awk "BEGIN { printf \"%.1f\", $batt_i_signed / 10.0 }")

    # Derived values
    pv1_p=$(awk "BEGIN { printf \"%.1f\", ($pv1_v_raw / 10.0) * ($pv1_i_raw / 10.0) }")
    pv2_p=$(awk "BEGIN { printf \"%.1f\", ($pv2_v_raw / 10.0) * ($pv2_i_raw / 10.0) }")
    pv_total_p=$(awk "BEGIN { printf \"%.1f\", (($pv1_v_raw / 10.0) * ($pv1_i_raw / 10.0)) + (($pv2_v_raw / 10.0) * ($pv2_i_raw / 10.0)) }")
    batt_p=$(awk "BEGIN { printf \"%.1f\", ($batt_v_raw / 100.0) * ($batt_i_signed / 10.0) }")

    # Write known-good values
    write_line "solis_pv1_voltage" "$pv1_v"
    write_line "solis_pv1_current" "$pv1_i"
    write_line "solis_pv2_voltage" "$pv2_v"
    write_line "solis_pv2_current" "$pv2_i"
    write_line "solis_grid_voltage" "$grid_v"
    write_line "solis_grid_frequency" "$grid_f"
    write_line "solis_grid_power" "$grid_p_signed"
    write_line "solis_battery_soc" "$batt_soc"
    write_line "solis_battery_voltage" "$batt_v"

    # Candidate values you currently want monitored
    write_line "solis_battery_current" "$batt_i"
    write_line "solis_battery_direction_flag" "$batt_dir_raw"

    # Derived values
    write_line "solis_pv1_power" "$pv1_p"
    write_line "solis_pv2_power" "$pv2_p"
    write_line "solis_pv_total_power" "$pv_total_p"
    write_line "solis_battery_power" "$batt_p"

    # Useful diagnostics
    write_line "solis_poll_count" "$poll_count"
    write_line "solis_read_errors" "$read_errors"
    write_line "solis_lock_timeouts" "$lock_timeouts"

    sleep 5
done

