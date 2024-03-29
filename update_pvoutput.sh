#!/bin/bash
last_energy_generated=$(curl -s 'http://localhost:8086/query?pretty=false' --data-urlencode "db=power" --data-urlencode "q=SELECT LAST(*) FROM total_lifetime_energy_generated" | jq -M '.results[].series[].values[][-1]')
last_energy_used=$(curl -s 'http://localhost:8086/query?pretty=false' --data-urlencode "db=power" --data-urlencode "q=SELECT LAST(*) FROM total_lifetime_energy_used" | jq -M '.results[].series[].values[][-1]')

echo  "last_energy_generated = $last_energy_generated"
echo  "last_energy_used      = $last_energy_used"

energy_used_last_5_mins=$(curl -s 'http://localhost:8086/query?pretty=false' --data-urlencode "db=power" --data-urlencode "q=SELECT MEAN(value) / 12 FROM grid_real_power WHERE time >= now() - 5m" | jq -M '.results[].series[].values[][-1]')
if [[ $energy_used_last_5_mins == -* ]]; then
    energy_used_last_5_mins=0
fi
energy_generated_last_5_mins=$(curl -s 'http://localhost:8086/query?pretty=false' --data-urlencode "db=power" --data-urlencode "q=SELECT MEAN(value) / 12 FROM solar_real_power WHERE time >= now() - 5m" | jq -M '.results[].series[].values[][-1]')
if [[ $energy_generated_last_5_mins == -* ]]; then
    energy_generated_last_5_mins=0
fi
echo  "energy_used_last_5_mins      = $energy_used_last_5_mins"
echo  "energy_generated_last_5_mins = $energy_generated_last_5_mins"


avg_power_usage_last_5_mins=$(curl -s 'http://localhost:8086/query?pretty=false' --data-urlencode "db=power" --data-urlencode "q=SELECT MEAN(value)  FROM grid_real_power WHERE time >= now() - 5m" | jq -M '.results[].series[].values[][-1]')
if [[ $avg_power_usage_last_5_mins == -* ]]; then
    avg_power_usage_last_5_mins=0
fi
avg_power_generated_last_5_mins=$(curl -s 'http://localhost:8086/query?pretty=false' --data-urlencode "db=power" --data-urlencode "q=SELECT MEAN(value)  FROM solar_real_power WHERE time >= now() - 5m" | jq -M '.results[].series[].values[][-1]')
if [[ $avg_power_generated_last_5_mins == -* ]]; then
    avg_power_generated_last_5_mins=0
fi

echo  "avg_power_usage_last_5_mins     = $avg_power_usage_last_5_mins"
echo  "avg_power_generated_last_5_mins = $avg_power_generated_last_5_mins"

new_energy_used="$( bc <<<"$energy_used_last_5_mins + $last_energy_used" )"
curlstring=$(echo "total_lifetime_energy_used,host=server01,source=grid value=$new_energy_used")
curl -i -XPOST 'http://localhost:8086/write?db=power' --data-binary "$curlstring"

new_energy_generated="$( bc <<<"$energy_generated_last_5_mins + $last_energy_generated" )"
if [[ $new_energy_generated == -* ]]; then
    new_energy_generated=0
fi
curlstring=$(echo "total_lifetime_energy_generated,host=server01,source=grid value=$new_energy_generated")
curl -i -XPOST 'http://localhost:8086/write?db=power' --data-binary "$curlstring"

echo  "new_energy_used      = $new_energy_used"
echo  "new_energy_generated = $new_energy_generated"





date=$(date +%Y%m%d)
time=$(date +%H:%M)
echo "strings = curl -d \"d=$date\" -d \"t=$time\" -d \"v1=$new_energy_generated\" -d \"v2=$avg_power_generated_last_5_mins\" -d \"v3=$new_energy_used\" -d \"v4=$avg_power_usage_last_5_mins\" -d \"c1=1\" -H \"X-Pvoutput-Apikey: 4a50099c09c275f52484aa4b4f3c308f334807d7\" -H \"X-Pvoutput-SystemId: 6417\" https://pvoutput.org/service/r2/addstatus.jsp"
curl -d "d=$date" -d "t=$time" -d "v1=$new_energy_generated" -d "v2=$avg_power_generated_last_5_mins" -d "v3=$new_energy_used" -d "v4=$avg_power_usage_last_5_mins" -d "c1=1"  -H "X-Pvoutput-Apikey: 4a50099c09c275f52484aa4b4f3c308f334807d7" -H "X-Pvoutput-SystemId: 6417" https://pvoutput.org/service/r2/addstatus.jsp

