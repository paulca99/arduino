#!/bin/bash
processData (){
        value=$(echo $1 | awk -v position=$2 '{split($0,a,","); print a[position]}')
        echo "Value $1=$value"
        writeDB "$3" $value
}
writeDB (){
        curlstring=$(echo "$1,host=server01,source=grid value=$2")
        echo "curlstring=$curlstring"
    curl -i -XPOST 'http://localhost:8086/write?db=power' --data-binary "$curlstring"
}
while :
do
sleep 1

#grid.realPower,grid.apparentPower,grid.Vrms,grid.Irms,grid.powerFactor,solar.realPower,solar.apparentPower,solar.Vrms,solar.Irms,solar.powerFactor,realHomePower,appHomePower;
 

inverter_xml=$(curl --connect-timeout 0.2 http://192.168.1.119/real_time_data.xml)
arduino_values=$(curl --max-time 5.5 esp32-3B6098:8080)
processData $arduino_values 1 "batt_volts"
processData $arduino_values 2 "grid_power"
processData $arduino_values 3 "grid_voltage"
processData $arduino_values 4 "resistance"
processData $arduino_values 5 "charger_power"
processData $arduino_values 6 "gti_power"
processData $arduino_values 7 "hour"
processData $arduino_values 8 "gti_enabled"
processData $arduino_values 9 "psuVolts1"
processData $arduino_values 10 "psuVolts2"
processData $arduino_values 11 "psuVolts3"
processData $arduino_values 12 "psuVolts4"
processData $arduino_values 13 "psuVolts5"
processData $arduino_values 14 "chargerPLimit"
inverterpower=$(echo $inverter_xml | xmllint --xpath "//real_time_data/p-ac/text()" -)
invertervoltage=$(echo $inverter_xml | xmllint --xpath "//real_time_data/v-grid/text()" -)
inverterfreq=$(echo $inverter_xml | xmllint --xpath "//real_time_data/f-grid/text()" -)
writeDB "inverter-power" $inverterpower
writeDB "inverter-voltage" $invertervoltage
writeDB "inverter-freq" $inverterfreq

 	echo "Press [CTRL+C] to stop.."
done
