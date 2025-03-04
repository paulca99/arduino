#!/bin/bash

processData (){
        value=$(echo $1 | awk -v position=$2 '{split($0,a,","); print a[position]}')
        echo "Value $1=$value"
        curlstring=$(echo "$3,host=server01,source=grid value=$value")
        echo "curlstring=$curlstring"
    curl -i -XPOST 'http://localhost:8086/write?db=power' --data-binary "$curlstring"
}
while :
do


#grid.realPower,grid.apparentPower,grid.Vrms,grid.Irms,grid.powerFactor,solar.realPower,solar.apparentPower,solar.Vrms,solar.Irms,solar.powerFactor,realHomePower,appHomePower;
 
arduino_values=$(curl --max-time 5.5 192.168.1.104:8080)
processData $arduino_values 1 "batt_volts"
processData $arduino_values 2 "grid_power"
processData $arduino_values 3 "grid_voltage"
processData $arduino_values 4 "resistance"
processData $arduino_values 5 "charger_power"
processData $arduino_values 6 "gti_power"
processData $arduino_values 7 "hour"
processData $arduino_values 8 "gti_enabled"

 	echo "Press [CTRL+C] to stop.."
done