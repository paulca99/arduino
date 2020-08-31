#!/bin/bash

processData (){
        value=$(echo $1 | awk -v position=$2 '{split($0,a,","); print a[position]}')
        curlstring=$(echo "$3,host=server01,source=grid value=$value")
    curl -i -XPOST 'http://localhost:8086/write?db=power' --data-binary "$curlstring"
}
while :
do


#grid.realPower,grid.apparentPower,grid.Vrms,grid.Irms,grid.powerFactor,solar.realPower,solar.apparentPower,solar.Vrms,solar.Irms,solar.powerFactor,realHomePower,appHomePower;
 
arduino_values=$(curl --max-time 5.5 192.168.1.177)
processData $arduino_values 1 "grid_real_power"
processData $arduino_values 2 "grid_apparent_power"
processData $arduino_values 3 "grid_vrms"
processData $arduino_values 4 "grid_irms"
processData $arduino_values 5 "grid_pfactor"
processData $arduino_values 6 "solar_real_power"
processData $arduino_values 7 "solar_apparent_power"
processData $arduino_values 8 "solar_vrms"
processData $arduino_values 9 "solar_irms"
processData $arduino_values 10 "solar_pfactor"
processData $arduino_values 11 "home_real_power"
processData $arduino_values 12 "home_app_power"
processData $arduino_values 13 "b1volts"
processData $arduino_values 14 "b2volts"
processData $arduino_values 15 "b3volts"
processData $arduino_values 16 "b4volts"


 	echo "Press [CTRL+C] to stop.."
sleep 1
done
