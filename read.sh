#!/bin/bash
while :
do


#(grid.realPower,grid.apparentPower,grid.Vrms,grid.Irms,grid.powerFactor)

arduino_values=$(curl --max-time 5.5 192.168.1.177)
grid_real_power=$(echo $arduino_values | awk '{split($0,a,","); print a[1]}')
echo "grid_real_power=$grid_real_power"
grid_apparent_power=$(echo $arduino_values | awk '{split($0,a,","); print a[2]}')
echo "grid_apparent_power=$grid_apparent_power"
grid_vrms=$(echo $arduino_values | awk '{split($0,a,","); print a[3]}')
echo "grid_vrms=$grid_vrms"
grid_irms=$(echo $arduino_values | awk '{split($0,a,","); print a[4]}')
echo "grid_irms=$grid_irms"
grid_pfactor=$(echo $arduino_values | awk '{split($0,a,","); print a[5]}')
echo "grid_pfactor=$grid_pfactor"

solar_real_power=$(echo $arduino_values | awk '{split($0,a,","); print a[6]}')
echo "solar_real_power=$solar_real_power"
solar_apparent_power=$(echo $arduino_values | awk '{split($0,a,","); print a[7]}')
solar_vrms=$(echo $arduino_values | awk '{split($0,a,","); print a[8]}')
solar_irms=$(echo $arduino_values | awk '{split($0,a,","); print a[9]}')
solar_pfactor=$(echo $arduino_values | awk '{split($0,a,","); print a[10]}')
real_home_power=$(echo $arduino_values | awk '{split($0,a,","); print a[11]}')
app_home_power=$(echo $arduino_values | awk '{split($0,a,","); print a[12]}')

app_home_power_data=$(echo "app_home_power,host=server01,source=grid value=$app_home_power")
curl -i -XPOST 'http://localhost:8086/write?db=power' --data-binary "$app_home_power_data"

real_home_power_data=$(echo "real_home_power,host=server01,source=grid value=$real_home_power")
curl -i -XPOST 'http://localhost:8086/write?db=power' --data-binary "$real_home_power_data"

solar_pfactor_data=$(echo "solar_pfactor,host=server01,source=grid value=$solar_pfactor")
curl -i -XPOST 'http://localhost:8086/write?db=power' --data-binary "$solar_pfactor_data"
solar_irms_data=$(echo "solar_irms,host=server01,source=grid value=$solar_irms")
curl -i -XPOST 'http://localhost:8086/write?db=power' --data-binary "$solar_irms_data"
solar_vrms_data=$(echo "solar_vrms,host=server01,source=grid value=$solar_vrms")
curl -i -XPOST 'http://localhost:8086/write?db=power' --data-binary "$solar_vrms_data"
solar_apparent_power_data=$(echo "solar_apparent_power,host=server01,source=grid value=$solar_apparent_power")
curl -i -XPOST 'http://localhost:8086/write?db=power' --data-binary "$solar_apparent_power_data"
solar_real_power_data=$(echo "solar_real_power,host=server01,source=grid value=$solar_real_power")
curl -i -XPOST 'http://localhost:8086/write?db=power' --data-binary "$solar_real_power_data"
grid_pfactor_data=$(echo "grid_pfactor,host=server01,source=grid value=$grid_pfactor")
curl -i -XPOST 'http://localhost:8086/write?db=power' --data-binary "$grid_pfactor_data"
grid_real_power_data=$(echo "grid_real_power,host=server01,source=grid value=$grid_real_power")
curl -i -XPOST 'http://localhost:8086/write?db=power' --data-binary "$grid_real_power_data"
grid_apparent_power_data=$(echo "grid_apparent_power,host=server01,source=grid value=$grid_apparent_power")
curl -i -XPOST 'http://localhost:8086/write?db=power' --data-binary "$grid_apparent_power_data"
grid_vrms_data=$(echo "grid_vrms,host=server01,source=grid value=$grid_vrms")
curl -i -XPOST 'http://localhost:8086/write?db=power' --data-binary "$grid_vrms_data"
grid_irms_data=$(echo "grid_irms,host=server01,source=grid value=$grid_irms")
curl -i -XPOST 'http://localhost:8086/write?db=power' --data-binary "$grid_irms_data"
 	echo "Press [CTRL+C] to stop.."
sleep 1
done
