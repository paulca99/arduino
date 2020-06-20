    #!/bin/bash

    insertData (){
        echo "first=$1"
        echo "second=$2"
        echo "third=$3"
        curlstring=$(echo "$1,host=server01,source=grid value=$2 $3" )
        curl -i -XPOST 'http://localhost:8086/write?db=power' --data-binary "$curlstring"
   }

    while read REPLY; do
        echo "REPLy = $REPLY"
        # date=1 totalpower=2 powerstring1 voltagestring1 powerstring2 voltagestring2 temp acvolts totalKWh
        time=$(echo $REPLY | awk -v position=1 '{split($0,a," "); print a[position]}')
        timestampStr="$3-$2-$1T$time"
        timestamp1=$(date -d "$timestampStr" +%s)
        echo "***timestamp1=$timestamp1"
        timestamp="${timestamp1}000000000"
        echo "***timestamp=$timestamp"
        totalpower=$(echo $REPLY | awk -v position=2 '{split($0,a," "); print a[position]}')
        powerstring1=$(echo $REPLY | awk -v position=3 '{split($0,a," "); print a[position]}')
        voltstring1=$(echo $REPLY | awk -v position=4 '{split($0,a," "); print a[position]}')
        powerstring2=$(echo $REPLY | awk -v position=5 '{split($0,a," "); print a[position]}')
        voltstring2=$(echo $REPLY | awk -v position=6 '{split($0,a," "); print a[position]}')
        temp=$(echo $REPLY | awk -v position=7 '{split($0,a," "); print a[position]}')
        acvolts=$(echo $REPLY | awk -v position=8 '{split($0,a," "); print a[position]}')
        totalkwh=$(echo $REPLY | awk -v position=9 '{split($0,a," "); print a[position]}')
        insertData "inv_power" "$totalpower" "$timestamp"
        insertData "inv_powers1" "$powerstring1" "$timestamp"
        insertData "inv_volts1" "$voltstring1" "$timestamp"
        insertData "inv_powers2" "$powerstring2" "$timestamp"
        insertData "inv_volts2" "$voltstring2" "$timestamp"
        insertData "inv_temp" "$temp" "$timestamp"
        insertData "inv_ac_volt" "$acvolts" "$timestamp"
        insertData "inv_kwh" "$totalkwh" "$timestamp"

    done
