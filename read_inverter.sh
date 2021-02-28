#!/bin/bash

#/home/pi/aurora/2020/d2_20200618.txt

yesterday_year=$(date -d "3 day ago" '+%Y')
yesterday_month=$(date -d "3 day ago" '+%m')
yesterday_day=$(date -d "3 day ago" '+%d')
year=$(date +"%Y")
month=$(date +"%m")
day=$(date +"%d")
filename="/home/pi/aurora/$year/d2_$year$month$day.txt"
old_filename="/home/pi/aurora/$yesterday_year/d2_$yesterday_year$yesterday_month$yesterday_day.txt"
echo "filename=$filename"
until [ -f $filename ]
do
     sleep 60
done

rm -rf $old_filename

tail -f $filename | /home/pi/process_inv_data.sh $day $month $year



