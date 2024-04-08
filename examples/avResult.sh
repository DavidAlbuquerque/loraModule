#!/bin/bash

if [ $# -ne 3 ]
then
    echo "parameters missing"
    exit 1
fi

trial=$1
interval=$2
flagRtx=$3

if [ ! -d TestResult/ ]
then
    echo "No such directory"
    exit 1
fi

count=0;
total=0;
val=0;

metrics=("metricAll" "metricRegular" "metricAlarm")

for metric in "${metrics[@]}"
do
    if [ $metric == "metricRegular" ]
    then
        range="{80..1200..80}"
		deviceType="alarm"
    elif [ $metric == "metricAlarm" ]
    then
        range="{20..300..20}"
		deviceType="regular"
    else
        range="{100..1500..100}"
		deviceType="all"
    fi

    if [ $flagRtx -eq 0 ]
    then
        for f in {7..12}
        do
            touch ./TestResult/test$trial/traffic-$interval/$metric/result-SF$f.dat
            output="./TestResult/test$trial/traffic-$interval/$metric/result-SF$f.dat"
            echo "#numSta, Throughput(Kbps), ProbSucc(%), ProbLoss(%), avgDelay(Seconds)" > $output
            
            file="./TestResult/test$trial/traffic-$interval/$metric/result-STAs-$deviceType-SF$f.dat"

            if [ ! -f $file ]
            then 
                echo "no such file $file"
                exit 1
            fi

            for j in $(eval echo $range)
            do
                printf "$j, " >> $output
                for k in {2..5}
                do
                    for i in $( cat $file | grep -w $j | awk -v l="$k" '{ print $l; }' | sed 's/,//g' )
                    do 
                        total=$( echo $total+$i | bc )
                        ((count++))
                    done
                    val=$( echo "scale=5; $total/$count" | bc)
                    if [ $k -eq 5 ]
                    then
                        printf  "$val" >> $output
                    else
                        printf "$val, " >> $output
                    fi
                    total=0
                    count=0
                done
                printf "\n" >> $output
            done
        done
    fi

    if [ $flagRtx -eq 1 ]
    then
        for f in 7
        do
            touch ./TestResult/test$trial/traffic-$interval/$metric/result-RTX$f.dat
            output="./TestResult/test$trial/traffic-$interval/$metric/result-RTX$f.dat"
            echo "#numSta, sent, rtxQuant[0], rtxQuant[1], rtxQuant[2], rtxQuant[3]" > $output
            
            file="./TestResult/test$trial/traffic-$interval/$metric/result-STAs-$deviceType-RTX$f.dat"

            if [ ! -f $file ]
            then 
                echo "no such file $file"
                exit 1
            fi

            for j in $(eval echo $range)
            do
                printf "$j, " >> $output
                for k in {2..6}
                do
                    for i in $( cat $file | grep -w $j | awk -v l="$k" '{ print $l; }' | sed 's/,//g' )
                    do 
                        total=$( echo $total+$i | bc )
                        ((count++))
                    done
                    val=$( echo "$total/$count" | bc)
                    if [ $k -eq 6 ]
                    then
                        printf  "$val" >> $output
                    else
                        printf "$val, " >> $output
                    fi
                    total=0
                    count=0
                done
                printf "\n" >> $output
            done
        done
    fi
done
