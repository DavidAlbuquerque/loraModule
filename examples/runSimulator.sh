#!/bin/bash

if [ $# -ne 10 ]
then
	echo "parameters missing"
	exit 1
fi

pDevicesAlarm=$2
gwRing=$3
rad=$4
gwRad=$5
simTime=$6
interval=$7
pEDs=$8
flagRtx=$9
trial=${10}

echo "##### Simulation Start #####"

if [ ! -d scratch/TestResult/ ]
then
	mkdir scratch/TestResult/
fi

RANDOM=$$

if [ ! -d scratch/TestResult/test$trial/ ]
	then
	mkdir scratch/TestResult/test$trial/
fi

if [ ! -d scratch/TestResult/test$trial/traffic-$interval/ ]
	then
	mkdir scratch/TestResult/test$trial/traffic-$interval/
fi

if [ $1 -eq 0 ]
then

	touch ./TestResult/test$trial/traffic-$interval/result-STAs.dat
	file1="./TestResult/test$trial/traffic-$interval/result-STAs.dat"
	#echo "#numSta, Throughput(Kbps), ProbSucc(%), ProbLoss(%), avgDelay(Seconds)" > ./TestResult/test$trial/traffic-$interval/result-STAs.dat 
   	
	touch ./TestResult/test$trial/traffic-$interval/mac-STAs-GW-$gwRing.txt
	file2="./TestResult/test$trial/traffic-$interval/mac-STAs-GW-$gwRing.txt"

	for numSta in {0..10..2}
	do
			echo -ne "trial:$trial-numSTA:$numSta #"

			if [ ! -d TestResult/test$trial/traffic-$interval/pcap-sta-$numSta/ ]
			then
				mkdir TestResult/test$trial/traffic-$interval/pcap-sta-$numSta/
			fi

			touch TestResult/test$trial/time-record$numSta.txt

			echo "Time: $(date) $interval $numSta" >> TestResult/test$trial/time-record$numSta.txt

		for numSeed in {1..5}
		do
			echo -ne "$numSeed \r"
  			./ns3 run "lorawan-network-sim --nSeed=$RANDOM --nDevices=$numSta --nGateways=$gwRing --radius=$rad --gatewayRadius=$gwRad --simulationTime=$simTime --appPeriod=$interval --fileMetric=$file1 --fileData=$file2 --printEDs=$pEDs --trial=$trial"  > ./TestResult/test$trial/traffic-$interval/pcap-sta-$numSta/record-$numSta.txt 2>&1
		done
	echo ""
	done
elif [ $1 -eq 1 ]
then

	if [ ! -d ./scratch/TestResult/test$trial/traffic-$interval/metricR/ ]
		then
			mkdir ./scratch/TestResult/test$trial/traffic-$interval/metricRegular/
		fi
	
	if [ ! -d ./scratch/TestResult/test$trial/traffic-$interval/metricA/ ]
		then
			mkdir ./scratch/TestResult/test$trial/traffic-$interval/metricAlarm/
		fi
	
	if [ ! -d ./scratch/TestResult/test$trial/traffic-$interval/metricAll/ ]
		then
			mkdir ./scratch/TestResult/test$trial/traffic-$interval/metricAll/
		fi
	
	file1="./scratch/TestResult/test$trial/traffic-$interval/"
	#echo "#numSta, Throughput(Kbps), ProbSucc(%), ProbLoss(%), avgDelay(Seconds)" > ./TestResult/test$trial/traffic-$interval/result-STAs-SF7.dat 
		
	touch ./scratch/TestResult/test$trial/traffic-$interval/mac-STAs-GW-$gwRing.txt
	file2="./scratch/TestResult/test$trial/traffic-$interval/mac-STAs-GW-$gwRing.txt"


	for numSta in {100..1500..100}
	do
			echo "trial:$trial-numSTA:$numSta #"

			if [ ! -d scratch/TestResult/test$trial/traffic-$interval/pcap-sta-$numSta/ ]
			then
				mkdir scratch/TestResult/test$trial/traffic-$interval/pcap-sta-$numSta/
			fi

			

			touch scratch/TestResult/test$trial/time-record$numSta.txt

			echo "Time: $(date) $interval $numSta" >> scratch/TestResult/test$trial/time-record$numSta.txt

		for numSeed in {1..5}
		do
			echo -ne "$numSeed \r"
 			./ns3 run "lorawan-test-rtx --nSeed=$RANDOM --nDevicesTotally=$numSta --pDevicesAlarm=$pDevicesAlarm --nGateways=$gwRing --radius=$rad --gatewayRadius=$gwRad --simulationTime=$simTime --appPeriod=$interval --fileMetric=$file1 --fileData=$file2 --print=$pEDs --flagRtx=$flagRtx --trial=$trial" > ./scratch/TestResult/test$trial/traffic-$interval/pcap-sta-$numSta/record-$numSta.txt 2>&1
		done
	done
elif [ $1 -eq 2 ]
then

	file1="./TestResult/test$trial/traffic-$interval/result-regSTAs"
	#echo "#numSta, Throughput(Kbps), ProbSucc(%), ProbLoss(%), avgDelay(Seconds)" > ./TestResult/test$trial/traffic-$interval/result-regSTAs.dat 
	
	touch ./TestResult/test$trial/traffic-$interval/mac-almSTAs-GW-$gwRing.txt
	file4="./TestResult/test$trial/traffic-$interval/mac-almSTAs-GW-$gwRing.txt"

	for numSta in {100..4000..100}
	do
			echo "trial:$trial-numSTA:$numSta"

			if [ ! -d TestResult/test$trial/traffic-$interval/pcap-sta-$numSta/ ]
			then
				mkdir TestResult/test$trial/traffic-$interval/pcap-sta-$numSta/
			fi

			touch TestResult/test$trial/time-record$numSta.txt

			echo "Time: $(date) $interval $numSta" >> TestResult/test$trial/time-record$numSta.txt

		for numSeed in {1..5}
		do
			echo -ne "$numSeed \r"
  			./ns3 run "lorawan-network-wAlm-sim --nSeed=$RANDOM --nDevices=$numSta --nGateways=$gwRing --radius=$rad --gatewayRadius=$gwRad --simulationTime=$simTime --appPeriod=$interval --file1=$file1 --file2=$file2 --printEDs=$pEDs --trial=$trial"  > ./TestResult/test$trial/traffic-$interval/pcap-sta-$numSta/record-$numSta.txt 2>&1
		done
	done
else

	touch ./TestResult/test$trial/traffic-$interval/result-STAs-SF7.dat
	file1="./TestResult/test$trial/traffic-$interval/result-STAs-SF7.dat"
	#echo "#numSta, Throughput(Kbps), ProbSucc(%), ProbLoss(%), avgDelay(Seconds)" > ./TestResult/test$trial/traffic-$interval/result-STAs-SF7.dat 
		
	touch ./TestResult/test$trial/traffic-$interval/result-STAs-SF8.dat
	file2="./TestResult/test$trial/traffic-$interval/result-STAs-SF8.dat"
	#echo "#numSta, Throughput(Kbps), ProbSucc(%), ProbLoss(%), avgDelay(Seconds)" > ./TestResult/test$trial/traffic-$interval/result-STAs-SF8.dat 
	
	touch ./TestResult/test$trial/traffic-$interval/result-STAs-SF9.dat
	file3="./TestResult/test$trial/traffic-$interval/result-STAs-SF9.dat"
	#echo "#numSta, Throughput(Kbps), ProbSucc(%), ProbLoss(%), avgDelay(Seconds)" > ./TestResult/test$trial/traffic-$interval/result-STAs-SF9.dat 	

	touch ./TestResult/test$trial/traffic-$interval/mac-STAs-GW-$gwRing.txt
	file4="./TestResult/test$trial/traffic-$interval/mac-STAs-GW-$gwRing.txt"


	for numSta in {100..4000..100}
	do
			echo "trial:$trial-numSTA:$numSta #"

			if [ ! -d TestResult/test$trial/traffic-$interval/pcap-sta-$numSta/ ]
			then
				mkdir TestResult/test$trial/traffic-$interval/pcap-sta-$numSta/
			fi

			touch TestResult/test$trial/time-record$numSta.txt

			echo "Time: $(date) $interval $numSta" >> TestResult/test$trial/time-record$numSta.txt

		for numSeed in {1..5}
		do
			echo -ne "$numSeed \r"
  			./ns3 run "lorawan-network-wAlm-mClass-sim --nSeed=$RANDOM --nDevices=$numSta --nGateways=$gwRing --radius=$rad --gatewayRadius=$gwRad --simulationTime=$simTime --appPeriod=$interval --file1=$file1 --file2=$file2 --file3=$file3 --file4=$file4 --printEDs=$pEDs --trial=$trial" > ./TestResult/test$trial/traffic-$interval/pcap-sta-$numSta/record-$numSta.txt 2>&1
		done
	done


fi
echo "##### Simulation finish #####"
#echo "seinding email..."
#echo simulation finish | mail -s Simulator helderhdw@gmail.com


