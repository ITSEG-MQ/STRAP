#!/bin/bash

experiment(){

  scenarioPath=$1
  recordfile=$2
  savefolder=$3
  modified=$4
  module=$5

#  if [ ! -d $savefolder"/"$module ]
#  then
#    mkdir $savefolder"/"$module
#  fi

  python channel_monitor.py -n $savefolder"/"$recordfile &
  PID=$!
  sleep 1
  # play record

  if [ $module == 'signal' ];
  then
    cyber_recorder play -f $scenarioPath"/"$recordfile -k /apollo/perception/traffic_light
  fi

  if [ $module == 'perception' ];
  then
    cyber_recorder play -f $scenarioPath"/"$recordfile -k /apollo/perception/obstacles /apollo/prediction
  fi

  if [ $module == 'prediction' ];
  then
    cyber_recorder play -f $scenarioPath"/"$recordfile -k /apollo/prediction
  fi

  if [ $module == 'planning' ];
  then
    cyber_recorder play -f $scenarioPath"/"$recordfile -k /apollo/planning
  fi


  sleep 1
  # Kill it
  kill -INT $PID

}

testSystem=$1
benchmarkNum=$2
mutatedFile=$3
replacePath=$4
segmentsFolder=$5

if [ ! -d "results/"$testSystem ]
then
  mkdir "results/"$testSystem
fi


#if [ ! -d "results/"$testSystem"/benchmark"$benchmarkNum ]
#then
#  mkdir "results/"$testSystem"/benchmark"$benchmarkNum
#fi

if [ $testSystem == "signal" ]
then
  moduleName="Traffic"
  blockChannels="/apollo/perception/traffic_light"

fi

if [ $testSystem == "obstacle" ]
then
  moduleName="Perception"
  blockChannels="/apollo/perception/obstacles /apollo/prediction"

fi

if [ $testSystem == "planning" ]
then
  moduleName="Planning"
  blockChannels="/apollo/perception/obstacles /apollo/prediction"

fi

if [ $testSystem == "prediction" ]
then
  moduleName="Prediction"
  blockChannels="/apollo/perception/obstacles /apollo/prediction"

fi


../scripts/bootstrap.sh

#maps=("shalun" "gomentum" "cubetown")
maps=("cubetown")
for map in "${maps[@]}"

do
  echo $map
  if [ ! -d "results/"$testSystem"/"$map ]
  then
    mkdir "results/"$testSystem"/"$map
  fi

  if [ ! -d "results/"$testSystem"/"$map"/benchmark"$benchmarkNum ]
  then
        mkdir "results/"$testSystem"/"$map"/benchmark"$benchmarkNum
  fi

  if [ ! -d "results/"$testSystem"/"$map"/benchmark"$benchmarkNum"/"$segmentsFolder ]
  then
    mkdir "results/"$testSystem"/"$map"/benchmark"$benchmarkNum"/"$segmentsFolder

  fi

  if [ ! -d "results/"$testSystem"/"$map"/benchmark"$benchmarkNum"/"$segmentsFolder"/ori" ]
  then
    mkdir "results/"$testSystem"/"$map"/benchmark"$benchmarkNum"/"$segmentsFolder"/ori"

  fi

  saveFolder="results/"$testSystem"/"$map"/benchmark"$benchmarkNum"/"$segmentsFolder"/ori"

  # replay segments and save recording
  # shellcheck disable=SC1068
  segments=($(ls "data/record_segments/$map/$testSystem/"$segmentsFolder))
  for segment in "${segments[@]}"
  do
    	python3 dw.py -m $moduleName -p $map
  	sleep 30
  	experiment data/record_segments/$map/$testSystem/$segmentsFolder $segment $saveFolder 1 $testSystem
  	python3 dw.py -m $moduleName -d
  	sleep 15
  done

done
