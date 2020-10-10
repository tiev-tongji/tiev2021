#! /bin/bash
RUNPATH='../build/aarch64/bin'
MODELDIR='../net/TrafficLightModel'

$RUNPATH/detectnet-trafficlight 1 dog_out1.jpg \
	--prototxt=$MODELDIR/deploy.prototxt \
	--model=$MODELDIR/snapshot_iter_57000.caffemodel
