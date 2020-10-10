#! /bin/bash
RUNPATH='../build/aarch64/bin'
MODELDIR='../net/pedestrianModel'

$RUNPATH/detectnet-pedestrian 0 dog_out1.jpg \
	--prototxt=$MODELDIR/deploy.prototxt \
	--model=$MODELDIR/snapshot_iter_1710000.caffemodel
