#! /bin/bash
RUNPATH='../build/aarch64/bin'
MODELDIR='../net/warningModel'

$RUNPATH/detectnet-warning 0 dog_out1.jpg \
	--prototxt=$MODELDIR/deploy.prototxt \
	--model=$MODELDIR/snapshot_iter_39200.caffemodel
