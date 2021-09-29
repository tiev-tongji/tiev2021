#!/bin/bash
cd ~/tiev2020-code/src/modules/ControlCenterZLG_sim
if [ ! -d "build" ]
then
	mkdir build
fi
cd build
cmake .. && make -j8
if [ $? != 0 ]
then
	exit 2
fi
cd ..
./build/CONTROL | grep ANGLE