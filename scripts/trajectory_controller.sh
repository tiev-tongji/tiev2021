#!/bin/bash
cd ~/tiev2021/src/modules/Trajectory_Controller
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
./trajectory_controller
