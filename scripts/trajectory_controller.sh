#!/bin/bash
cd ~/tiev2020-code/src/modules/Trajectory_Controller
if [ ! -d "build" ]
then
	mkdir build
fi
cd build
cmake .. && make -j8
./trajectory_controller
