#!/bin/bash
cd ~/tiev2020-code/src/modules/planner
if [ ! -d "build" ]
then
	mkdir build
fi
cd build
cmake .. && make -j8
./src/visualization/visualization
