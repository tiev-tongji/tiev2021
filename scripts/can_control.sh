#!/bin/bash
cd ../src/modules/ControlCenterZLG
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
sudo ./build/CONTROL
