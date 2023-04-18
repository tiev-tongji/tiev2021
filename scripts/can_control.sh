#!/bin/bash
cd ../src/modules/ControlCenterZLG
if [ ! -d "build" ]
then
	mkdir build
fi
cd build
if [ $1 == "rebuild" ]
then
    rm -rf *
    echo "rebuild"
fi
cmake .. && make -j8
if [ $? != 0 ]
then
	exit 2
fi
cd ..
./build/CONTROL
