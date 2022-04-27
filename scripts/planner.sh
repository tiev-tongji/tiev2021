#!/bin/bash
cd ../src/modules/planner || fail
if [ ! -d "build" ]
then
	mkdir build
fi
cd build || fail
cmake .. && make -j8
if [ $? != 0 ]
then
	exit 2
fi
./src/planner
