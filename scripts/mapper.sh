#!/bin/bash
cd ../src/modules/IMU_Mapper_linux_zcm
if [ ! -d "build" ]
then
	mkdir build
fi
cd build
qmake .. && make -j8
if [ $? != 0 ]
then
	exit 2
fi
./IMU_Mapper_linux_zcm
