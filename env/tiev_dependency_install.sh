#!/bin/bash

#install protobuffer-c++
sudo apt install autoconf automake libtool curl make g++ unzip
unzip protobuf-cpp-3.18.0.zip
cd protobuf-3.18.0
./configure
make
make check
sudo make install
sudo ldconfig

#install qt5
sudo apt install qt5-default


#pre install opencv
sudo apt install libgtk2.0-dev pkg-config
sudo apt install libcanberra-gtk-module
# install opencv-3.4.12 in opencv dir
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D CMAKE_INSTALL_PREFIX=/usr/local ..
make -j8
sudo make install
# then cd /etc/ld.so.conf.d
# sudo vim opencv.conf
# add
# '/usr/local/lib' in opencv.conf
# configure env variable
# sudo vim ~/.bashrc
# add 
# 'PKG_CONFIG_PATH=$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig'
# export PKG_CONFIG_PATH
# validating opencv has been installed run:
# 'pkg-config opencv --modversion'
# if you see '3.4.15' in terminal
# Congratulations! OpenCV now is installed successfully!

# pre install zcm
sudo apt install python3-dev
sudo apt install libelf
sudo apt install libzmq3-dev
sudo apt install openjdk-16-jdk openjdk-16-jre
# install zcm in zcm dir
./waf configure --use-java --use-python --use-zmq --use-elf --use-ipc --use-udpm --use-serial --use-can
./waf build
sudo ./waf install
# set the ZCM_DEFAULT_URL in ~/.bashrc
# export ZCM_DEFAULT_URL=udpm://239.255.76.67:7667?ttl=1
# run zcm-spy
# if you see the error:Exception in thread "main" java.lang.UnsatisfiedLinkError: no zcmjni in java.library.path
#        at java.lang.ClassLoader.loadLibrary(ClassLoader.java:1867)
#        at java.lang.Runtime.loadLibrary0(Runtime.java:870)
#        at java.lang.System.loadLibrary(System.java:1122)
#        at zcm.zcm.ZCMJNI.<clinit>(ZCMJNI.java:7)
#        at zcm.zcm.ZCM.<init>(ZCM.java:37)
#        at zcm.spy.Spy.<init>(Spy.java:77)
#        at zcm.spy.Spy.main(Spy.java:481)
# please add env variable in ~/.bashrc
# export LD_LIBRARY_PATH=/usr/lib:/usr/local/lib

# install eigen3
sudo apt install libeigen3-dev

# install grpc++
sudo apt intall libgrpc++-dev

# install osqp in osqp dir
mkdir build & cd build
cmake .. & make -j8
sudo make install

# install boost-math
sudo apt install libboost-math-dev
