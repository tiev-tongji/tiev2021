#!/bin/bash

# echo "export ZCM_DEFAULT_URL=udpm://239.255.76.67:7667?ttl=1" >> ~/.bashrc
# echo "export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/usr/local/lib:/usr/lib/" >> ~/.bashrc
# source ~/.bashrc

cd /home/autolab/tiev2021/src/modules/planner/third_party/G1fitting/ && make && make install

cd /home/autolab/tiev2021/src/modules/planner/third_party/steering_functions
mkdir build
cd build/ && rm -rf * && cmake .. && make -j8 && make install

