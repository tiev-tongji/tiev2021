#!/bin/bash

# install some env use sudo apt
sudo apt install vim -y
sudo apt install git -y
sudo apt install g++ -y
sudo apt install cmake -y

# generate ssh rsa pub key
ssh-keygen -t rsa