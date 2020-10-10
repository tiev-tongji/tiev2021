# This is zcm installation for TiEV
## Installization for Python2.7
pip instal zcm
## Compile, build and install for c++ and Python3
### 0 Download zcm
git clone https://github.com/zerocm/zcm.git
### 1 Install the dependences
sudo ./scripts/install-deps-tiev.sh
### 2 Set Environment Variables
export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib or
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
export ZCM_DEFAULT_URL=ipc
### 3.1 Build for c++
./waf configure --use-java --use-zmq --use-ipc --use-udpm 
./waf build
sudo ./waf install
### 3.2 Build for python3(take python3.5 for example)
### Notice: must secify python version and don't use anaconda
sudo ./waf configure --use-java --use-zmq --use-ipc --use-udpm --use-python --python=`which python3.5`
sudo ./waf build
sudo ./waf install
sudo mv /usr/local/lib/python3/dist-packages/zerocm.cpython-35m-x86_64-linux-gnu.so /usr/local/lib/python3.5/dist-packages/zerocm.cpython-35m-x86_64-linux-gnu.so
### 4 Configuration
./scripts/set-muilticast-local.sh
./scripts/set-multicast-buffers.sh
### 5 Problems
if install is failed, and complaining about lack the waf configure, the configure and build zcm in root mode (sudo) may be required.
Then the JAVA_HOME should be set in /etc/environment

