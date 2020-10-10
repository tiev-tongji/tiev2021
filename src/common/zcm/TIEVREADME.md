# This is zcm installation for TiEV
## Installization for Python
pip instal zcm
## Compile, build and install
### 0 Download zcm
git clone https://github.com/zerocm/zcm.git
### 1 Install the dependences
sudo ./scripts/install-deps-tiev.sh
java
### 2 Set Environment Variables
export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib or
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib
export ZCM_DEFAULT_URL=ipc
### 3 Build
./waf configure --use-java --use-zmq --use-ipc --use-udpm
./waf build
sudo ./waf install
### 4 Configuration
./scripts/set-muilticast-local.sh
./scripts/set-multicast-buffers.sh
### 5 Problems
if install is failed, and complaining about lack the waf configure, the configure and build zcm in root mode (sudo) may be required.
Then the JAVA_HOME should be set in /etc/environment

