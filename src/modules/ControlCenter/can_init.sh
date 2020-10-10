modprobe can
modprobe can_raw
modprobe mttcan
ip link set can0 type can bitrate 500000
ip link set up can0
ip link set can1 type can bitrate 500000
ip link set up can1
#candump can1 >> null &
#candump can0 >> null &
