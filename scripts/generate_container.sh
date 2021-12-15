#!/bin/bash
DOCKER_GATEWAY=`ip route |grep 'docker0' | awk '{print $9}'`
cat /dev/null > /tmp/.docker.Xauthority
xauth -f /tmp/.docker.Xauthority add ${DOCKER_GATEWAY}:10 . `xauth list $DISPLAY | awk '{print $3}' | head -n 1`
socat -d -d tcp-listen:6010,bind=${DOCKER_GATEWAY},reuseaddr,fork unix:/tmp/.X11-unix/X1 &

sudo docker rm -f planner_docker 2>/dev/null
sudo nvidia-docker run -itd --privileged --net=host --ipc=host --name planner_docker -h planner_docker -e DISPLAY=${DOCKER_GATEWAY}:10.0 --mount type=bind,src="/tmp/.docker.Xauthority",dst="/root/.docker.Xauthority",ro -v /tmp:/tmp -v /home/autolab/tiev2021:/home/autolab/tiev2021 a8cadd2c1761 /bin/bash
