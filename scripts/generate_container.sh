#!/bin/bash
DOCKER_GATEWAY=`ip route |grep 'docker0' | awk '{print $9}'`
docker rm -f planner_docker
# docker run -it -v /home/autolab/tiev_docker/:/home/autolab/tiev2020-code/ --name finally22111 -h lane_parking -e DISPLAY=${DOCKER_GATEWAY}:10.0 --mount type=bind,src="/tmp/.docker.Xauthority",dst="/root/.docker.Xauthority",ro 53b9e6ed67d4 /bin/bash 
docker run -it  -v /home/autolab/tiev2021/:/home/autolab/tiev2021/  --privileged --net=host --ipc=host --name planner_docker -h planner_docker -e DISPLAY=${DOCKER_GATEWAY}:11.0 -v /tmp:/tmp -v /usr/lib:/usr/lib,ro a8cadd2c1761 /bin/bash && cd /home/autolab/tiev2021

#sudo nvidia-docker run -it --privileged --net=host --ipc=host --name lane_parking -h lane_parking -e DISPLAY=${DOCKER_GATEWAY}:10.0 --mount type=bind,src="/tmp/.docker.Xauthority",dst="/root/.docker.Xauthority",ro -v /tmp:/tmp -v /home/tiev-plus/Desktop/tiev-plus-code:/tiev-plus tiev-plus/vision:lane_parking /bin/bash 
