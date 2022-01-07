# set -x
DOCKER_USER=$(id -u ${USER})
DOCKER_USER_GROUP=$(id -g ${USER})
TIEV_CONTAINER="tiev"

xhost +local:root 1>/dev/null 2>&1

docker exec \
    -u "${DOCKER_USER}:${DOCKER_USER_GROUP}" \
    -it "${TIEV_CONTAINER}" \
    /bin/bash

set +x

xhost -local:root 1>/dev/null 2>&1