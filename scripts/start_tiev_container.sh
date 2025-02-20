#!/bin/bash
# set -u
# set -k
# CURR_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
# TIEV_ROOT_DIR="${CURR_DIR}"
TIEV_ROOT_DIR="/home/autolab/tiev2021"
TIEV_IMAGE="tiev_planner:v1"
TIEV_CONTAINER="tiev_planner"
INSIDE_CONTAINER="tiev_planner"
SHM_SIZE="8G"

function remove_container_if_exists() {
    local container="$1"
    if docker ps -a --format '{{.Names}}' | grep -q "${container}"; then
        echo "Removing existing TiEV container: ${container}"
        docker stop "${container}" >/dev/null
        docker rm -v -f "${container}" 2>/dev/null
    fi
    rm -rf /tmp/${USER}/${TIEV_CONTAINER}
    mkdir -p /tmp/${USER}/${TIEV_CONTAINER}
}

local_volumes="-v /tmp/${USER}/${TIEV_CONTAINER}:/home/${USER}"
local_volumes="${local_volumes} -v $TIEV_ROOT_DIR:/home/${USER}/tiev"
local_volumes="${local_volumes} -v /dev:/dev"
local_volumes="${local_volumes} -v /media:/media \
                    -v /tmp/${USER}/${TIEV_CONTAINER}/.X11-unix:/tmp/.X11-unix:rw \
                    -v /tmp/:/tmp/ \
                    -v /etc/localtime:/etc/localtime:ro \
                    -v /etc/passwd:/etc/passwd:ro \
                    -v /etc/group:/etc/group:ro \
                    -v /etc/shadow:/etc/shadow:ro \
                    -v /usr/src:/usr/src \
                    -v /usr/share/fonts:/usr/share/fonts \
                    -v /usr/share/fonts:/usr/local/lib/fonts \
                    -v /lib/modules:/lib/modules"

remove_container_if_exists ${TIEV_CONTAINER}
local_host="$(hostname)"
display="${DISPLAY:-:0}"
user="${USER}"
uid="$(id -u)"
group="$(id -g -n)"
gid="$(id -g)"

# set -x
nvidia-docker run -itd \
        --privileged \
        --name "${TIEV_CONTAINER}" \
        -e DISPLAY="${display}" \
        -e DOCKER_USER="${user}" \
        -e USER="${user}" \
        -e DOCKER_USER_ID="${uid}" \
        -e DOCKER_GRP="${group}" \
        -e DOCKER_GRP_ID="${gid}" \
        -e DOCKER_IMG="${TIEV_IMAGE}" \
        ${local_volumes} \
        --net host \
        --ipc host \
        --gpus all \
        -w /home/${USER}/tiev/scripts \
        --add-host "${INSIDE_CONTAINER}:127.0.0.1" \
        --add-host "${local_host}:127.0.0.1" \
        --hostname "${INSIDE_CONTAINER}" \
        --shm-size "${SHM_SIZE}" \
        --pid=host \
        -v /dev/null:/dev/null \
        "${TIEV_IMAGE}" \
        /bin/bash

set +x
