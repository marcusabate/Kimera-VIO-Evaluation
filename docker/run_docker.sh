#!/bin/bash

# Exit if any cmd fails
set -e

SCRIPT_CONTEXT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd -P)"
OUTPUT_LOGS_CONTEXT=/home/marcus/kimera_ws
DATASET_CONTEXT=/data/datasets

CONTAINER_NAME=kimera-evaluation

# Make sure to clone Kimera-VIO next to ford-Kimera-VIO-ROS so that params are easily updated.
# Also, apply the patch:
# cd Kimera-VIO && git apply ../ford-Kimera-VIO-ROS/install/add_dbow_and_opengv_headers_kimera_vio.patch
# It will then be mounted into the docker container so you can change params as needed.

xhost +local:docker
docker run \
    -it \
    --rm \
    --user=$USER \
    --name=$CONTAINER_NAME \
    --net=host \
    -v $SCRIPT_CONTEXT/..:/home/$USER/Kimera-Evaluation \
    -v $OUTPUT_LOGS_CONTEXT:$OUTPUT_LOGS_CONTEXT \
    -v /home/marcus/ford_ws:/home/marcus/ford_ws \
    -v /home/marcus/code/VINS-Fusion:/home/marcus/code/VINS-Fusion \
    -v /home/marcus/papers:/home/marcus/papers \
    -v $DATASET_CONTEXT:$DATASET_CONTEXT \
    kimera-evaluation:master \
    /bin/bash
xhost -local:docker
