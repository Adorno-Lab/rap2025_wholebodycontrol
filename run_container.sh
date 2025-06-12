#!/bin/bash
echo "Name:" $1
# Argument validation check
if [ $# -eq 0 ]; then
    echo "Error: No arguments provided."
    echo "Example: sh run_container.sh ubuntu_24_dqrobotics"
    exit 1
fi
xhost +local:root
docker run -it --name=$1 \
--rm --privileged --network=host --env=DISPLAY \
$1 /bin/bash