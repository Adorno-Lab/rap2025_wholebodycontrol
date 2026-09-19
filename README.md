![GitHub License](https://img.shields.io/github/license/Adorno-Lab/sas_robot_driver_unitree_z1)![Static Badge](https://img.shields.io/badge/ROS2-Jazzy-blue)![Static Badge](https://img.shields.io/badge/powered_by-DQ_Robotics-red)![Static Badge](https://img.shields.io/badge/SmartArmStack-green)![Static Badge](https://img.shields.io/badge/Ubuntu-24.04_LTS-orange)

# rap2025_wholebodycontrol

## Docker installation
```shell
sudo apt install docker.io docker-compose-v2
sudo usermod -aG docker $USER
newgrp docker
```
The command `usermod` modifies the user account, with the flag `-a` being used to append a new group without removing existing ones, and `-G docker` being used add $USER to the docker supplementary group.

The command `newgrp docker` starts a new shell with docker as the current group. Changes take effect immediately and logging out and in is unnecessary.

## Clone repository
```shell
mkdir -p ~/git/ && cd /git/
git clone https://github.com/Adorno-Lab/rap2025_wholebodycontrol.git --recursive
```


## Run simulation

```shell
cd ~/git/rap2025_wholebodycontrol/docker/demo_b1z1_simulation
xhost +local:root
docker compose up --build
```

## Real platform

Custom network configuration used for this repository

| PC | Description | IP |
| ------------- | ------------- |------------- |
| Desktop  | CoppeliaSim scene  | 192.168.8.100 |
| B1Z1-white  | kinematic control  | 192.168.8.170 |
| B1Z1-black  | kinematic control  | 192.168.8.226 |


## Build and save the Docker image

```shell
cd ~/git/rap2025_wholebodycontrol/
docker build --no-cache -f docker/sas_unitree_b1z1_rap_control_template/Dockerfile -t sas_unitree_b1z1_rap_control_template .
```

```shell
docker save -o sas_unitree_b1z1_rap_control_template.tar.gz sas_unitree_b1z1_rap_control_template:latest
```

## Send the image to the B1 computer 

For this case, we send the image to the  B1Z1-white unit (192.168.8.170)
```shell
scp -r sas_unitree_b1z1_rap_control_template.tar.gz unitree@192.168.8.170:/home/unitree/
```

## From the B1 computer, load the Docker image

Enter the B1 computer
```shell
ssh unitree@192.168.8.170
```
Load the image
```shell
docker load --input sas_unitree_b1z1_rap_control_template.tar.gz
```

## On your Desktop computer

```shell
cd ~/git/rap2025_wholebodycontrol/docker/demo_b1z1/
xhost +local:root
docker compose up --build
```

## On your B1 computer

Run the image
```shell
docker run -it --name="sas_unitree_b1z1_rap_control_template" --rm --privileged --network=host sas_unitree_b1z1_rap_control_template /bin/bash
```


