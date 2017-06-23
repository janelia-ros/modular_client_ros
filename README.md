# modular_client_ros

Authors:

    Peter Polidoro <polidorop@janelia.hhmi.org>

License:

    BSD

## Running

```shell
roscore
```

```shell
rosrun modular_client_ros audio_controller.py
```

```shell
rosservice list
```

```shell
rosservice call /audio_controller/audio_controller/play_tone 5000 ALL
```

```shell
rosservice call /audio_controller/audio_controller/stop
```

## Setup ROS Package to Interface to a Specific Modular Device

```shell
source ~/virtualenvs/modular_client/bin/activate
cd ~/catkin_ws/src/modular_client_ros
```

### Save Modular Device API

Plug modular device into USB port.

If only one USB port on the host computer is connected to a modular
device:

```shell
./scripts/save_device_api
```

Specify modular device port if more than one USB port on the host
computer is connected to a modular device:

```shell
./scripts/save_device_api -p /dev/ttyACM0
```

### Customize ROS Package from Modular Device API

## Installation

### Setup ROS Workspace

```shell
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
cd ~/catkin_ws/src
git clone https://github.com/janelia-ros/modular_client_ros.git
```

### Install Python Dependencies

```shell
sudo pip install ipython --upgrade
sudo pip install modular_client
sudo pip install jinja2
```
