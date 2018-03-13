# modular_client_ros

Authors:

    Peter Polidoro <polidorop@janelia.hhmi.org>

License:

    BSD

## Running

```shell
roslaunch modular_client_ros modular_client.launch
```

```shell
rosservice list
rosservice call /modular_client/<MODULAR_DEVICE_NAME>/modular_server/get_device_id
```

## Setup ROS Package to Interface to a Specific Modular Device

### Cleanup Previous Modular Device API

```shell
cd ~/catkin_ws/src/modular_client_ros
git clean -xdf
```

### Save Modular Device API

Plug modular device into USB port.

If only one USB port on the host computer is connected to a modular
device:

```shell
cd ~/catkin_ws/src/modular_client_ros
./scripts/save_device_api
```

Specify modular device port if more than one USB port on the host
computer is connected to a modular device:

```shell
cd ~/catkin_ws/src/modular_client_ros
./scripts/save_device_api -p /dev/ttyACM0
```

### Customize ROS Package from Modular Device API

```shell
cd ~/catkin_ws/src/modular_client_ros
./scripts/setup_package_using_api
```

### Compile ROS Package After Setup

```shell
cd ~/catkin_ws/
catkin_make
source ./devel/setup.bash

```

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

### Updating Python Dependencies

#### Python 3

```shell
sudo -H python3 -m pip install modular_client --upgrade
```

#### Python 2

```shell
sudo -H python -m pip install modular_client --upgrade
```
