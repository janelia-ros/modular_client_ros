# modular_client_ros

Authors:

    Peter Polidoro <polidorop@janelia.hhmi.org>

License:

    BSD

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
sudo apt-get install virtualenv
mkdir ~/virtualenvs
cd ~/virtualenvs
virtualenv modular_client
source ~/virtualenvs/modular_client/bin/activate
pip install modular_client
```
