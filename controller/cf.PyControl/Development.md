# Local Workspace Setup: cf.PyControl

This document describes how to setup the development environment for cf.PyControl

## System Prerequisites

Tested with:

- Ubuntu 24.04 (6.8.0-39-generic)
- Python 3.12.3
- Crazyflie 2.x

### Install Required Packages

```shell
sudo apt install git graphviz
sudo apt install python3-pip python3-virtualenv libxcb-xinerama0 libxcb-cursor0
```

### USB Permissions

To use USB Radio and Crazyflie 2 over USB without being root:

```shell
sudo groupadd plugdev
sudo usermod -a -G plugdev $USER

cat <<EOF | sudo tee /etc/udev/rules.d/99-bitcraze.rules > /dev/null
# Crazyradio (normal operation)
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="7777", MODE="0664", GROUP="plugdev"
# Bootloader
SUBSYSTEM=="usb", ATTRS{idVendor}=="1915", ATTRS{idProduct}=="0101", MODE="0664", GROUP="plugdev"
# Crazyflie (over USB)
SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="5740", MODE="0664", GROUP="plugdev"
EOF

sudo udevadm control --reload-rules
sudo udevadm trigger
```

- See also: [USB permissions ](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/)


## Workspace Setup

### Create Virtual Environment

Create and activate the virtual environment:

```shell
virtualenv venv 
# or: python3 -m venv venv
source venv/bin/activate
```

### Install Packages

> [!IMPORTANT]
> The first option is the recommended way of installing the Crazyflie libraries.
> Manual installation is just for demonstration purposes.

**Install Packages from requirements.txt:**

```shell
pip3 install -r requirements.txt
```

**Manual Installation:**

To use the latest release of the cfclient and cflib, you can directly clone both repositories and install them in the virtual environment:

```shell
git clone https://github.com/bitcraze/crazyflie-lib-python.git
git clone https://github.com/bitcraze/crazyflie-clients-python

cd crazyflie-lib-python
pip3 install -e .
cd ..
cd crazyflie-clients-python
pip install -e .
```

You have to install the required python libraries yourself. See `requirements.txt` for the respective versions.



## Getting Started

- Connect the Crazyradio 2.0 to a USB port
- Switch on the Crazyflie 2.X drone
- Start the Crazyflie Client: 
  - `cfclient`
- See [README.md](README.md) on how to use the Crazyflie 2.x REST API to control the Crazyflie drone

