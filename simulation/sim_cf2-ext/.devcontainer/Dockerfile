FROM osrf/ros:humble-desktop-full
ENV USERNAME=user
ENV USER_UID=1000
ENV USER_GID=$USER_UID

ENV DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]
WORKDIR /home/$USERNAME/dev_ws

# Create the user and add sudo support
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt update \
    && apt install -y sudo udev curl usbutils nano graphviz git \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

#### ROS2 Setup
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
RUN apt-get install -y python3-colcon-common-extensions
RUN pip3 install transforms3d
RUN apt-get install -y ros-humble-ros-gz ros-humble-gazebo-* ros-humble-rviz2 ros-humble-rosbridge-*




WORKDIR /home/$USERNAME/dev_ws/libs




#### Crazyflie firmware with modifications for SITL 
# see https://github.com/CrazyflieTHI/crazyflie-firmware

RUN apt install -y make build-essential libncurses5-dev
# Get the crazyflie-firmware
# Check network connectivity first
RUN curl -I https://github.com 
RUN git clone https://github.com/CrazyflieTHI/crazyflie-firmware.git && \
    cd crazyflie-firmware && \
    git submodule update --init --recursive --depth 1

#### GSL
# Copy remote GSL source into the image
#ADD https://ftp.gnu.org/gnu/gsl/gsl-2.8.tar.gz /home/$USERNAME/dev_ws/libs/gsl-2.8.tar.gz
ADD https://ftp.uni-hannover.de/gnu/gsl/gsl-2.8.tar.gz /home/$USERNAME/dev_ws/libs/gsl-2.8.tar.gz
RUN mkdir /home/$USERNAME/dev_ws/libs/gsl-2.8 \
    && tar -xzf gsl-2.8.tar.gz -C /home/$USERNAME/dev_ws/libs/gsl-2.8 --strip-components=1 \
    && rm gsl-2.8.tar.gz
# Copy local GSL source directory into the image
# COPY gsl-2.8 /home/$USERNAME/dev_ws/libs/gsl-2.8
RUN cd /home/$USERNAME/dev_ws/libs/gsl-2.8 && \
    ./configure && \
    make -j && \
    sudo make install && \
    sudo ldconfig
# Check if GSL is installed:
# ldconfig -p | grep gsl

#### Crazyflie Firmware with SITL
RUN mkdir -p /home/$USERNAME/dev_ws/libs/crazyflie-firmware/build
COPY crazyflie-firmware/build/.config /home/$USERNAME/dev_ws/libs/crazyflie-firmware/build
RUN cd crazyflie-firmware && \
    make oldconfig && \
    make -j

#### cflib with Crazyflie CRTP sim driver
RUN pip3 install posix-ipc
RUN pip3 install --upgrade pip setuptools wheel
# (!) make sure to run ./sync-upstream.sh first (!)
COPY crazyflie-lib-python /home/${USERNAME}/dev_ws/libs/crazyflie-lib-python
RUN cd /home/${USERNAME}/dev_ws/libs/crazyflie-lib-python && \
    pip install -e .




WORKDIR /home/$USERNAME/dev_ws/ros2




#### ROS2 Gazebo Simulation for Crazyflie
# https://github.com/CrazyflieTHI/sim_cf2
RUN mkdir -p src
RUN apt install -y cmake build-essential genromfs ninja-build
RUN apt install -y protobuf-compiler libgoogle-glog-dev libeigen3-dev libxml2-utils
RUN apt install -y ros-humble-rmw-cyclonedds-cpp
RUN apt install -y ros-humble-xacro
RUN cd src && \
    git clone https://github.com/CrazyflieTHI/sim_cf2.git
COPY sim_cf2/launch /home/$USERNAME/dev_ws/ros2/src/sim_cf2/launch
COPY sim_cf2/worlds /home/$USERNAME/dev_ws/ros2/src/sim_cf2/worlds
COPY sim_cf2/models /home/$USERNAME/dev_ws/ros2/src/sim_cf2/models
RUN cd src && \
    git clone https://github.com/CrazyflieTHI/mav_comm.git
RUN source /opt/ros/humble/setup.bash && \
    colcon build




WORKDIR /home/$USERNAME/dev_ws/libs




#### Crazyflie Controller
RUN apt-get install -y \
    python3-virtualenv python3-venv python3-pip python3-pyqt5 libxcb-xinerama0 libxcb-cursor0
COPY ./cf.PyControl/src /home/$USERNAME/dev_ws/libs/crazyflie-controller/src
COPY ./cf.PyControl/bin /home/$USERNAME/dev_ws/libs/crazyflie-controller/bin
COPY ./cf.PyControl/.devcontainer/requirements_nocf.txt /home/$USERNAME/dev_ws/libs/crazyflie-controller/requirements.txt
RUN chmod +x /home/$USERNAME/dev_ws/libs/crazyflie-controller/bin/cf-ctrl.sh
RUN cd /home/$USERNAME/dev_ws/libs/crazyflie-controller && \
    pip3 install -r requirements.txt




WORKDIR /home/$USERNAME/dev_ws




#### Entrypoint
COPY .devcontainer/entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]