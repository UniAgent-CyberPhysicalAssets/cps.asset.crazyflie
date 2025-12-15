FROM osrf/ros:humble-desktop-full
ENV ROS_DISTRO=humble
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




WORKDIR /home/$USERNAME/dev_ws/libs




#### CF2 Controller
# RUN pip3 install cflib
# RUN pip3 install cfclient==2024.11
# cfclient Setup
# see also: https://github.com/bitcraze/crazyflie-clients-python/issues/518
RUN apt-get install -y \
    python3-virtualenv python3-venv python3-pip python3-pyqt5 libxcb-xinerama0 libxcb-cursor0
COPY ./src /home/$USERNAME/dev_ws/libs/crazyflie-controller/src
COPY ./bin /home/$USERNAME/dev_ws/libs/crazyflie-controller/bin
COPY requirements.txt /home/$USERNAME/dev_ws/libs/crazyflie-controller
RUN chmod +x /home/$USERNAME/dev_ws/libs/crazyflie-controller/bin/cfpyctrl.sh
RUN cd /home/$USERNAME/dev_ws/libs/crazyflie-controller && \
    pip3 install -r requirements.txt

# ds-crazyflie support
RUN apt update && apt install -y \
    python3-colcon-common-extensions \
    ros-humble-rosidl-default-generators \
    ros-humble-rosidl-generator-py \
    ros-humble-std-msgs \
    ros-humble-geometry-msgs \
    ros-humble-rmw-cyclonedds-cpp
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
COPY ./vendor/crazyflie_interfaces \
    /home/$USERNAME/dev_ws/libs/crazyflie-controller/src/crazyflie_interfaces
COPY ./vendor/crazyflie_webots_gateway_interfaces \
     /home/$USERNAME/dev_ws/libs/crazyflie-controller/src/crazyflie_webots_gateway_interfaces


RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd /home/$USERNAME/dev_ws/libs/crazyflie-controller && \
    colcon build --packages-select crazyflie_interfaces crazyflie_webots_gateway_interfaces"
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc && \
    echo "source /home/$USERNAME/dev_ws/libs/crazyflie-controller/install/setup.bash" >> /etc/bash.bashrc




WORKDIR /home/$USERNAME/dev_ws/libs/crazyflie-controller/bin




#### Entrypoint
COPY .devcontainer/entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]