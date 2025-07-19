FROM osrf/ros:jazzy-desktop-full
ENV USERNAME=user
ENV USER_UID=1000
ENV ALT_USER_UID=1001
ARG USER_GID=$USER_UID

SHELL ["/bin/bash", "-c"]
WORKDIR /home/$USERNAME/dev_ws

RUN TARGET_ID=$USER_GID; \
    if getent group $USER_GID > /dev/null; then \
        echo "GID $USER_GID exists. Using alternative GID $ALT_USER_UID"; \
        TARGET_ID=$ALT_USER_UID; \
    fi \
    && groupadd --gid $TARGET_ID $USERNAME \
    && useradd --uid $TARGET_ID --gid $TARGET_ID -m $USERNAME \
    && apt update \
    && apt install -y sudo udev curl usbutils nano lsb-release gnupg \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME


#### ROS2 Setup
# https://www.bitcraze.io/2024/09/crazyflies-adventures-with-ros-2-and-gazebo/
RUN apt-get install -y python3-pip python3-venv python3-virtualenv python3-setuptools python3-colcon-common-extensions libboost-program-options-dev libusb-1.0-0-dev
RUN apt-get install -y ros-jazzy-teleop-twist-keyboard
RUN apt-get install -y ros-jazzy-rmw-cyclonedds-cpp
RUN apt-get install -y ros-jazzy-slam-toolbox
# https://imrclab.github.io/crazyswarm2/installation.html
RUN pip install --break-system-packages rowan nicegui 
RUN apt-get install -y ros-jazzy-motion-capture-tracking
RUN pip install --break-system-packages cflib transforms3d
RUN apt-get install -y ros-jazzy-tf-transformations

#### Install gz-harmonic
# https://gazebosim.org/docs/harmonic/install_ubuntu/
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update \
    && apt-get install -y gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

#### ROS2 Gazebo Simulation for the Crazyflie
RUN mkdir -p /home/$USERNAME/dev_ws/simulation
RUN mkdir -p /home/$USERNAME/dev_ws/ros2/src
RUN mkdir -p /home/$USERNAME/dev_ws/crazyflie-firmware

WORKDIR /home/$USERNAME/dev_ws/simulation
RUN git clone https://github.com/bitcraze/crazyflie-simulation.git

WORKDIR /home/$USERNAME/dev_ws/ros2/src
RUN git clone https://github.com/knmcguire/crazyflie_ros2_multiranger.git
RUN git clone https://github.com/knmcguire/ros_gz_crazyflie
RUN git clone https://github.com/IMRCLab/crazyswarm2 --recursive

##### Add build_scripts executable configuration to setup.cfg files
RUN sed -i '/^\[build_scripts\]/!b;n;c\executable = /usr/bin/env python3' /home/$USERNAME/dev_ws/ros2/src/crazyflie_ros2_multiranger/crazyflie_ros2_multiranger_simple_mapper/setup.cfg || echo -e '\n[build_scripts]\nexecutable = /usr/bin/env python3' >> /home/$USERNAME/dev_ws/ros2/src/crazyflie_ros2_multiranger/crazyflie_ros2_multiranger_simple_mapper/setup.cfg \
    && sed -i '/^\[build_scripts\]/!b;n;c\executable = /usr/bin/env python3' /home/$USERNAME/dev_ws/ros2/src/crazyflie_ros2_multiranger/crazyflie_ros2_multiranger_wall_following/setup.cfg || echo -e '\n[build_scripts]\nexecutable = /usr/bin/env python3' >> /home/$USERNAME/dev_ws/ros2/src/crazyflie_ros2_multiranger/crazyflie_ros2_multiranger_wall_following/setup.cfg

##### Add customization
COPY ./dev_ws /home/$USERNAME/dev_ws

RUN source /opt/ros/jazzy/setup.bash && \
    colcon build --cmake-args -DBUILD_TESTING=ON

#### Set up software-in-the-loop simulation
WORKDIR /home/$USERNAME/dev_ws/
COPY libs/swig-4.1.1.tar.gz ./
RUN tar -xzf swig-4.1.1.tar.gz \
    && cd swig-4.1.1 \
    && ./configure
RUN cd swig-4.1.1 \
    && make
# RUN cd swig-4.1.1 \
#     && make check 
RUN cd swig-4.1.1 \
    && make install
RUN rm -rf swig-4.1.1 swig-4.1.1.tar.gz
RUN swig -version

RUN git clone --recursive https://github.com/bitcraze/crazyflie-firmware.git
RUN cd crazyflie-firmware && \
    make cf2_defconfig && \
    make bindings_python && \
    cd build && \
    python3 setup.py install --user
RUN echo "export PYTHONPATH=/home/user/dev_ws/crazyflie-firmware/build:\$PYTHONPATH" >> /etc/bash.bashrc

##### Install Python Dependencies
WORKDIR /home/$USERNAME/dev_ws/
COPY dev_ws/requirements.txt ./
RUN pip install --break-system-packages -r requirements.txt

#### Entrypoint
COPY .devcontainer/entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["/bin/bash"]