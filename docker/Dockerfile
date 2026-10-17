# sas_robot_driver_mujoco on ROS 2 Jazzy, with cpp-interface-mujoco (MuJoCo 3.16) and the models of
# cpp-interface-mujoco-robots (in /root/git/cpp-interface-mujoco-robots/models).
#
# Build it from the root of this repository (or with docker/compose.yaml):
#   docker build -f docker/Dockerfile -t sas_robot_driver_mujoco_jazzy .
FROM murilomarinho/sas:jazzy
SHELL ["/bin/bash", "-c"]
ENV BASH_ENV="/etc/bash_env"

########################################################################################################
# Dependencies of cpp-interface-mujoco: the build tools, GLFW and OpenGL (the window), EGL and Mesa (offscreen
# rendering of the cameras, also without a display), and DQ Robotics (dqrobotics PPA, configured in the base
# image). MuJoCo itself is built from source by cpp-interface-mujoco.
RUN apt-get update && \
    apt-get install -y \
        git \
        build-essential \
        cmake \
        libeigen3-dev \
        libglfw3-dev \
        libgl1-mesa-dev \
        libgl1-mesa-dri \
        libegl-dev \
        libegl-mesa0 \
        libdqrobotics && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

########################################################################################################
# Update the SAS packages of the base image (the driver needs sas::ShutdownSignaler and the control loop callback of
# sas::RobotDriver), with libmarinholab-sas-core, the ROS-free core of SAS, which cpp-interface-mujoco also uses. Only
# the SAS packages already installed: the wildcard also matches packages whose dependencies are not available here.
# The debug symbols are removed first, as they pin the old versions.
RUN apt-get update && \
    apt-get purge -y 'ros-jazzy-sas-*-dbgsym' && \
    apt-get install -y --only-upgrade 'ros-jazzy-sas-*' && \
    apt-get install -y libmarinholab-sas-core && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

########################################################################################################
# Create the git folder to clone github repositories
RUN mkdir -p ~/git

########################################################################################################
# Clone cpp-interface-mujoco into ~/git, then configure, build, and install it (without its tests and examples)
RUN cd ~/git/ && \
    git clone https://github.com/juanjqo/cpp-interface-mujoco.git && \
    cd ~/git/cpp-interface-mujoco/ && \
    cmake -S . -B build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DDQ_MUJOCO_BUILD_TESTS=OFF \
        -DDQ_MUJOCO_BUILD_EXAMPLES=OFF && \
    cmake --build build -j$(nproc) && \
    cmake --install build && \
    ldconfig

########################################################################################################
# Clone cpp-interface-mujoco-robots into ~/git, for its models (the scenes given to the driver, e.g.,
# ~/git/cpp-interface-mujoco-robots/models/universal_robots_ur3e/scene.xml), then build and install its library
# (the robots, with their kinematics, for the controllers)
RUN cd ~/git/ && \
    git clone https://github.com/juanjqo/cpp-interface-mujoco-robots.git && \
    cd ~/git/cpp-interface-mujoco-robots/ && \
    cmake -S . -B build \
        -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DDQ_MUJOCO_ROBOTS_BUILD_TESTS=OFF \
        -DDQ_MUJOCO_ROBOTS_BUILD_EXAMPLES=OFF && \
    cmake --build build -j$(nproc) && \
    cmake --install build && \
    ldconfig

########################################################################################################
# The ROS 2 workspace, with this package
WORKDIR /app
RUN mkdir -p /app/ros2_ws/src
COPY . /app/ros2_ws/src/sas_robot_driver_mujoco

# Add some useful shortcuts
RUN echo "source /opt/ros/jazzy/setup.bash" >> /etc/bash_env
RUN echo "alias cdros2='cd /app/ros2_ws/' " >> /etc/bash_env
RUN echo "alias buildros2='cdros2 && colcon build && source install/setup.bash' " >> /etc/bash_env
RUN echo "export MUJOCO_ROBOTS_MODELS=/root/git/cpp-interface-mujoco-robots/models" >> /etc/bash_env

# Build the ROS 2 packages
RUN bash -c "source /opt/ros/jazzy/setup.bash; cd /app/ros2_ws/; colcon build"

RUN echo "source /app/ros2_ws/install/setup.bash" >> /etc/bash_env

RUN echo "source /etc/bash_env" >> ~/.bashrc

# docker stop (and docker compose down, or Ctrl+C on docker compose up) sends SIGINT instead of SIGTERM: ros2 launch, the
# main process of the containers, then stops the driver as Ctrl+C does (it ignores SIGTERM as the main process, so the
# driver would be killed after the grace period)
STOPSIGNAL SIGINT
