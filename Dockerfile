# Dockerfile for LEAP Hand with ROS 1/2 options
# Build with: docker build --build-arg ROS_VERSION=<1 or 2> -t leap-hand-ros .
# Run with: docker run --privileged -v /dev:/dev --device-cgroup-rule='c 188:* rmw' leap-hand-ros

ARG ROS_VERSION=2
ARG ROS_DISTRO_1=noetic
ARG ROS_DISTRO_2=humble

FROM ubuntu:20.04 AS ros1-base
FROM ubuntu:22.04 AS ros2-base
FROM ros${ROS_VERSION}-base AS leap-hand-ros

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=UTC
ENV MAKEFLAGS="-j4"
ENV COLCON_PARALLEL_JOBS="4"

RUN apt-get update && apt-get install -y \
    git \
    python3-pip \
    python3-dev \
    build-essential \
    cmake \
    lsb-release \
    curl \
    gnupg2 \
    udev \
    && rm -rf /var/lib/apt/lists/*

# ROS 1 installation
FROM leap-hand-ros AS ros1-install
ARG ROS_DISTRO_1
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && apt-get update \
    && apt-get install -y \
        ros-${ROS_DISTRO_1}-desktop \
        python3-rosdep \
        python3-rosinstall \
        python3-rosinstall-generator \
        python3-wstool \
        python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/* \
    && rosdep init \
    && rosdep update

# ROS 2 installation
FROM leap-hand-ros AS ros2-install
ARG ROS_DISTRO_2
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
    && apt-get update \
    && apt-get install -y \
        ros-${ROS_DISTRO_2}-desktop \
        ros-dev-tools \
    && rm -rf /var/lib/apt/lists/*

FROM ros${ROS_VERSION}-install AS workspace-setup
RUN pip install dynamixel_sdk numpy
RUN mkdir -p /leap_ws/src
WORKDIR /leap_ws

COPY cpp/ /leap_ws/src/cpp/
COPY python/ /leap_ws/src/python/
COPY useful_tools/ /leap_ws/src/useful_tools/

FROM workspace-setup AS ros1-setup
COPY ros_module/ /leap_ws/src/ros_module/
WORKDIR /leap_ws

FROM workspace-setup AS ros2-setup
COPY ros2_module/ /leap_ws/src/ros2_module/
WORKDIR /leap_ws

FROM ros${ROS_VERSION}-setup AS final
ARG ROS_VERSION
ARG ROS_DISTRO_1
ARG ROS_DISTRO_2

RUN find /leap_ws/src -name "*.py" -exec chmod +x {} \;

RUN if [ "$ROS_VERSION" = "1" ]; then \
        echo "Building for ROS 1..." && \
        . /opt/ros/${ROS_DISTRO_1}/setup.sh && \
        catkin_make; \
    else \
        echo "Building for ROS 2..." && \
        . /opt/ros/${ROS_DISTRO_2}/setup.sh && \
        colcon build --symlink-install; \
    fi

RUN if [ "$ROS_VERSION" = "1" ]; then \
        echo "source /opt/ros/${ROS_DISTRO_1}/setup.bash" >> ~/.bashrc && \
        echo "source /leap_ws/devel/setup.bash" >> ~/.bashrc && \
        echo '#!/bin/bash\nsource /opt/ros/'${ROS_DISTRO_1}'/setup.bash\nsource /leap_ws/devel/setup.bash\nroslaunch leap_hand_ros leap.launch' > /launch.sh; \
    else \
        echo "source /opt/ros/${ROS_DISTRO_2}/setup.bash" >> ~/.bashrc && \
        echo "source /leap_ws/install/setup.bash" >> ~/.bashrc && \
        echo '#!/bin/bash\nsource /opt/ros/'${ROS_DISTRO_2}'/setup.bash\nsource /leap_ws/install/setup.bash\nros2 launch launch_leap.py' > /launch.sh; \
    fi

RUN chmod +x /launch.sh

RUN echo '#!/bin/bash\n\
# Detect ROS version based on installed packages\n\
if [ -d "/opt/ros/noetic" ]; then\n\
    source /opt/ros/noetic/setup.bash\n\
    if [ -f "/leap_ws/devel/setup.bash" ]; then\n\
        source /leap_ws/devel/setup.bash\n\
    fi\n\
elif [ -d "/opt/ros/humble" ]; then\n\
    source /opt/ros/humble/setup.bash\n\
    if [ -f "/leap_ws/install/setup.bash" ]; then\n\
        source /leap_ws/install/setup.bash\n\
    fi\n\
else\n\
    echo "No ROS installation detected"\n\
fi\n\
exec "$@"' > /ros_entrypoint.sh \
    && chmod +x /ros_entrypoint.sh

WORKDIR /leap_ws
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/bin/bash"]

