ARG ROS_DISTRO="jazzy"
FROM osrf/ros:$ROS_DISTRO-desktop-full
ARG BRANCH="main"

# Install Utilities
# hadolint ignore=DL3008
RUN apt-get update && \
    apt-get install -y --no-install-recommends \
    sudo xterm init systemd snapd vim net-tools \
    curl wget git build-essential cmake cppcheck \
    gnupg libeigen3-dev libgles2-mesa-dev \
    lsb-release pkg-config protobuf-compiler \
    python3-dbg python3-pip python3-venv python3-pexpect \
    python-is-python3 python3-future python3-wxgtk4.0 \
    qtbase5-dev ruby dirmngr gnupg2 nano xauth \
    software-properties-common htop libtool \
    x11-apps mesa-utils bison flex automake \
    && rm -rf /var/lib/apt/lists/

# Prereqs for Ardupilot - ArduRover
ADD --chown=root:root --chmod=0644 https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list /etc/ros/rosdep/sources.list.d/00-gazebo.list
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" |  tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends \
    libgz-sim8-dev rapidjson-dev libopencv-dev \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    && rm -rf /var/lib/apt/lists/

# Locale for UTF-8
RUN truncate -s0 /tmp/preseed.cfg && \
   (echo "tzdata tzdata/Areas select Etc" >> /tmp/preseed.cfg) && \
   (echo "tzdata tzdata/Zones/Etc select UTC" >> /tmp/preseed.cfg) && \
   debconf-set-selections /tmp/preseed.cfg && \
   rm -f /etc/timezone && \
   dpkg-reconfigure -f noninteractive tzdata
# hadolint ignore=DL3008
RUN apt-get update && \
    apt-get -y install --no-install-recommends locales tzdata \
    && rm -rf /tmp/*
RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    export LANG=en_US.UTF-8

# Install ROS-Gazebo framework
ADD https://raw.githubusercontent.com/IOES-Lab/saildrone/$BRANCH/\
extras/ros-jazzy-gz-harmonic-install.sh install.sh
RUN bash install.sh

# Install Ardupilot - ArduRover
ADD https://raw.githubusercontent.com/IOES-Lab/saildrone/$BRANCH/\
extras/ardurover-ubuntu-install.sh install.sh
RUN bash install.sh
# Install mavros
RUN apt-get update && \
    apt-get -y install --no-install-recommends ros-jazzy-mavros* \
    && rm -rf /tmp/*
WORKDIR /opt/mavros_ws
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    bash ./install_geographiclib_datasets.sh

# Set up Saildrone workspace
ENV SAILDRONE_WS=/opt/saildrone_ws
WORKDIR $SAILDRONE_WS/src

ADD https://raw.githubusercontent.com/IOES-Lab/saildrone/$BRANCH/\
extras/repos/dave.$ROS_DISTRO.repos $SAILDRONE_WS/dave.repos
RUN vcs import --shallow --input $SAILDRONE_WS/dave.repos

# Install Saildrone dependencies
RUN apt-get update && rosdep update && \
    rosdep install -iy --from-paths . && \
    rm -rf /var/lib/apt/lists/

# Compile Saildrone
WORKDIR $SAILDRONE_WS
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && \
    colcon build
WORKDIR /

# Set up bashrc for root
RUN echo "source /opt/ros/jazzy/setup.bash" >> /root/.bashrc && \
    echo "source /opt/saildrone_ws/install/setup.bash" >> /root/.bashrc && \
    echo "export PATH=/opt/ardupilot_ws/ardupilot/Tools/autotest:\$PATH" >> /root/.bashrc && \
    echo "export PATH=/opt/ardupilot_ws/ardupilot/build/sitl/bin:\$PATH" >> /root/.bashrc && \
    echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ardupilot_ws/ardupilot_gazebo/build:\$GZ_SIM_SYSTEM_PLUGIN_PATH" >> /root/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=/opt/ardupilot_ws/ardupilot_gazebo/models:/opt/ardupilot_ws/ardupilot_gazebo/worlds:\$GZ_SIM_RESOURCE_PATH" >> /root/.bashrc && \
    echo "export PS1='\[\e[1;36m\]\u@DAVE_docker\[\e[0m\]\[\e[1;34m\](\$(hostname | cut -c1-12))\[\e[0m\]:\[\e[1;34m\]\w\[\e[0m\]\$ '" >> /root/.bashrc

RUN touch /root/.saildrone_entrypoint && printf '\033[1;36m =====\n' >> /root/.saildrone_entrypoint && \
    printf '  ____    _    _____ _       _     ____   ____   _   _   _____  \n' >> /root/.saildrone_entrypoint && \
    printf ' / ___|  / \  |  ___| |     | |   |  _ \\ / ___| | \\ | | | ____| \n' >> /root/.saildrone_entrypoint && \
    printf '| |     / _ \ | |_  | |     | |   | | | | |     |  \\| | |  _|   \n' >> /root/.saildrone_entrypoint && \
    printf '| |___ / ___ \|  _| | |___  | |   | |_| | |___  | |\\  | | |___  \n' >> /root/.saildrone_entrypoint && \
    printf ' \\____/_/   \\_\\_|   |_____| |_|   |____/ \\____| |_| \\_| |_____| \n\033[0m' >> /root/.saildrone_entrypoint && \
    printf '\033[1;32m\n =====\n\033[0m' >> /root/.saildrone_entrypoint && \
    printf "\\033[1;32m 👋 Hi! This is Docker virtual environment for Saildrone (fork from Dave)\n\\033[0m" \
    >> /root/.saildrone_entrypoint && \
    printf "\\033[1;33m\tROS2 Jazzy - Gazebo Harmonic (w ardupilot(ardurover) + mavros)\n\n\\033[0m" \
    >> /root/.saildrone_entrypoint && \
    echo 'cat /root/.saildrone_entrypoint' >> /root/.bashrc

WORKDIR /root
