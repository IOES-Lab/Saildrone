# FROM arm64v8/ubuntu:24.04
# # Set RDP and SSH environments
# ARG X11Forwarding=true
# # hadolint ignore=DL3008,DL3015,DL3009
# RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
#         apt-get install -y ubuntu-desktop-minimal dbus-x11 xrdp sudo; \
#     [ $X11Forwarding = 'true' ] && apt-get install -y openssh-server; \
#     apt-get autoremove --purge; \
#     apt-get clean; \
#     rm /run/reboot-required*

# ARG USER=docker
# ARG PASS=docker

# RUN useradd -s /bin/bash -m $USER -p "$(openssl passwd "$PASS")"; \
#     usermod -aG sudo $USER; echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers; \
#     adduser xrdp ssl-cert; \
#     # Setting the required environment variables
#     echo 'LANG=en_US.UTF-8' >> /etc/default/locale; \
#     echo 'export GNOME_SHELL_SESSION_MODE=ubuntu' > /home/$USER/.xsessionrc; \
#     echo 'export XDG_CURRENT_DESKTOP=ubuntu:GNOME' >> /home/$USER/.xsessionrc; \
#     echo 'export XDG_SESSION_TYPE=x11' >> /home/$USER/.xsessionrc; \
#     # Enabling log to the stdout
#     sed -i "s/#EnableConsole=false/EnableConsole=true/g" /etc/xrdp/xrdp.ini; \
#     # Disabling system animations and reducing the
#     # image quality to improve the performance
#     sed -i 's/max_bpp=32/max_bpp=16/g' /etc/xrdp/xrdp.ini; \
#     gsettings set org.gnome.desktop.interface enable-animations true; \
#     # Listening on wildcard address for X forwarding
#     [ $X11Forwarding = 'true' ] && \
#         sed -i 's/#X11UseLocalhost yes/X11UseLocalhost no/g' /etc/ssh/sshd_config || \
#         sed -i 's/#PasswordAuthentication yes/PasswordAuthentication yes/g' /etc/ssh/sshd_config || \
#         :;

# # Disable initial welcome window
# RUN echo "X-GNOME-Autostart-enabled=false" >> /etc/xdg/autostart/gnome-initial-setup-first-login.desktop

# # Install basics
# ENV DEBIAN_FRONTEND noninteractive
# ENV DEBCONF_NONINTERACTIVE_SEEN=true
# # hadolint ignore=DL3008
# RUN apt-get update && \
#     apt-get install -y --no-install-recommends \
#     sudo xterm init systemd snapd vim net-tools \
#     curl wget git build-essential cmake cppcheck \
#     gnupg libeigen3-dev libgles2-mesa-dev \
#     lsb-release pkg-config protobuf-compiler \
#     python3-dbg python3-pip python3-venv \
#     qtbase5-dev ruby dirmngr gnupg2 nano xauth \
#     software-properties-common htop libtool \
#     x11-apps mesa-utils bison flex automake && \
#     rm -rf /var/lib/apt/lists/

# Locale for UTF-8
# RUN truncate -s0 /tmp/preseed.cfg && \
#    (echo "tzdata tzdata/Areas select Etc" >> /tmp/preseed.cfg) && \
#    (echo "tzdata tzdata/Zones/Etc select UTC" >> /tmp/preseed.cfg) && \
#    debconf-set-selections /tmp/preseed.cfg && \
#    rm -f /etc/timezone && \
#    dpkg-reconfigure -f noninteractive tzdata
# # hadolint ignore=DL3008
# RUN apt-get update && \
#     apt-get -y install --no-install-recommends locales tzdata \
#     && rm -rf /tmp/*
# RUN locale-gen en_US en_US.UTF-8 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
#     export LANG=en_US.UTF-8

# # Run
# EXPOSE 3389/tcp
# EXPOSE 22/tcp
# # hadolint ignore=DL3025
# CMD sudo rm -f /var/run/xrdp/xrdp*.pid >/dev/null 2>&1; \
#     sudo service dbus restart >/dev/null 2>&1; \
#     sudo /usr/lib/systemd/systemd-logind >/dev/null 2>&1 & \
#     [ -f /usr/sbin/sshd ] && sudo /usr/sbin/sshd; \
#     sudo xrdp-sesman --config /etc/xrdp/sesman.ini; \
#     sudo xrdp --nodaemon --config /etc/xrdp/xrdp.ini

# Using the pre-built image for above commented out dockerfile code lines
# hadolint ignore=DL3007
FROM woensugchoi/ubuntu-arm-rdp-base:latest
ARG USER=docker

# ROS-Gazebo arg
ARG BRANCH="main"
ARG ROS_DISTRO="jazzy"

# Update OS
RUN apt update && apt full-upgrade -y && apt autoremove -y

# Install ROS-Gazebo framework
ADD https://raw.githubusercontent.com/IOES-Lab/saildrone/$BRANCH/\
extras/ros-jazzy-gz-harmonic-install.sh install.sh
RUN sudo bash install.sh

# Install wave sim dependencies
RUN apt-get update && \
    apt-get install -y libcgal-dev libfftw3-dev \
    && rm -rf /var/lib/apt/lists/

# Prereqs for Ardupilot - ArduRover
ENV DEBIAN_FRONTEND=noninteractive
ENV DEBCONF_NONINTERACTIVE_SEEN=true
# hadolint ignore=DL3008
ADD --chown=root:root --chmod=0644 https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list /etc/ros/rosdep/sources.list.d/00-gazebo.list
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" |  tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null \
    && apt-get -q update && \
    apt-get install -y --no-install-recommends \
    python-is-python3 python3-future python3-wxgtk4.0 python3-pexpect \
    libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    libgz-sim8-dev rapidjson-dev libopencv-dev libasio-dev \
    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl \
    && rm -rf /var/lib/apt/lists/
# Install mavros
RUN apt-get update && \
    apt-get -y install --no-install-recommends ros-jazzy-mavros* \
    && rm -rf /tmp/*
WORKDIR /opt/mavros_ws
RUN wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    bash ./install_geographiclib_datasets.sh

# Download the background image from GitHub raw content URL
# hadolint ignore=DL3047
RUN wget -O /usr/share/backgrounds/custom-background.png -q \
    https://raw.githubusercontent.com/IOES-Lab/saildrone/$BRANCH/\
extras/background.png && \
    mv /usr/share/backgrounds/warty-final-ubuntu.png \
        /usr/share/backgrounds/warty-final-ubuntu.png.bak && \
    mv /usr/share/backgrounds/custom-background.png \
        /usr/share/backgrounds/warty-final-ubuntu.png && \
    cp /usr/share/backgrounds/warty-final-ubuntu.png \
        /usr/share/backgrounds/ubuntu-wallpaper-d.png

# Install Ardupilot - ArduRover
USER docker
RUN wget -O /tmp/install.sh https://raw.githubusercontent.com/IOES-Lab/saildrone/$BRANCH/extras/ardurover-ubuntu-install-local.sh
RUN chmod +x /tmp/install.sh && bash /tmp/install.sh

# Set up Dave workspace
ENV DAVE_UNDERLAY=/home/$USER/saildrone_ws
WORKDIR $DAVE_UNDERLAY/src
RUN wget -O /home/$USER/saildrone_ws/saildrone.repos -q https://raw.githubusercontent.com/IOES-Lab/saildrone/$BRANCH/\
extras/repos/saildrone.$ROS_DISTRO.repos
RUN vcs import --shallow --input "/home/$USER/saildrone_ws/saildrone.repos"

USER root
# hadolint ignore=DL3027
RUN apt update && apt --fix-broken install && \
    rosdep init && rosdep update --rosdistro $ROS_DISTRO && \
    rosdep install --rosdistro $ROS_DISTRO -iy --from-paths . && \
    rm -rf /var/lib/apt/lists/

# Install VSCode
RUN wget -qO- https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > /usr/share/keyrings/microsoft.gpg && \
    echo "deb [arch=arm64 signed-by=/usr/share/keyrings/microsoft.gpg] https://packages.microsoft.com/repos/code stable main" \
    > /etc/apt/sources.list.d/vscode.list && \
    apt-get update && apt-get install -y code && \
    rm -rf /var/lib/apt/lists/*

USER docker
# Build Saildrone workspace
WORKDIR $DAVE_UNDERLAY
RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" && colcon build

# Patch for wave sim
USER root
RUN ln -s /opt/ros/jazzy/opt/gz_ogre_next_vendor/lib/libOgreNextMain.so.2.3.3 /opt/ros/jazzy/opt/gz_ogre_next_vendor/lib/libOgreNextMain.so.2.3.1

# Set User as user
USER docker
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc && \
    echo "source $DAVE_UNDERLAY/install/setup.bash" >> ~/.bashrc && \
    echo "export GEOGRAPHICLIB_GEOID_PATH=/usr/local/share/GeographicLib/geoids" >> ~/.bashrc && \
    echo "export PYTHONPATH=\$PYTHONPATH:/opt/gazebo/install/lib/python" >> ~/.bashrc && \
    echo "export PATH=/home/$USER/ardupilot_ws/ardupilot/Tools/autotest:\$PATH" >> ~/.bashrc && \
    echo "export PATH=/home/$USER/ardupilot_ws/ardupilot/build/sitl/bin:\$PATH" >> ~/.bashrc && \
    echo "export GZ_SIM_SYSTEM_PLUGIN_PATH=/home/$USER/ardupilot_ws/ardupilot_gazebo/build:\$GZ_SIM_SYSTEM_PLUGIN_PATH:/home/docker/saildrone_ws/install/wave/lib" >> ~/.bashrc && \
    echo "export GZ_GUI_PLUGIN_PATH=\$GZ_GUI_PLUGIN_PATH:/home/docker/saildrone_ws/install/wave/src/gui/plugins/waves_control/build" >> ~/.bashrc && \
    echo "export GZ_SIM_RESOURCE_PATH=/home/$USER/ardupilot_ws/ardupilot_gazebo/models:/home/$USER/ardupilot_ws/ardupilot_gazebo/worlds:\$GZ_SIM_RESOURCE_PATH" >> ~/.bashrc && \
    echo "\n\n" >> ~/.bashrc && echo "if [ -d ~/HOST ]; then chown $USER:$USER ~/HOST; fi" >> ~/.bashrc  && \
    echo "export PS1='\[\e[1;36m\]\u@DAVE_docker\[\e[0m\]\[\e[1;34m\](\$(hostname | cut -c1-12))\[\e[0m\]:\[\e[1;34m\]\w\[\e[0m\]\$ '" >>  ~/.bashrc

# Other environment variables
RUN echo "export XDG_RUNTIME_DIR=~/.xdg_log" >> ~/.bashrc && \
    echo "unset SESSION_MANAGER" >> ~/.bashrc

# Create and write the welcome message to a new file
RUN mkdir -p /home/docker/.config/autostart && \
    printf '\033[1;36m =====\n' >> ~/.hi && \
    printf ' ▗▄▄▖ ▗▄▖ ▗▄▄▄▖▗▖   ▗▄▄▄ ▗▄▄▖  ▗▄▖ ▗▖  ▗▖▗▄▄▄▖\n' >> ~/.hi && \
    printf '▐▌   ▐▌ ▐▌  █  ▐▌   ▐▌  █▐▌ ▐▌▐▌ ▐▌▐▛▚▖▐▌▐▌   \n' >> ~/.hi && \
    printf ' ▝▀▚▖▐▛▀▜▌  █  ▐▌   ▐▌  █▐▛▀▚▖▐▌ ▐▌▐▌ ▝▜▌▐▛▀▀▘\n' >> ~/.hi && \
    printf '▗▄▄▞▘▐▌ ▐▌▗▄█▄▖▐▙▄▄▖▐▙▄▄▀▐▌ ▐▌▝▚▄▞▘▐▌  ▐▌▐▙▄▄▖\n' >> ~/.hi && \
    printf '\033[0m' >> ~/.hi && \
    printf '\033[1;32m\n =====\n\033[0m' >> ~/.hi && \
    printf "\\033[1;32m 👋 Hi! This is Docker virtual environment for SailDrone (fork from Dave)\n\\033[0m" \
    >> ~/.hi && \
    printf "\\033[1;33m\tROS2 Jazzy - Gazebo Harmonic (w ardupilot(ardurover) + mavros)\n\n\n\\033[0m" \
    >> ~/.hi

# Remove sudo message
RUN touch /home/docker/.sudo_as_admin_successful

# Autostart terminal
# hadolint ignore=SC3037
RUN echo "[Desktop Entry]\nType=Application" \
    > /home/docker/.config/autostart/terminal.desktop && \
    echo "Exec=gnome-terminal -- bash -c 'cat ~/.hi; exec bash'" \
    >> /home/docker/.config/autostart/terminal.desktop && \
    echo -e "X-GNOME-Autostart-enabled=true" \
    >> /home/docker/.config/autostart/terminal.desktop
