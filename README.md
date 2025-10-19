# Saildrone

[![Publish a Docker image (AMD64; Common X86_64 Linux Machine)](https://github.com/IOES-Lab/Saildrone/actions/workflows/docker-amd64.yml/badge.svg)](https://github.com/IOES-Lab/Saildrone/actions/workflows/docker-amd64.yml)
[![Publish a Docker image (ARM64; Apple Silicon)](https://github.com/IOES-Lab/dave/actions/workflows/docker-arm64v8.yml/badge.svg?branch=ros2)](https://github.com/IOES-Lab/Saildrone/actions/workflows/docker-arm64v8.yml)

The original Documentation is currently at [http://dave-ros2.notion.site](http://dave-ros2.notion.site)

## Installation on Mac (Apple Silicon)

### 🐳 Install Docker for Mac

- Install Docker Desktop with following official document
    - https://docs.docker.com/desktop/install/mac-install/

### 🏗️ Install Brew, Xauth, Python, Rocker

- On Terminal (find it at spotlight)
- Install [Homebrew](https://brew.sh/) package manager (something like app store)
    
    ```bash
    /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    ```
    
- Install python3 with Homebrew and update pip (the python package manager)
    
    ```bash
    brew install python3
    # (below is optional)
    python3 -m pip install --upgrade pip
    ```
    
- Install xauth with Homebrew
    
    ```jsx
    brew install xauth
    ```
    
- Set-up virtual python environment for rocker installation
    
    ```bash
    # Make or Choose a directory for saildrone workspace
    # here, populating saildrone_ws directory at user home
    mkdir -p ~/saildrone/src && cd ~/saildrone
    
    # Make python virtual environment with python3-venv
    python3 -m venv saildrone_venv
    # Set-up virtual environment (terminal will now show saildrone_venv)
    source ~/saildrone_ws/saildrone_venv/bin/activate
    
    # Install VCS tool in virtual environment
    pip install vcstool
    ```
    
    - If run correctly, you will see `(saildrone_venv)` your terminal showing that you are in the virtual environment
- Copy saildrone source codes and install rocker in virtual environment
    
    ```bash
    cd ~/saildrone_ws/src && curl -fsSL https://raw.githubusercontent.com/IOES-Lab/Saildrone/master/extras/repos/saildrone.jazzy.repos | vcs import --skip-existing --shallow --input -
    ```
    
- Install downloaded rocker(the Open Robotics Docker CLI)
    
    ```bash
    python3 -m pip install rocker/.
    ```

## 🛠️ Build Saildrone environment

- Use pre-built docker environment (`ros2-arm-rdp` means that the image is for arm architecture).
    
    ```bash
    # Download pre-build image from docker hub
    # You may have to have Docker Desktop application running
    docker pull ioeslab/saildrone:ros2-arm-rdp
    # Rename downloaded image to saildrone:latest
    docker tag ioeslab/saildrone:ros2-arm-rdp saildrone:latest
    ```
    
- (Optional) If you want to build the image of your own, follow below. If you are using pre-built image, skip this part.
    - Unlike the Linux machine cases above, we won’t build image using `build.bash` of dockwater. We will use docker command instead
        - This takes quite long (longer than linux versions about 10~40 minutes depending on your machine). You may use prebuilt image (go down below)
            - You need to have docker desktop running on your machine.
            
            ```bash
            docker build -f ~/saildrone_ws/src/saildrone/.docker/jazzy.arm64v8.rdp.dockerfile -t saildrone:latest .
            ```

## 🚀 Start Saildrone Environment

    💡 Docker Desktop should be running!

- We are using Dockwater (wrapper of rocker) to launch the Saildrone environment
    - we will use special tag `-r` (added RDP remote desktop connection capability and removed NVIDIA, sadly no 3D hardware acceleration) for the docker `run.bash`
    
    ```bash
    source ~/saildrone_ws/saildrone_venv/bin/activate
    cd ~/saildrone_ws/src/dockwater
    # Now run! (it may take some time at first run)
    ./run.bash -r saildrone:latest
    ```
    
    - It will launch the docker image and stop on hold
        - You will see the terminal screen stopped at this message
            
            ```bash
            [20240919-01:26:20] [INFO ] xrdp_listen_pp done
            ```
    
- Download and install RDP client
    - https://apps.apple.com/in/app/windows-app/id1295203466?mt=12
- Connect to [`localhost`](http://localhost) with username and password both `docker`

    💡 If you see only black screen when connected using RDP client, turn `./run.bash -r saildrone:latest` off with `Control + C` and re-run it

    💡 The host machine’s home directory is mounted at /home/docker/HOST.

- Launch demo rexrov ROV on ocean waves with below command on Terminal app inside RDP
    ```bash
    ros2 launch dave_demos dave_robot.launch.py namespace:=wamv world_name:=wamv_waves paused:=false

    # It may take some time to load and may show NOT RESPONDING
    # Just wait until the window shows up
    ```

    - Saildrone source code is already compiled and installed at `/home/docker/saildrone_ws`
    - If you want something of your own, the host machine’s home directory is mounted at `/home/docker/HOST`


## ROVER Demo
- Installed at `/home/docker/ros_gz_rover`

- To make this work you need to modify `run.bash` at `~/dave_ws/src/dockwater` before running `./run.bash -r saildrone:latest`
    - Add port for QGroundControl communication
    ```bash 
    --port "$HOST_RDP_PORT":3389 --port 14551:14551/udp
    ```
    - At QGroundControl application, set the UDP link to port 14551

- To launch the rover demo, run the following command inside the RDP terminal:

    ```
    # Terminal 1:
    ros2 launch ros_gz_rover rover.launch.py

    # Terminal 2:
    sim_vehicle.py -N -v Rover -f rover-skid --model JSON --mavproxy-args="--out=udp:host.docker.internal:14551"

    # Terminal 3:
    ros2 launch ros_gz_rover mavros.launch.py

    # Terminal 4:
    gz sim -v4 -g
    ```

