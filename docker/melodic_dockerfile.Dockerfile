FROM osrf/ros:melodic-desktop-full-bionic

# Set default shell
SHELL ["/bin/bash", "-c"]

# Add new user
RUN useradd --system --create-home --home-dir /home/user --shell /bin/bash --gid root --groups sudo,video,users --uid 1000 --password user@123 user && \ 
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

ENV QT_X11_NO_MITSHM=1 \
    USER=user \
    LANG=en_US.UTF-8 \
    HOME=/home/user \
    XDG_RUNTIME_DIR=/run/user/1000 \
    TZ=America/New_York 

# ENV NVIDIA_VISIBLE_DEVICES \
#     ${NVIDIA_VISIBLE_DEVICES:-all}
# ENV NVIDIA_DRIVER_CAPABILITIES \
#     ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

USER $USER
WORKDIR $HOME
# custom Bash prompt
RUN { echo && echo "PS1='\[\e]0;\u \w\a\]\[\033[01;32m\]\u\[\033[00m\] \[\033[01;34m\]\w\[\033[00m\] \\\$ '" ; } >> .bashrc

RUN /bin/bash -c "apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654"

# Basic setup
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends --allow-unauthenticated \
    software-properties-common \
    build-essential \
    g++ \
    git \
    ca-certificates \
    make \
    automake \
    autoconf \
    libtool \
    pkg-config \
    python \
    libxext-dev \
    libx11-dev \
    doxygen \
    tmux \
    tzdata \
    xclip \
    # mesa-utils \
    x11proto-gl-dev && \
    sudo rm -rf /var/lib/apt/lists/*

# Setup tmux config
ADD --chown=user:1000 https://raw.githubusercontent.com/kanishkaganguly/dotfiles/master/tmux/.tmux.bash.conf $HOME/.tmux.conf

# Set datetime and timezone correctly
# Remove duplicate sources
RUN sudo ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo '$TZ' | sudo tee -a /etc/timezone

# Install OpenCV
# RUN cd ${HOME} && \
#     git clone https://github.com/opencv/opencv.git && \
#     git -C opencv checkout 4.x && \
#     mkdir -p build && cd build && \
#     cmake ../opencv && \
#     make -j4 && \
#     sudo make install

# Install ROS packages
RUN sudo apt-get update && sudo apt-get install -y \
    python-catkin-tools \
    ros-melodic-moveit-visual-tools \
    ros-melodic-moveit-* \
    ros-melodic-usb-cam \
    ros-melodic-fiducial-msgs \
    ros-melodic-aruco-detect \
    libeigen3-dev

# Install nvidia drivers
# RUN sudo apt-get install -y nvidia-driver-510

# Setup ROS workspace directory
RUN mkdir -p $HOME/workspace/src && \
    catkin init --workspace $HOME/workspace/ && \
    cd $HOME/workspace/src

# cmake 3.16
ADD https://cmake.org/files/v3.16/cmake-3.16.9-Linux-x86_64.sh /opt/cmake-3.16.9-Linux-x86_64.sh
WORKDIR /opt/
RUN sudo chmod +x /opt/cmake-3.16.9-Linux-x86_64.sh && \
    bash -c "yes Y | sudo /opt/cmake-3.16.9-Linux-x86_64.sh" && \
    bash -c "sudo ln -s /opt/cmake-3.16.9-Linux-x86_64/bin/* /usr/local/bin"

# Set up ROS
RUN source /opt/ros/melodic/setup.bash && \
    cd ${HOME}/workspace && \
    catkin build && \
    source $HOME/workspace/devel/setup.bash

# Setup UR Drivers
RUN source /opt/ros/melodic/setup.bash && \
    cd ~/workspace && \
    git clone https://github.com/UniversalRobots/Universal_Robots_ROS_Driver.git src/Universal_Robots_ROS_Driver && \
    git clone -b calibration_devel https://github.com/fmauch/universal_robot.git src/fmauch_universal_robot && \
    sudo apt update -qq && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y && \
    catkin build && \
    source devel/setup.bash

#Install Gripper Controllers
RUN sudo apt-get install ros-melodic-gripper-action-controller

# Set up working directory and bashrc
WORKDIR ${HOME}/workspace/
RUN echo 'source /opt/ros/melodic/setup.bash' >> $HOME/.bashrc && \
    echo 'source $HOME/workspace/devel/setup.bash' >> $HOME/.bashrc
CMD /bin/bash
