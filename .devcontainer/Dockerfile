# Use the official ROS Humble desktop image directly, no ARG required
FROM osrf/ros:humble-desktop

# Arguments for creating a new user with the same UID and GID as the host
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create a new user and add it to the sudo group
RUN if ! id -u $USER_UID > /dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME && \
        usermod -aG sudo $USERNAME; \
    fi

# Install sudo, update apt packages, and give the new user passwordless sudo access
RUN apt-get update && \
    apt-get install -y sudo git && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME && \
    rm -rf /var/lib/apt/lists/*

# Add the user to the 'video' group to allow webcam access
RUN usermod --append --groups video $USERNAME

# Install ROS Humble packages
RUN apt-get update && \
    apt-get install -y \
    ros-humble-twist-mux \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-joint-state-publisher-gui \
    ros-humble-slam-toolbox \
    ros-humble-controller-manager \
    ros-humble-xacro \
    ros-humble-gazebo-ros \
    ros-humble-rplidar-ros \
    ros-humble-usb-cam \
    ros-humble-image-transport \
    ros-humble-diff-drive-controller \
    ros-humble-joint-state-broadcaster && \
    rm -rf /var/lib/apt/lists/*

# Switch to the non-root user
USER $USERNAME

# ROS Setup: Source the ROS setup file on container start
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# ROS dep update for installing dependencies
RUN sudo rosdep update

# Custom setup goes below
##########################################
## ADD CUSTOM SETUP BELOW
##########################################
