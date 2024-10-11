FROM osrf/ros:humble-desktop

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi

# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then sudo rm /etc/ros/rosdep/sources.list.d/20-default.list; fi && \
    sudo rosdep init && rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

################################
## ADD ANY CUSTOM SETUP BELOW ##
################################
# Install ROS 2 packages
RUN sudo apt install -y \
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
    ros-humble-joint-state-broadcaster