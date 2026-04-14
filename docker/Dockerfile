ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop

# Disable interactive prompts
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# 1. Install Basic Tools
RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends \
    tree \
    build-essential \
    git \
    docker.io \
    wget \
    curl \
    nano \
    vim \
    neovim \
    lsb-release \
    gnupg2 \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    python3-vcstool \
    software-properties-common \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-teleop-twist-keyboard \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get upgrade -y && rm -rf /var/lib/apt/lists/*

# 2. Setup Non-Root User (ros)
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# If the user 'ubuntu' or 'ros' already exists with UID 1000, we rename it to our choice
# Otherwise, we create it.
RUN if id -u $USER_UID >/dev/null 2>&1; then \
        # Get the current name of user 1000
        EXISTING_USER=$(getent passwd $USER_UID | cut -d: -f1); \
        echo "User with UID $USER_UID already exists ($EXISTING_USER), reusing it."; \
        if [ "$EXISTING_USER" != "$USERNAME" ]; then \
            usermod -l $USERNAME $EXISTING_USER; \
            groupmod -n $USERNAME $(getent group $USER_GID | cut -d: -f1); \
            usermod -d /home/$USERNAME -m $USERNAME; \
        fi; \
    else \
        groupadd --gid $USER_GID $USERNAME && \
        useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash; \
    fi \
    && apt-get update && apt-get install -y sudo \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

# 3. Create Workspace and Fix Permissions
WORKDIR /home/$USERNAME/ros2_ws
RUN mkdir -p /home/$USERNAME/ros2_ws/src/quadruped \
    && chown -R $USERNAME:$USERNAME /home/$USERNAME/ros2_ws

# 4. Copy Code (Using the user flag to own the files)
COPY src/ /home/ros/ros2_ws/src/

# 5. Switch to User to install dependencies and build
USER $USERNAME

# Initialize rosdep if not already done by base image
RUN sudo rosdep init || true && rosdep update

# Install dependencies defined in package.xml
# We use 'from-paths' to look at the src folder we just copied
RUN sudo apt-get update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    sudo rm -rf /var/lib/apt/lists/*

# 6. Build the workspace (Build time, NOT run time)
# We source ROS 2, build, and then ensure the install setup is sourced in bashrc
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --symlink-install

# 7. Setup Environment
COPY --chown=$USERNAME:$USERNAME docker/bash_aliases.txt /home/$USERNAME/.bash_aliases
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "source /home/$USERNAME/ros2_ws/install/setup.bash" >> /home/$USERNAME/.bashrc && \
    echo "if [ -f ~/.bash_aliases ]; then . ~/.bash_aliases; fi" >> /home/$USERNAME/.bashrc

# 8. Entrypoint
COPY --chown=$USERNAME:$USERNAME docker/entrypoint.sh /home/$USERNAME/entrypoint.sh
RUN sudo chmod +x /home/$USERNAME/entrypoint.sh

ENTRYPOINT ["/home/ros/entrypoint.sh"]
CMD ["/bin/bash"]
