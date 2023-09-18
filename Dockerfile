FROM osrf/ros:rolling-desktop-full

# Disable interactive frontend
# NOTE: for this to take effect when using "sudo" apt install, we need the -E flag to preserve the environment variable inside the sudo command
ARG DEBIAN_FRONTEND=noninteractive

# Add vscode user with same UID and GID as your host system
# (copied from https://code.visualstudio.com/remote/advancedcontainers/add-nonroot-user#_creating-a-nonroot-user)
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
# Switch from root to user
USER $USERNAME

# Change directory to user home
WORKDIR /home/$USERNAME

# Install Webots
# We install keyboard-configuration first, with DEBIAN_FRONTEND disabled because installing keyboard-configuration can lock up the docker build
RUN sudo -E apt install -y wget software-properties-common
RUN wget -qO- https://cyberbotics.com/Cyberbotics.asc | sudo apt-key add -
RUN sudo apt-add-repository 'deb https://cyberbotics.com/debian/ binary-amd64/'
RUN sudo apt update
RUN sudo -E apt install webots -y

# Clone WebotsLolaController
# RUN git clone https://github.com/Bembelbots/WebotsLoLaController.git

# Install pip
RUN sudo -E apt install python3-pip -y

# Copy ijnek_wrestle into container
COPY --chown=$USERNAME:$USERNAME ijnek_wrestle ijnek_wrestle/.

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Create workspace and change directory into it
RUN mkdir ws
WORKDIR /home/$USERNAME/ws

# Copy the cloned repositories into container
COPY --chown=$USERNAME:$USERNAME src src/.

# Install dependencies ("|| true" is required to prevent a failure return code that happens if rosdep couldn't find some binary dependencies)
RUN rosdep install -y --from-paths src --ignore-src --rosdistro rolling -r || true

# Copy the colcon defaults file
COPY --chown=$USERNAME:$USERNAME colcon/defaults.yaml /home/$USERNAME/.colcon/defaults.yaml

# Set some ROS 2 logging env variables in ~/.bashrc
RUN echo 'export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] {file_name}:{line_number} - {message}"' >> ~/.bashrc
RUN echo 'export RCUTILS_COLORIZED_OUTPUT=1' >> ~/.bashrc

# Switch to bash
SHELL ["/bin/bash", "-c"]
# Enable flag to ignore the absence of the meshes and texture directories, used in nao_description's CMakeLists.txt
ENV IGNORE_MESHES_AND_TEXTURE_DIRS=1
# Source and build
RUN source /opt/ros/rolling/setup.bash && colcon build --packages-ignore nao_lola --cmake-args -DIGNORE_MESHES_AND_TEXTURE_DIRS=1

# Change directory to user home
WORKDIR /home/$USERNAME
