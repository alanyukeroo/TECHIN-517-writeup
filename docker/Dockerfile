ARG CUDA_VERSION=13.1.0 #G14 5060
#ARG CUDA_VERSION=13.0.0 #4090

ARG OS_VERSION=22.04

### build image ###
FROM nvidia/cuda:${CUDA_VERSION}-base-ubuntu${OS_VERSION} AS build

# Lecture code reference: Env vars for the nvidia-container-runtime.
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1
ENV DEBIAN_FRONTEND=noninteractive

RUN echo 'Acquire::Retries "3";' > /etc/apt/apt.conf.d/80retries

# Install basic setup tools and ROS 2 repository
RUN apt-get update && apt-get install -y --no-install-recommends \
    software-properties-common \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
    && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8

# Add ROS 2 GPG key and repo
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Lecture code reference: Install ROS 2 Humble Desktop and other apt packages from the screenshots
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    bash-completion \
    build-essential \
    cmake \
    gdb \
    git \
    openssh-client \
    python3-argcomplete \
    python3-pip \
    python-is-python3 \
    ros-dev-tools \
    ros-humble-ament-* \
    ros-humble-moveit \
    ros-humble-realsense2-* \
    ros-humble-usb-cam \
    ros-humble-topic-tools \
    vim \
    && rm -rf /var/lib/apt/lists/*

# Lecture code reference: Install pyrealsense2
RUN pip install pyrealsense2

# Install lerobot system dependencies based on its dockerfile
RUN apt-get update && apt-get install -y --no-install-recommends \
    libglib2.0-0 libegl1-mesa-dev ffmpeg \
    libusb-1.0-0-dev speech-dispatcher libgeos-dev portaudio19-dev \
    pkg-config ninja-build \
    && rm -rf /var/lib/apt/lists/*

# For devcontainers, copy the local project folder to install lerobot.
# (If mounting a volume over this later, you will still have the installed dependencies)
COPY third_party/lerobot /tmp/lerobot
RUN pip install '/tmp/lerobot[all]' && \
    pip uninstall torch torchvision torchaudio -y && \
    pip install torch==2.7.1 torchvision==0.22.1 torchaudio==2.7.1 \
        --index-url https://download.pytorch.org/whl/cu128

### dev ###
FROM build AS dev 

# Lecture code reference: change user
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi

# Lecture code reference: Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Lecture code reference: Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video,plugdev $USERNAME

# Switch from root to user
USER $USERNAME

# Lecture code reference: general dev tools
RUN sudo apt-get update && sudo apt-get install -y --no-install-recommends \
    vim \
    nano \
    less \
    tmux \
    && sudo rm -rf /var/lib/apt/lists/*

# Lecture code reference: environment settings in the bashrc
RUN echo "export PATH=\"\$HOME/.local/bin:\$PATH\"" >> /home/ubuntu/.bashrc && \
    echo "source /opt/ros/humble/setup.bash" >> /home/ubuntu/.bashrc  && \
    echo "source /home/ubuntu/techin517/ros2_ws/install/setup.bash" >> /home/ubuntu/.bashrc && \
    echo "export ROS_LOCALHOST_ONLY=1" >> /home/ubuntu/.bashrc && \
    echo "export HF_USER=ubuntu" >> /home/ubuntu/.bashrc && \
    echo "export XDG_RUNTIME_DIR=/tmp/runtime-ubuntu" >> /home/ubuntu/.bashrc

# Default shell
CMD ["/bin/bash"]
