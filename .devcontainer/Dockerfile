# Start with Ubuntu 22.04 base image
FROM mcr.microsoft.com/devcontainers/base:jammy

# Set environment variables for locale
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Avoid interactive prompts during package install
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies and set locale
RUN apt-get update && apt-get install -y --no-install-recommends \
    locales \
    curl \
    wget \
    software-properties-common \
    && locale-gen en_US en_US.UTF-8 \
    && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS2 apt source dynamically based on GitHub release
RUN export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') \
    && curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" \
    && dpkg -i /tmp/ros2-apt-source.deb \
    && rm /tmp/ros2-apt-source.deb

# Update apt and install ROS2 Humble Desktop
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# Azure Kinect SDK setup
# -----------------------------
RUN wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/k/k4a-tools/k4a-tools_1.4.2_amd64.deb && \
    wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4/libk4a1.4_1.4.2_amd64.deb && \
    wget https://packages.microsoft.com/ubuntu/18.04/prod/pool/main/libk/libk4a1.4-dev/libk4a1.4-dev_1.4.2_amd64.deb && \
    wget http://archive.ubuntu.com/ubuntu/pool/universe/libs/libsoundio/libsoundio1_1.1.0-1_amd64.deb && \
    dpkg -i libsoundio1_1.1.0-1_amd64.deb || true && \
    dpkg -i libk4a1.4_1.4.2_amd64.deb || true && \
    dpkg -i libk4a1.4-dev_1.4.2_amd64.deb || true && \
    dpkg -i k4a-tools_1.4.2_amd64.deb || true && \
    ACCEPT_EULA=Y apt-get install -f -y || true && \
    rm -f *.deb

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

# Add Kinect udev rules (ignored if not supported in Docker)
RUN bash -c 'cat > /etc/udev/rules.d/99-k4a.rules <<EOF || true \
    ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097a", MODE="0666", GROUP="plugdev" \
    ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097b", MODE="0666", GROUP="plugdev" \
    ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097c", MODE="0666", GROUP="plugdev" \
    ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097d", MODE="0666", GROUP="plugdev" \
    ATTRS{idVendor}=="045e", ATTRS{idProduct}=="097e", MODE="0666", GROUP="plugdev" \ 
    EOF' || true
