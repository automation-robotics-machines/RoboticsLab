# You can change humble -> foxy / galactic / iron / jazzy / etc
FROM ros:kilted

ENV DEBIAN_FRONTEND=noninteractive

# Install development + GUI deps
RUN apt-get update && apt-get install -y \
    ros-kilted-rviz2 \
    ros-kilted-navigation2 \
    ros-kilted-topic-tools \
    ros-kilted-turtlebot3* \
    mesa-utils \
    x11-apps \
    build-essential \
    python3-colcon-common-extensions \
    terminator \
    && rm -rf /var/lib/apt/lists/*

# Create default user for VS Code
ARG USERNAME=vscode
ARG USER_UID=2001
ARG USER_GID=2001

RUN if ! getent group ${USER_GID} >/dev/null; then \
        groupadd --gid ${USER_GID} ${USERNAME}; \
    fi

# Create user if UID is not already taken
RUN if ! id -u ${USER_UID} >/dev/null 2>&1; then \
        useradd -m --uid ${USER_UID} --gid ${USER_GID} -s /bin/bash ${USERNAME}; \
    else \
        echo "UID ${USER_UID} already exists, using existing user."; \
    fi
# ROS entrypoint
# COPY setup-ros.sh /usr/local/bin/setup-ros.sh
# RUN chmod +x /usr/local/bin/setup-ros.sh

USER $USERNAME
SHELL ["/bin/bash", "-c"]

