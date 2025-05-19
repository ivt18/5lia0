FROM osrf/ros:noetic-desktop
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi

RUN apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

USER $USERNAME
RUN sudo usermod --append --groups video $USERNAME
RUN sudo apt update && sudo apt upgrade -y
RUN sudo apt install -y git iputils-ping x11-apps sshfs sshpass net-tools netcat openssh-server avahi-daemon libnss-mdns iproute2 tmux vim nano curl
RUN rosdep update
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

ARG HOME_DIR=/home/$USERNAME
ARG ROS_MASTER_URI=http://jetbot.local:11311
ARG CLIENT_IP=192.168.8.131 # change to your host's ip

COPY EVC $HOME_DIR/EVC
# COPY talker.py $HOME_DIR/
# COPY listener.py $HOME_DIR/

RUN sudo chown -R $USERNAME:$USERNAME $HOME_DIR/EVC

RUN echo "export ROS_MASTER_URI=$ROS_MASTER_URI" >> ~/.bashrc
RUN echo "export ROS_IP=$CLIENT_IP" >> ~/.bashrc
#RUN echo "export DISPLAY=$CLIENT_IP:0" >> ~/.bashrc
RUN echo "export DISPLAY=:0" >> ~/.bashrc
