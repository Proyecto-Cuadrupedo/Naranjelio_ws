FROM ros:humble


# Example of installing programs
RUN apt-get update \
    && apt-get install -y \ 
    nano \
    python3-pip \
    python3-dev \
    i2c-tools \
    && rm -rf /var/lib/apt/lists/*


# Create a non-root user
ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID

RUN pip3 install numpy pandas requests Adafruit_PCA9685 smbus

RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  && mkdir /home/$USERNAME/.config && chown $USER_UID:$USER_GID /home/$USERNAME/.config


# Set up sudo
RUN apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  && rm -rf /var/lib/apt/lists/*


# Copy the entrypoint and bashrc scripts so we have 
# our container's environment set up correctly
COPY entrypoint.sh /entrypoint.sh
COPY bashrc /home/${USERNAME}/.bashrc


# Set up entrypoint and default command
ENTRYPOINT ["/bin/bash", "/entrypoint.sh"]
CMD ["bash"]