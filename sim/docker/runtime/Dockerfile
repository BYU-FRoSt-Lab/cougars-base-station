# Use the specified base image
FROM frostlab/holoocean:2.0-runtime-ubuntu22.04-cuda12.4-ros

# Source ROS setup script
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

USER root
RUN apt update && apt install -y --no-install-recommends \
    ros-humble-gps-msgs \
    ros-humble-geometry-msgs \
    ros-humble-nav-msgs \ 
    ros-humble-tf-transformations


# Install Dev tools for building and other tasks
RUN apt install -y python3-colcon-common-extensions \
    build-essential \
    cmake \
    python3-vcstool \
    sudo 

#### BELOW IS A SOLUTION TO THE TF TRANSFORMATIONS ISSUE
# Install pip
RUN apt update && apt install -y python3-pip
# Switch back to the ue4 user
USER ue4
# Upgrade this package to work with tf transformations and numpy 2.0
RUN pip install --upgrade transforms3d
#### END OF SOLUTION TO THE TF TRANSFORMATIONS ISSUE

RUN mkdir -p /home/ue4/sim_ws/src/holo-translator && \
    touch /home/ue4/sim_ws/setup.bash && \
    echo "source /opt/ros/humble/setup.bash && \
    source /home/ue4/coug_ros2/install/setup.bash && \
    source /home/ue4/ros2_ws/install/setup.bash && \
    source /home/ue4/sim_ws/install/setup.bash" >> /home/ue4/sim_ws/setup.bash

RUN echo "source /home/ue4/config/cougarsrc.sh" >> /home/ue4/.bashrc

USER ue4
WORKDIR /home/ue4/sim_ws
# Default command
CMD ["/bin/bash"]
