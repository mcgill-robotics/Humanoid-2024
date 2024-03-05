FROM ros:noetic-ros-base
COPY requirements.txt .
RUN apt-get -y update && apt-get install -y python-is-python3 python3-pip && \
    apt-get install -y ros-noetic-tf && \
    apt-get install -y ros-noetic-tf2-geometry-msgs && \
    pip install -r requirements.txt && rm requirements.txt && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    apt-get install -y python3-catkin-tools python3-pip python-is-python3 && \
    apt-get install -y dos2unix && \
    echo "source /root/Humanoid/catkin_ws/devel/setup.bash" >> ~/.bashrc && \
    apt-get install ros-noetic-rosserial-arduino && \
    apt-get install ros-noetic-rosserial