FROM ros:noetic
COPY requirements.txt .
RUN apt-get -y update && apt-get install -y python-is-python3 python3-pip && \
    pip install -r requirements.txt && rm requirements.txt && \
    echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc && \
    echo "source /root/Humanoid/catkin_ws/install/setup.bash" >> ~/.bashrc