FROM ros:humble-ros-base

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /root/Humanoid/ros2_ws/install/setup.bash && source /root/Humanoid/ros2_ws/install/local_setup.bash" >> ~/.bashrc