source /opt/ros/humble/setup.bash

mkdir -p ~/microros_ws/src
cd ~/microros_ws/src

git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git
cd ..

rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/local_setup.bash
