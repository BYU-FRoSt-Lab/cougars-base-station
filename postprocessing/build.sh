
# Run in docker container

cd ~/ros2_ws
colcon build --packages-select gps_msgs seatrac_interfaces frost_interfaces dvl_msgs
source ~/ros2_ws/install/setup.bash

cd ~/base_station/postprocessing
colcon build --packages-select data_reader