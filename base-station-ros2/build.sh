
# USE THIS instead of "colcon build" to build the base-station

# base-station-ros2 uses packages from cougars-ros2
# So to build the package, we need to build and source from that repository
# This script is designed to run in the docker container

# build required packages in cougars-ros2 and source
cd ~/ros2_ws
colcon build --packages-select seatrac seatrac_interfaces 
source install/setup.bash

# 
