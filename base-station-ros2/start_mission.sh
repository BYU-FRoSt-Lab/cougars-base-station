
source install/setup.bash

# user inputs target vehicle
echo ""
echo "Enter Target Vehicle (0 for all vehicles):"
read beacon_id

echo ""
echo "Enter a folder name for the rosbag:"
read folder

# makes a service call to the base station to start mission on a specified vehicle
ros2 service call "/init_coug_service" "base_station_interfaces/srv/BeaconId" "{beacon_id: $beacon_id}"