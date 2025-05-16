
source install/setup.bash

# user inputs target vehicle
echo ""
echo "Enter Target Vehicle (0 for all vehicles):"
read beacon_id

echo ""
echo "Would you like to record? (Enter On or Off): "
read record

echo ""
echo "Enter a folder name for the rosbag:"
read folder

# makes a service call to the base station to start mission on a specified vehicle
ros2 service call "/start_mission_service" "base_station_interfaces/srv/StartMission" "{beacon_id: $beacon_id, record: $record, folder: $folder}"