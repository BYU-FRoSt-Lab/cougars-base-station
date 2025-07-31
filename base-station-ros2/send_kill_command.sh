

source install/setup.bash


# user inputs the target vehicle
echo ""
echo "Enter Target Vehicle (0 for all vehicles):"
read beacon_id


# makes a service call to the base station to send an emergency kill command to specified vehicle
ros2 service call "/e_kill_service" "base_station_interfaces/srv/BeaconId" "{beacon_id: $beacon_id}"







