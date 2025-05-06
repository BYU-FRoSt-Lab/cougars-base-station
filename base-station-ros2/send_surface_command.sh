source install/setup.bash
ros2 service call "/emergency_surface_service" "base_station_interfaces/srv/BeaconId" "{beacon_id: $1}"