
source install/setup.bash

ros2 service call "/init_coug_service" "base_station_interfaces/srv/BeaconId" "{beacon_id: $1}"