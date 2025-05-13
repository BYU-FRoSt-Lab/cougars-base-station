
source install/setup.bash

ros2 service call "/emergency_kill_service" "std_srvs/srv/SetBool" 