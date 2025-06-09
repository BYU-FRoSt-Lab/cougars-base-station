source install/setup.sh

while true; do
  vehicle_id=$((1 + RANDOM % 4))
  dvl_running=$([ $((RANDOM % 2)) -eq 0 ] && echo true || echo false)
  gps_connection=$([ $((RANDOM % 2)) -eq 0 ] && echo true || echo false)
  leak_detection=$([ $((RANDOM % 2)) -eq 0 ] && echo true || echo false)
  ros2 topic pub -1 /status base_station_interfaces/msg/Status "{
    vehicle_id: $vehicle_id,
    x: $((RANDOM % 100)),
    y: $((RANDOM % 100)),
    depth: $((RANDOM % 20)),
    heading: $((RANDOM % 256)),
    waypoint: $((RANDOM % 10)),
    dvl_vel: $((RANDOM % 20)),
    battery_voltage: $((RANDOM % 100)),
    dvl_running: $dvl_running,
    gps_connection: $gps_connection,
    leak_detection: $leak_detection
  }"

  c1=$([ $((RANDOM % 2)) -eq 0 ] && echo true || echo false)
  c2=$([ $((RANDOM % 2)) -eq 0 ] && echo true || echo false)
  c3=$([ $((RANDOM % 2)) -eq 0 ] && echo true || echo false)
  c4=$([ $((RANDOM % 2)) -eq 0 ] && echo true || echo false)
  p1=$((RANDOM % 31))
  p2=$((RANDOM % 31))
  p3=$((RANDOM % 31))
  p4=$((RANDOM % 31))
  ct=$((RANDOM % 2))
  ros2 topic pub -1 /connections base_station_interfaces/msg/Connections "{
    connection_type: $ct,
    connections: [$c1, $c2, $c3, $c4],
    last_ping: [$p1, $p2, $p3, $p4]
  }"
  bool1=$([ $((RANDOM % 2)) -eq 0 ] && echo true || echo false)
  bool2=$([ $((RANDOM % 2)) -eq 0 ] && echo true || echo false)
  ros2 topic pub -1 confirm_e_surface std_msgs/msg/Bool "{data: $bool1}"
  ros2 topic pub -1 confirm_e_kill std_msgs/msg/Bool "{data: $bool2}"
done