read -p "Do you want to build the workspace? (y/n) : " build


if [[ "$build" == "y" || "$build" == "Y" ]]; then
    docker exec -it holoocean bash -c "source /opt/ros/humble/setup.bash && cd ~/ros2_ws && colcon build"
fi

docker exec -it holoocean bash -c "source ~/ros2_ws/install/setup.bash && ros2 launch reverse_converters full_launch.py"