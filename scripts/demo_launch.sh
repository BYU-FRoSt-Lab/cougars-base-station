#!/bin/bash

if [[ ! $1 =~ ^[0-9]+$ ]]; then
    # Run your command here
    echo "Running the command because the first argument is not a number"
    VEHICLE_ID=$(./select_vehicle.sh)
else
    echo "First argument is a number, not running the command."
    VEHICLE_ID="coug$1"
fi

tmux new-session -d -s cougars -n "demo"

tmux send-keys "bash coug_ssh.sh $VEHICLE_ID" Enter

tmux send-keys "cd CoUGARs" Enter

tmux send-keys "bash compose.sh" Enter
tmux send-keys "cd ros2_ws" Enter

tmux send-keys "bash launch.sh demo" Enter
tmux send-keys "" Enter
tmux attach-session -t cougars
