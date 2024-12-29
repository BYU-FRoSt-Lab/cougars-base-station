#!/bin/bash
#
# Starts or enters the tmux session
# - Use 'bash tmux.sh kill' to kill the session

source ../../config/bash_vars.sh

function printInfo {
  echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
  echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
  echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

cd ../sim-packages/docker
bash setup_container.sh


case $1 in
  "kill")
    printWarning "Killing the tmux session..."
    tmux kill-session -t holoocean
    ;;
  *)
    # Check if the tmux session is already running
    if [ -z "$(tmux list-sessions | grep holoocean)" ]; then

      printInfo "Starting the tmux session..."

      ### FIRST WINDOW - ROS SCRIPTS ###

      # Start the tmux session
      tmux new-session -d -s holoocean -n "coug"
      tmux split-window -h -t holoocean
      tmux select-pane -t holoocean:coug.0

      tmux send-keys -t holoocean:coug.0 "docker exec -it holoocean bash" ENTER
      tmux send-keys -t holoocean:coug.0 "clear" ENTER
      tmux send-keys -t holoocean:coug.1 "docker exec -it holoocean bash" ENTER
      tmux send-keys -t holoocean:coug.1 "clear" ENTER

      tmux send-keys -t holoocean:coug.0 "cd ~/ros2_ws" ENTER
      sleep 1
      tmux send-keys -t holoocean:coug.0 "source install/setup.bash" ENTER
      sleep 1
      tmux send-keys -t holoocean:coug.0 "ros2 launch reverse_converters full_launch.py" # Don't start just yet

      tmux send-keys -t holoocean:coug.1 "source ~/ros2_ws/install/setup.bash" ENTER
      tmux send-keys -t holoocean:coug.1 "cd ~/ros2_ws/src/holoocean-ros/holoocean_main/config" ENTER
      tmux send-keys -t holoocean:coug.1 "cat config.yaml" ENTER # Don't start just yet

    else
      printInfo "Attaching to the tmux session..."
    fi

    # Attach to the tmux session
    tmux attach-session -t holoocean
    ;;
esac
