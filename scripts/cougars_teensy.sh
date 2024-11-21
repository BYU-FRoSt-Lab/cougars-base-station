#!/bin/bash

function printInfo {
  echo -e "\033[0m\033[36m[INFO] $1\033[0m"
}

function printWarning {
  echo -e "\033[0m\033[33m[WARNING] $1\033[0m"
}

function printError {
  echo -e "\033[0m\033[31m[ERROR] $1\033[0m"
}

function printSuccess {
  echo -e "\033[0m\033[32m[SUCCESS] $1\033[0m"
}

function printFailure {
  echo -e "\033[0m\033[31m[FAIL] $1\033[0m"
}

SSH_PASSWORD="frostlab"

# Function for "build"
build_local() {
    echo "Building cougars teensy project inside the Docker container..."
    CONTAINER_NAME="cougars"
    COMMAND="bash /home/frostlab/teensy_ws/build.sh"

    # Run the command inside the Docker container
    if docker ps | grep -q "$CONTAINER_NAME"; then
        docker exec "$CONTAINER_NAME" $COMMAND
        echo "Cougars Build completed inside the container."
    else
        echo "Error: Container '$CONTAINER_NAME' is not running."
    fi
}

# Function for "upload"
upload() {
    local firmware_path="$COUG_WORKSPACE_DIR/cougars-teensy/cougars/.pio/build/teensy41/firmware.hex"
    local remote_device=""
    local remote_username="frostlab"
    local remote_folder="/home/frostlab/CoUGARs/cougars-teensy/firmware_options"
    local remote_file_name=""
    
    # Check if firmware exists
    if [[ ! -f "$firmware_path" ]]; then
        echo "Error: Firmware file not found at $firmware_path."
        return 1
    fi

    # Prompt for the device name
    read -p "Enter the device name (e.g., coug1): " remote_device
    if [[ -z "$remote_device" ]]; then
        echo "Error: Device name cannot be empty."
        return 1
    fi
    remote_device="${remote_device}.local"

    # Prompt for the name to save the firmware as on the remote device
    read -p "Enter the name for the firmware file (without extension): " remote_file_name
    if [[ -z "$remote_file_name" ]]; then
        echo "Error: File name cannot be empty."
        return 1
    fi
    remote_file_name="${remote_file_name}.hex"

    # Copy the firmware file to the remote device
    printInfo "Copying firmware to $remote_device:$remote_folder/$remote_file_name..."
    sshpass -p "$SSH_PASSWORD" scp "$firmware_path" "${remote_username}@${remote_device}:${remote_folder}/${remote_file_name}"
    if [[ $? -ne 0 ]]; then
        echo "Error: Failed to copy the firmware file."
        return 1
    fi
    printInfo "Firmware successfully copied."

    # Execute the command on the remote device
    printInfo "Uploading firmware on the remote device using Docker..."
    sshpass -p "$SSH_PASSWORD" ssh "${remote_username}@${remote_device}" "docker exec cougars bash /home/frostlab/teensy_ws/upload.sh ${remote_file_name}"
    if [[ $? -ne 0 ]]; then
        printError "Failed to execute the upload command on the remote device."
        return 1
    fi
    printSuccess "Firmware upload completed successfully."
}

if ! command -v sshpass &> /dev/null; then
    printError "sshpass is not installed. Install it using your package manager."
    exit 1
fi

# Main menu loop
while true; do
      echo "
      ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
      ><(((°>                           ><(((°>
               __|__
           ---@--@---
        ___/_______\___
       |               |
       |  FROSTLAB     |            ><(((°>
       |               |
       |_______________|
          \  1) Build Local
           \  2) Upload
            \  3) Build Local and Upload
             \  4) Quit
              \_____________
                 ~~~~~~~~~~~~
                ~~~~~~~~~~~~~~
               ~~~~~~~~~~~~~~~~
"
    # echo "1) Build Local"
    # echo "2) Upload"
    # echo "3) Build Local and Upload"
    # echo "4) Quit"
    echo "=========================="
    read -p "Select an option (1-4): " choice

    case $choice in
        1)
            build_local
            ;;
        2)
            upload
            ;;
        3)
            build_local
            upload
            ;;
        4)
            printInfo "Quitting the program. Goodbye!"
            break
            ;;
        *)
            printError "Invalid option. Please try again."
            ;;
    esac
    echo ""
done