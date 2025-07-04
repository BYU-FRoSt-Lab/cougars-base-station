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
if [[ ! $1 =~ ^[0-9]+$ ]]; then
  VEHICLE_ID=$(./select_vehicle.sh)
else
  VEHICLE_ID="coug$1"
fi

printWarning "Make sure you are running this inside the container"
printWarning "TODO create setup ssh keys script -Braden"

# Extract the last character of VEHICLE_ID and use it for IP_ADDRESS
VEHICLE_SUFFIX=${VEHICLE_ID: -1}
IP_ADDRESS="192.168.0.10$VEHICLE_SUFFIX"

printInfo "Using IP address: $IP_ADDRESS"
# Variables
REMOTE_USER="frostlab"
REMOTE_FOLDER="/home/frostlab/cougars/bag"
LOCAL_FOLDER="$HOME/bag/$VEHICLE_ID"

# TODO add a setup ssh script to setup ssh keys

# SSH_PASSWORD="frostlab"  # Replace with your SSH password

# # Ensure sshpass is installed
# if ! command -v sshpass &> /dev/null; then
#     printError "sshpass is not installed. Install it using your package manager."
#     exit 1
# fi

# Create the local folder if it doesn't exist
mkdir -p "$LOCAL_FOLDER"

# Run rsync with sshpass to avoid password prompt
rsync -avz --progress \
    "${REMOTE_USER}@$IP_ADDRESS:${REMOTE_FOLDER}/" "$LOCAL_FOLDER/"

# # Check if rsync was successful
# if [ $? -eq 0 ]; then
#     printSuccess "Files copied successfully from ${VEHICLE_ID} to ${LOCAL_FOLDER}"
# else
#     printError "Failed to copy files from ${VEHICLE_ID}"
#     exit 1
# fi
