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

# Prompt the user to select the vehicle ID
echo "Select a vehicle ID:"
echo "1) coug1"
echo "2) coug2"
echo "3) coug3"
echo "4) coug4"
echo "5) coug5"
read -p "Enter the number (1-5): " vehicle_option

case $vehicle_option in
    1) VEHICLE_ID="coug1" ;;
    2) VEHICLE_ID="coug2" ;;
    3) VEHICLE_ID="coug3" ;;
    4) VEHICLE_ID="coug4" ;;
    5) VEHICLE_ID="coug5" ;;
    *) echo "Invalid option. Exiting."; exit 1 ;;
esac

# Variables
REMOTE_USER="frostlab"
REMOTE_FOLDER="/home/frostlab/CoUGARs/bag"
LOCAL_FOLDER="../../bag/$VEHICLE_ID"
SSH_PASSWORD="frostlab"  # Replace with your SSH password

# Ensure sshpass is installed
if ! command -v sshpass &> /dev/null; then
    printError "sshpass is not installed. Install it using your package manager."
    exit 1
fi

# Create the local folder if it doesn't exist
mkdir -p "$LOCAL_FOLDER"

# Run rsync with sshpass to avoid password prompt
sshpass -p "$SSH_PASSWORD" rsync -avz --progress \
    "${REMOTE_USER}@${VEHICLE_ID}.local:${REMOTE_FOLDER}/" "$LOCAL_FOLDER/"

# Check if rsync was successful
if [ $? -eq 0 ]; then
    printSuccess "Files copied successfully from ${VEHICLE_ID} to ${LOCAL_FOLDER}"
else
    printError "Failed to copy files from ${VEHICLE_ID}"
    exit 1
fi
