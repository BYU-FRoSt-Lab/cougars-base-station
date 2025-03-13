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




VEHICLE_ID=$(./select_vehicle.sh)


read -p "Enter the directory name the waypoint files are stored: " MISSION_NAME



python3 ../mission_processing/waypoint3D/process_missions.py "$VEHICLE_ID" "$MISSION_NAME"


# # Variables
REMOTE_USER="frostlab"
REMOTE_FOLDER="/home/frostlab/CoUGARs/nothing"
LOCAL_FOLDER="../mission_processing/waypoint3D/new_mission"
SSH_PASSWORD="frostlab"  # Replace with your SSH password

# Ensure sshpass is installed
if ! command -v sshpass &> /dev/null; then
    printError "sshpass is not installed. Install it using your package manager."
fi

# # Create the local folder if it doesn't exist
# mkdir -p "$LOCAL_FOLDER"

# Run rsync with sshpass to avoid password prompt
echo ''
sshpass -p "$SSH_PASSWORD" rsync -avz --progress \
   "$LOCAL_FOLDER/" "${REMOTE_USER}@coug0.local:${REMOTE_FOLDER}/" 

if [ $? -eq 0 ]; then
  printSuccess "Files copied successfully from local:${LOCAL_FOLDER} to coug0:${REMOTE_FOLDER}"
else
    printError "Failed to copy files from local:${LOCAL_FOLDER} to coug0:${REMOTE_FOLDER}"
fi
echo ''

sshpass -p "$SSH_PASSWORD" rsync -avz --progress \
   "$LOCAL_FOLDER/" "${REMOTE_USER}@coug1.local:${REMOTE_FOLDER}/" 
if [ $? -eq 0 ]; then
  printSuccess "Files copied successfully from local:${LOCAL_FOLDER} to coug1:${REMOTE_FOLDER}"
else
    printError "Failed to copy files from local:${LOCAL_FOLDER} to coug1:${REMOTE_FOLDER}"
fi
echo ''

sshpass -p "$SSH_PASSWORD" rsync -avz --progress \
   "$LOCAL_FOLDER/" "${REMOTE_USER}@coug2.local:${REMOTE_FOLDER}/" 
if [ $? -eq 0 ]; then
  printSuccess "Files copied successfully from local:${LOCAL_FOLDER} to coug2:${REMOTE_FOLDER}"
else
    printError "Failed to copy files from local:${LOCAL_FOLDER} to coug2:${REMOTE_FOLDER}"
fi
echo ''
sshpass -p "$SSH_PASSWORD" rsync -avz --progress \
   "$LOCAL_FOLDER/" "${REMOTE_USER}@coug3.local:${REMOTE_FOLDER}/" 
if [ $? -eq 0 ]; then
  printSuccess "Files copied successfully from local:${LOCAL_FOLDER} to coug3:${REMOTE_FOLDER}"
else
    printError "Failed to copy files from local:${LOCAL_FOLDER} to coug3:${REMOTE_FOLDER}"
fi
echo ''

sshpass -p "$SSH_PASSWORD" rsync -avz --progress \
   "$LOCAL_FOLDER/" "${REMOTE_USER}@coug4.local:${REMOTE_FOLDER}/" 

echo ''
if [ $? -eq 0 ]; then
  printSuccess "Files copied successfully from local:${LOCAL_FOLDER} to coug4:${REMOTE_FOLDER}"
else
    printError "Failed to copy files from local:${LOCAL_FOLDER} to coug4:${REMOTE_FOLDER}"
    
fi
echo ''
sshpass -p "$SSH_PASSWORD" rsync -avz --progress \
   "$LOCAL_FOLDER/" "${REMOTE_USER}@coug5.local:${REMOTE_FOLDER}/" 

if [ $? -eq 0 ]; then
  printSuccess "Files copied successfully from local:${LOCAL_FOLDER} to coug5:${REMOTE_FOLDER}"
else
    printError "Failed to copy files from local:${LOCAL_FOLDER} to coug5:${REMOTE_FOLDER}"
fi


# Check if rsync was successful


python3 ../mission_processing/waypoint3D/clear_new_mission.py "$VEHICLE_ID" "$MISSION_NAME"
