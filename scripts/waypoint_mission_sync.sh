#!/bin/bash
#
# Waypoint Mission Sync Script
#
# This script provides a menu to select a vehicle and then a menu to select
# waypoint mission files to copy to that vehicle. It handles finding the correct
# remote directory and uses rsync for efficient file transfer.

# --- Helper Functions for Colored Output ---

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

# --- Configuration ---

# Define vehicles and their corresponding IP address suffixes
# Associative array: vehicle_name -> ip_suffix
declare -A VEHICLES=(
    ["coug1"]="101"
    ["coug2"]="102"
    ["coug3"]="103"
    ["coug4"]="104"
    ["coug5"]="105"
)

REMOTE_USER="frostlab"
SSH_PASSWORD="frostlab" # The password for the remote user
IP_BASE="192.168.0."

# Source directory for waypoint mission files.
# The script expects to be run from a directory where this relative path is valid.
SOURCE_DIR="../waypoint_planner/waypoint_missions"

# --- Pre-run Checks ---

# Ensure sshpass is installed
if ! command -v sshpass &> /dev/null; then
    printError "sshpass is not installed. Please install it first."
    printError "On Debian/Ubuntu, run: sudo apt install sshpass"
    exit 1
fi

# --- Main Script Logic ---

# 1. Select Vehicle
# Create a numerically sorted indexed array from the associative array keys for the menu.
# This prevents the unordered behavior of associative arrays.
mapfile -t VEHICLE_NAMES < <(printf "%s\n" "${!VEHICLES[@]}" | sort -V)

printInfo "Please select a vehicle to sync files with:"
select VEHICLE_NAME in "${VEHICLE_NAMES[@]}"; do
    if [[ -n "$VEHICLE_NAME" ]]; then
        VEHICLE_IP="${IP_BASE}${VEHICLES[$VEHICLE_NAME]}"
        printInfo "You selected ${VEHICLE_NAME} (${VEHICLE_IP})"
        break
    else
        printWarning "Invalid selection. Please try again."
    fi
done

if [[ -z "$VEHICLE_IP" ]]; then
    printError "No vehicle selected. Exiting."
    exit 1
fi

# 2. Check for Source Directory
if [ ! -d "$SOURCE_DIR" ]; then
    printError "Source directory not found at: $(realpath "$SOURCE_DIR")"
    printError "Please run this script from the correct directory."
    exit 1
fi

# 3. Select Files
# Find all files in the source directory
mapfile -t files < <(find "$SOURCE_DIR" -maxdepth 1 -type f | sort)

if [ ${#files[@]} -eq 0 ]; then
    printError "No mission filthe rest of the commands.



bash script directory:

~/Code/CoUGARs/cougars-base-station/scripts$ 



files to copy directory:

es found in $SOURCE_DIR"
    exit 1
fi

printInfo "Select files to copy to ${VEHICLE_NAME}:"
printInfo "(enter numbers separated by spaces, e.g., '1 3 4', or 'a' for all)"

# Display numbered list of files
for i in "${!files[@]}"; do
  echo "  $((i+1))) $(basename "${files[$i]}")"
done

read -rp "Your choice(s): " choices

SELECTED_FILES=()
if [[ "$choices" == "a" || "$choices" == "A" ]]; then
    printInfo "Selecting all files."
    SELECTED_FILES=("${files[@]}")
else
    for choice in $choices; do
        # Check if the choice is a valid number within the range
        if [[ "$choice" =~ ^[0-9]+$ ]] && [ "$choice" -ge 1 ] && [ "$choice" -le ${#files[@]} ]; then
            # Adjust for 0-based array index
            SELECTED_FILES+=("${files[$((choice-1))]}")
        else
            printWarning "Ignoring invalid choice: $choice"
        fi
    done
fi

if [ ${#SELECTED_FILES[@]} -eq 0 ]; then
    printError "No valid files selected. Exiting."
    exit 1
fi

echo # Newline for readability
printInfo "Files to be copied:"
for file in "${SELECTED_FILES[@]}"; do
    echo "  - $(basename "$file")"
done
echo # Newline

# 4. Determine Remote Destination Directory
printInfo "Checking remote directory structure on ${VEHICLE_IP}..."
REMOTE_DEST_BASE_PATH="/home/${REMOTE_USER}"
REMOTE_DEST_SUB_PATH="/cougars-ros2/src/cougars_control/waypoint_missions/"
REMOTE_DEST_DIR=""

# Use sshpass with ssh to test for the directory. A 5-second timeout prevents long hangs.
# We check for the 'CoUGARS' directory first.
if sshpass -p "$SSH_PASSWORD" ssh -o ConnectTimeout=5 "${REMOTE_USER}@${VEHICLE_IP}" "[ -d ${REMOTE_DEST_BASE_PATH}/CoUGARS ]"; then
    REMOTE_DEST_DIR="${REMOTE_DEST_BASE_PATH}/CoUGARS${REMOTE_DEST_SUB_PATH}"
    printInfo "Found 'CoUGARS' base directory."
# Then check for the 'cougars' directory.
elif sshpass -p "$SSH_PASSWORD" ssh -o ConnectTimeout=5 "${REMOTE_USER}@${VEHICLE_IP}" "[ -d ${REMOTE_DEST_BASE_PATH}/cougars ]"; then
    REMOTE_DEST_DIR="${REMOTE_DEST_BASE_PATH}/cougars${REMOTE_DEST_SUB_PATH}"
    printInfo "Found 'cougars' base directory."
else
    # If neither exists, default to 'CoUGARS' and let rsync create it.
    printWarning "Neither 'CoUGARS' nor 'cougars' directory found on remote."
    REMOTE_DEST_DIR="${REMOTE_DEST_BASE_PATH}/CoUGARS${REMOTE_DEST_SUB_PATH}"
    printInfo "Will attempt to create and use: $REMOTE_DEST_DIR"
fi


# 5. Execute File Transfer with rsync
printInfo "Starting file transfer to ${VEHICLE_NAME}..."

# Prepend sshpass to the rsync command to handle password authentication automatically.
# The --rsync-path option ensures the full remote directory path is created before copying.
sshpass -p "$SSH_PASSWORD" rsync -avz --progress \
      --rsync-path="mkdir -p ${REMOTE_DEST_DIR} && rsync" \
      "${SELECTED_FILES[@]}" \
      "${REMOTE_USER}@${VEHICLE_IP}:${REMOTE_DEST_DIR}"

# Check if rsync was successful
if [ $? -eq 0 ]; then
    printSuccess "Files copied successfully to ${VEHICLE_NAME} at ${REMOTE_DEST_DIR}"
else
    printError "Failed to copy files to ${VEHICLE_NAME}"
    exit 1
fi
