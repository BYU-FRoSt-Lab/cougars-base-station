#!/bin/bash

# Ensure the required environment variable is set
if [ -z "$COUG_WORKSPACE_DIR" ]; then
    echo "Error: COUG_WORKSPACE_DIR is not set."
    exit 1
fi
source $COUG_WORKSPACE_DIR/cougars-ros2/install/setup.bash

# Prompt the user to select the vehicle ID
echo "Select a vehicle ID:"
echo "1) coug1"
echo "2) coug2"
echo "3) coug3"
echo "4) coug4"
echo "5) coug5"
read -p "Enter the number (1-5): " vehicle_option

# Map the selected option to a vehicle ID
case $vehicle_option in
    1) VEHICLE_ID="coug1" ;;
    2) VEHICLE_ID="coug2" ;;
    3) VEHICLE_ID="coug3" ;;
    4) VEHICLE_ID="coug4" ;;
    5) VEHICLE_ID="coug5" ;;
    *) echo "Invalid option. Exiting."; exit 1 ;;
esac

# Define the target directory
TARGET_DIR="$COUG_WORKSPACE_DIR/bag/$VEHICLE_ID"

# Check if the target directory exists
if [ ! -d "$TARGET_DIR" ]; then
    echo "Error: Directory $TARGET_DIR does not exist."
    exit 1
fi

# Find directories starting with "UL" and process them
cd $COUG_WORKSPACE_DIR/cougars-base-station/dataplot || { echo "Error: Unable to navigate to $COUG_WORKSPACE_DIR/cougars-base-station/dataplot"; exit 1; }

#TODO: THINK ABOUT PULLING IN THE REF LAT AND LON from the yaml file!!!

for dir in "$TARGET_DIR"/UL*; do
    if [ -d "$dir" ]; then
        # Check if a PNG file already exists in the directory
        if ls "$dir"/*.png 1> /dev/null 2>&1; then
            echo "Skipping $dir (PNG file already exists)."
            continue
        fi

        echo "Processing $dir..."

        python3 run.py -b -l ul "$dir"

        # Check if the output file exists and move it
        if [ -f "output.png" ]; then
            echo "Moving output.png to $dir..."
            mv output.png "$dir/"
        else
            echo "Warning: output.png not found for $dir."
        fi
    fi
done

