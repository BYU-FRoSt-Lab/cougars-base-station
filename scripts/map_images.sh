#!/bin/bash

# Source the required setup file
source $COUG_WORKSPACE_DIR/cougars-ros2/install/setup.bash

VEHICLE_ID=$(./select_vehicle.sh)

# Run the Python script
DIR="$COUG_WORKSPACE_DIR/bag/$VEHICLE_ID"
cd $COUG_WORKSPACE_DIR/cougars-base-station/dataplot
python3 map_images.py "$DIR"