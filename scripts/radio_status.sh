#!/bin/bash

# Check which directory the script is being run from
if [ "$PWD" = "/home/bja1701/Code/CoUGARs/cougars-base-station/scripts/" ]; then
    cd /home/bja1701/Code/CoUGARs/cougars-base-station/base_station_radio/
else
    cd /home/frostlab/base_station/base_station_radio/
fi

# Execute the radio_status.py Python file
python3 radio_status.py