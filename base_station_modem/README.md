

# Base Station Modem

The Base Station Modem is connected to a surface computer and can be used to manage the CoUGARs UV 
fleet while underwater.


## Disabling the coug Thruster

To disable the coug thruster from the command line

1. Build the topside_modem application.  While you have an internet connection, open the 
    'topside_modem' folder in the terminal and run `bash build.sh`.  You only need to do this step once.
2. Move out of the topside_modem folder into the base_station_modem folder and run 
    `bash send_kill_command.sh <target_id> <serial_port>`. This command takes two arguements: 
    <target_id>, the id of the beacon to emergency stop (default 0), and <serial_port>, the serial 
    port of the base station modem (default "/dev/ttyUSB0"). If <target_id> is 0, then the modem 
    will disable the thrusters of all the deployed vehicles.




## Manually Changing Settings and calibrating Modem

To change settings such as the beacon id on the modem

1. Build the seatrac_setup_tool application.  While you have an internet connection, open the 
    'seatrac_setup_tool' folder in the termanl and run `bash build.sh`.  You only need to do this step once.
2. Run `bash run.sh` in the 'seatrac_setup_tool' folder. This should open a terminal application. An 
    example using the application to change the beacon id is shown here
    
    ```
    cougars-base-station/base_station_modem/seatrac_setup_tool$ bash ./run.sh
    === Seatrac Beacon Setup Tool ===

    Enter Serial Port (or blank for default '/dev/ttyUSB0'): /dev/ttyUSB0
    Connecting to Beacon... Done
    View current settings (y/n)? n
    How would you like to enter beacon settings?
    'm': manual,  'c': config file,  's': skip to calibration  (m/c/s)? m
    Current Beacon Id: 15
    Change Beacon Id (y/n)? y
    Enter New Beacon Id (integer between 1 and 15 inclusive): 1
    Setting Beacon Id to 1... done

    Current Water Salinity Setting: 0 ppt
    Fresh water has salinity of 0 ppt. Salt water has salinity of 35 ppt.
    Change Water Salinity Setting (y/n)? n
    View and modify serial report settings (y/n)? n
    View and modify transciever and sensor settings (y/n)? n
    Manual Settings upload complete.

    Calibrate Magnetometer (y/n)? n
    Calibrate Accelerometer (y/n)? n
    Beacon setup complete
    Review changes to settings (y/n)? n
    Save Settings to permanent EEPROM memory? 
    If you chose not to, settings will still be saved in beacon ram. (y/n)? y
    Saving Settings... done

    Beacon setup complete
    Closing connection

    Setup another beacon (y/n)? n
    ```

