
# CoUGARs UV Mission Processes

Processes to start and run multi-agent CoUGARs UV missions from the backend perspective

---

## Start Mission Process

1. Base Station Startup / CoUGARs Startup:
    * Secure wifi connection to each vehicle
    * ROS sensors and cougars_coms launch
    * Verify sensors (gps, voltage, modem connection, dvl connection, leak, etc.)

2. Create Mission Button:  Convert Ardupilot waypoints into moos mission files

3. Load Mission Button:  Copy moos mission files onto each vehicle

4. Put vehicles in water

5. Start Mission Button:
    1. Verify
        *  Base Station sends VERIFY to each vehicle individually 
        *  Turn on dvl acoustics
        *  ROS mission launch (factor_graph, converters, etc.)
        *  Verify sensors (gps, voltage, modem, dvl data, leak, etc.)
        *  Vehicle responds with CONFIRM_VERIFY
    2. Launch
        *  Base Station sends LAUNCH to each vehicle individually with `start_time`
        *  Vehicles respond with CONFIRM_LAUNCH and wait for `start_time`
        *  At `start_time`: arm thruster and start mission

---

## Modem Coms Schedule

Base Station talks to each vehicle one at a time in order coug1, coug2, etc.
Each vehicle gets 4 seconds to talk.

On vehicle turn:

1. Base Station sends REQUEST_STATUS
2. Vehicle modem sends first response with range to base station
3. Vehicle sends second response with VEHICLE_STATUS message
4. Base Station logs response and updates gui

Even though VEHICLE_STATUS messages are addressed to the Base Station, all vehicles
will recieve and record that message.

#### Contents of Vehicle Status

Length: 13 bytes

* VEHICLE_STATUS message header (1 byte)
* Vehicle status flags (1 byte)
    * vehicle state (0 for nominal, 1 for error)
    * gps on
    * dvl on
    * voltage good
    * state estimate good
* Next Waypoint (1 byte)
* Timestamp (2 bytes)
* State Estimate (8 bytes)
    * X position (2 bytes)
    * Y position (2 bytes)
    * Depth (2 bytes)
    * Heading (2 bytes)

---

## Other Mission Processes

1. Emergency Kill Thruster Command (already implemented):
    Modem command to disable thruster on vehicle

2. Return To Home Command:
    Commands vehicles to abort the mission and return back for retrieval

3. Surface Command:
    Commands vehicles to abort mission, turn fins up, and swim to surface.

anything else?









