These scripts convert a `.waypoints` file generated from Ardupilot's [Mission Planner](https://ardupilot.org/planner/docs/mission-planner-installation.html#) into a mission file that one of our vehicles can use.  Use:


* wptconvertor-coug.py: Used for the COUGs.  The output file is a `.bhv` file that can read by the COUG's software.

The useage of the scripts is simple.  Once you have created a waypoint mission in Mission Planner, save it to a file, then move the resulting `.waypoints` file to the waypoint-files directory (cougars-base-station/waypoint-files)

```bash
python3 wptconvertor-coug.py filename.waypoints -o different_name
```
will output `different_name.bhv`.

You do not need to include the file extension when you specify a different output name.  However, if you do, the script will not include the extension twice (ie. putting in `-o foo.wpt` will not result in a file named `foo.wpt.wpt`).

If a file exists with the same name as the output, then the script will prompt you asking if you want to overwrite it.


The contents of this directory, including this README, have been copied/adapted by Matthew McMurray from code written by Tristan Hodgins. 

Tristan:
> Contact information
> Phone: (614) 927-7408
> Email: tristanahodgins@gmail.com
> Email: tah88@byu.edu

Matthew:
> Contact information
> Phone: (208) 914-3376
> Email: mmcmurray123@gmail.com
