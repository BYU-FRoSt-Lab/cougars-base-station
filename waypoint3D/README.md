These scripts convert a `.waypoints` file generated from Ardupilot's [Mission Planner](https://ardupilot.org/planner/docs/mission-planner-installation.html#) into a mission file that one of our vehicles can use.  There are two python scripts currently:

1. wptconvertor-wamv.py: Used for the WAM-V.  The output file is a `.wpt` file that can be imported to the WAM-V's waypoint planner
2. wptconvertor-coug.py: Used for the COUGs.  The output file is a `.bhv` file that can read by the COUG's software.

The useage of the scripts is simple.  Once you have created a waypoint mission in Mission Planner, save it to a file, then move the resulting `.waypoints` file to the same directory as the script you wish to use.  Then, run: 

```bash
python3 wptconvertor-wamv.py file_name.waypoints
```
The line above will output `file_name.wpt`.

The output have the same filename as the input unless specified by -o.  For example,

```bash
python3 wptconvertor-coug.py filename.waypoints -o different_name
```
will output `different_name.bhv`.

You do not need to include the file extension when you specify a different output name.  However, if you do, the script will not include the extension twice (ie. putting in `-o foo.wpt` will not result in a file named `foo.wpt.wpt`).

If a file exists with the same name as the output, then the script will prompt you asking if you want to overwrite it.

Also, if you would like to include a screenshot of your mission from ardupilot, just put it in the main directory with the template files, and you will be prompted to enter it in, and it will be copied over to the docker container.

Questions about this script may be directed to Tristan Hodgins.
> Contact information
> Phone: (614) 927-7408
> Email: tristanahodgins@gmail.com
> Email: tah88@byu.edu
