import numpy as np
import matplotlib.pyplot as plt
import pickle
import math
from plotter_utility import config, cordinatehandling
from plotter_utility.printing import update
from tqdm import tqdm
import os

#ROS BAG INTERPRETER
from plotter_utility.classes_ros import *
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

# TODO: Throw out data when heading rate is too high
# Interpolate between data points to understand potential overlap errors (not sure on best way to do this)

# The data doesn't always line up, so we need to find the data
# that is closest to the beam range timestamp
def get_closest_timestamp(arr, target):
    # Returns element closest to target in arr[]
    def findClosest(arr, n, target):
    
        # Corner cases
        if (target <= arr[0].timestamp):
            return arr[0]
        if (target >= arr[n - 1].timestamp):
            return arr[n - 1]
    
        # Doing binary search
        i = 0; j = n; mid = 0
        while (i < j):
            mid = (i + j) // 2
    
            if (arr[mid].timestamp == target):
                return arr[mid]
    
            # If target is less than array
            # element, then search in left
            if (target < arr[mid].timestamp) :
    
                # If target is greater than previous
                # to mid, return closest of two
                if (mid > 0 and target > arr[mid - 1].timestamp):
                    return getClosest(arr[mid - 1], arr[mid], target)
    
                # Repeat for left half
                j = mid
            
            # If target is greater than mid
            else :
                if (mid < n - 1 and target < arr[mid + 1].timestamp):
                    return getClosest(arr[mid], arr[mid + 1], target)
                    
                # update i
                i = mid + 1
            
        # Only single element left after search
        return arr[mid]
    
    
    # Method to compare which one value is closer.
    # We find the closest by taking the difference
    # between the target and both values. It assumes
    # that val2 is greater than val1 and target lies
    # between these two.
    def getClosest(val1, val2, target):
    
        if (target - val1.timestamp >= val2.timestamp - target):
            return val2
        else:
            return val1
    
    # Driver code
    return findClosest(arr, len(arr), target)


def read_bhv_file(file_path):
    """
    Reads in a .bhv file and returns a Python list of xy waypoints as tuples.
    
    Args:
        file_path (str): Path to the .bhv file.

    Returns:
        list[tuple]: A list of (x, y) waypoints as tuples.
    """
    waypoints = []

    with open(file_path, 'r') as file:
        for line in file:
            line = line.strip()
            if line.startswith("points ="):
                # Extract the points string
                points_str = line.split("=")[1].strip()
                # Split into individual point strings
                point_pairs = points_str.split(":")
                # Convert each point to a tuple of floats
                waypoints = [tuple(map(float, point.split(","))) for point in point_pairs]
            elif line.startswith("slip_radius"):
                slip_radius = points_str = line.split("=")[1].strip()
                slip_radius = float(slip_radius)
            elif line.startswith("capture_radius"):
                capture_radius = points_str = line.split("=")[1].strip()
                capture_radius = float(capture_radius)

    radii = (capture_radius, slip_radius)  
    return waypoints, radii


#TODO: make functions that parse each data item

def run(args):
    update('Starting interpretation', True)
    

    data_dict = {
        config.INTERPRETED_HEADING: [],
        config.INTERPRETED_IMU: [],
        config.INTERPRETED_BEAM_RANGE: [],
        config.INTERPRETED_BEAM_INTENSITY: [],
        config.INTERPRETED_GPS: [],
        config.INTERPRETED_GPS_ODOM: [],
        config.INTERPRETED_FACTOR: [],
    }

    if args.folder == 'null':
        update('ERROR: A valid rosbag file is required when running the interpreter', True)
        quit(2)

    file = os.path.basename(args.folder.rstrip("/"))
    db3_path = args.folder + '/'+ file + "_0.db3"
    update('Opening rosbag file: ' + db3_path)

    try:
        # Set up the rosbag2 reader
        storage_options = rosbag2_py.StorageOptions(uri=db3_path, storage_id='sqlite3')
        converter_options = rosbag2_py.ConverterOptions('', '')
        reader = rosbag2_py.SequentialReader()
        reader.open(storage_options, converter_options)

        # Get the list of topics and types from the rosbag metadata
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}

        # Iterate through messages in the rosbag
        update('Iterating over rosbag file:', True)
        while reader.has_next():
            (topic, msg, t) = reader.read_next()

            # Deserialize the message
            msg_type = get_message(type_map[topic])
            deserialized_msg = deserialize_message(msg, msg_type)

            # Process messages based on the topic name
            if topic.endswith('/gps_odom'):
                data_dict[config.INTERPRETED_GPS_ODOM].append(ODOM(deserialized_msg))
            elif topic.endswith('/fix'):
                data_dict[config.INTERPRETED_GPS].append(GPS_FIX(deserialized_msg))
            elif topic.endswith('/smoothed_output'):
                data_dict[config.INTERPRETED_FACTOR].append(ODOM(deserialized_msg))
            # elif topic.endswith('/fix'):
            #     data_dict[config.INTERPRETED_GPS].append(GPS_FIX(deserialized_msg))
    except Exception as e:
        update(f"ERROR: Failed to read rosbag: {str(e)}", True)
        quit(2)
    
    output_dict = {}

    update('Finding the reference point for the mapper')
    # reference point (lat and long) in degrees. It is the median point.

    if args.location == 'ul':
        #UTAH LAKE LOCATION
        ref_lat = 40.23844451917827
        ref_lon = -111.73952716770287
    elif args.location == 'byu':
        # BYU in front of the MARB
        ref_lat = 40.2466104
        ref_lon = -111.6488273
    else:
        ref_lat, ref_lon = cordinatehandling.FindCenter([curr.gps_lat for curr in data_dict[config.INTERPRETED_GPS]], [curr.gps_long for curr in data_dict[config.INTERPRETED_GPS]])


    output_dict[config.PROCESSED_CENTER] = np.array([ref_lat, ref_lon])

    # convert reference point to radians
    # DO WE USE THIS?
    update('Converting reference cords to radians')
    ref_lat_rad = math.radians(ref_lat)
    ref_lon_rad = math.radians(ref_lon)

    # list of GPS coordinates (latitude, longitude)
    # convert GPS coordinates to xy coordinates
    update('Initalizing coordinate tables')
    local_coords = []
    bathymetry = []
    gps_odom = []
    factor = []

    #TODO: Do something here for depth map
    depth = []
    altitude = []

    data_reduction = args.samplereduction

    update("Processing data:", True)
    # Iterate over the data
    for i in range(len(data_dict[config.INTERPRETED_GPS])):
        lat = data_dict[config.INTERPRETED_GPS][i].gps_lat
        long = data_dict[config.INTERPRETED_GPS][i].gps_long
        elevation = data_dict[config.INTERPRETED_GPS][i].elevation

        # Use NumPy to check for NaNs
        if np.isnan(lat) or np.isnan(long) or np.isnan(elevation):
            continue

        x, y = cordinatehandling.CalculateHaversine(ref_lat, ref_lon, lat, long)
        z = elevation
        local_coords.append((x, y, z))
    
    # gps odom values to list
    for i in range(len(data_dict[config.INTERPRETED_GPS_ODOM])):
        x = data_dict[config.INTERPRETED_GPS_ODOM][i].x
        y = data_dict[config.INTERPRETED_GPS_ODOM][i].y
        z = data_dict[config.INTERPRETED_GPS_ODOM][i].z
        gps_odom.append((x, y, z))
    
    # factor values to list
    for i in range(len(data_dict[config.INTERPRETED_FACTOR])):
        x = data_dict[config.INTERPRETED_FACTOR][i].x
        y = data_dict[config.INTERPRETED_FACTOR][i].y
        z = data_dict[config.INTERPRETED_FACTOR][i].z
        factor.append((x, y, z))

    #TODO print out what percent of gps data was thrown out due to nan


    # plot bathymetry in 3D
    bathymetry = np.array(bathymetry)
    path = np.array(local_coords)
    odom = np.array(gps_odom)
    factor_array = np.array(factor)
    
    waypoints  = []

    if args.behavior:
        # Call your function to process the file
        bhv_file = args.folder + '/coug.bhv'
        waypoints, radii = read_bhv_file(bhv_file)
        wapoints_array = np.array(waypoints)
        output_dict[config.PROCESSED_WPT] = wapoints_array
        output_dict[config.PROCESSED_RAD] = radii
    else:
        update("No .bhv file provided. Please specify a file with the -b option.")

    output_dict[config.PROCESSED_BATHYMETRY] = bathymetry
    output_dict[config.PROCESSED_PATH] = path
    output_dict[config.PROCESSED_GPS_ODOM] = odom
    output_dict[config.PROCESSED_FACTOR] = factor_array

    # Pickle bathymetry data so it can be used elsewhere
    with open(config.PROCESSOR_OUTPUT, 'wb') as file:
        pickle.dump(output_dict, file)
    