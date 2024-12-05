import googlemaps
# from googlemaps.maps import static_map
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image  as pltimg
from matplotlib.collections import LineCollection
from matplotlib.colors import Normalize
from matplotlib.patches import Circle
from datetime import datetime
import pickle
from tqdm import tqdm

from plotter_utility import config, cordinatehandling
from plotter_utility.printing import update


def plot_waypoints(waypoints, radii, ax):
    x_waypoints = waypoints[:, 0]
    y_waypoints = waypoints[:, 1]

    capture_radius = radii[0]
    slip_radius = radii[1]
    # Plot the waypoints
    ax.scatter(x_waypoints, y_waypoints, color='black', marker='o', label='Waypoints')

    # Add circles at each waypoint
    for x, y in zip(x_waypoints, y_waypoints):
        # Green inner circle with radius 5
        inner_circle = Circle((x, y), radius=capture_radius, color='green', fill=True, alpha=0.5)
        ax.add_patch(inner_circle)
        
        # Red outer circle with radius 10
        outer_circle = Circle((x, y), radius=slip_radius, color='red', fill=False, linewidth=2)
        ax.add_patch(outer_circle)

def add_colored_line(ax, x, y, z, colormap):
        """Adds a colored line to the axis based on z-values."""
        # Create segments for the line
        points = np.array([x, y]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)
        
        # Normalize z-values and apply a colormap
        norm = Normalize(vmin=np.min(z), vmax=np.max(z))
        lc = LineCollection(segments, cmap=colormap, norm=norm)
        lc.set_array(z)
        lc.set_linewidth(2.5)  # Adjust line thickness
        
        # Add the LineCollection to the axis
        ax.add_collection(lc)
        return lc

def run(args):

    update('Starting map generation', True)

    # Load in the data
    update("Unpackaging processed data")
    with open(config.PROCESSOR_OUTPUT, 'rb') as fb:
        data_dict = pickle.load(fb)
    centerCords = data_dict[config.PROCESSED_CENTER]
    path = data_dict[config.PROCESSED_PATH]

    # Determin the zoom level and picture radius
    update("Determining zoom level and picture size")
    maxX = np.max(np.abs(path[0]))
    maxY = np.max(np.abs(path[1]))
    maxDistance = maxX if maxX >= maxY else maxY
    #TODO: Fix the max distance calculation. Not sure how it works
    maxDistance = 1
    print("Center Coords", centerCords)
    zoomLevel, pictureRadius = cordinatehandling.DetermineMinimumZoomLevel(centerCords[0], maxDistance)
    print("Picture Radius", pictureRadius)
    # Determine the style of the map
    if   (args.maptype == 'non'):
        makeMap = False
    elif (args.maptype == 'rdm'):
        makeMap = True
        mapType = "roadmap"
    elif (args.maptype == 'sat'):
        makeMap = True
        mapType = "satellite"

    if args.location == 'ul':
        load_map = config.UTAH_LAKE_MAP
    elif args.location == 'byu':
        load_map = config.BYU_MAP
    elif makeMap:
        # Initializing the gmaps object
        update("Connecting with Google Maps")
        try:
            gmaps = googlemaps.Client(key = config.GMAPS_API_KEY)
        except ValueError:
            quit(1)

        # Puts the map at the specified location
        update("Generating Google Map")
        mapAtLocation = gmaps.static_map(size=config.DEFAULT_OUTPUT_IMAGE_SIZE, center=cordinatehandling.CordToString(centerCords), maptype=mapType, zoom=zoomLevel)

        update("Saving Google Map")
        output = open(config.EMPTY_MAP, 'wb')
        for chunk in mapAtLocation:
            if chunk:
                output.write(chunk)
        output.close()
        load_map = config.EMPTY_MAP

    x_values = []
    y_values = []
    z_values = path[:, 2]

    gps_odom = data_dict[config.PROCESSED_GPS_ODOM]
    x_odom = gps_odom[:,0]
    y_odom = gps_odom[:,1]
    z_odom = gps_odom[:,2]
    
    factor = data_dict[config.PROCESSED_FACTOR]
    x_factor = factor[:,0]
    y_factor = factor[:,1]
    z_factor = factor[:,2]


    if (args.latlongaxis):
        update("Converting path data to lat/long coordinates", True)
        for i in tqdm(range(len(path))):
            newCoords = cordinatehandling.MetersToLatLang(centerCords, np.array([path[i, 0], path[i, 1]]))
            x_values.append(newCoords[1])
            y_values.append(newCoords[0])
        x_values = np.array(x_values)
        y_values = np.array(y_values)

        topRightCorner = cordinatehandling.MetersToLatLang(centerCords, np.array([pictureRadius, pictureRadius]))
        bottomLeftCorner = cordinatehandling.MetersToLatLang(centerCords, np.array([-pictureRadius, -pictureRadius]))
        x_min = bottomLeftCorner[1]
        x_max = topRightCorner[1]
        y_min = bottomLeftCorner[0]
        y_max = topRightCorner[0]
    else:
        update("Importing path data")
        x_values = path[:,0]
        y_values = path[:,1]
        x_min = - pictureRadius
        x_max = pictureRadius
        y_min = - pictureRadius
        y_max = pictureRadius

    # plt.figure(figsize=[9, 6.4])
    # Create the plot
    fig, ax = plt.subplots()
    if makeMap:
        map = pltimg.imread(load_map)
        plt.imshow(map, extent=[y_min, y_max, x_min, x_max])

    update("Generating scatterplot from path data")
    # plt.scatter(y_values, x_values, c=z_values, cmap=args.colormap, s=0.1)
    # plt.plot(y_values, x_values, color='red')
    # plt.plot(x_odom, y_odom, color='blue')
    # plt.plot(x_factor, y_factor, color='green')

    update("Configuring scatterplot")
    # plt.colorbar()
    if (args.latlongaxis):
        plt.xlabel('Longitude (Degrees)')
        plt.ylabel('Latitude (Degrees)')
    else:
        plt.xlabel('Distance from center (Meters)')
        plt.ylabel('Distance from Center (Meters)')
    
    plt.axis('tight')
    
    plt.ticklabel_format(style='plain', useOffset=False)
    # plt.title('Digital Elevation Model (DEM)')
    plt.tight_layout(rect=(0.01, 0.05, 0.99, 0.99))

    # start_time = data_dict[config.PROCESSED_TIME_START]
    # date = datetime.fromtimestamp(start_time).strftime('%a, %d %b %Y at %H:%M %Z')
    # bottomText = 'Date recorded: {date}\nTime Elapsed: {time:.1f} minutes'.format(date=date, time=data_dict[config.PROCESSED_TIME_DIFF])
    # plt.figtext(0.01, 0.01, bottomText, va='bottom', fontsize='x-small')

    update("Saving scatterplot")
    # plt.show()
    # prompt input for naming of map 

    update("Map generation complete", True)


    # Add the colored lines
    lc1 = add_colored_line(ax, y_values, x_values, z_values, args.colormap)
    lc2 = add_colored_line(ax, x_odom, y_odom, z_odom, args.colormap)
    lc3 = add_colored_line(ax, x_factor, y_factor, z_factor, args.colormap)

    # Add colorbars for each line
    # cbar1 = plt.colorbar(lc1, ax=ax, orientation='vertical', label='Z-Values Line 1')
    # cbar2 = plt.colorbar(lc2, ax=ax, orientation='vertical', label='Z-Values Odom')
    cbar3 = plt.colorbar(lc3, ax=ax, orientation='vertical', label='Z-Values Factor')

    #Plot the waypoints
    waypoints = data_dict[config.PROCESSED_WPT]
    radii = data_dict[config.PROCESSED_RAD]
    plot_waypoints(waypoints, radii, ax)
    
    # Show the plot
    # plt.show()
    plt.savefig("output.png", pad_inches=1)