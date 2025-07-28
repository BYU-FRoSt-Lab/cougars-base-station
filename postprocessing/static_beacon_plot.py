import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import math
import post_mission_processor_config as CONFIG
from utils import plotter_utils as util
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize
# from plotter_utility import config, cordinatehandling
from mpl_toolkits.mplot3d.art3d import Line3D
import post_mission_processor_config as CONFIG


# This script will plot the path of the vehicle according to static beacons viewpoint
# Will only work for static beacons. This script could be updated to also work for vehicles pinging each other.




# plots modem line colors according to these colors
MODEM_COLORS = {
    1: 'red',
    2: 'orange',
    3: 'yellow',
    4: 'green',
    5: 'blue',
    6: 'purple',
    15: "blue"
}
PLOTTING_DELAY = 0.1   # sets the delay between points plotted. Set to 0 to skip to end
CLEAR_TRAIL = True      # If true, will only keep last 5 points for a less cluttered few. If false will plot entire line
# MISSION_NAME = "/converted__1.0_static_beacons-2025-07-15-11-16-35"
MISSION_NAME ="/converted__2.0_lawnmower_varying_depth-2025-07-15-12-30-44"
modem_positions_corrected = {}
center = CONFIG.MODEM_POSITIONS[CONFIG.CENTRAL_MODEM]

# Gets the x,y,z for each modem based on the center modem
for modem in CONFIG.MODEM_POSITIONS:
    modem_positions_corrected[modem] = {}
    x,y = util.CalculateHaversine(center['lat'], center['lon'], CONFIG.MODEM_POSITIONS[modem]['lat'], CONFIG.MODEM_POSITIONS[modem]['lon'])
    z = CONFIG.MODEM_POSITIONS[modem]['depth']
    modem_positions_corrected[modem]['x'] = x
    modem_positions_corrected[modem]['y'] = y
    modem_positions_corrected[modem]['z'] = z

# adds a singular timestamp field instead of seconds and nanoseconds for easier indexing
def load_csv_with_timestamp(file):
    df = pd.read_csv(CONFIG.SAVES_DIR+ MISSION_NAME + file)
    df['timestamp'] = pd.to_datetime(df["header.stamp.sec"] + df["header.stamp.nanosec"] * 1e-9, unit="s")
    df.set_index("timestamp", inplace=True)
    return df


# calculates the vehicles position in the global frame from the modem pings
def raw_modem_rec_to_global(line, modem_data, imu_data):
    modem_id, range_dist, azimuth, elevation, timestamp = get_modem_data(line, modem_data)
    x, y, z = util.spherical_to_cartesian(range_dist, azimuth, elevation)
    x_orientaion, y_orientation, z_orientaion, w_orientaion = get_imu_data(timestamp, imu_data)
    vector = np.array([x,y,z])
    quaternion = [x_orientaion, y_orientation, z_orientaion, w_orientaion]
    x_rot, y_rot, z_rot, = util.rotate_vector(vector, quaternion)
    x_fixed_frame, y_fixed_frame, z_fixed_frame = fix_modem_origin(x_rot,y_rot,z_rot,modem_id)

    return x_fixed_frame, y_fixed_frame, z_fixed_frame, modem_id, timestamp

def plot_modem_locations(ax):
    for modem_id, position in modem_positions_corrected.items():
        x = position['x']
        y = position['y']
        z = position['z']
        ax.scatter([x], [y], [z], color=MODEM_COLORS[modem_id], label=f'Modem {modem_id}')
        ax.text(x + 0.1, y + 0.1, z + 0.1, f'Modem {modem_id}', fontsize=12)


def plot_modem_locations_2d(ax):
    for modem_id, position in modem_positions_corrected.items():
        x = position['x']
        y = position['y']
        ax.plot(x, y, 'o', color=MODEM_COLORS[modem_id], label=f'Modem {modem_id}')
        ax.text(x + 0.1, y + 0.1, f'Modem {modem_id}', fontsize=12)



def plot_static_beacon(modem_data, imu_data, ax, live=False):
    if imu_data.index.name != 'timestamp':
        imu_data.set_index("timestamp", inplace=True)
    if modem_data.index.name != 'timestamp':
        modem_data.set_index("timestamp", inplace=True)
    plot_modem_locations_2d(ax)
    processed_modem_data = process_modem_location(modem_data, imu_data)
    ax = plot_vehicle_location(processed_modem_data, ax, live)
    return ax


def process_modem_location(modem_rec_data, imu_data):
    modem_points = {modem_id: {'timestamp':[], 'x':[], 'y':[], 'z':[]} for modem_id in CONFIG.MODEM_POSITIONS}
    for i in range(len(modem_rec_data['range_dist'])):
        x, y, z, id, timestamp= raw_modem_rec_to_global(i, modem_rec_data, imu_data)
        modem_points[id]['timestamp'].append(timestamp)
        modem_points[id]['x'].append(x)
        modem_points[id]['y'].append(y)
        modem_points[id]['z'].append(z)
    return modem_points

def plot_vehicle_location(modem_points, ax, live=False):
    combined = []
    for modem_id, data in modem_points.items():
        for t, x, y in zip(data['timestamp'], data['x'], data['y']):
            combined.append((t, modem_id, x, y))

    combined.sort()

    # Track last positions per modem
    last_pos = {}  # modem_id -> (x, y)

    for t, modem_id, x, y in combined:
        print(f"Time: {t}, Modem: {modem_id}, Pos: ({x}, {y})")
        
        if modem_id in last_pos:
            lastx, lasty = last_pos[modem_id]
            ax.plot(
                [lastx, x],
                [lasty, y],
                color=MODEM_COLORS[modem_id],
                linewidth=1
            )
        
        last_pos[modem_id] = (x, y)

        if PLOTTING_DELAY > 0 and live:
            plt.pause(PLOTTING_DELAY)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("Modem Range and Angle Points")
                
    return ax



def plot_vehicle_location_live(ax, modem_data, imu_data):
    modem_scatters = {modem_id: [] for modem_id in MODEM_COLORS}
    modem_lines = {modem_id: [] for modem_id in MODEM_COLORS}
    modem_last_pos = {}
    
    processed_modem_data = process_modem_location(modem_data, imu_data)
    combined = []

    for modem_id, data in processed_modem_data.items():
        for t, x, y, z in zip(data['timestamp'], data['x'], data['y'], data['z']):
            combined.append((t, modem_id, x, y, z))

    # Sort all the points by timestamp
    combined.sort()

    # Now iterate through them in order
    for t, modem_id, x, y, z in combined:
        # print(f"Time: {t}, Modem: {modem_id}, Pos: ({x}, {y}, {z})")

        if modem_id in modem_last_pos:
            plot_line_with_trail(ax, modem_lines[modem_id], modem_last_pos[modem_id], (x, y, z), MODEM_COLORS[modem_id], max_trail=5, clear_trail=CLEAR_TRAIL)
        plot_scatter_with_trail(ax, modem_scatters[modem_id], (x, y, z), MODEM_COLORS[modem_id], size=15, max_trail=5, clear_trail=CLEAR_TRAIL)

        modem_last_pos[modem_id] = (x, y, z)
        print("plotting")
        if(PLOTTING_DELAY > 0):
            plt.pause(PLOTTING_DELAY)

def plot_line_with_trail(ax, lines, start_point, end_point, color, linewidth=1, max_trail=5, clear_trail=True):
    line = Line3D(
        [start_point[0], end_point[0]],
        [start_point[1], end_point[1]],
        [start_point[2], end_point[2]],
        color=color, linewidth=linewidth
    )
    ax.add_line(line)
    lines.append(line)
    if len(lines) > max_trail and clear_trail:
        lines.pop(0).remove()

def plot_scatter_with_trail(ax, scatters, point, color, size=15, max_trail=5, clear_trail=True):
    scatter = ax.scatter(point[0], point[1], point[2], color=color, s=size)
    scatters.append(scatter)
    if len(scatters) > max_trail and clear_trail:
        scatters.pop(0).remove()
    return scatter

# Corrects the position according to the location of the modem
def fix_modem_origin(x,y,z,modem_id):
    x = x + modem_positions_corrected[modem_id]['x']
    y = y + modem_positions_corrected[modem_id]['y']
    z = z + modem_positions_corrected[modem_id]['z']
    return x, y, z

# gets the modem at a certain line in data
def get_modem_data(line, modem_data):
    range = modem_data['range_dist'][line] / 10
    azimuth = modem_data['usbl_azimuth'][line] / 10
    elevation = modem_data['usbl_elevation'][line] / 10
    timestamp = modem_data.index[line]
    modem_id = modem_data['src_id'][line]
    return modem_id, range, azimuth, elevation, timestamp

# gets the imu data at the closest timestamp to the input
def get_imu_data(timestamp, imu_data):
    closest_timestamp = imu_data.index.get_indexer([timestamp], method='nearest')[0]
    orientation_x = imu_data.iloc[closest_timestamp]['orientation.x']
    orientation_y = imu_data.iloc[closest_timestamp]['orientation.y']
    orientation_z = imu_data.iloc[closest_timestamp]['orientation.z']
    orientation_w = imu_data.iloc[closest_timestamp]['orientation.w']
    return orientation_x, orientation_y, orientation_z, orientation_w


def main():
    fig, ax = plt.subplots()
    # ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    # ax.set_zlabel('Z')
    # plot_modem_locations(ax)
    modem_rec_data = load_csv_with_timestamp('/coug3.modem_rec.csv')
    modem_imu_data = load_csv_with_timestamp('/coug3.modem_imu.csv')

    plot_static_beacon(modem_rec_data, modem_imu_data, ax, True)
    # plot_vehicle_location_live(ax, modem_rec_data, modem_imu_data)
    plt.show()


if __name__ == "__main__":
    main()