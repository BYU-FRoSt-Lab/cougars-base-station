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
modem_colors = {
    1: 'red',
    2: 'orange',
    3: 'yellow',
    4: 'green',
    5: 'blue',
    6: 'purple',
    15: "blue"
}

modem_positions_corrected = {}
center = CONFIG.MODEM_POSITIONS[CONFIG.CENTRAL_MODEM]

# Gets the x,y,z for each modem based on the center modem
for modem in CONFIG.MODEM_POSITIONS:
    modem_positions_corrected[modem] = {}
    y,x = util.CalculateHaversine(center['lat'], center['lon'], CONFIG.MODEM_POSITIONS[modem]['lat'], CONFIG.MODEM_POSITIONS[modem]['lon'])
    z = CONFIG.MODEM_POSITIONS[modem]['depth']
    modem_positions_corrected[modem]['x'] = x
    modem_positions_corrected[modem]['y'] = y
    modem_positions_corrected[modem]['z'] = z

# adds a singular timestamp field instead of seconds and nanoseconds for easier indexing
def load_csv_with_timestamp(file):
    df = pd.read_csv(CONFIG.SAVES_DIR+ CONFIG.MISSION_NAME + file)
    df['timestamp'] = pd.to_datetime(df["header.stamp.sec"] + df["header.stamp.nanosec"] * 1e-9, unit="s")
    df.set_index("timestamp", inplace=True)
    return df


modem_rec_data = load_csv_with_timestamp('/coug3.modem_rec.csv')
modem_imu_data = load_csv_with_timestamp('/coug3.modem_imu.csv')
dvl_data = load_csv_with_timestamp('/coug3.dvl.dead_reckoning.csv')
gps_data = load_csv_with_timestamp('/coug3.gnss_1.llh_position.csv')


# calculates the vehicles position in the global frame from the modem pings
def calc_point(line):
    modem_id, range_dist, azimuth, elevation, timestamp = get_modem_data(line)
    x, y, z = util.spherical_to_cartesian(range_dist, azimuth, elevation)
    x_orientaion, y_orientation, z_orientaion, w_orientaion = get_imu_data(timestamp)
    vector = np.array([x,y,z])
    quaternion = [x_orientaion, y_orientation, z_orientaion, w_orientaion]
    x_rot, y_rot, z_rot, = util.rotate_vector(vector, quaternion)
    x_fixed_frame, y_fixed_frame, z_fixed_frame = fix_modem_origin(x_rot,y_rot,z_rot,modem_id)
    # x_fixed_frame, y_fixed_frame, z_fixed_frame = apply_offset(x_fixed_frame, y_fixed_frame, z_fixed_frame, modem_id)

    return x_fixed_frame, y_fixed_frame, z_fixed_frame, modem_id, timestamp

def plot_modem_locations(ax):
    for modem_id, position in modem_positions_corrected.items():
        x = modem_positions_corrected[modem_id]['x']
        y = modem_positions_corrected[modem_id]['y']
        z = modem_positions_corrected[modem_id]['z']
        ax.plot(x, y, z, 'o', color=modem_colors[modem_id],label=f'Modem {modem_id}')
        ax.text(x + 0.1, y + 0.1, z + 0.1, f'Modem {modem_id}', fontsize=12)

def process_modem_location(modem_rec_data):
    modem_points = {modem_id: {'timestamp':[], 'x':[], 'y':[], 'z':[]} for modem_id in CONFIG.MODEM_POSITIONS}
    for i in range(len(modem_rec_data)):
        x, y, z, id, timestamp = calc_point(i)
        modem_points[id]['timestamp'] = timestamp
        modem_points[id]['x'].append(x)
        modem_points[id]['y'].append(y)
        modem_points[id]['z'].append(z)
    return modem_points

def plot_vehicle_location(modem_points, ax):
    print(len(modem_points[1]))
    for modem, points in modem_points.items():
        for i in range(len(points)):
            if i != 0:
                
                ax.plot(
                    [points['x'][i], points['x'][i-1]],
                    [points['y'][i], points['y'][i-1]],
                    color=modem_colors[i], linewidth=1
                )
                
    return ax
    
def plot_static_beacon(modem_data, ax):
    processed_modem_data = process_modem_location(modem_data)
    ax = plot_vehicle_location(processed_modem_data, ax)
    return ax

            

def plot_vehicle_location_live(fig, ax):
    modem_scatters = {modem_id: [] for modem_id in modem_colors}
    modem_lines = {modem_id: [] for modem_id in modem_colors}
    modem_last_pos = {}
    modem_data_final = {modem_id: [] for modem_id in modem_colors}

    gps_scatters = []
    gps_lines = []
    dvl_scatters = []
    dvl_lines = []
    last_gps = None
    last_dvl = None

    for i in range(len(modem_rec_data)):
        x, y, z, modem_id, timestamp = calc_point(i)
        
        if CONFIG.PLOT_DVL:
            dx, dy, dz = get_gps_data(timestamp)
            
            if last_dvl:
                lx, ly, lz = last_dvl
                plot_line_with_trail(ax, dvl_lines, (lx,ly,lz), (dx,dy,dz), 'brown', clear_trail=CONFIG.CLEAR_TRAIL)
            plot_scatter_with_trail(ax, dvl_scatters, (dx,dy,dz), 'brown', clear_trail=CONFIG.CLEAR_TRAIL)
            last_dvl = (dx, dy, dz)

        if CONFIG.PLOT_GPS:
            gx, gy, covariance = get_gps_data(timestamp)
            if covariance < 20:                
                if last_gps:
                    lx,ly = last_gps
                    plot_line_with_trail(ax, gps_lines, (lx,ly,0), (dx,dy,0), 'gray', clear_trail=CONFIG.CLEAR_TRAIL)
                plot_scatter_with_trail(ax, dvl_scatters, (dx,dy,0), 'gray', clear_trail=CONFIG.CLEAR_TRAIL)
                last_gps = (gx,gy,0)

        if modem_id in modem_last_pos:
            plot_line_with_trail(ax, modem_lines[modem_id], modem_last_pos[modem_id], (x, y, z), modem_colors[modem_id], max_trail=5, clear_trail=CONFIG.CLEAR_TRAIL)
        plot_scatter_with_trail(ax, modem_scatters[modem_id], (x, y, z), modem_colors[modem_id], size=15, max_trail=5, clear_trail=CONFIG.CLEAR_TRAIL)

        modem_last_pos[modem_id] = (x, y, z)
        if(CONFIG.PLOTTING_DELAY > 0):
            plt.pause(CONFIG.PLOTTING_DELAY)

    # print(compute_best_offset(modem_data_final[15], modem_data_final[1]))
    # print(compute_best_offset(modem_data_final[15], modem_data_final[2]))

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
def get_modem_data(line):
    range = modem_rec_data['range_dist'][line] / 10
    azimuth = modem_rec_data['usbl_azimuth'][line] / 10
    elevation = modem_rec_data['usbl_elevation'][line] / 10
    timestamp = modem_rec_data.index[line]
    modem_id = modem_rec_data['src_id'][line]
    return modem_id, range, azimuth, elevation, timestamp

# gets the imu data at the closest timestamp to the input
def get_imu_data(timestamp):
    closest_timestamp = modem_imu_data.index.get_indexer([timestamp], method='nearest')[0]
    orientation_x = modem_imu_data.iloc[closest_timestamp]['orientation.x']
    orientation_y = modem_imu_data.iloc[closest_timestamp]['orientation.y']
    orientation_z = modem_imu_data.iloc[closest_timestamp]['orientation.z']
    orientation_w = modem_imu_data.iloc[closest_timestamp]['orientation.w']
    return orientation_x, orientation_y, orientation_z, orientation_w

# gets the gps data at the closest timestamp to the input
def get_gps_data(timestamp):
    closest_timestamp = gps_data.index.get_indexer([timestamp], method='nearest')[0]
    x, y = util.CalculateHaversine(center['lat'], center['lon'], gps_data.iloc[closest_timestamp]['latitude'], gps_data.iloc[closest_timestamp]['longitude'])
    covariance = gps_data.iloc[closest_timestamp]['COVARIANCE_TYPE_DIAGONAL_KNOWN']
    return x, y, covariance

# gets the dvl data at the closest timestamp to the input
def get_dvl_data(timestamp):
    closest_timestamp = dvl_data.index.get_indexer([timestamp], method='nearest')[0]
    x = gps_data.iloc[closest_timestamp]['position.x']
    y = gps_data.iloc[closest_timestamp]['position.y']
    z = gps_data.iloc[closest_timestamp]['position.z']
    return x,y,z



# def run():
#     fig, ax = plt.subplots(figsize=(10, 10))
#     ax = fig.add_subplot(111, projection='3d')
#     ax.set_xlabel('X')
#     ax.set_ylabel('Y')
#     ax.set_zlabel('Z')
#     plot_modem_locations(ax)
#     plot_vehicle_location(fig, ax)
#     plt.show()

def main():
    fig, ax = plt.subplots(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plot_modem_locations(ax)
    plot_vehicle_location_live(fig, ax)
    plt.show()

# def apply_offset(x,y,z,modem_id):
#     if modem_id == 1:
#         x += -0.51190371
#         y += -0.51190371
#         z += -2.0174028
#     elif modem_id == 2:
#         x += -1.363379
#         y += 0.9284069
#         z += -0.05922599
#     return x,y,z


if __name__ == "__main__":
    main()