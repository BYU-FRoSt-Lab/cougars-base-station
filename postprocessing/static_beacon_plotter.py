from utils import plotter_utils as p_utils
import numpy as np
import pandas as pd
import yaml
import matplotlib.pyplot as plt
import time

CONFIG_PATH="/home/frostlab/base_station/postprocessing/post_mission_processor_config.yaml"


MODEM_COLORS = {
    1: 'red',
    2: 'orange',
    3: 'yellow',
    4: 'green',
    5: 'blue',
    6: 'purple',
    15: "blue"
}


def run(modem_data, imu_data, modem_offsets, run_live, ax, plot_seperate):
    modemxyz = modem_raw_to_global(modem_data, imu_data, modem_offsets)
    plot_modems(ax, modem_offsets)
    if run_live:
        plot_modem_data_live(ax, modemxyz, plot_seperate, 0.2)
    else:
        plot_modem_data(ax, modemxyz, plot_seperate)

def plot_modem_data(ax, modemxyz, plot_seperate):
    modemxyz.sort_values(by='timestamp', inplace=True)
    if plot_seperate:
        for modem_id, group in modemxyz.groupby('modem_id'):
            xs = group['x'].values
            ys = group['y'].values
            color = MODEM_COLORS.get(modem_id, 'black')  # fallback to black if not defined

            ax.plot(xs, ys, color=color, label=f"Modem {modem_id}")
        return ax
    else:
        xs = modemxyz['x'].values
        ys = modemxyz['y'].values
        ax.plot(xs, ys, color="blue")

def plot_modem_data_live(ax, modemxyz, plot_separate, delay=0.05):
    modemxyz.sort_values(by='timestamp', inplace=True)
    
    prev_by_modem = {}

    for _, row in modemxyz.iterrows():
        modem_id = row['modem_id']
        color = MODEM_COLORS.get(modem_id, 'black')

        if modem_id in prev_by_modem:
            prev = prev_by_modem[modem_id]
            ax.plot([prev['x'], row['x']], [prev['y'], row['y']], color=color)
        prev_by_modem[modem_id] = row

        plt.pause(delay)
    else:
        prev = None
        for _, row in modemxyz.iterrows():
            if prev is not None:
                ax.plot([prev['x'], row['x']], [prev['y'], row['y']], color="blue")
                plt.pause(delay)
            prev = row
    return ax

def modem_raw_to_global(modem_data, imu_data, modem_offsets):
    if imu_data.index.name != 'timestamp':
        imu_data.set_index("timestamp", inplace=True)
    if modem_data.index.name != 'timestamp':
        modem_data.set_index("timestamp", inplace=True)
    
    modem_global = []

    for timestamp in modem_data.index:
        modem_id, range_dist, azimuth, elevation = get_modem_data(timestamp, modem_data)
        x_orientaion, y_orientation, z_orientaion, w_orientaion = get_imu_data(timestamp, imu_data)
        x,y,z = p_utils.spherical_to_cartesian(range_dist, azimuth, elevation)
        x,y,z = p_utils.rotate_vector(np.array([x,y,z]), [x_orientaion, y_orientation, z_orientaion, w_orientaion])
        #Seconds rotation to to make north the positive y-axis
        x,y,z = p_utils.rotate_vector(np.array([x,y,z]), [0, 0, -np.sqrt(2)/2, np.sqrt(2)/2])
        x,y,z = fix_modem_origin(x,y,z, modem_offsets[modem_id])
        modem_global.append({'timestamp' : timestamp, 'modem_id' : modem_id, 'x' : x, 'y' : y, 'z' : z})
    
    return pd.DataFrame(modem_global)


        
def get_modem_data(timestamp, modem_data):
    range = modem_data['range_dist'][timestamp] / 10
    azimuth = modem_data['usbl_azimuth'][timestamp] / 10
    elevation = modem_data['usbl_elevation'][timestamp] / 10
    modem_id = modem_data['src_id'][timestamp]
    return modem_id, range, azimuth, elevation

def get_imu_data(timestamp, imu_data):
    closest_timestamp = imu_data.index.get_indexer([timestamp], method='nearest')[0]
    orientation_x = imu_data.iloc[closest_timestamp]['orientation.x']
    orientation_y = imu_data.iloc[closest_timestamp]['orientation.y']
    orientation_z = imu_data.iloc[closest_timestamp]['orientation.z']
    orientation_w = imu_data.iloc[closest_timestamp]['orientation.w']
    return orientation_x, orientation_y, orientation_z, orientation_w

def fix_modem_origin(x,y,z,modem_vector):
    x = x + modem_vector[0]
    y = y + modem_vector[1]
    z = z + modem_vector[2]
    return x, y, z

def plot_modems(ax, modem_offsets):
    for modem_id, vector in modem_offsets.items():
        x = vector[0]
        y = vector[1]
        ax.scatter([x], [y], color=MODEM_COLORS[modem_id], label=f'Modem {modem_id}')
        ax.text(x + 0.1, y + 0.1, f'Modem {modem_id}', fontsize=12)

def load_csv_with_timestamp(file):
    with open(CONFIG_PATH,"r") as f:
        config=yaml.safe_load(f)
    MISSION_NAME ="/converted__2.0_lawnmower_varying_depth-2025-07-15-12-30-44"
    df = pd.read_csv(config.get("SAVES_DIR")+ MISSION_NAME + file)
    df['timestamp'] = pd.to_datetime(df["header.stamp.sec"] + df["header.stamp.nanosec"] * 1e-9, unit="s")
    df.set_index("timestamp", inplace=True)
    return df

def test():
    fig, ax = plt.subplots()
    ax.set_xlabel('X')
    ax.set_ylabel('Y')

    modem_positions_corrected = {}
    with open(CONFIG_PATH,"r") as f:
        config=yaml.safe_load(f)
    center = config.get("MODEM_POSITIONS")[config.get("CENTRAL_MODEM")]
    for modem in config.get("MODEM_POSITIONS"):
        modem_positions_corrected[modem] = []
        x,y = p_utils.CalculateHaversine(center['lat'], center['lon'], config.get("MODEM_POSITIONS")[modem]['lat'], config.get("MODEM_POSITIONS")[modem]['lon'])
        z = config.get("MODEM_POSITIONS")[modem]['depth']
        modem_positions_corrected[modem] = [x,y,z]

    modem_rec_data = load_csv_with_timestamp('/coug3.modem_rec.csv')
    modem_imu_data = load_csv_with_timestamp('/coug3.modem_imu.csv')
    run(modem_rec_data, modem_imu_data, modem_positions_corrected, True, ax, True)
    plt.show()

# test()