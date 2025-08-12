import utils.rosbags_converter as rc
from utils.seatrac_enums import CST_E
import utils.plotter_utils as p_utils
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import yaml
import static_beacon_plotter as beacon_plot
import os
from pathlib import Path
import plot_gps_pings as gps_plot
CONFIG_PATH="/home/frostlab/base_station/postprocessing/post_mission_processor_config.yaml"
with open(CONFIG_PATH,"r") as f:
    config=yaml.safe_load(f)
BAGPATH = config.get("BAGPATH")
ROSMSGS_DIR = config.get("ROSMSGS_DIR")
SAVES_DIR = config.get("SAVES_DIR")
PLOT_GPS_LOCKS = config.get("PLOT_GPS_LOCKS")
PLOT_DEAD_RECKONING = config.get("PLOT_DEAD_RECKONING")
PLOT_COV_ELL = config.get("PLOT_COV_ELL")
PLOT_DIRECTION_LINE = config.get("PLOT_DIRECTION_LINE")
PLOT_STATIC_BEACON_TEST = config.get("PLOT_STATIC_BEACON_TEST")
RUN_LIVE = config.get("RUN_LIVE")
PLOT_SEPERATE = config.get("PLOT_SEPERATE")
MODEM_POSITIONS = config.get("MODEM_POSITIONS")
CENTRAL_MODEM = config.get("CENTRAL_MODEM")
PLOT_STATIC_BEACON_DVL = config.get("PLOT_STATIC_BEACON_DVL")
RELOAD=config.get("RELOAD")
VERBOSE=config.get("VERBOSE")
# print(config)
# Convert Rosbags or get data from already converted CSVs

if RELOAD:
    print("converting rosbags")
    dataframes = p_utils.get_dataframes(
        rosbags_dir= BAGPATH, rosmsgs_dir= ROSMSGS_DIR, csv_dir= SAVES_DIR,
        keywords=None, topics=None,verbose= VERBOSE)
else:
    convertedbags=os.listdir( SAVES_DIR)
    for bagn in range(len(convertedbags)):
        convertedbags[bagn]=convertedbags[bagn].removeprefix('processed_')
    print(f"skipping: {convertedbags}")
    dataframes = p_utils.get_dataframes(
        rosbags_dir=BAGPATH, rosmsgs_dir= ROSMSGS_DIR, csv_dir= SAVES_DIR,
        keywords=None, topics=None,verbose= VERBOSE,excluded_bags=convertedbags)
    print(f"loading dataframes from { SAVES_DIR}")
    dataframes = rc.load_dataframes( SAVES_DIR, keywords=None, verbose= VERBOSE)
    if len(dataframes)==0:
        raise RuntimeError("Lenth of dataframes is 0. Dataframes may not be loaded")
print("dataframes loaded")
# print(dataframes.keys())
def plot_dead_reckoning(bag, ax):
    # print(f"Graphing: {path}")
    dvl_odom = p_utils.get_topic(bag, "/dvl/dead_reckoning")
    if dvl_odom is not None:
        ax = p_utils.plot_pose_w_cov(dvl_odom, ax=ax, plot_direction_line=PLOT_DIRECTION_LINE, plot_cov_ell=PLOT_COV_ELL)
    return ax


def plot_static_beacon(bag, ax):
    modem_rec = p_utils.get_topic(bag, "/modem_rec")
    imu_data = p_utils.get_topic(bag, "/modem_imu")

    modem_offsets = {
        modem: {
            'x': (xy := p_utils.CalculateHaversine(MODEM_POSITIONS[CENTRAL_MODEM]['lat'], MODEM_POSITIONS[CENTRAL_MODEM]['lon'], pos['lat'], pos['lon']))[0],
            'y': xy[1],
            'z': pos['depth']
        }
        for modem, pos in MODEM_POSITIONS.items()
    }

    beacon_plot.run(modem_rec, imu_data, modem_offsets, RUN_LIVE, ax, PLOT_SEPERATE )
    return ax

def plot_gps_pings(bag, ax):
    gps_data = p_utils.get_topic(bag, "llh_position")
    if gps_data is not None:
        lat = gps_data['latitude']
        lon = gps_data['latitude']
        cenlat, cenlon = p_utils.FindCenter(lat, lon)
        gps_plot.plot_gps(ax, cenlat, cenlon, lat, lon)
        return ax
    return None



# make timestamp field in dataframes
p_utils.insert_timestamps(dataframes)
fig, ax = plt.subplots()

if PLOT_DEAD_RECKONING:
    for path, bag in dataframes.items():
        plot_dead_reckoning(bag, ax)
        savepath=Path( SAVES_DIR).joinpath(path,'dead_reckoning_w_cov.png')
        print(f"Saving to {savepath}")
        plt.savefig(savepath)
        ax.cla()

        # plt.show()
if PLOT_STATIC_BEACON_TEST:
    for path, bag in dataframes.items():
        print(bag)
        plot_static_beacon(bag, ax)
        savepath=Path( SAVES_DIR).joinpath(path,'static_beacon_test.png')
        print(f"Saving {savepath}")
        plt.savefig(savepath)
        ax.cla()
        # plt.show()
if  PLOT_GPS_LOCKS:
    for path, bag in dataframes.items():
        plot_gps_pings(bag, ax)
        savepath=Path( SAVES_DIR).joinpath(path,'gps_path.png')
        print(f"Saving {savepath}")
        plt.savefig(savepath)
        ax.cla()




