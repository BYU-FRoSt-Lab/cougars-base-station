import utils.rosbags_converter as rc
from utils.seatrac_enums import CST_E
import utils.plotter_utils as p_utils
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import post_mission_processor_config as CONFIG
import static_beacon_plotter as beacon_plot
import os
from pathlib import Path
import plot_gps_pings as gps_plot
import argparse

def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('true', 't', '1', 'yes', 'y'):
        return True
    elif v.lower() in ('false', 'f', '0', 'no', 'n'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')



argparser=argparse.ArgumentParser(description="Mission Postprocessing Script")
argparser.add_argument("plot_dead_reckoning", help="True or False to plot dead reckoning info",type=str2bool)
argparser.add_argument("plot_static_beacon", help="True or False to plot for a static beacon test",type=str2bool)
argparser.add_argument("plot_gps_pings", help="True or False to plot gps pings",type=str2bool)
argparser.add_argument("--plot_static_beacon_dvl", help="True or False to plot dvl data for static beacon stuff",type=str2bool)
argparser.add_argument("--plot_covariance", help="True or False to plot covariance ellipsoids for dead reckoning plots",type=str2bool,default=False)
argparser.add_argument("--plot_direction_line", help="True or False to plot heading line on dead reckoning plots",type=str2bool,default=False)
argparser.add_argument("--run_live", help="True or False to run static beacon live",type=str2bool,default=False)
args=argparser.parse_args()

BAGPATH = CONFIG.BAGPATH
ROSMSGS_DIR = CONFIG.ROSMSGS_DIR
SAVES_DIR = CONFIG.SAVES_DIR
PLOT_SAVES_DIR = CONFIG.PLOT_SAVES_DIR

PLOT_DEAD_RECKONING = CONFIG.PLOT_DEAD_RECKONING
PLOT_COV_ELL = CONFIG.PLOT_COV_ELL
PLOT_DIRECTION_LINE = CONFIG.PLOT_DIRECTION_LINE

PLOT_STATIC_BEACON_TEST = CONFIG.PLOT_STATIC_BEACON_TEST
RUN_LIVE = CONFIG.RUN_LIVE
PLOT_SEPERATE = CONFIG.PLOT_SEPERATE
MODEM_POSITIONS = CONFIG.MODEM_POSITIONS
CENTRAL_MODEM = CONFIG.CENTRAL_MODEM

PLOT_STATIC_BEACON_DVL = CONFIG.PLOT_STATIC_BEACON_DVL
# Convert Rosbags or get data from already converted CSVs

if CONFIG.RELOAD:
    print("converting rosbags")
    dataframes = p_utils.get_dataframes(
        rosbags_dir=CONFIG.BAGPATH, rosmsgs_dir=CONFIG.ROSMSGS_DIR, csv_dir=CONFIG.SAVES_DIR,
        keywords=None, topics=None,verbose=CONFIG.VERBOSE)
else:
    convertedbags=os.listdir(CONFIG.SAVES_DIR)
    for bagn in range(len(convertedbags)):
        convertedbags[bagn]=convertedbags[bagn].removeprefix('processed_')
    print(f"skipping: {convertedbags}")
    dataframes = p_utils.get_dataframes(
        rosbags_dir=BAGPATH, rosmsgs_dir=CONFIG.ROSMSGS_DIR, csv_dir=CONFIG.SAVES_DIR,
        keywords=None, topics=None,verbose=CONFIG.VERBOSE,excluded_bags=convertedbags)
    print(f"loading dataframes from {CONFIG.SAVES_DIR}")
    dataframes = rc.load_dataframes(CONFIG.SAVES_DIR, keywords=None, verbose=CONFIG.VERBOSE)
    if len(dataframes)==0:
        raise RuntimeError("Lenth of dataframes is 0. Dataframes may not be loaded")
print("dataframes loaded")
print(dataframes.keys())
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
        savepath=Path(CONFIG.SAVES_DIR).joinpath(path,'dead_reckoning_w_cov.png')
        print(f"Saving to {savepath}")
        plt.savefig(savepath)
        ax.cla()

        # plt.show()
if PLOT_STATIC_BEACON_TEST:
    for path, bag in dataframes.items():
        print(bag)
        plot_static_beacon(bag, ax)
        savepath=Path(CONFIG.SAVES_DIR).joinpath(path,'static_beacon_test.png')
        print(f"Saving {savepath}")
        plt.savefig(savepath)
        ax.cla()
        # plt.show()
if CONFIG.PLOT_GPS_LOCKS:
    for path, bag in dataframes.items():
        plot_gps_pings(bag, ax)
        savepath=Path(CONFIG.SAVES_DIR).joinpath(path,'gps_path.png')
        print(f"Saving {savepath}")
        plt.savefig(savepath)
        ax.cla()




