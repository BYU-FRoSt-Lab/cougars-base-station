import utils.rosbags_converter as rc
from utils.seatrac_enums import CST_E
import utils.plotter_utils as p_utils
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import post_mission_processor_config as CONFIG
import static_beacon_plot as beacon_plot
import os



print("Post Mission Processing Script Running")

# Convert Rosbags or get data from already converted CSVs
if CONFIG.RELOAD:
    print("converting rosbags")
    dataframes = p_utils.get_dataframes(
        rosbags_dir=CONFIG.BAGPATH, rosmsgs_dir=CONFIG.ROSMSGS_DIR, csv_dir=CONFIG.SAVES_DIR,
        keywords=None, topics=None,verbose=CONFIG.VERBOSE)
else:
    print(f"loading dataframes from {CONFIG.SAVES_DIR}")
    dataframes = rc.load_dataframes(CONFIG.SAVES_DIR, keywords=None, verbose=CONFIG.VERBOSE)
    if len(dataframes)==0:
        raise RuntimeError("Lenth of dataframes is 0. Dataframes may not be loaded")
print("dataframes loaded")

# make timestamp field in dataframes
p_utils.insert_timestamps(dataframes)
fig, ax = plt.subplots()

if CONFIG.PLOT_DEAD_RECKONING:
    for path, bag in dataframes.items():
        print(f"Graphing: {path}")
        dvl_odom = p_utils.get_topic(bag, "/dvl/dead_reckoning")
        if dvl_odom is None: continue
        ax = p_utils.plot_pose_w_cov(dvl_odom, ax=ax, plot_direction_line=True)
        mission_name = p_utils.clean_converted_bags_path(path)
        plt.savefig(CONFIG.PLOT_SAVES_DIR + mission_name + '_dead_reckoning_w_cov.png')
        # plt.show()
if CONFIG.PLOT_STATIC_BEACON_TEST:
    i=0
    for path, bag in dataframes.items():
        if i >0:
            modem_rec = p_utils.get_topic(bag, "/modem_rec")
            print(modem_rec)
            beacon_plot.plot_static_beacon(modem_rec, ax)
        i+=1
    plt.show()
