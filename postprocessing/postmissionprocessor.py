import utils.rosbags_converter as rc
from utils.seatrac_enums import CST_E
import utils.plotter_utils as p_utils
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os

# Paths
BAGPATH = "/home/frostlab/bag"
SAVES_DIR = "/home/frostlab/bag/converted_bags"
ROSMSGS_DIR="/home/frostlab/cougars-ros2/src"
RELOAD = False
VERBOSE = False
# Convert Rosbags
if RELOAD:
    print("converting rosbags")
    dataframes = p_utils.get_dataframes(
        rosbags_dir=BAGPATH, rosmsgs_dir=ROSMSGS_DIR, csv_dir=SAVES_DIR,
        keywords=None, topics=None,verbose=VERBOSE)
else:
    print("loading dataframes")
    dataframes = rc.load_dataframes(SAVES_DIR, keywords=None, verbose=VERBOSE)
    if len(dataframes)==0:
        raise RuntimeError("Lenth of dataframes is 0. Dataframes may not be loaded")
print("dataframes loaded")
p_utils.insert_timestamps(dataframes)

