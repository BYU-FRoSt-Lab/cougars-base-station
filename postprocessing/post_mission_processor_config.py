# Directory information
COUGARS_REPO = "/home/frostlab"
BAGPATH = COUGARS_REPO + "/bag"
SAVES_DIR = BAGPATH + "/converted_bags"
PLOT_SAVES_DIR = COUGARS_REPO + "/cougars-base-station/postprocessing/saved_plots"
ROSMSGS_DIR= COUGARS_REPO + "/cougars-ros2/src"



#If True converts bags to dataframes and csv. 
#If False gets data from csvs.
RELOAD = False

# controls how much is printed
VERBOSE = True



# If true, plots the dvl_data
PLOT_DEAD_RECKONING = True
PLOT_COV_ELL = True
PLOT_DIRECTION_LINE = True

PLOT_GPS_LOCKS = True

PLOT_STATIC_BEACON_TEST = False
CENTRAL_MODEM = 15      # sets lat and long of this modem to be (0,0) on graph
MODEM_POSITIONS = {             # Recorded modem lat, long, and depth. Depth should be negative unless the coug is flying.
    1: {'lon': -111.02801757, 'lat': 40.14641604, 'depth': -0.8128},
    2: {'lon': -111.02795235, 'lat': 40.14619182, 'depth': -1.164},
    15: {'lon': -111.02801804, 'lat': 40.14627160, 'depth': -1.3218}
}
RUN_LIVE = False
PLOT_SEPERATE = True
PLOT_STATIC_BEACON_DVL = False