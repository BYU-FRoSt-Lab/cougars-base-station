# Directory information
COUGARS_REPO = "/home/benwash/cougars"
BAGPATH = COUGARS_REPO + "/bag"
SAVES_DIR = BAGPATH + "/converted_bags"
PLOT_SAVES_DIR = COUGARS_REPO + "/cougars-base-station/postprocessing/saved_plots"
ROSMSGS_DIR= COUGARS_REPO + "/cougars-ros2/src"



#If True converts bags to dataframes and csv. 
#If False gets data from csvs.
RELOAD = False

# controls how much is printed
VERBOSE = False

# If true, plots the dvl_data
PLOT_DEAD_RECKONING = False


PLOT_STATIC_BEACON_TEST = True
CENTRAL_MODEM = 15      # sets lat and long of this modem to be (0,0) on graph
PLOT_DVL = False        # plots dvl path on map (currently doesn't work well)
PLOT_GPS = False        # plots gps path on map (currently doesn't work well)
PLOTTING_DELAY = 0.01   # sets the delay between points plotted. Set to 0 to skip to end
CLEAR_TRAIL = True      # If true, will only keep last 5 points for a less cluttered few. If false will plot entire line
MODEM_POSITIONS = {             # Recorded modem lat, long, and depth. Depth should be negative unless the coug is flying.
    1: {'lon': -111.02801757, 'lat': 40.14641604, 'depth': -0.8128},
    2: {'lon': -111.02795235, 'lat': 40.14619182, 'depth': -1.164},
    15: {'lon': -111.02801804, 'lat': 40.14627160, 'depth': -1.3218}
}
MISSION_NAME = "/converted__2.0_lawnmower_varying_depth-2025-07-15-12-30-44"