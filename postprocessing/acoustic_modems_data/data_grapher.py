
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from datetime import datetime

#   0-2:  timestamp, remote_beacon_id, position_flt_error, 
#   3-5:  yaw, pitch, roll, 
#   6-8:  local_depth, VOS, RSSI, 
#  9-12:  usbl_rssi[0], usbl_rssi[1], usbl_rssi[2], usbl_rssi[3], 
# 13-14:  range_time, range_dist, 
# 15-17  azimuth, elevation, fit_error
# 18-20:  easting, northing, remote_depth


modem_pd = pd.read_csv('csv_data/modem_data.csv')
modem_np = modem_pd.to_numpy(dtype=np.int64)
modem_np = modem_np[modem_np[:,2]==0]

modem_err_pd = pd.read_csv('csv_data/modem_error_data.csv')
modem_err_np = modem_err_pd.to_numpy(dtype=np.int64)

print(modem_np)
print(modem_err_np)

gps_pd = pd.read_csv('csv_data/gps_data.csv')
gps_times = gps_pd["timestamp"].to_numpy()
gps_lat = gps_pd[" latitude"].to_numpy()
gps_long = gps_pd[" longitude"].to_numpy()
gps_head = gps_pd[" heading"].to_numpy()

gps_modem_np = np.zeros((modem_np.shape[0],3), dtype=np.float128)
j=0
for i in range(len(gps_times)):
    if(gps_times[i]>modem_np[j,0]):
        gps_modem_np[j,0] = gps_lat[i-1]
        gps_modem_np[j,1] = gps_long[i-1]
        j+=1
        gps_modem_np[j,2] = gps_head[i-1]
        if(j>=modem_np.shape[0]):
            break

gps_modem_err_np = np.zeros((modem_err_np.shape[0],3), dtype=np.float128)
j=0
for i in range(len(gps_times)):
    if(gps_times[i]>modem_err_np[j,0]):
        gps_modem_err_np[j,0] = gps_lat[i-1]
        gps_modem_err_np[j,1] = gps_long[i-1]
        gps_modem_err_np[j,2] = gps_head[i-1]
        j+=1
        if j>=modem_err_np.shape[0]:
            break
print(gps_modem_err_np)



def easting_northing(beacon):
    f_modem_np = modem_np[modem_np[:,1]==beacon]
    easting_meters = f_modem_np[:,18]/10
    northing_meters = f_modem_np[:,19]/10 
    plt.plot(-easting_meters, -northing_meters, 'g--o', markersize=3)
    plt.plot([0],[0],'rx', markersize=12)


def lat_long_b3_sucesses():
    rec_lat = gps_modem_np[:,0]
    rec_long = gps_modem_np[250:,1]
    plt.plot(rec_lat, rec_long, 'b--o')
 

def range_rssi():
    modem_2_np = modem_np[modem_np[:,1]==2]
    modem_3_np = modem_np[modem_np[:,1]==3]
    modem_4_np = modem_np[modem_np[:,1]==4]
    modem_5_np = modem_np[modem_np[:,1]==5]
    range_2_meters = modem_2_np[:,14]/10
    range_3_meters = modem_3_np[:,14]/10
    range_4_meters = modem_4_np[:,14]/10
    range_5_meters = modem_5_np[:,14]/10
    rssi_2 = modem_2_np[:,8]/10
    rssi_3 = modem_3_np[:,8]/10
    rssi_4 = modem_4_np[:,8]/10
    rssi_5 = modem_5_np[:,8]/10
    plt.plot(range_2_meters,rssi_2, 'o', markersize=3)
    plt.figure()
    plt.plot(range_3_meters,rssi_3, 'o', markersize=3)
    plt.figure()
    plt.plot(range_4_meters,rssi_4, 'o', markersize=3)
    plt.figure()
    plt.plot(range_5_meters,rssi_5, 'o', markersize=3)
    plt.figure()
    plt.plot()


def range_time():
    f_modem_np = modem_np[modem_np[:,1]==5]
    range_meters = f_modem_np[:,14]/10
    time_sec = [datetime.fromtimestamp(timestamp*10e-9) for timestamp in f_modem_np[:,0]]

    rssi_db = f_modem_np[:,8]/10#+f_modem_np[:,9]+f_modem_np[:,10])/(3*10)
    normed_rssi = (rssi_db.mean() - rssi_db)*(np.std(range_meters)/np.std(rssi_db)) + range_meters.mean()
    avgd_rssi_db = (f_modem_np[:,8]+f_modem_np[:,9]+f_modem_np[:,10])/(3*10)
    normed_avgd_rssi = (avgd_rssi_db.mean() - avgd_rssi_db)*(np.std(range_meters)/np.std(avgd_rssi_db)) + range_meters.mean()

    plt.plot(time_sec, rssi_db, 'r--o', markersize=3)
    plt.figure()
    plt.plot(time_sec, avgd_rssi_db, 'b--o', markersize=3)
    plt.figure()
    plt.plot(time_sec, range_meters, 'g--o', markersize=3)


# def rssi_time():
#     f_modem_np = modem_np[modem_np[:,1]==5]
#     range_meters = f_modem_np[:,14]/10
#     time_sec = [datetime.fromtimestamp(timestamp*10e-9) for timestamp in f_modem_np[:,0]]

#     rssi_db = f_modem_np[:,8]/10#+f_modem_np[:,9]+f_modem_np[:,10])/(3*10)
#     normed_rssi = (rssi_db.mean() - rssi_db)*(np.std(range_meters)/np.std(rssi_db)) + range_meters.mean()
#     avgd_rssi_db = (f_modem_np[:,8]+f_modem_np[:,9]+f_modem_np[:,10])/(3*10)
#     normed_avgd_rssi = (avgd_rssi_db.mean() - avgd_rssi_db)*(np.std(range_meters)/np.std(avgd_rssi_db)) + range_meters.mean()

#     #plt.plot(time_sec, normed_rssi, 'r--o', markersize=3)
#     #plt.plot(time_sec, normed_avgd_rssi, 'b--o', markersize=3)
#     plt.plot(time_sec, range_meters, 'g--o', markersize=3)
# 



def beacon_oriented_xy():
    range_meters = modem_np[:,14]/10
    azimuth = modem_np[:,15]/10
    x_vertical_axis = range_meters*np.cos(azimuth*2*np.pi/360)
    y_horiz_axis = range_meters*np.sin(-azimuth*2*np.pi/360)
    plt.plot(y_horiz_axis, x_vertical_axis, 'go')
    plt.plot([0],[0],'rx', markersize=12)



def lat_long_successes_failures():
    rec_lat = gps_modem_np[:,0]
    rec_long = gps_modem_np[:,1]
    err_lat = gps_modem_err_np[:,0]
    err_long = gps_modem_err_np[:,1]
    all_lat = gps_lat
    all_long = gps_long
    plt.plot(all_lat,all_long, '-', color='gray')
    plt.plot(err_lat, err_long, 'rx')
    plt.plot(rec_lat, rec_long, 'go')



def full_gps_path():
    plt.plot(gps_lat, gps_long, '-', color='gray')


def end_gps_path():
    all_lat = gps_lat[17210:]
    all_long = gps_long[17210:]
    plt.plot(all_lat, all_long, '-', color='gray')



def gps_on_predictions():
    f_modem_np = modem_np[modem_np[:,1]==5]
    easting_meters = f_modem_np[:,18]/10
    northing_meters = f_modem_np[:,19]/10
    rec_lat = (gps_modem_np[modem_np[:,1]==5])[:,0]
    rec_long = (gps_modem_np[modem_np[:,1]==5])[:,1]
    all_lat = gps_lat[17210:]
    all_long = gps_long[17210:]
    R = 6371000
    long_met = (rec_long-rec_long.mean())*(np.std(easting_meters)/np.std(rec_long)) + easting_meters.mean()
    lat_met = (rec_lat-rec_lat.mean())*(np.std(northing_meters)/np.std(rec_lat)) + northing_meters.mean() #*np.cos(all_long*2*np.pi/360)

    plt.plot(lat_met, long_met, '-', color='gray')
    plt.plot()
    plt.plot(easting_meters, northing_meters, 'g--o', markersize=3)
    plt.plot([0],[0],'rx', markersize=12)



def elevation_over_range():
    elevation = modem_np[:,16]/-10
    range_meters = modem_np[:,14]/10
    relative_depth = range_meters*np.cos(elevation*2*np.pi/360)
    horiz_range = range_meters*np.sin(elevation*2*np.pi/360)
    #plt.plot(range_meters, elevation, 'o', color='purple')
    plt.plot(relative_depth, horiz_range, 'o', color='purple')




if __name__ == '__main__':
    full_gps_path()
    plt.figure()
    #elevation_over_range()
    easting_northing(2)
    plt.figure()
    easting_northing(3)
    plt.figure()
    easting_northing(4)
    plt.figure()
    easting_northing(5)
    #plt.figure()
    # lat_long_b3_sucesses()
    # end_gps_path()
    # range_rssi()
    # range_time()
    #gps_on_predictions()
    #lat_long_successes_failures()
    # beacon_oriented_xy()
    plt.show()



