import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from scipy.spatial.transform import Rotation as R

# --- Modem GPS Locations in Decimal Degrees ---
modem_positions = {
    1: {'lon': -111.02801757, 'lat': 40.14641604, 'depth': 0.8128},  # modem 1
    2: {'lon': -111.02795235, 'lat': 40.14619182, 'depth': 1.164},  # modem 2
    15: {'lon': -111.02801804, 'lat': 40.14627160, 'depth': 1.3208}  # modem 15
}

central_modem = 15

def get_point_from_lat_lon(lat, lon):
    center = modem_positions[central_modem]
    m_per_degree = 111320  # Approximate conversion factor for latitude/longitude to meters
    x = (lon - center['lon']) * m_per_degree
    y = (lat - center['lat']) * m_per_degree
    return x, y

def plot_modem_locations():
    """
    Plot the modem locations on a map.
    """
    fig, ax = plt.subplots(figsize=(10, 10))
    
    # Plot each modem location
    for modem_id, position in modem_positions.items():
        x, y = get_point_from_lat_lon(position['lat'], position['lon'])
        ax.plot(x, y, 'o', label=f'Modem {modem_id}')
        ax.text(x + 0.1, y + 0.1, f'Modem {modem_id}', fontsize=12)

    ax.set_xlabel('X Coordinate (m)')
    ax.set_ylabel('Y Coordinate (m)')
    ax.set_title('Modem Locations')
    ax.legend()
    plt.grid()
    plt.show()



def rotate_vector(vector, pitch, roll, yaw):
    # DVL angles are in degrees, convert to radians
    roll_offset = 0  # Adjust if needed
    pitch_offset = 0  # Adjust if needed
    yaw_offset = 0  # Adjust if needed
    roll = roll + roll_offset
    pitch = pitch + pitch_offset
    yaw = yaw + yaw_offset

    r = R.from_euler('zyx', [yaw, pitch, roll], degrees=True)
    return r.apply(vector)



def plot_vehicle_location():
    data = get_csv_data('/home/benwash/cougars/bag/converted_bags/coug2/converted__2.0_lawnmower_varying_depth-2025-07-15-12-30-44/coug3.modem_rec.csv')
    dvl_data = get_csv_data('/home/benwash/cougars/bag/converted_bags/coug2/converted__2.0_lawnmower_varying_depth-2025-07-15-12-30-44/coug3.dvl.position.csv')
    data['range_c'] = data['range_dist'] / 10
    data['elevation_c'] = np.radians(data['usbl_elevation'] / 10)
    data['azimuth_c'] = np.radians(data['usbl_azimuth'] / 10 - 90)

    modem_ids = [1, 2, 15]
    colors = {1: 'blue', 2: 'green', 15: 'red'}

    fig, ax = plt.subplots(figsize=(10, 10))
    # ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X Coordinate (m)')
    ax.set_ylabel('Y Coordinate (m)')
    # ax.set_zlabel('Z Coordinate (m)')

    ax.set_title('Manual Step-through Vehicle Location')
    ax.grid(True)

    for modem_id in modem_ids:
        x, y = get_point_from_lat_lon(modem_positions[modem_id]['lat'], modem_positions[modem_id]['lon'])
        z = -modem_positions[modem_id]['depth']
        ax.plot([x], [y], 'x', label=f'Modem {modem_id}', color=colors[modem_id], markersize=10)
    ax.legend()


    # Set axis limits based on all points




    # Combine and sort all modem data by timestamp
    combined_data = pd.concat([
        data[data['src_id'] == modem_id].assign(modem_id=modem_id)
        for modem_id in modem_ids
    ]).sort_values(['header.stamp.sec', 'header.stamp.nanosec']).reset_index(drop=True)

    step_event = {'next': False}

    def on_key(event):
        if event.key == ' ':
            step_event['next'] = False

    fig.canvas.mpl_connect('key_press_event', on_key)

    # Loop through each row in timestamp order
    modem_scatters = {modem_id: [] for modem_id in modem_ids}

    # Loop through each row in timestamp order
    for idx, row in combined_data.iterrows():
        modem_id = row['src_id']
        modem_x = row['range_c'] * np.cos(row['elevation_c']) * np.cos(row['azimuth_c'])
        modem_y = row['range_c'] * np.cos(row['elevation_c']) * np.sin(row['azimuth_c'])
        modem_z = row['range_c'] * np.sin(row['elevation_c'])

        modem_pos_x, modem_pos_y = get_point_from_lat_lon(modem_positions[modem_id]['lat'], modem_positions[modem_id]['lon'])
        modem_pos_z = -modem_positions[modem_id]['depth']

        # Find matching DVL index
        i = 0
        seconds = row['header.stamp.sec']
        nanoseconds = row['header.stamp.nanosec']
        while dvl_data['header.stamp.sec'].iloc[i] < seconds or (
            dvl_data['header.stamp.sec'].iloc[i] == seconds and dvl_data['header.stamp.nanosec'].iloc[i] < nanoseconds):
            i += 1

        vector = np.array([modem_x, modem_y, modem_z])
        rotated_vector = rotate_vector(vector, dvl_data['pitch'].iloc[i-1], dvl_data['roll'].iloc[i-1], dvl_data['yaw'].iloc[i-1])
        final_vector = rotated_vector + np.array([modem_pos_x, modem_pos_y, modem_pos_z])
        # if modem_id == 1:
        #     final_vector[0] += 5
        #     final_vector[1] += -33
        # if modem_id == 2:
        #     final_vector[0] += -5
        #     final_vector[1] += 15
        # final_vector[2] =0
        step_event['next'] = False

        # Remove oldest scatter if more than 4 exist
        if len(modem_scatters[modem_id]) >= 5:
            scatter_to_remove = modem_scatters[modem_id].pop(0)
            scatter_to_remove.remove()

        # Plot new point and store scatter
        scatter = ax.scatter(final_vector[0], final_vector[1], color=colors[modem_id], alpha=0.7, label=f'Modem {modem_id}')
        modem_scatters[modem_id].append(scatter)

        fig.canvas.draw()
        print(f"Modem {modem_id} - x: {final_vector[0]:.2f}, y: {final_vector[1]:.2f}, z: {final_vector[2]:.2f}")
        plt.pause(0.0001)

    plt.show()

def get_csv_data(file_path):
    return pd.read_csv(file_path)



# plot_modem_locations()
plot_vehicle_location()