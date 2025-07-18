import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.collections import LineCollection
from scipy.spatial.transform import Rotation as R
from scipy.optimize import minimize

modem_positions = {
    1: {'lon': -111.02801757, 'lat': 40.14641604, 'depth': 0.8128},  # modem 1
    2: {'lon': -111.02795235, 'lat': 40.14619182, 'depth': 1.164},  # modem 2
    15: {'lon': -111.02801804, 'lat': 40.14627160, 'depth': 1.3218}  # modem 15
}

modem_positions_corrected = {
    1: {'x': 0, 'y': 0, 'z': 0},  # modem 1
    2: {'x': 0, 'y': 0, 'z': 0},  # modem 2
    15: {'x': 0, 'y': 0, 'z': 0}  # modem 15
}
modem_colors = {
    1: 'green',
    2: 'red',
    15: "blue"
}

central_modem = 15

for modem in modem_positions:
    center = modem_positions[central_modem]
    m_per_degree = 111321  # Approximate conversion factor for latitude/longitude to meters
    x = (modem_positions[modem]['lon'] - center['lon']) * m_per_degree
    y = (modem_positions[modem]['lat'] - center['lat']) * m_per_degree
    z = -modem_positions[modem]['depth']
    modem_positions_corrected[modem]['x'] = x
    modem_positions_corrected[modem]['y'] = y
    modem_positions_corrected[modem]['z'] = z

bag = '/home/benwash/cougars/bag/converted_bags/coug2/converted__2.0_lawnmower_varying_depth-2025-07-15-12-30-44'
modem_rec_data = pd.read_csv(bag + '/coug3.modem_rec.csv')
dvl_data = pd.read_csv(bag + '/coug3.dvl.position.csv')

modem_lists = {1: [], 2: [], 15: []}
global modem_arrays
def plot_modem_locations(ax):
    for modem_id, position in modem_positions_corrected.items():
        x = modem_positions_corrected[modem_id]['x']
        y = modem_positions_corrected[modem_id]['y']
        z = modem_positions_corrected[modem_id]['z']
        ax.plot(x, y, z, 'o', color=modem_colors[modem_id],label=f'Modem {modem_id}')
        ax.text(x + 0.1, y + 0.1, z + 0.1, f'Modem {modem_id}', fontsize=12)

def plot_vehicle_location(fig,ax):
    modem_scatters = {modem_id: [] for modem_id in modem_colors}
    dvl_scatter = []
    for i in range(len(modem_rec_data)):

        # scatter = ax.scatter(dvl_data['position.x'][i*21], dvl_data['position.y'][i*21], dvl_data['position.z'][i*21], color='purple', alpha=0.7)
        # dvl_scatter.append(scatter)
        # if len(dvl_scatter) >= 5:
        #     scatter_to_remove = dvl_scatter.pop(0)
        #     scatter_to_remove.remove()


        modem_id, range_dist, azimuth, elevation, seconds, nanosec = get_modem_data(i)
        x, y, z = spherical_to_cartesian(range_dist, azimuth, elevation)
        roll, pitch, yaw = get_orientation(seconds, nanosec)
        # yaw -=90
        x_rot, y_rot, z_rot = account_for_vehicle_orienation(x, y, z, roll, pitch, yaw)
        x_fixed_frame, y_fixed_frame, z_fixed_frame = fix_frame(x_rot,y_rot,z_rot,modem_id)
        # x_fixed_frame, y_fixed_frame, z_fixed_frame = apply_offset(modem_id, x_fixed_frame, y_fixed_frame, z_fixed_frame)
        scatter = ax.scatter(x_fixed_frame, y_fixed_frame, z_fixed_frame,color=modem_colors[modem_id], s=15)
        modem_scatters[modem_id].append(scatter)
        if len(modem_scatters[modem_id]) >= 5:
            scatter_to_remove = modem_scatters[modem_id].pop(0)
            scatter_to_remove.remove()
        modem_lists[modem_id].append([x_fixed_frame, y_fixed_frame, z_fixed_frame])
        plt.pause(0.01)
    global modem_arrays
    modem_arrays = {mid: np.array(arr) for mid, arr in modem_lists.items()}

def apply_offset(id,x,y,z):
    y-=10
    if id == 1:
        return x+1.95169983, y-28.05396889,  z-0.46783286
    if id == 2:
        return x-5.04262761, y+17.24996007, z-0.71939448
    return x,y,z
    

def account_for_vehicle_orienation(x, y, z, roll, pitch, yaw):
    # Vector in body frame
    vector = np.array([x, y, z])

    # Create rotation matrix using yaw (Z), pitch (Y), roll (X), in degrees
    r = R.from_euler('zyx', [yaw, pitch, roll], degrees=True)

    # Apply the rotation
    rotated_vector = r.apply(vector)
    return rotated_vector[0], rotated_vector[1], rotated_vector[2]

def fix_frame(x,y,z,modem_id):
    x = x + modem_positions_corrected[modem_id]['x']
    y = y + modem_positions_corrected[modem_id]['y']
    z = z + modem_positions_corrected[modem_id]['z']
    return x, y, z

def spherical_to_cartesian(range_d, azimuth, elevation):
    elevation = np.radians(elevation)
    azimuth = np.radians(azimuth)
    x = range_d * np.cos(elevation) * np.cos(azimuth)
    y = range_d * np.cos(elevation) * np.sin(azimuth)
    z = range_d * np.sin(elevation)
    return x, y, z

def get_modem_data(point):

    range = modem_rec_data['range_dist'][point] / 10
    azimuth = modem_rec_data['usbl_azimuth'][point] / 10
    elevation = modem_rec_data['usbl_elevation'][point] / 10
    seconds = modem_rec_data['header.stamp.sec'][point]
    nanosec = modem_rec_data['header.stamp.nanosec'][point]
    modem_id = modem_rec_data['src_id'][point]
    return modem_id, range, azimuth, elevation, seconds, nanosec

def get_orientation(seconds, nanoseconds):
    mask = (
        (dvl_data['header.stamp.sec'] < seconds) | ((dvl_data['header.stamp.sec'] == seconds) & (dvl_data['header.stamp.nanosec'] <= nanoseconds))
    )
    filtered = dvl_data[mask]
    if filtered.empty:
        return dvl_data['roll'][0], dvl_data['pitch'][0], dvl_data['yaw'][0]
    last_idx = filtered.index[-1]
    return dvl_data.loc[last_idx, 'roll'], dvl_data.loc[last_idx, 'pitch'], dvl_data.loc[last_idx, 'yaw']



def position_error(offset, computed_positions, reference_positions):
    # Apply offset and calculate mean squared error to references
    corrected = computed_positions + offset
    return np.sqrt(((corrected - reference_positions)**2).mean())

def main():
    fig, ax = plt.subplots(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    plot_modem_locations(ax)
    plot_vehicle_location(fig, ax)
    plt.show()


    # computed_positions = modem_arrays[1]
    # reference_positions = modem_arrays[15]
    # min_len = min(computed_positions.shape[0], reference_positions.shape[0])
    # computed_aligned = computed_positions[:min_len]
    # reference_aligned = reference_positions[:min_len]

    # result = minimize(
    #     position_error, 
    #     x0=np.zeros(3),    # initial guess: no offset
    #     args=(computed_aligned, reference_aligned)
    # )

    # best_offset = result.x  # Apply this to your results
    # print(f"X: {result.x}")




main()