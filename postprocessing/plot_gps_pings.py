from utils import plotter_utils as p_utils
import matplotlib.pyplot as plt


def plot_gps(ax, cen_lat, cen_long, latlist, lonlist):
    xs = []
    ys = []
    lastx, lasty = p_utils.CalculateHaversine(cen_lat,cen_long,latlist[0],lonlist[0])
    xs.append(lastx)
    ys.append(lasty)

    for i in range(1,len(latlist)):
        x,y = p_utils.CalculateHaversine(cen_lat,cen_long,latlist[i],lonlist[i])
        print(f"Lat: {latlist[i]}, Lon: {lonlist[i]}")
        print(f"({x},{y})")
        ax.plot(
                    [x, lastx],
                    [y, lasty],
                    color='black', linewidth=1
                )
        lasty=y
        lastx=x
        xs.append(x)
        ys.append(y)

    ax.set_xlim(min(xs) - 5, max(xs) + 5)  
    ax.set_ylim(min(ys) - 5, max(ys) + 5)
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.set_title("Vehicle GPS Path")
    ax.set_aspect('equal')  # Optional: keep square aspect so path isn’t stretched
    ax.scatter(xs, ys, color='red', s=10)


