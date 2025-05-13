import tkinter
import tkinter.messagebox
import tkinter.simpledialog
from tkinter import ttk
from tkinter import filedialog
import tkintermapview
import os

class App:
    def __init__(self, root_widget):
        self.root_widget = root_widget
        self.root_widget.title("Waypoint Mission Planner")
        self.root_widget.geometry("1000x700")

        # --- Data Storage ---
        self.origin_data = None  # Will store {'lat': float, 'lon': float, 'alt': float, 'heading': float, 'marker': TkinterMapView.Marker}
        self.waypoints = []      # List of {'lat': float, 'lon': float, 'marker': TkinterMapView.Marker, 'map_marker_object': object}

        # --- UI Elements ---
        # Left Frame for Controls and Waypoint List
        self.control_frame = ttk.Frame(self.root_widget, padding=10)
        self.control_frame.pack(side=tkinter.LEFT, fill=tkinter.Y)

        # Origin Section
        ttk.Label(self.control_frame, text="Mission Origin", font=("Arial", 14, "bold")).pack(pady=(0, 5))
        self.set_origin_button = ttk.Button(self.control_frame, text="Set Origin", command=self.prompt_origin)
        self.set_origin_button.pack(pady=5, fill=tkinter.X)
        self.origin_info_label = ttk.Label(self.control_frame, text="Origin not set.", wraplength=280)
        self.origin_info_label.pack(pady=5)

        # Waypoints Section
        ttk.Label(self.control_frame, text="Waypoints", font=("Arial", 14, "bold")).pack(pady=(10, 5))
        self.waypoints_listbox = tkinter.Listbox(self.control_frame, height=15)
        self.waypoints_listbox.pack(pady=5, fill=tkinter.BOTH, expand=True)
        
        self.clear_waypoints_button = ttk.Button(self.control_frame, text="Clear All Waypoints", command=self.clear_mission_confirmation)
        self.clear_waypoints_button.pack(pady=5, fill=tkinter.X)

        # Mission Actions Section
        ttk.Label(self.control_frame, text="Mission Actions", font=("Arial", 14, "bold")).pack(pady=(10, 5))
        self.save_mission_button = ttk.Button(self.control_frame, text="Save Mission to .waypoints File", command=self.save_mission_to_file)
        self.save_mission_button.pack(pady=5, fill=tkinter.X)

        # Right Frame for Map
        self.map_frame = ttk.Frame(self.root_widget)
        self.map_frame.pack(side=tkinter.RIGHT, fill=tkinter.BOTH, expand=True)

        self.map_widget = tkintermapview.TkinterMapView(self.map_frame, width=700, height=700, corner_radius=0)
        self.map_widget.pack(fill=tkinter.BOTH, expand=True)
        # Set initial position (e.g., Provo, Utah or a global view)
        self.map_widget.set_position(40.2338, -111.6585) # Provo, UT
        self.map_widget.set_zoom(5)

        # Add right-click menu for adding waypoints
        self.map_widget.add_right_click_menu_command(label="Add Waypoint Here",
                                                     command=self.add_waypoint_on_map_click,
                                                     pass_coords=True)
        
        self._update_ui_state()

    def _update_ui_state(self):
        """Enable/disable buttons based on current state."""
        if self.origin_data:
            self.save_mission_button.config(state=tkinter.NORMAL)
            self.clear_waypoints_button.config(state=tkinter.NORMAL)
        else:
            self.save_mission_button.config(state=tkinter.DISABLED)
            self.clear_waypoints_button.config(state=tkinter.DISABLED)


    def prompt_origin(self):
        """Prompts the user for origin details."""
        try:
            lat_str = tkinter.simpledialog.askstring("Origin Latitude", "Enter Origin Latitude (e.g., 40.23851040):", parent=self.root_widget)
            if lat_str is None: return
            lat = float(lat_str)

            lon_str = tkinter.simpledialog.askstring("Origin Longitude", "Enter Origin Longitude (e.g., -111.73908950):", parent=self.root_widget)
            if lon_str is None: return
            lon = float(lon_str)

            alt_str = tkinter.simpledialog.askstring("Origin Altitude (meters)", "Enter Origin Altitude (e.g., 1400.0):", parent=self.root_widget)
            if alt_str is None: return
            alt = float(alt_str)

            head_str = tkinter.simpledialog.askstring("Origin Heading (degrees)", "Enter Origin Heading (0-360, e.g., 0.0):", parent=self.root_widget)
            if head_str is None: return
            heading = float(head_str)
            if not (0 <= heading <= 360):
                tkinter.messagebox.showerror("Invalid Input", "Heading must be between 0 and 360 degrees.")
                return

        except ValueError:
            tkinter.messagebox.showerror("Invalid Input", "Please enter valid numbers for all fields.")
            return
        except Exception as e:
            tkinter.messagebox.showerror("Error", f"An unexpected error occurred: {e}")
            return

        # Clear existing origin and waypoints if setting a new origin
        if self.origin_data or self.waypoints:
            self.clear_mission_data(show_confirmation=False)


        self.origin_data = {'lat': lat, 'lon': lon, 'alt': alt, 'heading': heading}
        
        # Add marker for origin
        origin_marker = self.map_widget.set_marker(lat, lon, text="H (Origin)", text_color="red", marker_color_circle="red")
        self.origin_data['map_marker_object'] = origin_marker


        self.origin_info_label.config(text=f"Origin: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.2f}m, Head={heading:.1f}°")
        self.map_widget.set_position(lat, lon)
        self.map_widget.set_zoom(15)
        
        self._update_waypoint_display()
        self._update_ui_state()
        tkinter.messagebox.showinfo("Origin Set", "Origin set successfully. You can now right-click on the map to add waypoints.")


    def add_waypoint_on_map_click(self, coords):
        """Adds a waypoint when the map is clicked."""
        if not self.origin_data:
            tkinter.messagebox.showwarning("Set Origin First", "Please set the mission origin before adding waypoints.")
            return

        lat, lon = coords
        wp_index = len(self.waypoints) + 1
        
        # Add marker for waypoint
        # Note: tkintermapview doesn't directly return the marker object from set_marker in all versions in a way that's easy to delete individually without custom tracking.
        # We'll store the coordinates and re-add markers if clearing. For simplicity, we'll clear all and re-add.
        # A more robust way would be to extend tkintermapview or manage marker objects carefully.
        map_marker = self.map_widget.set_marker(lat, lon, text=f"WP{wp_index}")
        
        self.waypoints.append({'lat': lat, 'lon': lon, 'map_marker_object': map_marker})
        self._update_waypoint_display()
        self._update_ui_state()

    def _update_waypoint_display(self):
        """Updates the listbox showing waypoints."""
        self.waypoints_listbox.delete(0, tkinter.END)
        if self.origin_data:
            self.waypoints_listbox.insert(tkinter.END, f"HOME: Lat={self.origin_data['lat']:.6f}, Lon={self.origin_data['lon']:.6f}, Alt={self.origin_data['alt']:.2f}m, Hdg={self.origin_data['heading']:.1f}°")
        
        for i, wp in enumerate(self.waypoints):
            # Altitude for subsequent waypoints is relative (0) to home's altitude.
            self.waypoints_listbox.insert(tkinter.END, f"WP {i+1}: Lat={wp['lat']:.6f}, Lon={wp['lon']:.6f} (Alt: Home + 0.0m)")
    
    def clear_mission_confirmation(self):
        if tkinter.messagebox.askyesno("Confirm Clear", "Are you sure you want to clear the current origin and all waypoints?"):
            self.clear_mission_data(show_confirmation=True)

    def clear_mission_data(self, show_confirmation=True):
        """Clears all mission data."""
        # Clear map markers
        self.map_widget.delete_all_marker()
        # Potentially re-add origin marker if it was cleared by delete_all_marker and we still have origin data
        # However, standard flow is to clear data first.

        self.origin_data = None
        self.waypoints = []
        
        self.origin_info_label.config(text="Origin not set.")
        self.map_widget.set_position(40.2338, -111.6585) # Reset to default view
        self.map_widget.set_zoom(5)
        
        self._update_waypoint_display()
        self._update_ui_state()
        if show_confirmation: # Avoid double message if called internally
             tkinter.messagebox.showinfo("Cleared", "Mission data cleared.")


    def save_mission_to_file(self):
        """Saves the mission to a .waypoints file."""
        if not self.origin_data:
            tkinter.messagebox.showerror("Error", "Cannot save mission. Origin not set.")
            return

        filepath = filedialog.asksaveasfilename(
            defaultextension=".waypoints",
            filetypes=[("Waypoint Files", "*.waypoints"), ("All Files", "*.*")],
            title="Save Mission File"
        )

        if not filepath:
            return # User cancelled

        try:
            with open(filepath, 'w') as f:
                f.write("QGC WPL 110\n")

                # Home position (Index 0)
                # Format: INDEX CURRENT_WP COORD_FRAME COMMAND PARAM1 PARAM2 PARAM3 PARAM4 LATITUDE LONGITUDE ALTITUDE AUTOCONTINUE
                # Home: Index 0, Current 1, Frame 0 (Global), Command 0 (special handling for home)
                # Param4 for home is heading. Altitude is absolute.
                home_line = (
                    f"0\t1\t0\t0\t"  # Index, Current, Frame, Command
                    f"0.00000000\t0.00000000\t0.00000000\t"  # Param1, Param2, Param3
                    f"{self.origin_data['heading']:.8f}\t"  # Param4 (Heading)
                    f"{self.origin_data['lat']:.8f}\t"      # Latitude
                    f"{self.origin_data['lon']:.8f}\t"      # Longitude
                    f"{self.origin_data['alt']:.6f}\t"      # Altitude (Absolute for Home)
                    f"1\n"                                  # Autocontinue
                )
                f.write(home_line)

                # Subsequent Waypoints (Index 1 onwards)
                # Frame 3 (Global Relative Alt), Command 16 (NAV_WAYPOINT)
                # Altitude for these waypoints is 0.0, meaning relative to Home's altitude.
                for i, wp in enumerate(self.waypoints):
                    wp_index = i + 1
                    wp_line = (
                        f"{wp_index}\t0\t3\t16\t"  # Index, Current, Frame, Command
                        f"0.00000000\t0.00000000\t0.00000000\t0.00000000\t"  # Param1-4 (Param4 Yaw is 0)
                        f"{wp['lat']:.8f}\t"      # Latitude
                        f"{wp['lon']:.8f}\t"      # Longitude
                        f"0.000000\t"             # Altitude (Relative to Home, 0 means same alt as home)
                        f"1\n"                    # Autocontinue
                    )
                    f.write(wp_line)
            
            tkinter.messagebox.showinfo("Success", f"Mission saved to {os.path.basename(filepath)}")

        except Exception as e:
            tkinter.messagebox.showerror("File Save Error", f"Could not save mission file: {e}")


if __name__ == "__main__":
    root = tkinter.Tk()
    app = App(root)
    root.mainloop()
