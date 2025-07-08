# Created by Seth Ricks, July 2025
import sys 
import random, time, os
import yaml
import json
from PyQt6.QtWidgets import (QScrollArea, QApplication, QMainWindow, 
    QWidget, QPushButton, QTabWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame,
    QSizePolicy, QSplashScreen, QCheckBox, QSpacerItem, QGridLayout, QToolBar, QSlider,
    QStyle, QLineEdit, QWidget, QDialog, QFileDialog, QDialogButtonBox, QMessageBox, QColorDialog, QStatusBar
)

from PyQt6.QtCore import QSize, Qt, QTimer, pyqtSignal, QObject, QEvent

from PyQt6.QtGui import QColor, QPalette, QFont, QPixmap, QKeySequence, QShortcut, QCursor, QPainter, QAction, QIcon, QActionGroup

from base_station_interfaces.srv import BeaconId, ModemControl
from functools import partial
from transforms3d.euler import quat2euler
import math
from base_station_gui2.temp_mission_control import deploy
from base_station_gui2.vehicles_calibrate import calibrate
from base_station_gui2.cougars_bringup.scripts import startup_call
import tkinter
from base_station_gui2.temp_waypoint_planner.temp_waypoint_planner import App as WaypointPlannerApp
import threading
import subprocess
import multiprocessing

class MainWindow(QMainWindow):
    # Initializes GUI window with a ros node inside
    update_connections_signal = pyqtSignal(object)
    update_console_signal = pyqtSignal(object, int)
    kill_confirm_signal = pyqtSignal(object)
    safety_status_signal = pyqtSignal(int, object)
    smoothed_ouput_signal = pyqtSignal(int, object)
    depth_data_signal = pyqtSignal(int, object)
    pressure_data_signal = pyqtSignal(int, object)
    battery_data_signal = pyqtSignal(int, object)
    surface_confirm_signal = pyqtSignal(object)
    update_wifi_signal = pyqtSignal(dict)

    def __init__(self, ros_node, vehicle_list):
        """
        Initializes GUI window with a ros node inside

        Parameters:
            ros_node (node): node passed in from ros in order to access the publisher
        """
        
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle(" ")

        self.button_padding = 15
        self.button_font_size = 15

        #light_mode, dark_mode, blue_pastel, brown_sepia, intense_dark, cadetblue
        self.set_color_theme("dark_mode", first_time=True) #default to dark mode

        # Create an exclusive action group for theme actions
        theme_action_group = QActionGroup(self)
        theme_action_group.setExclusive(True)

        dark_mode = QAction("Dark Mode", self)
        dark_mode.triggered.connect(lambda: self.set_color_theme("dark_mode"))
        dark_mode.setCheckable(True)
        theme_action_group.addAction(dark_mode)

        light_mode = QAction("Light Mode", self)
        light_mode.triggered.connect(lambda: self.set_color_theme("light_mode"))
        light_mode.setCheckable(True)
        theme_action_group.addAction(light_mode)

        blue_pastel = QAction("Blue Pastel", self)
        blue_pastel.triggered.connect(lambda: self.set_color_theme("blue_pastel"))
        blue_pastel.setCheckable(True)
        theme_action_group.addAction(blue_pastel)

        brown_sepia = QAction("Brown Sepia", self)
        brown_sepia.triggered.connect(lambda: self.set_color_theme("brown_sepia"))
        brown_sepia.setCheckable(True)
        theme_action_group.addAction(brown_sepia)

        intense_dark = QAction("Intense Dark", self)
        intense_dark.triggered.connect(lambda: self.set_color_theme("intense_dark"))
        intense_dark.setCheckable(True)
        theme_action_group.addAction(intense_dark)

        intense_light = QAction("Intense Light", self)
        intense_light.triggered.connect(lambda: self.set_color_theme("intense_light"))
        intense_light.setCheckable(True)
        theme_action_group.addAction(intense_light)
        
        cadetblue = QAction("Cadetblue", self)
        cadetblue.triggered.connect(lambda: self.set_color_theme("cadetblue"))
        cadetblue.setCheckable(True)
        theme_action_group.addAction(cadetblue)

        # Set the default checked action
        dark_mode.setChecked(True)

        self.setStatusBar(QStatusBar(self))
        menu = self.menuBar()
        file_menu = menu.addMenu("Theme")
        file_submenu = file_menu.addMenu("Set Theme")
        file_submenu.addAction(dark_mode)
        file_submenu.addAction(light_mode)
        file_submenu.addAction(blue_pastel)
        file_submenu.addAction(brown_sepia)        
        file_submenu.addAction(intense_dark)
        file_submenu.addAction(intense_light)
        file_submenu.addAction(cadetblue)

        self.selected_vehicles = vehicle_list

        #confirmation label dictionary
        self.confirm_reject_labels = {}

        ###This is how the vehicle info gets into the GUI
        self.feedback_dict = {
            #0->negative, 1->positive, 2->waiting
            #Vehicles 1-3 connections
            "Wifi": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},
            "Radio": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},
            "Modem": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},

            #0->negative, 1->positive, 2->waiting
            #Vehicles 1-3 sensors
            "DVL": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},
            "GPS": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},
            "IMU": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},
            "Battery": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},

            #Vehicles 1-3 status messages
            "Status_messages": {vehicle_num: "" for vehicle_num in self.selected_vehicles},

            #Vehicles 1-3 message logs, lists of strings
            "Console_messages": {vehicle_num: [] for vehicle_num in self.selected_vehicles},

            #Vehicles 1-3 message logs, lists of strings
            "Missions": {vehicle_num: "" for vehicle_num in self.selected_vehicles},    

            #Vehicles 1-3 seconds since last connection, list of ints
            "Modem_seconds": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},    

            #Vehicles 1-3 seconds since last radio connection, list of ints
            "Radio_seconds": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},     

            #Vehicles 1-3 X Position in the DVL frame
            "XPos": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},        
            
            #Vehicles 1-3 Y Position in the DVL frame
            "YPos": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},

            #Vehicles 1-3 Depth, list of ints
            "Depth": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},

            #Vehicles 1-3 Heading, list of ints
            "Heading": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},

            #Vehicles 1-3 Waypoint, list of ints
            "Waypoint": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},

            #Vehicles 1-3 Linear Velocities, list of ints
            "DVL_vel": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},

            #Vehicles 1-3 Angular Velocities, list of ints
            "Angular_vel": {vehicle_num: 2 for vehicle_num in self.selected_vehicles},

            #Vehicles 1-3 Pressures, list of ints
            "Pressure": {vehicle_num: 2 for vehicle_num in self.selected_vehicles}
        }

        #this is used to connect the feedback_dict keys to what is actually printed, in _update_status_gui
        self.key_to_text_dict = {
            "XPos": "x (m): ",
            "YPos": "y (m): ",
            "Depth": "Depth (m): ",
            "Heading": "Heading (deg): ",
            "Waypoint": "Current Waypoint: ",
            "DVL_vel": "DVL Velocity <br>(m/s): ",
            "Battery": "Battery (V): ",
            "Pressure": "Pressure (Pa): ",
            "Angular_vel": "Angular Velocity <br>(rad/s): "
        }

        self.option_map = {
            "Start the node": "start_node",
            "Record rosbag": "record_rosbag",
            "Enter rosbag prefix (string): ": "rosbag_prefix",
            "Arm Thruster": "arm_thruster",
            "Start DVL": "start_dvl"
        }

        #This is a dictionary to map the feedback_dict to the correct symbols
        #"x" symbol -> SP_MessageBoxCritical
        #"check" symbol -> SP_DialogApplyButton
        # "waiting" symbol -> SP_TitleBarContextHelpButton
        self.icons_dict = {
            0: QStyle.StandardPixmap.SP_MessageBoxCritical,
            1: QStyle.StandardPixmap.SP_DialogApplyButton,
            2: QStyle.StandardPixmap.SP_TitleBarContextHelpButton
        }

        #Create the tabs
        self.tabs = QTabWidget()
        #Orient the tabs at the tob of the screen
        self.tabs.setTabPosition(QTabWidget.TabPosition.North)
        #The tabs' order can't be changed or moved
        self.tabs.setMovable(False)

        #Placeholders for the tabs layout, to be accessed later. 
        tab_names = ["General"] + [f"Vehicle {i}" for i in self.selected_vehicles]
        self.tab_dict = {name: [None, QHBoxLayout()] for name in tab_names}

        #create the widgets from the tab dict, assign layouts, and add each to self.tabs
        for name in self.tab_dict:
            # Create content layout
            content_widget = QWidget()
            content_layout = QVBoxLayout()

            # Main content widget
            content = QWidget()
            content.setLayout(self.tab_dict[name][1])

            # Add line and content to layout
            content_layout.addWidget(self.make_hline())

            label = QLabel("Confirmation/Rejection messages from command buttons will appear here")
            label.setStyleSheet(f"color: {self.text_color}; font-size: 14px;") 
            self.confirm_reject_labels[name] = label

            if name.lower() != "general":
                vehicle_number = int(name.split()[-1])  # This works for any number of digits in custom numbers
                # For Vehicle tabs, add specific widgets and console log
                content_layout.addWidget(self.set_specific_vehicle_widgets(vehicle_number))
                content_layout.addWidget(self.make_hline())
                content_layout.addWidget(self.create_specific_vehicle_console_log(vehicle_number))
                content_layout.addWidget(label, alignment=Qt.AlignmentFlag.AlignTop)
            else:
                # For General tab, add general widgets
                content_layout.addWidget(content)
                self.set_general_page_widgets()
                content_layout.addWidget(label, alignment=Qt.AlignmentFlag.AlignTop)

            # Set the combined layout
            content_widget.setLayout(content_layout)
            self.tab_dict[name][0] = content_widget

            # Add to tabs
            self.tabs.addTab(content_widget, name)
            self.set_background(content_widget, self.background_color)

        # Connect tab change to scroll-to-bottom for console logs
        self.tabs.currentChanged.connect(self.scroll_console_to_bottom_on_tab)

        #The overall layout is vertical
        self.main_layout = QVBoxLayout()
        #Add the tabs to the main layout
        self.main_layout.addWidget(self.tabs)

        #create a container widget, and place the main layout inside of it
        self.container = QWidget()
        self.container.setObjectName("MyContainer")
        self.container.setLayout(self.main_layout)
        #the container with the main layout is set as the central widget
        self.setCentralWidget(self.container)

        #creates a signal that is sent from recieve_connections, and sent to _update_connections_gui. 
        #this avoids the error of the gui not working on the main thread
        self.update_connections_signal.connect(self._update_connections_gui)
        self.update_console_signal.connect(self._update_console_gui)
        self.kill_confirm_signal.connect(self._update_kill_confirmation_gui)
        self.surface_confirm_signal.connect(self._update_surf_confirmation_gui)
        self.safety_status_signal.connect(self._update_safety_status_information)
        self.smoothed_ouput_signal.connect(self._update_gui_smoothed_output)
        self.depth_data_signal.connect(self.update_depth_data)
        self.pressure_data_signal.connect(self.update_pressure_data)
        self.battery_data_signal.connect(self.update_battery_data)
        self.update_wifi_signal.connect(self.update_wifi_widgets)

        self.get_IP_addresses()
        self.recieve_console_update(f"These are the Vehicle IP Addresses that were both selected and in the config.json: {self.Vehicle_IP_addresses}", 0) #declared in get_IP_addresses

        self.ping_timer = QTimer(self)
        self.ping_timer.timeout.connect(self.ping_vehicles_via_wifi)
        self.ping_timer.start(3000) #try to ping the vehicles every 3 seconds 

    def get_IP_addresses(self):
        config_path = os.path.join(
            os.path.dirname(__file__),
            "temp_mission_control",
            "deploy_config.json"
        )
        with open(config_path, "r") as f:
            config = json.load(f)
        vehicles = config["vehicles"]
        self.Vehicle_IP_addresses = []
        self.ip_to_vehicle = {} 
        for num in self.selected_vehicles:
            if str(num) in vehicles:
                ip = vehicles[str(num)]['remote_host']
                self.Vehicle_IP_addresses.append(ip)
                self.ip_to_vehicle[ip] = num 
            else:
                err_msg = f"❌ Vehicle {num} not found in config, consider adding to config.json"
                self.recieve_console_update(err_msg, num)

    def ping_vehicles_via_wifi(self):
        def do_ping():
            try:
                IPs_reachable = {}
                for ip in self.Vehicle_IP_addresses:
                    result = subprocess.run(["ping", "-c", "1", "-W", "2", ip], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                    reachable = 1 if result.returncode == 0 else 0
                    IPs_reachable[ip] = reachable
                self.update_wifi_signal.emit(IPs_reachable)
            except Exception as e:
                print("Exception in do_ping:", e)
        threading.Thread(target=do_ping, daemon=True).start()

    def update_wifi_widgets(self, IPs_dict):
        try:
            for ip, reachable in IPs_dict.items():
                vehicle_number = self.ip_to_vehicle.get(ip)
                if vehicle_number is None:
                    print(f"IP {ip} not found in ip_to_vehicle mapping.")
                    continue
                wifi_status = self.feedback_dict["Wifi"][vehicle_number]
                if wifi_status != reachable:
                    self.recieve_console_update(
                        f"{'Ping successful for' if reachable == 1 else 'Unable to Ping'} vehicle{vehicle_number}",
                        vehicle_number
                    )
                    self.feedback_dict["Wifi"][vehicle_number] = reachable
                    self.replace_general_page_icon_widget(vehicle_number, "Wifi")
                    self.replace_specific_icon_widget(vehicle_number, "Wifi")
                    self.modem_shut_off_service(bool(reachable), vehicle_number)

        except Exception as e:
                print("Exception in update_wifi_widgets:", e)

    def set_color_theme(self, color_theme, first_time=False):
        
        # Define a base style for pop-up windows
        base_pop_up_style = """
            QDialog {{
                background-color: {bg};
                color: {text};
            }}
            
            QLabel, QCheckBox {{
                color: {text};
            }}

            QCheckBox::indicator {{
                width: 13px;
                height: 13px;
            }}

            QLineEdit {{
                background-color: {bg};
                color: {text};
                border: 1px solid {text};
                padding: 2px;
            }}
        """

        # Extra checkbox rules for dark themes
        extra_checkbox_rules = """
            QCheckBox::indicator:checked {{
                border: 1px solid {text};
            }}

            QCheckBox::indicator:unchecked {{
                background-color: {text};
                border: 1px solid {text};
            }}
        """

        theme = color_theme.lower()

        #dark mode
        if theme == "dark_mode":
            self.background_color = "#0F1C37"

            self.border_outline = "#FFFFFF"
            self.text_color = "#FFFFFF"
            self.normal_button_color = "#28625a"
            self.danger_button_color = "#953f10"
            self.danger_button_style_sheet = f"background-color: {self.danger_button_color}; color: {self.text_color}; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.normal_button_style_sheet = f"background-color: {self.normal_button_color}; color: {self.text_color}; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.selected_tab_color = self.text_color
            self.selected_tab_text_color = self.background_color
            self.not_selected_tab_color = self.background_color
            self.not_selected_tab_text_color = self.text_color
            self.check_box_color = self.text_color
            self.dark_icon_bkgrnd_color = self.text_color
            self.light_icon_bkgrnd_color = self.background_color
            self.pop_up_window_style = (
                base_pop_up_style.format(bg=self.background_color, text=self.text_color) +
                extra_checkbox_rules.format(text=self.text_color)
            )

        # light mode
        elif theme == "light_mode":
            self.background_color = "#f4f6fc"
            self.border_outline = "#000000"
            self.text_color = "#000000"
            self.danger_button_color = "#faa94a"
            self.normal_button_color = "#99d1c5"
            self.danger_button_style_sheet = f"background-color: {self.danger_button_color}; color: {self.text_color}; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.normal_button_style_sheet = f"background-color: {self.normal_button_color}; color: {self.text_color}; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.selected_tab_color = self.normal_button_color
            self.selected_tab_text_color = self.border_outline
            self.not_selected_tab_color = self.background_color
            self.not_selected_tab_text_color = self.text_color
            self.dark_icon_bkgrnd_color = self.background_color
            self.light_icon_bkgrnd_color = self.background_color
            self.pop_up_window_style = base_pop_up_style.format(bg=self.background_color, text=self.text_color)

        # blue pastel
        elif theme == "blue_pastel":
            self.background_color = "#caedee"
            self.border_outline = "#000000"
            self.text_color = "#000000"
            self.danger_button_color = "#ffdb4f"
            self.normal_button_color = "#81b673"
            self.danger_button_style_sheet = f"background-color: {self.danger_button_color}; color: {self.text_color}; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.normal_button_style_sheet = f"background-color: {self.normal_button_color}; color: {self.text_color}; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.selected_tab_color = self.normal_button_color
            self.selected_tab_text_color = self.border_outline
            self.not_selected_tab_color = self.background_color
            self.not_selected_tab_text_color = self.text_color
            self.dark_icon_bkgrnd_color = self.background_color
            self.light_icon_bkgrnd_color = self.background_color
            self.pop_up_window_style = base_pop_up_style.format(bg=self.background_color, text=self.text_color)

        # brown sepia
        elif theme == "brown_sepia":
            self.background_color = "#f2edd1"
            self.border_outline = "#44312b"
            self.text_color = "#44312b"
            self.danger_button_color = "#44312b"
            self.normal_button_color = "#44312b"
            self.danger_button_style_sheet = f"background-color: {self.danger_button_color}; color: {self.background_color}; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.normal_button_style_sheet = f"background-color: {self.normal_button_color}; color: {self.background_color}; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.selected_tab_color = self.normal_button_color
            self.selected_tab_text_color = self.background_color
            self.not_selected_tab_color = self.background_color
            self.not_selected_tab_text_color = self.text_color
            self.dark_icon_bkgrnd_color = self.background_color
            self.light_icon_bkgrnd_color = self.background_color
            self.pop_up_window_style = base_pop_up_style.format(bg=self.background_color, text=self.text_color)

        # Seth's special mode for the color haters
        #Cadetblue
        elif theme == "cadetblue":
            self.background_color = "cadetblue"
            self.border_outline = "#44312b"
            self.text_color = "#000000"
            self.danger_button_color = "red"
            self.normal_button_color = "blue"
            self.danger_button_style_sheet = f"background-color: {self.danger_button_color}; color: #FFFFFF; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.normal_button_style_sheet = f"background-color: {self.normal_button_color}; color: #FFFFFF; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.selected_tab_color = "blue"
            self.selected_tab_text_color = "white"
            self.not_selected_tab_color = "grey"
            self.not_selected_tab_text_color = "black"
            self.dark_icon_bkgrnd_color = self.background_color
            self.light_icon_bkgrnd_color = self.background_color
            self.pop_up_window_style = base_pop_up_style.format(bg=self.background_color, text=self.text_color)

        # Intense Dark
        elif theme == "intense_dark":
            self.background_color = "black"
            self.border_outline = "white"
            self.text_color = "white"
            self.danger_button_color = "white"
            self.normal_button_color = "white"
            self.danger_button_style_sheet = f"background-color: {self.danger_button_color}; color: {self.background_color}; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.normal_button_style_sheet = f"background-color: {self.normal_button_color}; color: {self.background_color}; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.selected_tab_color = "white"
            self.selected_tab_text_color = "black"
            self.not_selected_tab_color = "black"
            self.not_selected_tab_text_color = "white"
            self.dark_icon_bkgrnd_color = self.text_color
            self.light_icon_bkgrnd_color = self.background_color
            self.pop_up_window_style = (
                base_pop_up_style.format(bg=self.background_color, text=self.text_color) +
                extra_checkbox_rules.format(text=self.text_color)
            )

        # Intense Light
        elif theme == "intense_light":
            self.background_color = "white"
            self.border_outline = "black"
            self.text_color = "black"
            self.danger_button_color = "black"
            self.normal_button_color = "black"
            self.danger_button_style_sheet = f"background-color: {self.danger_button_color}; color: {self.background_color}; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.normal_button_style_sheet = f"background-color: {self.normal_button_color}; color: {self.background_color}; border: 2px solid {self.border_outline}; padding-top: {self.button_padding}px; padding-bottom: {self.button_padding}px; font-size: {self.button_font_size}px;"
            self.selected_tab_color = "black"
            self.selected_tab_text_color = "white"
            self.not_selected_tab_color = "white"
            self.not_selected_tab_text_color = "black"
            self.dark_icon_bkgrnd_color = self.background_color
            self.light_icon_bkgrnd_color = self.background_color
            self.pop_up_window_style = base_pop_up_style.format(bg=self.background_color, text=self.text_color)

        #the first time widgets aren't created yet, so no need to change them
        if not first_time: self.apply_theme_to_widgets()

    def apply_theme_to_widgets(self):

        #get current wideth, and recolor the tabs themselves
        width_px = self.width() // (len(self.selected_vehicles) + 1) - 10
        self.repaintTabs(width_px)

        bg_color, text_color = self.background_color, self.text_color
        for name in self.tab_dict:
            widget = self.tab_dict[name][0]
            #set background color of each tab
            self.set_background(widget, self.background_color)
            #set text color of each console log
            self.set_console_log_colors(self.text_color, self.background_color)
            for button in self.findChildren(QPushButton):
                # Danger buttons
                if (
                    "recall" in button.text().lower()
                    or "emergency" in button.text().lower()
                    or "clear console" in button.text().lower()   # <-- Add this line
                ):
                    button.setStyleSheet(self.danger_button_style_sheet)
                # Normal buttons
                else:
                    button.setStyleSheet(self.normal_button_style_sheet)
            #set text color of the labels
            for label in self.findChildren(QLabel):
                label.setStyleSheet(f"color: {self.text_color};")

            # Update line colors
            for line in self.findChildren(QFrame):
                if line.frameShape() in (QFrame.Shape.HLine, QFrame.Shape.VLine):
                    line.setStyleSheet(f"background-color: {self.text_color};")

            # Find all QLabel widgets whose objectName starts with "icon"
            icon_labels = [label for label in self.findChildren(QLabel) if label.objectName().startswith("icon")]
            for ic_label in icon_labels:
                # Use the original icon pixmap if available
                orig_pixmap = getattr(ic_label, "_original_icon_pixmap", None)
                if orig_pixmap is not None:
                    icon_type = getattr(ic_label, "_icon_type", None)
                    if icon_type == QStyle.StandardPixmap.SP_MessageBoxCritical:
                        icon_bkgrnd = self.light_icon_bkgrnd_color
                    elif icon_type == QStyle.StandardPixmap.SP_DialogApplyButton:
                        icon_bkgrnd = self.light_icon_bkgrnd_color
                    elif icon_type == QStyle.StandardPixmap.SP_TitleBarContextHelpButton:
                        icon_bkgrnd = self.dark_icon_bkgrnd_color
                    else:
                        print("Unknown icon type.")
                        continue
                    new_pixmap = self.paintIconBackground(orig_pixmap, bg_color=icon_bkgrnd)
                    ic_label.setPixmap(new_pixmap)

    def repaint_icon(self, ic_label):
        # Use the original icon pixmap if available
        orig_pixmap = getattr(ic_label, "_original_icon_pixmap", None)
        if orig_pixmap is not None:
            icon_type = getattr(ic_label, "_icon_type", None)
            if icon_type == QStyle.StandardPixmap.SP_MessageBoxCritical:
                icon_bkgrnd = self.light_icon_bkgrnd_color
            elif icon_type == QStyle.StandardPixmap.SP_DialogApplyButton:
                icon_bkgrnd = self.light_icon_bkgrnd_color
            elif icon_type == QStyle.StandardPixmap.SP_TitleBarContextHelpButton:
                icon_bkgrnd = self.dark_icon_bkgrnd_color
            else:
                print("Unknown icon type.")
            new_pixmap = self.paintIconBackground(orig_pixmap, bg_color=icon_bkgrnd)
            ic_label.setPixmap(new_pixmap)
    
    def set_console_log_colors(self, text_color, background_color):
        """
        Sets the background and text color of all console log QLabel widgets.
        """
        for vehicle_number in self.selected_vehicles:
            scroll_area = getattr(self, f"vehicle{vehicle_number}_console_scroll_area", None)
            if scroll_area:
                scroll_area.setStyleSheet(
                    f"border: 2px solid {self.border_outline}; border-radius: 6px; background: {self.background_color};"
                )
                # Find the label inside the scroll area and set its text color
                label = scroll_area.findChild(QLabel, f"Console_messages{vehicle_number}")
                if label:
                    label.setStyleSheet(f"color: {text_color};")

    def handle_console_log(self, msg):
        if msg.vehicle_number == 0:
            for i in self.selected_vehicles:
                self.recieve_console_update(msg.message, i)
        elif msg.vehicle_number in self.selected_vehicles:
            self.recieve_console_update(msg.message, msg.vehicle_number)

    def scroll_console_to_bottom_on_tab(self, index):
        """
        Ensures the console log for a Vehicle tab is always scrolled to the bottom when the tab is selected.

        This function is connected to the QTabWidget's currentChanged signal. When the user switches
        to a Vehicle tab, it finds the corresponding QScrollArea for that Vehicle's console log and scrolls
        it to the bottom, so the latest messages are always visible.

        Parameters:
            index (int): The index of the newly selected tab.
        """
        # Get the name of the tab at the given index
        tab_name = self.tabs.tabText(index)
        # Only act if the tab is a Vehicle tab (e.g., "Vehicle 1", "Vehicle 2", "Vehicle 3")
        if tab_name.startswith("Vehicle"):
            # Extract the Vehicle number from the tab name
            vehicle_number = int(tab_name.split()[-1])
            # Get the scroll area for this Vehicle's console log
            scroll_area = getattr(self, f"vehicle{vehicle_number}_console_scroll_area", None)
            if scroll_area:
                # Process any pending events to ensure the layout is up to date
                QApplication.processEvents()
                # Scroll the vertical scrollbar to the maximum (bottom)
                scroll_area.verticalScrollBar().setValue(scroll_area.verticalScrollBar().maximum())

    def clear_console(self, vehicle_number):

        if vehicle_number == 0: 
            msg = f"Clear Console Called for All Vehicles"
            vehicle_numbers = self.selected_vehicles
            window_title = f"Clear All Consoles?"
            confirm_message = f"Are you sure you want to clear all the consoles? This can't be undone."
        else: 
            msg = f"Clear Console Called for Vehicle{vehicle_number}"
            vehicle_numbers = [vehicle_number]
            window_title = f"Clear Vehicle{vehicle_number} Console?"
            confirm_message = f"Are you sure you want to clear the console for vehicle{vehicle_number}? This can't be undone."

        self.replace_confirm_reject_label(msg)

        # Confirm with the user
        dlg = ConfirmationDialog(
            window_title,
            confirm_message,
            self,
            background_color=self.background_color,
            text_color=self.text_color,
            pop_up_window_style=self.pop_up_window_style
        )

        if dlg.exec(): 
            
            for vehicle in vehicle_numbers:
                try:
                    label = self.findChild(QLabel, f"Console_messages{vehicle}")
                    if label:
                        label.setText("")
                        label.setStyleSheet(f"color: {self.text_color};")
                        # Scroll to the bottom of the scroll area only if user was already at the bottom
                except Exception as e:
                    print(f"Exception in clear_console for Vehicle {vehicle}: {e}")
        
        else: 
            for vehicle in vehicle_numbers:
                self.recieve_console_update("Clear Console Log Command Canceled", vehicle)
            
    def replace_confirm_reject_label(self, confirm_reject_text):
        for label in self.confirm_reject_labels.values():
            label.setText(confirm_reject_text)

    "/*Override the resizeEvent method in the sub class*/"
    def resizeEvent(self, event):
        size = self.size()
        width_px = self.width() // (len(self.selected_vehicles) + 1) - 10
        self.repaintTabs(width_px)
        # Dynamically resize each console scroll area
        for i in self.selected_vehicles:  # Assuming Vehicle 1-3
            scroll_area = getattr(self, f"vehicle{i}_console_scroll_area", None)
            if scroll_area:
                scroll_area.setFixedHeight(int(self.height() * 0.2))
            # Dynamically set width of column0_widget
            column0_widget = getattr(self, f"vehicle{i}_column0_widget", None)
            if column0_widget:
                column0_widget.setMaximumWidth(int(self.width() * 0.16))  # 16% of window width
            column01_widget = getattr(self, f"vehicle{i}_column01_widget", None)
            if column01_widget:
                column01_widget.setMaximumWidth(int(self.width() * 0.16))

        super().resizeEvent(event)

    "/*resize the tabs according to the width of the window*/"
    def repaintTabs(self, width_px):
        # Sets the stylesheet for tab width and appearance.
        self.tabs.setStyleSheet(f"""
        QTabBar::tab {{
            height: 30px;
            width: {width_px - 15}px;
            font-size: 12pt;
            padding: 5px;
            background: {self.not_selected_tab_color};  /* background color of non-selected tab */
            color: {self.not_selected_tab_text_color};           /* font color of non-selected tab */
            border: 2px solid {self.not_selected_tab_text_color}; 
        }}
        QTabBar::tab:selected {{
            background: {self.selected_tab_color};       /* background color of selected tab */
            color: {self.selected_tab_text_color};           /* font color of selected tab */
            border: 2px solid {self.selected_tab_text_color}; 
            font-weight: bold;
        }}
        """)

    def set_background(self, widget, color):
        # Sets the background color of a widget.
        palette = widget.palette()
        palette.setColor(widget.backgroundRole(), QColor(color))
        widget.setAutoFillBackground(True)
        widget.setPalette(palette)
        widget.setStyleSheet(f"background-color: {color};")

    def load_missions_button(self):
        self.replace_confirm_reject_label("Loading the missions...")
        for i in self.selected_vehicles: self.recieve_console_update("Loading the missions...", i)

        def deploy_in_thread(selected_files):
            try:
                origins = []
                spec_paths_dict = {}

                # for file in selected_files:
                for idx, vehicle_number in enumerate(self.selected_vehicles):
                    file = selected_files[idx]

                    spec_paths_list = []

                    # load the yaml file
                    with open(file, 'r') as f:
                        mission_data = yaml.safe_load(f)

                    # get the origin long and lat
                    # For the waypoint YAML, it's under 'origin_lla'
                    origin_lla = mission_data.get('origin_lla')
                    if not origin_lla:
                        err_msg = f"Warning: ⚠️ File {file} missing 'origin_lla' section."
                        self.recieve_console_update(err_msg, vehicle_number)
                        self.replace_confirm_reject_label(err_msg)
                    else:
                        origin_lat = origin_lla.get('latitude')
                        origin_long = origin_lla.get('longitude')

                        if origin_lat is None or origin_long is None:
                            err_msg = f"Warning: ⚠️ File {file} missing latitude or longitude in 'origin_lla'."
                            self.recieve_console_update(err_msg, vehicle_number)
                            self.replace_confirm_reject_label(err_msg)

                        else:
                            # add it to origins list
                            origins.append((origin_lat, origin_long))

                    waypoints = mission_data.get('waypoints', [])
                    # check that each file has waypoints
                    if not waypoints:
                        err_msg = f"Warning: ⚠️ File {file} doesn't have waypoints."
                        self.recieve_console_update(err_msg, vehicle_number)
                        self.replace_confirm_reject_label(err_msg)
                    else:
                        for wp in waypoints:
                            x = wp['position_enu']['x']
                            y = wp['position_enu']['y']
                            spec_paths_list.append((x, y))

                        spec_paths_dict[vehicle_number] = spec_paths_list

                # see if they are all the same
                if origins: 
                    first_origin = origins[0]
                    if not all(origin == first_origin for origin in origins):
                        # raise an exception if it is not the same
                        err_msg = f"Warning: ⚠️ Not all mission files have the same origin (latitude, longitude). Not publishing map viz origin data."
                        for i in self.selected_vehicles: self.recieve_console_update(err_msg, i)
                        self.replace_confirm_reject_label(err_msg)
                    else:
                        # publish origin message
                        self.ros_node.publish_origin((origin_lat, origin_long))

                # publish waypoint paths for each vehicle
                if spec_paths_dict:
                    for num, path_msg in spec_paths_dict.items():
                        self.ros_node.publish_path(path_msg, num)
                else:
                    err_msg = f"Warning: ⚠️ No paths found in files. Not publishing map viz path data."
                    for i in self.selected_vehicles: self.recieve_console_update(err_msg, i)
                    self.replace_confirm_reject_label(err_msg)
                
                deploy.main(self.ros_node, self.selected_vehicles, selected_files)
                self.replace_confirm_reject_label("Loading Mission Command Complete")

            except Exception as e:
                err_msg = f"Mission loading failed: {e}"
                print(err_msg)
                self.replace_confirm_reject_label(err_msg)
                for i in self.selected_vehicles: self.recieve_console_update(err_msg, i)

        dlg = LoadMissionsDialog(parent=self, background_color=self.background_color, text_color=self.text_color, pop_up_window_style=self.pop_up_window_style, selected_vehicles=self.selected_vehicles)
        if dlg.exec():
            start_config = dlg.get_states()
            selected_files = list(start_config['selected_files'].values())
            threading.Thread(target=deploy_in_thread, args=(selected_files,), daemon=True).start()
        else:
            err_msg = "Mission Loading command was cancelled."
            for i in self.selected_vehicles: self.recieve_console_update(err_msg, i)
            self.replace_confirm_reject_label(err_msg)

    def start_missions_button(self):
        # Handler for 'Start Missions' button on the general tab.
        self.replace_confirm_reject_label("Starting all missions...")
        for i in self.selected_vehicles: self.recieve_console_update("Starting the missions...", i)

        def deploy_in_thread(start_config):
            try:
                startup_call.publish_system_control(self.ros_node, self.selected_vehicles, start_config)
                self.replace_confirm_reject_label("Starting Mission Command Complete")

            except Exception as e:
                err_msg = f"Mission starting failed: {e}"
                print(err_msg)
                self.replace_confirm_reject_label(err_msg)
                for i in self.selected_vehicles:
                    self.recieve_console_update(err_msg, i)

        options = ["Start the node", "Record rosbag", "Enter rosbag prefix (string): ", "Arm Thruster", "Start DVL"]
        dlg = StartMissionsDialog(options, parent=self, passed_option_map=self.option_map, background_color=self.background_color, text_color=self.text_color, pop_up_window_style=self.pop_up_window_style)
        if dlg.exec():
            start_config = dlg.get_states()
            threading.Thread(target=deploy_in_thread, args=(start_config,), daemon=True).start()
        else:
            err_msg = "Starting All Missions command was cancelled."
            for i in self.selected_vehicles: self.recieve_console_update(err_msg, i)
            self.replace_confirm_reject_label(err_msg)

    def spec_load_missions_button(self, vehicle_number):
        msg = f"Loading Vehicle{vehicle_number} mission..."
        self.replace_confirm_reject_label(msg)
        self.recieve_console_update(msg, vehicle_number)

        def deploy_in_thread(selected_file):
            try:
                deploy.main(self.ros_node, [vehicle_number], [selected_file])
                self.replace_confirm_reject_label(f"Loading Vehicle{vehicle_number} Mission Command Complete")

            except Exception as e:
                err_msg = f"Mission loading for vehicle{vehicle_number} failed: {e}"
                print(err_msg)
                self.replace_confirm_reject_label(err_msg)
                self.recieve_console_update(err_msg, vehicle_number)

        dlg = LoadMissionsDialog(parent=self, vehicle=vehicle_number, background_color=self.background_color, text_color=self.text_color, pop_up_window_style=self.pop_up_window_style)
        if dlg.exec():
            start_config = dlg.get_states()
            print(f"config files chosen: {start_config}")
            threading.Thread(target=deploy_in_thread, args=(start_config['selected_file'],), daemon=True).start()
        else:
            err_msg = "Mission Loading command was cancelled."
            self.recieve_console_update(err_msg, vehicle_number)
            self.replace_confirm_reject_label(err_msg)

    def spec_start_missions_button(self, vehicle_number):
        # Handler for 'Start Mission' button on a specific Vehicle tab.
        self.replace_confirm_reject_label(f"Starting Vehicle {vehicle_number} mission...")
        self.recieve_console_update(f"Starting Vehicle {vehicle_number} mission...", vehicle_number)

        def deploy_in_thread(start_config):
            try:
                startup_call.publish_system_control(self.ros_node, [vehicle_number], start_config)
                self.replace_confirm_reject_label(f"Starting Mission Vehicle{vehicle_number} Command Complete")

            except Exception as e:
                err_msg = f"Mission starting failed: {e}"
                print(err_msg)
                self.replace_confirm_reject_label(err_msg)
                self.recieve_console_update(err_msg, vehicle_number)

        options = list(self.option_map.keys())
        dlg = StartMissionsDialog(options, parent=self, passed_option_map=self.option_map, vehicle=vehicle_number, background_color=self.background_color, text_color=self.text_color, pop_up_window_style=self.pop_up_window_style)
        if dlg.exec():
            start_config = dlg.get_states()
            threading.Thread(target=deploy_in_thread, args=(start_config,), daemon=True).start()
        else:
            err_msg = f"Starting Vehicle{vehicle_number} Mission command was cancelled."
            self.recieve_console_update(err_msg, vehicle_number)
            self.replace_confirm_reject_label(err_msg)

    def load_waypoint_button(self): 
        msg = f"Loading waypoint planner on general page..."
        self.replace_confirm_reject_label(msg)
        for i in self.selected_vehicles: self.recieve_console_update(msg, i)

        def run_waypoint_planner():
            import tkinter
            from base_station_gui2.temp_waypoint_planner.temp_waypoint_planner import App as WaypointPlannerApp
            root = tkinter.Tk()
            app = WaypointPlannerApp(root)
            root.mainloop()

        # Use multiprocessing to avoid GUI conflicts
        p = multiprocessing.Process(target=run_waypoint_planner)
        p.start()

        # Poll for process completion and update label
        def check_planner_closed():
            if not p.is_alive():
                msg = "Waypoint planner closed successfully"
                self.replace_confirm_reject_label(msg)
                for i in self.selected_vehicles: self.recieve_console_update(msg, i)
            else:
                QTimer.singleShot(500, check_planner_closed)  # check again in 0.5s

        QTimer.singleShot(500, check_planner_closed)

    def copy_bags(self):
        msg = "Starting bag sync for all vehicles..."
        self.replace_confirm_reject_label(msg)
        for i in self.selected_vehicles: 
            self.recieve_console_update(msg, i)
            threading.Thread(target=self.run_sync_bags, args=(i,), daemon=True).start()  

    def spec_copy_bags(self, vehicle_number):
        msg = f"Starting bag sync for Vehicle {vehicle_number}..."
        self.replace_confirm_reject_label(msg)
        self.recieve_console_update(msg, vehicle_number)
        # Run the sync operation in a separate thread to avoid blocking the GUI
        threading.Thread(target=self.run_sync_bags, args=(vehicle_number,), daemon=True).start()  

    def run_calibrate_script(self, vehicle_number):
        threading.Thread(target=self.run_calibrate_script_threaded, args=(vehicle_number,), daemon=True).start()

    def run_calibrate_script_threaded(self, vehicle_number):
        if not vehicle_number: vehicles = self.selected_vehicles
        else: vehicles = [vehicle_number]
        calibrate.main(self.ros_node, vehicles)

    #used by copy bags
    def run_sync_bags(self, vehicle_number):
        try:
            # Path to the sync_bags.sh script
            script_path = os.path.join(
                os.path.expanduser("~"),  # Start from home directory
                "base_station", 
                "mission_control", 
                "sync_bags.sh"
            )

            # Run the script with the vehicle number as argument
            result = subprocess.run(
                [script_path, str(vehicle_number)], 
                capture_output=True, 
                text=True,
                cwd=os.path.dirname(script_path)  # Run from mission_control directory
            )
                            
            if result.returncode == 0:
                success_msg = f"Bag sync completed successfully for Vehicle {vehicle_number}"
                self.replace_confirm_reject_label(success_msg)
                self.recieve_console_update(success_msg, vehicle_number)
                # Also show any output from the script
                if result.stdout:
                    self.recieve_console_update(f"Script output: {result.stdout.strip()}", vehicle_number)
            else:
                error_msg = f"Bag sync failed for Vehicle {vehicle_number}. Exit code: {result.returncode}"
                self.replace_confirm_reject_label(error_msg)
                self.recieve_console_update(error_msg, vehicle_number)
                if result.stderr:
                    self.recieve_console_update(f"Error: {result.stderr.strip()}", vehicle_number)


        except Exception as e:
            error_msg = f"Failed to run bag sync script: {str(e)}"
            self.replace_confirm_reject_label(error_msg)
            self.recieve_console_update(error_msg, vehicle_number)

    def calibrate_fins(self):
        def publish_fins(vehicle_num, fins):
            # Publish to ROS node
            fins_out = [float(f) for f in fins]
            self.ros_node.publish_fins(fins_out, vehicle_num)
        for i in self.selected_vehicles:
            self.recieve_console_update("Loading Fin Calibration Window...", i)
        dlg = CalibrateFinsDialog(
            parent=self,
            background_color=self.background_color,
            text_color=self.text_color,
            pop_up_window_style=self.pop_up_window_style,
            selected_vehicles=self.selected_vehicles,
            passed_ros_node=self.ros_node,
            on_slider_change=publish_fins
        )
        if dlg.exec():
            for i in self.selected_vehicles:
                self.recieve_console_update("Fin Calibration Saved to Params (no logic yet)", i)
        else:
            for i in self.selected_vehicles:
                self.recieve_console_update("Canceling Fin Calibration", i)

    #Connected to the "kill" signal
    def emergency_shutdown_button(self, vehicle_number):
        # Handler for 'Emergency Shutdown' button, with confirmation dialog.
        message = BeaconId.Request()
        message.beacon_id = vehicle_number
        dlg = ConfirmationDialog("Emergency Shutdown?", "Are you sure you want to initiate emergency shutdown?", self, background_color=self.background_color, text_color=self.text_color, pop_up_window_style=self.pop_up_window_style)
        if dlg.exec():
            self.replace_confirm_reject_label("Starting Emergency Shutdown...")
            self.recieve_console_update(f"Starting Emergency Shutdown for Vehicle {vehicle_number}", vehicle_number)
            future = self.ros_node.cli.call_async(message)
            # Add callback to handle response
            future.add_done_callback(partial(self.handle_service_response, action="Emergency Shutdown", vehicle_number=vehicle_number))
            return future
        else:
            self.replace_confirm_reject_label("Canceling Emergency Shutdown command...")
            self.recieve_console_update(f"Canceling Emergency Shutdown for Vehicle {vehicle_number}", vehicle_number)

    #Connected to the "surface" signal
    def emergency_surface_button(self, vehicle_number):
        # Handler for 'Emergency Surface' button, with confirmation dialog.
        message = BeaconId.Request()
        message.beacon_id = vehicle_number
        dlg = ConfirmationDialog("Emergency Surface?", "Are you sure you want to initiate emergency surface?", self, background_color=self.background_color, text_color=self.text_color, pop_up_window_style=self.pop_up_window_style)
        if dlg.exec():
            self.replace_confirm_reject_label("Starting Emergency Surface...")
            self.recieve_console_update(f"Starting Emergency Surface for Vehicle {vehicle_number}", vehicle_number)
            future = self.ros_node.cli2.call_async(message)
            # Add callback to handle response
            future.add_done_callback(partial(self.handle_service_response, action="Emergency Surface", vehicle_number=vehicle_number))
            return future
        else:
            self.replace_confirm_reject_label("Canceling Emergency Surface command...")
            self.recieve_console_update(f"Canceling Emergency Surface for Vehicle {vehicle_number}", vehicle_number)

    #Connected to the "ModemControl" service in base_station_interfaces
    def modem_shut_off_service(self, shutoff:bool, vehicle_id:int):
        #shutoff: True-> shutoff modem, False-> Turn on Modem
        message = ModemControl.Request()
        message.modem_shut_off = shutoff
        message.vehicle_id = vehicle_id
        if shutoff: self.replace_confirm_reject_label("Wifi Connected, Shutting Off Modem")
        else: self.replace_confirm_reject_label("Wifi Disconnected, Turning On Modem")
        
        if shutoff: self.recieve_console_update(f"Wifi Connected, Shutting Off Modem", vehicle_id)
        else: self.recieve_console_update(f"Wifi Disconnected, Turning On Modem", vehicle_id)

        future = self.ros_node.cli3.call_async(message)
        # Add callback to handle response
        future.add_done_callback(partial(self.handle_service_response, action="Modem Shut off Service", vehicle_number=vehicle_id)) #0->for all vehicles
        return future

    #used by various buttons to handle services dynamically
    def handle_service_response(self, future, action, vehicle_number):
        # Handles the result of an asynchronous ROS service call.
        try:
            response = future.result()
            if response.success:
                message = f"{action} Service Initiated Successfully"
                self.replace_confirm_reject_label(message)
                if not vehicle_number:
                    for i in self.selected_vehicles:
                        self.recieve_console_update(message, i)
                elif vehicle_number in self.selected_vehicles:
                    self.recieve_console_update(message, vehicle_number)
            else:
                message = f"{action} Service Initialization Failed"
                self.replace_confirm_reject_label(message)
                if not vehicle_number:
                    for i in self.selected_vehicles:
                        self.recieve_console_update(message, i)
                elif vehicle_number in self.selected_vehicles:
                    self.recieve_console_update(message, vehicle_number)
       
        except Exception as e:
            self.replace_confirm_reject_label(f"{action} service call failed: {e}")
            if vehicle_number in self.selected_vehicles: self.recieve_console_update(f"{action} service call failed: {e}", vehicle_number)
            else: print(f"{action} service call failed: {e}")

    #(No Signal) -> not yet connected to a signal
    def recall_vehicles(self):
        # Handler for 'Recall Vehicles' button on the general tab, with confirmation dialog.
        dlg = ConfirmationDialog("Recall Vehicles?", "Are you sure that you want recall the Vehicles? This will abort all the missions, and cannot be undone.", self, background_color=self.background_color, text_color=self.text_color, pop_up_window_style=self.pop_up_window_style)
        if dlg.exec():
            self.replace_confirm_reject_label("Recalling the Vehicles...")
            for i in self.selected_vehicles: self.recieve_console_update("Recalling the Vehicles...", i)
        else:
            self.replace_confirm_reject_label("Canceling Recall All Vehicles Command...")
            for i in self.selected_vehicles: self.recieve_console_update("Canceling Recall All Vehicles Command...", i)
    
    #(No Signal) -> not yet connected to a signal
    def recall_spec_vehicle(self, vehicle_number):
        # Handler for 'Recall Vehicle' button on a specific Vehicle tab, with confirmation dialog.
        dlg = ConfirmationDialog("Recall Vehicle?", "Are you sure that you want to recall this Vehicle?", self, background_color=self.background_color, text_color=self.text_color, pop_up_window_style=self.pop_up_window_style)
        if dlg.exec():
            self.replace_confirm_reject_label(f"Recalling Vehicle {vehicle_number}...")
            self.recieve_console_update(f"Recalling Vehicle {vehicle_number}...", vehicle_number)
        else:
            self.replace_confirm_reject_label("Canceling Recall Vehicle Command...")
            self.recieve_console_update(f"Canceling Recall Vehicle {vehicle_number} Command...", vehicle_number)

    #template to make a vertical line
    def make_vline(self):
        # Returns a vertical line QFrame for use in layouts.
        Vline = QFrame()
        Vline.setFrameShape(QFrame.Shape.VLine)
        Vline.setFrameShadow(QFrame.Shadow.Sunken)
        Vline.setStyleSheet(f"background-color: {self.text_color};")
        return Vline

    #template to make a horizontal line
    def make_hline(self):
        # Returns a horizontal line QFrame for use in layouts.
        Hline = QFrame()
        Hline.setFrameShape(QFrame.Shape.HLine)
        Hline.setFrameShadow(QFrame.Shadow.Sunken)
        Hline.setStyleSheet(f"background-color: {self.text_color};")
        return Hline

    #used to set all of the widgets on the "general" page tab
    def set_general_page_widgets(self):
        # Sets up the widgets and layouts for the General tab.
        self.general_page_layout = self.tab_dict["General"][1]

        # Create the first column (General Options)
        self.general_page_C0_widget = QWidget()
        self.general_page_C0_layout = QVBoxLayout()
        self.general_page_C0_widget.setLayout(self.general_page_C0_layout)

        # Store widgets and layouts for each Vehicle column
        self.general_page_vehicle_widgets = {}
        self.general_page_vehicle_layouts = {}

        # Add the first column to the layout
        self.general_page_layout.addWidget(self.general_page_C0_widget)

        # For each selected Vehicle, create a column and add to the layout, separated by vertical lines
        for idx, vehicle_number in enumerate(self.selected_vehicles):
            self.general_page_layout.addWidget(self.make_vline())
            widget = QWidget()
            layout = QVBoxLayout()
            widget.setLayout(layout)
            self.general_page_layout.addWidget(widget)
            self.general_page_vehicle_widgets[vehicle_number] = widget
            self.general_page_vehicle_layouts[vehicle_number] = layout

        # Add the buttons to the first column
        self.set_general_page_C0_widgets()

        # Set the widgets for each Vehicle column
        for vehicle_number in self.selected_vehicles:
            self.set_general_page_column_widgets(self.general_page_vehicle_layouts[vehicle_number], vehicle_number)

    #set the widgets of the first column on the general page
    def set_general_page_C0_widgets(self):

        #Create and style the label
        general_label = QLabel("General Options:")
        general_label.setFont(QFont("Arial", 17, QFont.Weight.Bold))
        general_label.setStyleSheet(f"color: {self.text_color};")
        general_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)

        #Load All Missions button
        self.Load_missions_button = QPushButton("Load All Missions")
        self.Load_missions_button.clicked.connect(self.load_missions_button)
        self.Load_missions_button.setStyleSheet(self.normal_button_style_sheet)

        #Start All Missions button
        self.Start_missions_button = QPushButton("Start All Missions")
        self.Start_missions_button.clicked.connect(self.start_missions_button)
        self.Start_missions_button.setStyleSheet(self.normal_button_style_sheet)

        #Plot Waypoints button
        self.plot_waypoints_button = QPushButton("Plot Waypoints")
        self.plot_waypoints_button.clicked.connect(self.load_waypoint_button)
        self.plot_waypoints_button.setStyleSheet(self.normal_button_style_sheet)        
        
        #Copy Bags to Base Station
        self.copy_bags_button = QPushButton("Copy Bags to Base Station")
        self.copy_bags_button.clicked.connect(self.copy_bags)
        self.copy_bags_button.setStyleSheet(self.normal_button_style_sheet)

        #Calibrate All Vehicles 
        self.sync_all_vehicles_button = QPushButton("Calibrate All Vehicles (BUGGY)")
        self.sync_all_vehicles_button.clicked.connect(lambda: self.run_calibrate_script(0))
        self.sync_all_vehicles_button.setStyleSheet(self.normal_button_style_sheet)

        #Recall all the vehicles button
        self.calibrate_fins_button = QPushButton("Calibrate Fins")
        self.calibrate_fins_button.clicked.connect(self.calibrate_fins)
        self.calibrate_fins_button.setStyleSheet(self.normal_button_style_sheet)

        #Recall all the vehicles button
        self.recall_all_vehicles = QPushButton("Recall Vehicles (No Signal)")
        self.recall_all_vehicles.clicked.connect(self.recall_vehicles)
        self.recall_all_vehicles.setStyleSheet(self.danger_button_style_sheet)

        # Add widgets to the layout
        self.general_page_C0_layout.addWidget(general_label, alignment=Qt.AlignmentFlag.AlignTop)
        self.general_page_C0_layout.addSpacing(20)
        self.general_page_C0_layout.addWidget(self.Load_missions_button, alignment=Qt.AlignmentFlag.AlignTop)
        self.general_page_C0_layout.addSpacing(20)
        self.general_page_C0_layout.addWidget(self.Start_missions_button)
        self.general_page_C0_layout.addSpacing(20)
        self.general_page_C0_layout.addWidget(self.plot_waypoints_button)
        self.general_page_C0_layout.addSpacing(20)
        self.general_page_C0_layout.addWidget(self.copy_bags_button)
        self.general_page_C0_layout.addSpacing(20)
        self.general_page_C0_layout.addWidget(self.sync_all_vehicles_button)
        self.general_page_C0_layout.addSpacing(20)
        self.general_page_C0_layout.addWidget(self.calibrate_fins_button)
        self.general_page_C0_layout.addSpacing(20)

        # Add spacer to push the rest of the buttons down
        self.general_page_C0_layout.addWidget(self.recall_all_vehicles)
        self.general_page_C0_layout.addSpacing(20)
        
        # Add remaining buttons (red recall vehicles at the bottom)
        spacer = QSpacerItem(0, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)
        self.general_page_C0_layout.addItem(spacer)
            
    #template to set the rest of widgets on the rest of the columns on the general page
    def set_general_page_column_widgets(self, layout, vehicle_number):
        # Sets up the widgets for each Vehicle column on the General tab.
        # Create and style the header label for each vehicle column
        title_label = QLabel(f"Vehicle {vehicle_number}:")
        title_label.setFont(QFont("Arial", 17, QFont.Weight.Bold))
        title_label.setStyleSheet(f"color: {self.text_color};")
        title_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        layout.addWidget(title_label, alignment=Qt.AlignmentFlag.AlignTop)
        layout.addSpacing(20)

        #section labels for each column
        section_titles = ["Connections", "Sensors", "Emergency Status"]
        for title in section_titles:
            label = QLabel(title)
            label.setFont(QFont("Arial", 15))
            label.setStyleSheet(f"color: {self.text_color};")
            layout.addWidget(label, alignment=Qt.AlignmentFlag.AlignTop)
            layout.addSpacing(20)
            #repeated tab_spacing variable used throughout the file, to keep tabs consistent
            self.tab_spacing = 60

            #The connections section contains the wifi, radio, and modem connections for each Vehicle respectively
            if title == "Connections": 
                wifi_widget = self.create_icon_and_text("Wifi", self.icons_dict[self.feedback_dict["Wifi"][vehicle_number]], self.tab_spacing, vehicle_number, 0)
                layout.addWidget(wifi_widget)
                layout.addSpacing(20)

                radio_widget = self.create_icon_and_text("Radio", self.icons_dict[self.feedback_dict["Radio"][vehicle_number]], self.tab_spacing, vehicle_number, 0)
                layout.addWidget(radio_widget)
                layout.addSpacing(20)

                modem_widget = self.create_icon_and_text("Modem", self.icons_dict[self.feedback_dict["Modem"][vehicle_number]], self.tab_spacing, vehicle_number, 0)
                layout.addWidget(modem_widget)
                layout.addSpacing(40)

            #The connections section contains the DVL, GPS, and IMU sensors connections for each Vehicle respectively
            elif title == "Sensors":
                DVL_sensor_widget = self.create_icon_and_text("DVL", self.icons_dict[self.feedback_dict["DVL"][vehicle_number]], self.tab_spacing, vehicle_number, 0)
                layout.addWidget(DVL_sensor_widget)
                layout.addSpacing(20)

                GPS_sensor_widget = self.create_icon_and_text("GPS", self.icons_dict[self.feedback_dict["GPS"][vehicle_number]], self.tab_spacing, vehicle_number, 0)
                layout.addWidget(GPS_sensor_widget)
                layout.addSpacing(20)
                
                IMU_sensor_widget = self.create_icon_and_text("IMU", self.icons_dict[self.feedback_dict["IMU"][vehicle_number]], self.tab_spacing, vehicle_number, 0)
                layout.addWidget(IMU_sensor_widget)
                layout.addSpacing(40)

            #The status section contains the status message for each Vehicle respectively
            elif title == "Emergency Status":
                status = "No Data Recieved"
                label = QLabel(f"{status}", font=QFont("Arial", 13))
                label.setObjectName(f"Status_messages{vehicle_number}")
                label.setStyleSheet(f"color: {self.text_color};")
                layout.addWidget(label, alignment=Qt.AlignmentFlag.AlignTop)
                layout.addSpacing(40)

        # Add spacer to push content up
        spacer = QSpacerItem(0, 0, QSizePolicy .Policy.Minimum, QSizePolicy.Policy.Expanding)
        layout.addItem(spacer)

    #This is used to set the widgets on the other tabs, namely Vehicle1, Vehicle2, Vehicle3, etc
    def set_specific_vehicle_widgets(self, vehicle_number):
        # Sets up the widgets for a specific Vehicle tab.
        #temporary container for the entire tab
        temp_container = QWidget()
        temp_container.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        temp_layout = QHBoxLayout(temp_container)
        temp_layout.setSpacing(0)
        temp_layout.setContentsMargins(0, 0, 0, 0)

        #col0 contains connections and sensor icons
        temp_layout.addWidget(self.create_specific_vehicle_column0(vehicle_number), alignment=Qt.AlignmentFlag.AlignTop)
        #col1 contains mission and vehicle1 status
        temp_layout.addWidget(self.create_specific_vehicle_column01(vehicle_number), alignment=Qt.AlignmentFlag.AlignTop)

        #vline
        temp_layout.addWidget(self.make_vline())
        #col2 - seconds since last connected and buttons
        temp_layout.addWidget(self.create_vehicle_buttons_column(vehicle_number))
        return temp_container

    #The scrolling log at the bottom of the specific vehicle tabs. 
    def create_specific_vehicle_console_log(self, vehicle_number): 
        # Creates the scrollable console log area for a specific Vehicle tab.
        temp_container = QWidget()
        temp_layout = QVBoxLayout(temp_container)
        setattr(self, f"vehicle{vehicle_number}_console_layout", temp_layout)
        setattr(self, f"vehicle{vehicle_number}_console_widget", temp_container)

        # First text label (bold title)
        title_text = "Console information/message log"
        title_label = QLabel(title_text)
        title_label.setWordWrap(True)
        title_label.setFont(QFont("Arial", 15, QFont.Weight.Bold))
        title_label.setStyleSheet(f"color: {self.text_color};")
        title_label.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        temp_layout.addWidget(title_label)

        # Second text label (wrapped long message)
        message_text = ""

        # Create a QLabel from the message_text for displaying the log
        message_label = QLabel(message_text)
        message_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        message_label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse | Qt.TextInteractionFlag.TextSelectableByKeyboard)
        message_label.setWordWrap(True) # Enable word wrapping for readability
        font = QFont()
        #second font is for emojis, that aren't available in arial
        font.setFamily("Arial, Noto Color Emoji")
        font.setPointSize(13)
        message_label.setFont(font)
        message_label.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop) 
        message_label.setContentsMargins(0, 0, 0, 0)
        message_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        message_label.setObjectName(f"Console_messages{vehicle_number}")

        # Create a QWidget to hold the message label, and a layout for it
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setContentsMargins(0, 0, 0, 0)
        scroll_layout.addWidget(message_label)

        # Create QScrollArea and set scroll content
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setWidget(scroll_content)
        scroll_area.setStyleSheet(
            f"border: 2px solid {self.border_outline}; border-radius: 6px; background: {self.background_color};"
        )
        # Store the scroll area as an attribute for dynamic resizing
        setattr(self, f"vehicle{vehicle_number}_console_scroll_area", scroll_area)

        # Add scroll_area to your layout
        temp_layout.addWidget(scroll_area)

        # Add a spacer to push content up and allow for vertical expansion
        spacer = QSpacerItem(0, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)
        temp_layout.addItem(spacer)

        # Return the container widget holding the scrollable log
        return temp_container


    def paintIconBackground(self, icon_pixmap, bg_color="#28625a", diameter=24):
        """
        Draws a colored circle behind the given icon pixmap.

        Parameters:
            icon_pixmap (QPixmap): The icon to draw.
            bg_color (str): The background color (hex or color name).
            diameter (int): The diameter of the background circle.

        Returns:
            QPixmap: The new pixmap with the background.
        """
        # Create a transparent pixmap
        result = QPixmap(diameter, diameter)
        result.fill(Qt.GlobalColor.transparent)
        painter = QPainter(result)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setBrush(QColor(bg_color))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawEllipse(0, 0, diameter, diameter)
        # Center the icon in the circle
        icon_size = icon_pixmap.size()
        x = (diameter - icon_size.width()) // 2
        y = (diameter - icon_size.height()) // 2
        painter.drawPixmap(x, y, icon_pixmap)
        painter.end()
        return result

    def make_icon_label(self, icon, text, vehicle_number, icon_pg_type): 
        icon_label = QLabel()
        icon_pixmap = self.style().standardIcon(icon).pixmap(16, 16)
        icon_label._original_icon_pixmap = icon_pixmap  # Store original
        icon_label._icon_type = icon # Store the icon type (e.g., QStyle.StandardPixmap.SP_MessageBoxCritical)
        
        if icon == QStyle.StandardPixmap.SP_MessageBoxCritical:
            icon_bkgrnd = self.light_icon_bkgrnd_color
        elif icon == QStyle.StandardPixmap.SP_DialogApplyButton:
            icon_bkgrnd = self.light_icon_bkgrnd_color
        elif icon == QStyle.StandardPixmap.SP_TitleBarContextHelpButton:
            icon_bkgrnd = self.dark_icon_bkgrnd_color
        else:
            print("Unknown icon type.")
            return
        
        bg_pixmap = self.paintIconBackground(icon_pixmap, bg_color=icon_bkgrnd)
        icon_label.setPixmap(bg_pixmap)
        icon_label.setObjectName(f"icon_{text}{vehicle_number}{icon_pg_type}")
        # print(f"icon created with name: icon_{text}{vehicle_number}{icon_pg_type}")

        icon_label.setContentsMargins(0, 0, 0, 0)
        icon_label.setFixedSize(24, 24)
        return icon_label


    #used to create an icon next to text in a pre-determined fashion
    def create_icon_and_text(self, text, icon=None, temp_tab_spacing=None, vehicle_number=None, icon_pg_type=None):
        """
        Creates a QWidget containing an icon (optional) and a text label, arranged horizontally.

        Parameters:
            text (str): The text to display next to the icon.
            icon (QStyle.StandardPixmap, optional): The standard Qt icon to display. If None, no icon is shown.
            temp_tab_spacing (int, optional): Left margin for the layout, used for tab alignment.
            vehicle_number(int, optional): used to name the icon labels
            icon_pg_type(int (0,1), optional):  used to name the icon labels. 0-> general 1->specific

        Returns:
            QWidget: A container widget with the icon and text label.
        """
        # Create a container widget and a horizontal layout for icon and text
        temp_container = QWidget()
        temp_layout = QHBoxLayout(temp_container)

        # If a tab spacing value is provided, set the left margin accordingly
        if temp_tab_spacing: 
            temp_layout.setContentsMargins(temp_tab_spacing, 0, 0, 0)
        temp_layout.setSpacing(20)  # Space between icon and text

        # If an icon is provided, create a QLabel for it and add to the layout
        if icon:
            icon_label = self.make_icon_label(icon, text, vehicle_number, icon_pg_type)
            temp_layout.addWidget(icon_label, alignment=Qt.AlignmentFlag.AlignVCenter)

        # Create the text label and add to the layout
        text_label = QLabel(text)
        text_label.setFont(QFont("Arial", 13))
        text_label.setContentsMargins(0, 0, 0, 0)
        text_label.setStyleSheet(f"color: {self.text_color};")
        temp_layout.addWidget(text_label, alignment=Qt.AlignmentFlag.AlignVCenter)

        # Return the container widget with icon and text
        return temp_container

    #used to create the title labels throughout the window
    def create_title_label(self, text):
        # Create and style the label
        temp_label = QLabel(text)
        temp_label.setFont(QFont("Arial", 15, QFont.Weight.Bold))
        temp_label.setStyleSheet(f"color: {self.text_color};")
        temp_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        return temp_label

    #used to create the buttons column on the specific vehicle pages
    def create_vehicle_buttons_column(self, vehicle_number):
        """
        Creates the button column for a specific Vehicle tab, including mission control and emergency buttons,
        as well as labels for seconds since last connection.

        Parameters:
            vehicle_number (int): The vehicle number for which to create the button column.

        Returns:
            QWidget: The vertical container widget holding all buttons and labels for the Vehicle.
        """
        # Create temporary containers and layouts for organizing buttons and labels
        temp_sub_container1 = QWidget()
        temp_layout1 = QVBoxLayout(temp_sub_container1)
        temp_layout1.setAlignment(Qt.AlignmentFlag.AlignTop)

        #temp container for the second column of buttons
        temp_sub_container2 = QWidget()
        temp_layout2 = QVBoxLayout(temp_sub_container2)
        temp_layout2.setAlignment(Qt.AlignmentFlag.AlignTop)

        #temp container for the button columns put together horizontally
        temp_container = QWidget()
        temp_layout = QHBoxLayout(temp_container)

        #temp container for the entire column, the buttons and the last connected labels
        temp_V_container = QWidget()
        temp_V_layout = QVBoxLayout(temp_V_container)
        setattr(self, f"vehicle{vehicle_number}_buttons_column_widget", temp_V_container)
        setattr(self, f"vehicle{vehicle_number}_buttons_column_layout", temp_V_layout)

        #load mission (normal button)
        self.create_vehicle_button(vehicle_number, "load_mission", "Load Mission", lambda: self.spec_load_missions_button(vehicle_number))
        #start mission (normal button)
        self.create_vehicle_button(vehicle_number, "start_mission", "Start Mission", lambda: self.spec_start_missions_button(vehicle_number))
        #start mission (normal button)
        self.create_vehicle_button(vehicle_number, "copy_bag", "Copy Bag to Base Station", lambda: self.spec_copy_bags(vehicle_number))
        #sync vehicle (normal button)
        self.create_vehicle_button(vehicle_number, "sync", "Calibrate Vehicle (BUGGY)", lambda: self.run_calibrate_script(vehicle_number))


        #emergency surface (danger button)
        self.create_vehicle_button(vehicle_number, "emergency_surface", "Emergency Surface", lambda: self.emergency_surface_button(vehicle_number), danger=True)
        #abort mission (danger button)
        self.create_vehicle_button(vehicle_number, "recall", f"Recall Vehicle (No Signal)", lambda: self.recall_spec_vehicle(vehicle_number), danger=True)
        #emergency shutdown (danger button)
        self.create_vehicle_button(vehicle_number, "emergency_shutdown", "Emergency Shutdown", lambda: self.emergency_shutdown_button(vehicle_number), danger=True)
        #clear console (danger button)
        self.create_vehicle_button(vehicle_number, "clear_console", "Clear Console", lambda: self.clear_console(vehicle_number), danger=True)

        temp_spacing = 20
        # Add buttons to the first and second sub-columns with spacing
        temp_layout1.addWidget(getattr(self, f"load_mission_vehicle{vehicle_number}_button"))
        temp_layout1.addSpacing(temp_spacing)
        temp_layout1.addWidget(getattr(self, f"start_mission_vehicle{vehicle_number}_button"))
        temp_layout1.addSpacing(temp_spacing)
        temp_layout1.addWidget(getattr(self, f"copy_bag_vehicle{vehicle_number}_button"))
        temp_layout1.addSpacing(temp_spacing)
        temp_layout1.addWidget(getattr(self, f"sync_vehicle{vehicle_number}_button"))
        temp_layout1.addSpacing(temp_spacing)
        temp_layout2.addWidget(getattr(self, f"emergency_surface_vehicle{vehicle_number}_button"))
        temp_layout2.addSpacing(temp_spacing)
        temp_layout2.addWidget(getattr(self, f"recall_vehicle{vehicle_number}_button"))
        temp_layout2.addSpacing(temp_spacing)
        temp_layout2.addWidget(getattr(self, f"emergency_shutdown_vehicle{vehicle_number}_button"))
        temp_layout2.addSpacing(temp_spacing)
        temp_layout2.addWidget(getattr(self, f"clear_console_vehicle{vehicle_number}_button"))

        # Add the two button columns to the main horizontal layout
        temp_layout.addWidget(temp_sub_container1)
        temp_layout.addWidget(temp_sub_container2)

        # Add a title label and connection time labels to the vertical layout
        temp_V_layout.addWidget(self.create_title_label("Seconds since last connected"))
        self.insert_label(temp_V_layout, "Radio: xxx", vehicle_number, 1)
        self.insert_label(temp_V_layout, "Accoustics: xxx", vehicle_number, 0)
        temp_V_layout.addWidget(self.make_hline())
        temp_V_layout.addWidget(temp_container)
        
        # Return the vertical container holding all buttons and labels
        return temp_V_container

    def insert_label(self, temp_layout, text, vehicle_number, conn_type):
        """
        Inserts a QLabel into the given layout for displaying the seconds since last connection
        for either radio or modem, and stores it as an attribute for later access.

        Parameters:
            temp_layout (QLayout): The layout to add the label to.
            text (str): The text to display in the label.
            vehicle_number (int): The vehicle number (Vehicle) this label is for.
            conn_type (int): 1 for radio, 0 for modem (used to determine label name).
        """
        text_label = QLabel(text)
        # Set the object name based on connection type for easy lookup later
        if conn_type:
            name = f"vehicle{vehicle_number}_radio_seconds_widget"
        else:
            name = f"vehicle{vehicle_number}_modem_seconds_widget"
        setattr(self, name, text_label)
        text_label.setObjectName(name)
        text_label.setFont(QFont("Arial", 13))
        text_label.setContentsMargins(0, 0, 0, 0)
        text_label.setStyleSheet(f"color: {self.text_color};")
        # Add the label to the layout, vertically centered
        temp_layout.addWidget(text_label, alignment=Qt.AlignmentFlag.AlignVCenter)

    def create_seconds_label(self, conn_type, seconds):
        """
        Creates a QLabel displaying the seconds since last connection for either radio or modem.

        Parameters:
            conn_type (int): 1 for radio, 0 for modem.
            seconds (int): The number of seconds since last connection.

        Returns:
            QLabel: The label displaying the connection time.
        """
        # Set the label text based on connection type
        if conn_type:
            text = f"Radio: {seconds}"
        else:
            text = f"Accoustics: {seconds}"
        text_label = QLabel(text)
        text_label.setFont(QFont("Arial", 13))
        text_label.setContentsMargins(0, 0, 0, 0)
        text_label.setStyleSheet(f"color: {self.text_color};")
        return text_label

    #Dynamically creates a QPushButton with the given properties and stores it as an attribute.
    def create_vehicle_button(self, vehicle_number, name, text, callback, danger=False):
        """
        Dynamically creates a QPushButton with the given properties and stores it as an attribute.

        Parameters:
            vehicle_number (int): Which Vehicle this button is for.
            name (str): Short functional name for the button (e.g., "start_mission", "disarm_thruster").
            text (str): Text to display on the button.
            callback (function): Function to call when the button is clicked.
            danger (bool): used to change between color buttons, dangerous and normal
        """
        button = QPushButton(text)
        button.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        button.clicked.connect(callback)
        button.setStyleSheet(self.danger_button_style_sheet) if danger else button.setStyleSheet(self.normal_button_style_sheet)
        attr_name = f"{name}_vehicle{vehicle_number}_button"
        setattr(self, attr_name, button)

    def create_specific_vehicle_column0(self, vehicle_number):
        """
        Creates the first column for a specific Vehicle tab, displaying connection and sensor status icons.

        Parameters:
            vehicle_number (int): The vehicle number for which to create the column.

        Returns:
            QWidget: The container widget holding all connection and sensor status icons for the Vehicle.
        """
        # Create a vertical layout for the column and store it as an attribute
        temp_layout = QVBoxLayout()
        setattr(self, f"vehicle{vehicle_number}_column0_layout", temp_layout)
        temp_layout.setContentsMargins(0, 0, 0, 0)
        temp_layout.setSpacing(0) 

        # Create the container widget for this column and store it as an attribute
        temp_container = QWidget()
        setattr(self, f"vehicle{vehicle_number}_column0_widget", temp_container)
        container_layout = QVBoxLayout(temp_container)
        container_layout.addLayout(temp_layout)

        # Add the Vehicle title label at the top
        temp_layout.addWidget(self.create_title_label(f"Vehicle {vehicle_number}"), alignment=Qt.AlignmentFlag.AlignTop)
        
        # Section: Connections
        temp_label = QLabel("Connections")
        temp_label.setFont(QFont("Arial", 15, QFont.Weight.Bold))
        temp_label.setStyleSheet(f"color: {self.text_color};")
        temp_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        temp_layout.addSpacing(20)
        temp_layout.addWidget(temp_label)

        # Add connection status icons (Wifi, Radio, Modem)
        wifi_widget = self.create_icon_and_text("Wifi", self.icons_dict[self.feedback_dict["Wifi"][vehicle_number]], 0, vehicle_number, 1)
        temp_layout.addWidget(wifi_widget)

        radio_widget = self.create_icon_and_text("Radio", self.icons_dict[self.feedback_dict["Radio"][vehicle_number]], 0, vehicle_number, 1)
        temp_layout.addWidget(radio_widget)

        modem_widget = self.create_icon_and_text("Modem", self.icons_dict[self.feedback_dict["Modem"][vehicle_number]], 0, vehicle_number, 1)
        temp_layout.addWidget(modem_widget)
        temp_layout.addSpacing(20)

        # Section: Sensors
        temp_label = QLabel("Sensors")
        temp_label.setFont(QFont("Arial", 15, QFont.Weight.Bold))
        temp_label.setStyleSheet(f"color: {self.text_color};")

        temp_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        temp_layout.addSpacing(20)
        temp_layout.addWidget(temp_label)

        # Add sensor status icons (DVL, GPS, IMU)
        DVL_sensor_widget = self.create_icon_and_text("DVL", self.icons_dict[self.feedback_dict["DVL"][vehicle_number]], 0, vehicle_number, 1)
        temp_layout.addWidget(DVL_sensor_widget)

        GPS_sensor_widget = self.create_icon_and_text("GPS", self.icons_dict[self.feedback_dict["GPS"][vehicle_number]], 0, vehicle_number, 1)
        temp_layout.addWidget(GPS_sensor_widget)
        
        IMU_sensor_widget = self.create_icon_and_text("IMU", self.icons_dict[self.feedback_dict["IMU"][vehicle_number]], 0, vehicle_number, 1)
        temp_layout.addWidget(IMU_sensor_widget)

        # Return the container widget holding all status icons
        return temp_container

    #create the second sub-column in the first column of the specific vehiclear pages (starts with "Nodes")
    def create_specific_vehicle_column01(self, vehicle_number):
        """     
        Creates the second sub-column in the first column of the specific Vehicle pages.
        This column displays the mission section and the status widgets for the given Vehicle.

        Parameters:
            vehicle_number (int): The vehicle number for which to create the column.

        Returns:
            QWidget: The container widget holding the mission and status widgets for the Vehicle.
        """
        # Create a vertical layout for the column and set margins and spacing
        temp_layout = QVBoxLayout()
        temp_layout.setContentsMargins(0, 0, 0, 0)
        # temp_layout.setSpacing(0) 

        # Create the container widget for this column and store it as an attribute
        temp_container = QWidget()
        # Optionally set a maximum width for the container
        # temp_container.setMaximumWidth(220)
        setattr(self, f"vehicle{vehicle_number}_column01_layout", temp_layout)
        setattr(self, f"vehicle{vehicle_number}_column01_widget", temp_container)
        container_layout = QVBoxLayout(temp_container)
        container_layout.addLayout(temp_layout)

        # Add an (optional) title label at the top
        temp_layout.addWidget(self.create_title_label(f""), alignment=Qt.AlignmentFlag.AlignTop)

        # Status widgets section
        temp_layout.addSpacing(10)
        temp_layout.addWidget(self.create_title_label(f"Status"), alignment=Qt.AlignmentFlag.AlignTop)
        temp_layout.addSpacing(10)
        temp_layout.addWidget(self.create_normal_label("x (m): x", f"XPos{vehicle_number}"), alignment=Qt.AlignmentFlag.AlignVCenter)
        temp_layout.addSpacing(10)
        temp_layout.addWidget(self.create_normal_label("y (m): y", f"YPos{vehicle_number}"), alignment=Qt.AlignmentFlag.AlignVCenter)
        temp_layout.addSpacing(10)
        temp_layout.addWidget(self.create_normal_label("Depth (m): d", f"Depth{vehicle_number}"), alignment=Qt.AlignmentFlag.AlignVCenter)
        temp_layout.addSpacing(10)
        temp_layout.addWidget(self.create_normal_label("Heading (deg): h", f"Heading{vehicle_number}"), alignment=Qt.AlignmentFlag.AlignVCenter)
        temp_layout.addSpacing(10)
        temp_layout.addWidget(self.create_normal_label("Current Waypoint: w", f"Waypoint{vehicle_number}"), alignment=Qt.AlignmentFlag.AlignVCenter)
        temp_layout.addSpacing(10)
        temp_layout.addWidget(self.create_normal_label("DVL Velocity <br>(m/s): v", f"DVL_vel{vehicle_number}"), alignment=Qt.AlignmentFlag.AlignVCenter)
        temp_layout.addSpacing(10)
        temp_layout.addWidget(self.create_normal_label("Angular Velocity <br>(rad/s): a", f"Angular_vel{vehicle_number}"), alignment=Qt.AlignmentFlag.AlignVCenter)
        temp_layout.addSpacing(10)
        temp_layout.addWidget(self.create_normal_label("Battery (V): b", f"Battery{vehicle_number}"), alignment=Qt.AlignmentFlag.AlignVCenter)
        temp_layout.addSpacing(10)
        temp_layout.addWidget(self.create_normal_label("Pressure (Pa):<br>p", f"Pressure{vehicle_number}"), alignment=Qt.AlignmentFlag.AlignVCenter)

        # Return the container widget holding the mission and status widgets
        return temp_container

    def create_normal_label(self, text, name): 
        """
        Creates a QLabel with the given text and object name, sets its font and margins, 
        and stores it as an attribute for later access.

        Parameters:
            text (str): The text to display in the label.
            name (str): The object name and attribute name for the label.

        Returns:
            QLabel: The created label.
        """
        text_label = QLabel(text)
        text_label.setTextFormat(Qt.TextFormat.RichText)
        setattr(self, name, text_label)
        text_label.setObjectName(name) 
        text_label.setFont(QFont("Arial", 13))
        text_label.setStyleSheet(f"color: {self.text_color};")
        text_label.setContentsMargins(0, 0, 0, 0)
        text_label.setWordWrap(True)  # Allow text to wrap if it's long
        text_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        return text_label

    def recieve_safety_status_message(self, vehicle_number, safety_message):
        # #header
        # std_msgs/Header header
        # #0 = good, 1 = not publishing
        # std_msgs/Int8 depth_status
        # #imu status
        # std_msgs/Bool imu_published
        # #0=good, bit 1 = publishing?, bit 2 = good enough gps fix?
        # std_msgs/Int8 gps_status
        # #0 = good, 1= not publishing
        # std_msgs/Int8 modem_status
        # #0=good, bit 1=publishing?, bit 2= standard dev below threshold
        # std_msgs/Int8 dvl_status
        # #0= ok, 1 = surfacing, 2=surfaced/disarmed
        # std_msgs/Int8 emergency_status
        # #what node is sending this, 1= vehicle, 0= base station
        #or, 1= wifi, 0= modem
        # std_msgs/Int8 sender_id
        self.safety_status_signal.emit(vehicle_number, safety_message)
    
    def _update_safety_status_information(self, vehicle_number, safety_message):
        #NOTE: wifi is updated by pinging directly. (ping_vehicles_via_wifi)

        #logic is opposite, switch 0 and 1
        if safety_message.gps_status.data: gps_data = 0
        else: gps_data = 1
        self.feedback_dict["GPS"][vehicle_number] = gps_data

        if safety_message.dvl_status.data: dvl_data = 0
        else: dvl_data = 1
        self.feedback_dict["DVL"][vehicle_number] = dvl_data 
        
        if safety_message.imu_published.data: imu_data = 1
        else: imu_data = 0
        self.feedback_dict["IMU"][vehicle_number] = imu_data
        
        #replace general page widgets
        self.replace_general_page_icon_widget(vehicle_number, "GPS")
        self.replace_general_page_icon_widget(vehicle_number, "DVL")
        # self.replace_general_page_icon_widget(vehicle_number, "Wifi")
        self.replace_general_page_icon_widget(vehicle_number, "IMU")

        #replace specific page widgets
        self.replace_specific_icon_widget(vehicle_number, "GPS")
        self.replace_specific_icon_widget(vehicle_number, "DVL")
        # self.replace_specific_icon_widget(vehicle_number, "Wifi")
        self.replace_specific_icon_widget(vehicle_number, "IMU")

        #replace emergency status label
        if self.feedback_dict["Status_messages"][vehicle_number] != safety_message.emergency_status.data:
            self.feedback_dict["Status_messages"][vehicle_number] = safety_message.emergency_status.data
            layout = self.general_page_vehicle_layouts.get(vehicle_number)
            widget = self.general_page_vehicle_widgets.get(vehicle_number)
            new_status_label = self.get_status_label(vehicle_number, self.feedback_dict["Status_messages"][vehicle_number])
            existing_label = widget.findChild(QLabel, f"Status_messages{vehicle_number}")
            if existing_label: existing_label.setText(new_status_label.text())

    def recieve_smoothed_output_message(self, vehicle_number, msg):
        self.smoothed_ouput_signal.emit(vehicle_number, msg)

    def _update_gui_smoothed_output(self, vehicle_number, msg):
        position = msg.pose.pose.position
        x = position.x
        y = position.y
        #won't use z, will use depth data instead
        
        # Quaternion from Odometry message
        q = (
            msg.pose.pose.orientation.w,  # transforms3d expects (w, x, y, z)
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z
        )

        # Convert to roll, pitch, yaw in radians (defaults to 'sxyz' convention)
        _, _, yaw = quat2euler(q)

        # heading
        heading_deg = math.degrees(yaw) % 360

        l_vel = msg.twist.twist.linear
        a_vel = msg.twist.twist.angular

        total_linear_vel = math.sqrt(l_vel.x**2 + l_vel.y**2 + l_vel.z**2)
        total_angular_vel = math.sqrt(a_vel.x**2 + a_vel.y**2 + a_vel.z**2)

        #update feedback dict 
        self.feedback_dict["XPos"][vehicle_number] = round(x, 2)
        self.feedback_dict["YPos"][vehicle_number] = round(y, 2)
        self.feedback_dict["DVL_vel"][vehicle_number] = round(total_linear_vel, 2)
        self.feedback_dict["Angular_vel"][vehicle_number] = round(total_angular_vel, 2)
        self.feedback_dict["Heading"][vehicle_number] = round(heading_deg, 2)

        #replace specific page status widget
        self.replace_specific_status_widget(vehicle_number, "XPos")
        self.replace_specific_status_widget(vehicle_number, "YPos")
        self.replace_specific_status_widget(vehicle_number, "DVL_vel")
        self.replace_specific_status_widget(vehicle_number, "Angular_vel")
        self.replace_specific_status_widget(vehicle_number, "Heading")

    def recieve_depth_data_message(self, vehicle_number, msg):
        self.depth_data_signal.emit(vehicle_number, msg)

    def update_depth_data(self, vehicle_number, msg):
        #update feedback dict 
        self.feedback_dict["Depth"][vehicle_number] = round(msg.pose.pose.position.z, 2)
        #replace specific page status widget
        self.replace_specific_status_widget(vehicle_number, "Depth")

    def recieve_pressure_data_message(self, vehicle_number, msg):
        self.pressure_data_signal.emit(vehicle_number, msg)

    def update_pressure_data(self, vehicle_number, msg):
        #update feedback dict 
        self.feedback_dict["Pressure"][vehicle_number] = round(msg.fluid_pressure, 2)
        #replace specific page status widget
        self.replace_specific_status_widget(vehicle_number, "Pressure")

    def recieve_battery_data_message(self, vehicle_number, msg):
        self.battery_data_signal.emit(vehicle_number, msg)
        
    def update_battery_data(self, vehicle_number, msg):
        #update feedback dict 
        self.feedback_dict["Battery"][vehicle_number] = round(msg.voltage, 1)
        #replace specific page status widget
        self.replace_specific_status_widget(vehicle_number, "Battery")

    def recieve_kill_confirmation_message(self, kill_message): 
        """
        Slot to receive a kill confirmation message from the ROS topic.
        Emits a signal to update the GUI with the received message.
        
        Parameters:
            kill_message: The message object received from the 'confirm_e_kill' topic (std_msgs/Bool).
        """
        self.kill_confirm_signal.emit(kill_message)

    def _update_kill_confirmation_gui(self, kill_message): 
        """
        Slot connected to kill_confirm_signal.
        Updates the GUI console with a confirmation or failure message for all Vehicles,
        depending on the value of the kill_message.
        
        Parameters:
            kill_message: The message object received from the 'confirm_e_kill' topic (std_msgs/Bool).
        """
        value = kill_message.data if hasattr(kill_message, 'data') else kill_message
        if value: 
            for i in self.selected_vehicles:
                self.recieve_console_update("Kill Command Confirmed", i)
        else: 
            for i in self.selected_vehicles:
                self.recieve_console_update("Kill Command Failed", i)

    def recieve_surface_confirmation_message(self, surf_message): 
        """
        Slot to receive a surface confirmation message from the ROS topic.
        Emits a signal to update the GUI with the received message.
        
        Parameters:
            surf_message: The message object received from the 'confirm_e_surface' topic (std_msgs/Bool).
        """
        self.surface_confirm_signal.emit(surf_message)

    def _update_surf_confirmation_gui(self, surf_message):
        """
        Slot connected to surface_confirm_signal.
        Updates the GUI console with a confirmation or failure message for all Vehicles,
        depending on the value of the surf_message.
        
        Parameters:
            surf_message: The message object received from the 'confirm_e_surface' topic (std_msgs/Bool).
        """
        value = surf_message.data if hasattr(surf_message, 'data') else surf_message
        if value: 
            for i in self.selected_vehicles:
                self.recieve_console_update("Surface Command Confirmed", i)
        else: 
            for i in self.selected_vehicles:
                self.recieve_console_update("Surface Command Failed", i)

    def recieve_connections(self, conn_message):
        """
        Slot to receive a Connections message and emit a signal to update the GUI.

        Parameters:
            conn_message: The Connections message object.
        """
        # message: Connections
        # std_msgs/Header header
        # 0 for acoustic modem, 1 for radio
        # uint8 connection_type
        # connection status, list of bool, representing connections of Vehicle1, Vehicle2, etc
        # bool[] connections
        # time since last ping response, representing last responses in seconds of Vehicle1, Vehicle2, etc
        # uint8[] last_ping
        self.update_connections_signal.emit(conn_message)

    def _update_connections_gui(self, conn_message):
        """
        Updates the GUI to reflect the latest connection status and ping times for each Vehicle.

        Parameters:
            conn_message: The Connections message object containing connection_type, connections, and last_ping.
        """
        print(f"connection_type: {conn_message.connection_type}, connections: {conn_message.connections}, last_ping: {conn_message.last_ping}")
        try:
            if conn_message.connection_type:
                feedback_key = "Radio"
                feedback_key_seconds = "Radio_seconds"
                conn_type = 1
            else:
                feedback_key = "Modem"
                feedback_key_seconds = "Modem_seconds"
                conn_type = 0

            # Update connection status icons for each Vehicle
            for i, vehicle_number in enumerate(conn_message.vehicle_ids):
                try:
                    if vehicle_number not in self.feedback_dict[feedback_key]:
                        continue
                    
                    # Use the index i instead of vehicle_number-1
                    status = 1 if conn_message.connections[i] else 0
                    self.feedback_dict[feedback_key][vehicle_number] = status
                    prefix = feedback_key.split("_")[0]
                    
                    # Update general page
                    layout = self.general_page_vehicle_layouts.get(vehicle_number)
                    widget = self.general_page_vehicle_widgets.get(vehicle_number)
                    if layout and widget:
                        self.replace_general_page_icon_widget(vehicle_number, feedback_key)

                    # Update specific page
                    layout = getattr(self, f"vehicle{vehicle_number}_column0_layout", None)
                    widget = getattr(self, f"vehicle{vehicle_number}_column0_widget", None)
                    if layout and widget:
                        self.replace_specific_icon_widget(vehicle_number, feedback_key)
                        
                except Exception as e:
                    print(f"Exception updating connection status for vehicle {vehicle_number}: {e}")

            # Update seconds since last ping for each Vehicle
            for i, vehicle_number in enumerate(conn_message.vehicle_ids):
                try:
                    if vehicle_number not in self.selected_vehicles:
                        continue
                    
                    # Use the index i instead of count
                    ping = conn_message.last_ping[i]
                    self.feedback_dict[feedback_key_seconds][vehicle_number] = ping
                    
                    layout = getattr(self, f"vehicle{vehicle_number}_buttons_column_layout", None)
                    widget = getattr(self, f"vehicle{vehicle_number}_buttons_column_widget", None)
                    
                    if layout and widget:
                        if conn_type:
                            old_label = f"vehicle{vehicle_number}_radio_seconds_widget"
                            existing_label = widget.findChild(QLabel, old_label)
                            new_text = f"Radio: {ping}"
                            if existing_label:
                                existing_label.setText(new_text)
                        else:
                            old_label = f"vehicle{vehicle_number}_modem_seconds_widget"
                            existing_label = widget.findChild(QLabel, old_label)
                            new_text = f"Accoustics: {ping}"
                            if existing_label:
                                existing_label.setText(new_text)
                                
                except Exception as e:
                    print(f"Exception updating ping time for vehicle {vehicle_number}: {e}")

        except Exception as e:
            print("Exception in update_connections_gui:", e)
            for i in self.selected_vehicles:
                self.recieve_console_update(f"Exception in update_connections_gui: {e}", i)
            
    def get_status_label(self, vehicle_number, status_message):
        if not status_message: message_text = "Good"
        elif status_message == 1: message_text = "EMERGENCY: <br>Recall Vehicle"
        elif status_message == 2: message_text = "Surfaced/Disarmed"
        else: message_text = "No Data Received"

        temp_label = QLabel(f"{message_text}", font=QFont("Arial", 13), alignment=Qt.AlignmentFlag.AlignTop)
        temp_label.setTextFormat(Qt.TextFormat.RichText)
        return temp_label

    def recieve_console_update(self, console_message, vehicle_number):
        """
        Slot to receive a console message and emit a signal to update the GUI.

        Parameters:
            console_message: The console message object.
        """
        self.update_console_signal.emit(console_message, vehicle_number)
    
    def _update_console_gui(self, console_message, vehicle_number):
        """
        Appends a new console message to the specific Vehicle's console log label.
        If vehicle_number is 0, send to all selected Vehicles.
        """
        # Determine which Vehicles to update
        if vehicle_number == 0:
            vehicle_numbers = self.selected_vehicles
        else:
            vehicle_numbers = [vehicle_number]

        for vehicle in vehicle_numbers:
            try:
                label = self.findChild(QLabel, f"Console_messages{vehicle}")
                if label:
                    current_text = label.text()
                    updated_text = f"{current_text}\n{console_message}" if current_text else console_message
                    label.setText(updated_text)
                    label.setStyleSheet(f"color: {self.text_color};")
                    # Scroll to the bottom of the scroll area only if user was already at the bottom
                    scroll_area = getattr(self, f"vehicle{vehicle}_console_scroll_area", None)
                    if scroll_area:
                        vbar = scroll_area.verticalScrollBar()
                        at_bottom = vbar.value() >= vbar.maximum() - 2  # Allow for rounding
                        def maybe_scroll():
                            if at_bottom:
                                vbar.setValue(vbar.maximum())
                        QTimer.singleShot(50, maybe_scroll)
                else:
                    print(f"Console log label not found for Vehicle {vehicle}")
            except Exception as e:
                print(f"Exception in _update_console_gui for Vehicle {vehicle}: {e}")

    def replace_general_page_icon_widget(self, vehicle_number, prefix):
        layout = self.general_page_vehicle_layouts.get(vehicle_number)
        widget = self.general_page_vehicle_widgets.get(vehicle_number)
        status = self.feedback_dict[prefix][vehicle_number]
        icon_type = self.icons_dict[status]
        existing_label = widget.findChild(QLabel, f"icon_{prefix}{vehicle_number}0")
        if existing_label: self.replace_icon_widget(existing_label, icon_type)
        else: print(f"icon_{prefix}{vehicle_number}0 label does not exist")
    
    def replace_specific_icon_widget(self, vehicle_number, prefix):
        layout = getattr(self, f"vehicle{vehicle_number}_column0_layout")
        widget = getattr(self, f"vehicle{vehicle_number}_column0_widget")
        status = self.feedback_dict[prefix][vehicle_number]
        icon_type = self.icons_dict[status]
        existing_label = widget.findChild(QLabel, f"icon_{prefix}{vehicle_number}1")
        if existing_label: self.replace_icon_widget(existing_label, icon_type)
        else: print(f"icon_{prefix}{vehicle_number}1 label does not exist")

    def replace_icon_widget(self, icon_label, icon_type):
        if icon_label: 
            icon_label._icon_type = icon_type
            # Update the original icon pixmap to the new icon
            icon_pixmap = self.style().standardIcon(icon_type).pixmap(16, 16)
            icon_label._original_icon_pixmap = icon_pixmap
            self.repaint_icon(icon_label)

    def replace_specific_status_widget(self, vehicle_number, prefix):
        layout = getattr(self, f"vehicle{vehicle_number}_column01_layout")
        widget = getattr(self, f"vehicle{vehicle_number}_column01_widget")
        new_text = self.key_to_text_dict[prefix] + str(self.feedback_dict[prefix][vehicle_number])
        existing_label = widget.findChild(QLabel, f"{prefix}{vehicle_number}")
        if existing_label: existing_label.setText(new_text)
        else: print(f"label with name {prefix}{vehicle_number} does not exist")

#used by ros to open a window. Needed in order to start PyQt on a different thread than ros
def OpenWindow(ros_node, borders=False):
    app = QApplication(sys.argv)
    window_width, window_height = 1200, 800

    # Prepare splash image
    img_path = os.path.join(os.path.dirname(__file__), "FRoSt_Lab.png")
    pixmap = QPixmap(img_path)
    pixmap = pixmap.toImage()
    pixmap.invertPixels()
    pixmap = QPixmap.fromImage(pixmap)
    if pixmap.isNull():
        print(f"Warning: ⚠️ Could not load splash image '{img_path}'. Using solid color instead.")
        pixmap = QPixmap(window_width, window_height)
        pixmap.fill(QColor(("#0F1C37")))
    else:
        background = QPixmap(window_width, window_height)
        background.fill(QColor("#0F1C37"))
        painter = QPainter(background)
        x = (window_width - pixmap.width()) // 2
        y = (window_height - pixmap.height()) // 2
        painter.drawPixmap(x, y, pixmap)
        painter.end()
        pixmap = background

    pixmap = pixmap.scaled(window_width, window_height, Qt.AspectRatioMode.KeepAspectRatio, Qt.TransformationMode.SmoothTransformation)
    splash = CustomSplash(pixmap)
    splash.setWindowFlag(Qt.WindowType.WindowStaysOnTopHint, False)
    splash.show()

    # Move splash to the center of the screen
    screen = app.primaryScreen()
    if QApplication.screens():
        mouse_pos = QCursor.pos()
        for scr in QApplication.screens():
            if scr.geometry().contains(mouse_pos):
                screen = scr
                break
    screen_geometry = screen.geometry()
    x = screen_geometry.x() + (screen_geometry.width() - window_width) // 2
    y = screen_geometry.y() + (screen_geometry.height() - window_height) // 2
    splash.move(x, y)

    app.processEvents()

    # Show configuration dialog ON TOP of splash
    options = [f"Vehicle {i}" for i in range(1, 5)] + ["select custom: "]
    dlg = ConfigurationWindow(options, parent=splash, background_color="#0F1C37", text_color="#FFFFFF")
    dlg.setWindowModality(Qt.WindowModality.ApplicationModal)
    dlg.setWindowFlag(Qt.WindowType.WindowStaysOnTopHint)
    dlg.move(
        x + (window_width - dlg.width()) // 2,
        y + (window_height - dlg.height()) // 2
    )

    selected_vehicles = None
    if dlg.exec():
        #example selected_vehicles: [4, 5, 7, 999] ##Vehicle selection can be any int
        selected_vehicles = dlg.get_states()
    else:
        sys.exit(0)

    # Raise the splash again in case it lost focus
    splash.raise_()
    splash.showMessage("Loading main window...")

    if borders:
        app.setStyleSheet("""*{border: 1px solid red;}""")

    result = {}

    def build_main_window():
        window = MainWindow(ros_node, selected_vehicles)
        window.resize(window_width, window_height)
        window.move(x, y)
        result['window'] = window
        QTimer.singleShot(3000, lambda: (
            window.show(),
            window.activateWindow(),
            splash.close()
        ))

    QTimer.singleShot(500, build_main_window)

    #return the app, the result, and the selected vehicle numbers
    return app, result, selected_vehicles

class CustomSplash(QWidget):
    def __init__(self, pixmap, parent=None):
        super().__init__(parent)
        self.setWindowFlags(Qt.WindowType.FramelessWindowHint | Qt.WindowType.WindowStaysOnTopHint)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.label = QLabel(self)
        self.label.setPixmap(pixmap)
        self.label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.resize(pixmap.size())
        # Optional: Add a message label
        self.message_label = QLabel("", self)
        self.message_label.setAlignment(Qt.AlignmentFlag.AlignBottom | Qt.AlignmentFlag.AlignHCenter)
        self.message_label.setStyleSheet("color: white; font-size: 18pt; font-weight: bold;")
        self.message_label.setGeometry(0, pixmap.height() - 60, pixmap.width(), 60)

    def showMessage(self, text):
        self.message_label.setText(text)

class ConfirmationDialog(QDialog):
    """
    Custom dialog for confirming or aborting mission-related actions (e.g., shutdown, recall).
    Presents a message and Accept/Decline buttons.

    Parameters:
        window_title (str): The title of the dialog window.
        message_text (str): The message to display in the dialog.
        parent (QWidget, optional): The parent widget.
    """
    def __init__(self, window_title, message_text, parent=None, background_color="white", text_color="black", pop_up_window_style=None):
        super().__init__(parent)

        self.setWindowTitle(window_title)
        self.setStyleSheet(pop_up_window_style)

        QBtn = (
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )

        self.buttonBox = QDialogButtonBox(QBtn)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

        # Change button labels
        ok_button = self.buttonBox.button(QDialogButtonBox.StandardButton.Ok)
        ok_button.setText("Accept")

        cancel_button = self.buttonBox.button(QDialogButtonBox.StandardButton.Cancel)
        cancel_button.setText("Decline")

        layout = QVBoxLayout()
        message = QLabel(message_text)
        layout.addWidget(message)
        layout.addWidget(self.buttonBox)
        self.setLayout(layout)

class LoadMissionsDialog(QDialog):
    """
    Custom dialog for loading the vehicle missions
    Presents a window with configuration options

    Parameters:
        
    """
    def __init__(self, parent=None, vehicle=0, background_color="white", text_color="black", pop_up_window_style=None, selected_vehicles=None):
        """
        Parameters:
            options (list of str): List of checkbox labels.
        """
        super().__init__(parent)
        if not vehicle: self.setWindowTitle("Load All Missions")
        else: self.setWindowTitle(f"Load Vehicle{vehicle} Mission")
        self.checkboxes = {}
        layout = QVBoxLayout()

        # Make the dialog not resizable
        self.setFixedSize(300, 200)  # Set to your preferred width and height

        self.setStyleSheet(pop_up_window_style)

        self.file_display_labels = {}  # Store file display labels for each tab

        if not vehicle:
            self.selected_files = {}  # Store files per tab
            #Create the tabs
            self.pop_up_tabs = QTabWidget()
            #Orient the tabs at the tob of the screen
            self.pop_up_tabs.setTabPosition(QTabWidget.TabPosition.North)
            #The tabs' order can't be changed or moved
            self.pop_up_tabs.setMovable(False)
            tab_names = [f"Vehicle {i}" for i in selected_vehicles]
            
            # Create separate content widgets for each tab
            for name in tab_names: 
                content_widget = QWidget()
                content_layout = QVBoxLayout(content_widget)
                
                # Add file selection widgets to each tab
                file_section_label = QLabel("Select Mission File:")
                file_section_label.setStyleSheet(f"font-weight: bold; color: {text_color};")
                content_layout.addWidget(file_section_label)

                # File display for this tab
                file_display_label = QLabel("No file selected")
                file_display_label.setWordWrap(True)
                file_display_label.setStyleSheet(f"border: 1px solid {text_color}; padding: 8px; min-height: 40px; color: {text_color}; background-color: {background_color};")
                content_layout.addWidget(file_display_label)

                # Store the file display label for this tab
                self.file_display_labels[name] = file_display_label
                
                # Browse button for this tab
                browse_button = QPushButton("Browse Files...")
                browse_button.setStyleSheet(f"background-color: {background_color}; color: {text_color}; border: 1px solid {text_color}; padding: 5px;")
                browse_button.clicked.connect(lambda checked, tab=name: self.browse_file(tab))
                content_layout.addWidget(browse_button)
                
                self.pop_up_tabs.addTab(content_widget, name)
            
            # Add the tab widget to the main layout
            layout.addWidget(self.pop_up_tabs)
            
        else:
            # For single vehicle, add file selection directly to layout
            # File selection section
            self.selected_file = None  # Initialize for single vehicle mode
            file_section_label = QLabel("Select Mission File:")
            file_section_label.setStyleSheet(f"font-weight: bold; color: {text_color};")
            layout.addWidget(file_section_label)

            # File display
            self.file_display_label = QLabel("No file selected")
            self.file_display_label.setWordWrap(True)
            self.file_display_label.setStyleSheet(f"border: 1px solid {text_color}; padding: 8px; min-height: 40px; color: {text_color}; background-color: {background_color};")
            layout.addWidget(self.file_display_label)
            
            # Browse button
            browse_button = QPushButton("Browse Files...")
            browse_button.setStyleSheet(f"background-color: {background_color}; color: {text_color}; border: 1px solid {text_color}; padding: 5px;")
            browse_button.clicked.connect(self.browse_file)
            layout.addWidget(browse_button)

        # OK/Cancel buttons
        buttonBox = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok)
        buttonBox.accepted.connect(self.validate_and_accept)
        button_row = QHBoxLayout()

        if not vehicle:
            applyAllButton = QPushButton("Apply to All")
            button_row.addWidget(applyAllButton)
            applyAllButton.clicked.connect(
                lambda: self.apply_to_all(
                    background_color=background_color,
                    text_color=text_color,
                    pop_up_window_style=pop_up_window_style
                )
            )

        button_row.addWidget(buttonBox)
        layout.addLayout(button_row)
        self.setLayout(layout)

    def apply_to_all(self, background_color=None, text_color=None, pop_up_window_style=None):
        # Get the current tab name
        current_tab_index = self.pop_up_tabs.currentIndex()
        current_tab_name = self.pop_up_tabs.tabText(current_tab_index)
        # Check if a file is selected for the current tab
        selected_file = self.selected_files.get(current_tab_name)
        if not selected_file:
            QMessageBox.warning(self, "No File Selected", "Please select a file for the current tab before applying to all.")
            return

        # Confirm with the user
        dlg = ConfirmationDialog(
            "Apply to All?",
            "Are you sure you want to apply this file to all vehicles? This will overwrite any other files you have already selected.",
            self,
            background_color=background_color,
            text_color=text_color,
            pop_up_window_style=pop_up_window_style
        )
        if dlg.exec():
            # Apply the selected file to all tabs (even if not previously selected)
            for tab_name in self.file_display_labels:
                self.selected_files[tab_name] = selected_file
                self.update_file_display(tab_name)
        # else: do nothing (user cancelled)


    def browse_file(self, tab_name=None):
        """Open file dialog to select a single mission file for specific tab"""
        if tab_name:
            # Tabbed mode
            file_path, _ = QFileDialog.getOpenFileName(
                self,
                f"Select Mission File for {tab_name}",
                "",
                "Mission Files (*.yaml *.yml *.json);;All Files (*)"
            )
            
            if file_path:
                self.selected_files[tab_name] = file_path
                self.update_file_display(tab_name)
        else:
            # Single vehicle mode
            file_path, _ = QFileDialog.getOpenFileName(
                self,
                "Select Mission File",
                "",
                "Mission Files (*.yaml *.yml *.json);;All Files (*)"
            )
            
            if file_path:
                self.selected_file = file_path
                self.update_file_display()

    def update_file_display(self, tab_name=None):
        """Update the display showing selected file"""
        if tab_name:
            # Update specific tab
            if tab_name in self.selected_files:
                file_name = os.path.basename(self.selected_files[tab_name])
                self.file_display_labels[tab_name].setText(file_name)
            else:
                self.file_display_labels[tab_name].setText("No file selected")
        else:
            # Update all tabs (for single vehicle mode)
            if hasattr(self, 'file_display_label') and hasattr(self, 'selected_file'):
                if self.selected_file:
                    file_name = os.path.basename(self.selected_file)
                    self.file_display_label.setText(file_name)
                else:
                    self.file_display_label.setText("No file selected")

    def validate_and_accept(self):
        if hasattr(self, 'selected_files'):
            # Tabbed mode - check all tabs have files
            if len(self.selected_files) == len(self.file_display_labels) and all(self.selected_files.values()):
                self.accept()
            else:
                from PyQt6.QtWidgets import QMessageBox
                QMessageBox.warning(self, "Files Required", "Please select a mission file for each Vehicle before continuing.")
        elif hasattr(self, 'selected_file') and self.selected_file:
            self.accept()
        else:
            from PyQt6.QtWidgets import QMessageBox
            QMessageBox.warning(self, "File Required", "Please select a mission file before continuing.")

    def get_states(self):
        """Returns a dict containing the selected file path(s)."""
        if hasattr(self, 'selected_files'):
            return {"selected_files": self.selected_files}
        else:
            return {"selected_file": getattr(self, 'selected_file', None)}

class StartMissionsDialog(QDialog):
    """
    Custom dialog for starting the vehicle missions
    Presents a window with configuration options

    Parameters:
        window_title (str): The title of the dialog window.
        message_text (str): The message to display in the dialog.
        parent (QWidget, optional): The parent widget.
    """
    def __init__(self, options, parent=None, passed_option_map=None, vehicle=0, background_color="white", text_color="black", pop_up_window_style=None):
        """
        Parameters:
            options (list of str): List of checkbox labels.
        """
        super().__init__(parent)
        if not vehicle: self.setWindowTitle("Start All Missions Configuration")
        else: self.setWindowTitle(f"Start Vehicle{vehicle} Mission Configuration")
        self.checkboxes = {}
        self.passed_option_map = passed_option_map
        layout = QVBoxLayout()

        # Make the dialog not resizable
        self.setFixedSize(300, 200)  # Set to your preferred width and height

        self.setStyleSheet(pop_up_window_style)

        self.inputs = {}

        for opt in options:
            if "rosbag prefix" in opt.lower():
                le = QLineEdit()
                le.setPlaceholderText("Enter Rosbag Prefix...")
                self.inputs[opt] = le
                layout.addWidget(le)
            else:
                cb = QCheckBox(opt)
                cb.setChecked(False)
                self.checkboxes[opt] = cb
                layout.addWidget(cb)

        # OK/Cancel buttons
        buttonBox = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok)
        buttonBox.accepted.connect(self.validate_and_accept)
        layout.addWidget(buttonBox)
        self.setLayout(layout)
    
    def validate_and_accept(self):
        states = self.get_states()
        #if record rosbag was chosen, a prefix must be given, as well as the opposite
        if states["record_rosbag"] and not states["rosbag_prefix"]: 
            QMessageBox.warning(self, "Prefix Required to Record Rosbag", "Please enter a rosbag prefix before continuing")
        elif not states["record_rosbag"] and states["rosbag_prefix"]: 
            QMessageBox.warning(self, "Record Rosbag Required to have Prefix", "Please select record rosbag before continuing")
        else:
            self.accept()
            return

    def get_states(self):
        """
        Returns a dict of {easy_key: value} for each option.
        """
        result = {}
        for opt, cb in self.checkboxes.items():
            result[self.passed_option_map[opt]] = cb.isChecked()
        for opt, le in self.inputs.items():
            result[self.passed_option_map[opt]] = le.text()
        return result

class ConfigurationWindow(QDialog):
    """
    Custom configuration dialog with multiple checkboxes.
    Returns the checked states as a dictionary if accepted.
    """
    def __init__(self, options, parent=None, background_color="white", text_color="black"):
        """
        Parameters:
            options (list of str): List of checkbox labels.
        """
        super().__init__(parent)
        self.setWindowTitle("Configuration")
        self.checkboxes = {}
        self.custom_inputs = []
        self.custom_plus_buttons = []
        layout = QVBoxLayout()
        self.background_color = background_color
        self.text_color = text_color
        self.MAX_VEHICLES = 4
        self.HIGHEST_VEHICLE_LABEL = 999

        # Make the dialog not resizable
        # self.setFixedSize(300, 200)  
        self.setMinimumWidth(300)
        self.resize(300, 200)

        self.setStyleSheet(f"""
            QDialog {{
                background-color: {self.background_color};
                color: {self.text_color};
            }}
            
            QLabel, QCheckBox {{
                color: {self.text_color};
            }}

            QCheckBox::indicator {{
                width: 13px;
                height: 13px;
            }}

            QCheckBox::indicator:checked {{
                border: 1px solid {self.text_color};
            }}

            QCheckBox::indicator:unchecked {{
                background-color: {self.text_color};
                border: 1px solid {self.text_color};
            }}

            QLineEdit {{
                background-color: {self.background_color};
                color: {self.text_color};
                border: 1px solid {self.text_color};
                padding: 2px;
            }}
        """)

        self.inputs = []

        # Create a checkbox for each option
        for opt in options:
            if "select custom:" not in opt.lower():
                cb = QCheckBox(opt)
                cb.setChecked(False)
                self.checkboxes[opt] = cb
                layout.addWidget(cb)

        # Container for custom Vehicle inputs
        self.custom_container = QVBoxLayout()
        layout.addLayout(self.custom_container)

        # Add the first "+" button
        self.add_custom_plus_button()

        # OK/Cancel buttons
        buttonBox = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok)
        buttonBox.accepted.connect(self.validate_and_accept)
        layout.addWidget(buttonBox)
        self.setLayout(layout)

    def add_custom_plus_button(self):
        custom_input_style = f"background-color: {self.background_color}; color: {self.text_color}; border: 2px solid {self.text_color};"
        plus_btn = QPushButton("+ Add Custom Vehicle")
        plus_btn.setStyleSheet(custom_input_style)
        plus_btn.clicked.connect(lambda: self.add_custom_input(plus_btn))
        self.custom_container.addWidget(plus_btn)
        self.custom_plus_buttons.append(plus_btn)

    def add_custom_input(self, plus_btn):
        # Count currently selected Vehicles (checkboxes + custom inputs)
        current_count = sum(cb.isChecked() for cb in self.checkboxes.values())
        current_count += len(self.custom_inputs)
        if current_count >= self.MAX_VEHICLES:
            QMessageBox.warning(self, "Limit Reached", f"You cannot add more than {self.MAX_VEHICLES} Vehicles.")
            return
        # Remove the plus button that was clicked
        self.custom_container.removeWidget(plus_btn)
        plus_btn.hide()
        # Add a new QLineEdit for custom input
        le = QLineEdit()
        le.setPlaceholderText("Enter Custom Number...")
        self.custom_inputs.append(le)
        self.custom_container.addWidget(le)
        # Add a new plus button below this input
        self.add_custom_plus_button()
        # Adjust the dialog size to fit new content
        self.adjustSize()
        
    def validate_and_accept(self):
        states = self.get_states()
        valid_custom = True
        valid_vehicle_number = True
        if not states:
            QMessageBox.warning(self, "Selection Required", "Please select at least one Vehicle before continuing.")
        elif len(states) > self.MAX_VEHICLES:
            QMessageBox.warning(self, "Max Vehicle Limit Reached", "Selection of more than 4 Vehicles not allowed")
        else:
            for value in states:
                try:
                    num = int(value)
                    if num > self.HIGHEST_VEHICLE_LABEL or num <= 0: valid_vehicle_number = False
                except:
                    valid_custom = False
            if len(states) != len(set(states)):
                QMessageBox.warning(self, "Duplicate Vehicles", "Please ensure all Vehicle numbers are unique.")
            elif not valid_custom:
                QMessageBox.warning(self, "Invalid Custom", "Please enter a valid integer for custom Vehicle number.")
            elif not valid_vehicle_number:
                QMessageBox.warning(self, "Invalid Vehicle Number", "Please enter an integer from 1-999.")
            else:
                self.accept()

    def get_states(self):
        selected_vehicles = []
        for opt, cb in self.checkboxes.items():
            if cb.isChecked():
                try:
                    num = int(opt.split()[-1])
                    selected_vehicles.append(num)
                except Exception:
                    pass
        for le in self.custom_inputs:
            value = le.text().strip()
            if value:
                try:
                    selected_vehicles.append(int(value))
                except ValueError:
                    selected_vehicles.append(value)
        return selected_vehicles

class CalibrateFinsDialog(QDialog):
    """
    Custom dialog for fin calibration.
    """
    def __init__(self, parent=None, background_color="white", text_color="black", pop_up_window_style=None, selected_vehicles=None, passed_ros_node=None, on_slider_change=None):
        super().__init__(parent)
        self.setWindowTitle("Fin Calibration:")
        self.fin_sliders = {}
        self.on_slider_change = on_slider_change  # <-- store callback
        layout = QVBoxLayout()
        self.setFixedSize(300, 200)
        self.setStyleSheet(pop_up_window_style)
        self.pop_up_tabs = QTabWidget()
        self.pop_up_tabs.setTabPosition(QTabWidget.TabPosition.North)
        self.pop_up_tabs.setMovable(False)
        tab_names = [f"Vehicle {i}" for i in selected_vehicles]

        for idx, name in enumerate(tab_names):
            vehicle_num = selected_vehicles[idx]
            content_widget = QWidget()
            content_widget.setStyleSheet(f"background-color: {background_color};")
            content_layout = QVBoxLayout(content_widget)
            self.fin_sliders[name] = []
            for i in range(1, 4):
                row = QHBoxLayout()
                fin_label = QLabel(f"Fin {i}: ")
                fin_label.setStyleSheet(f"font-weight: bold; color: {text_color};")
                row.addWidget(fin_label)
                fin_slider = QSlider(Qt.Orientation.Horizontal)
                fin_slider.setMinimum(-180)
                fin_slider.setMaximum(180)
                fin_slider.setValue(0)
                fin_slider.setTickInterval(1)
                # moves one with the arrows
                fin_slider.setSingleStep(1)
                # moves one with the page up/down buttons
                fin_slider.setPageStep(5)
                fin_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
                fin_slider.setStyleSheet(f"color: {text_color};")
                row.addWidget(fin_slider)
                value_label = QLabel(str(fin_slider.value()))
                value_label.setStyleSheet(f"color: {text_color};")
                fin_slider.valueChanged.connect(lambda val, lbl=value_label: lbl.setText(str(val)))

                if self.on_slider_change:
                    fin_slider.valueChanged.connect(
                        lambda _, vnum=vehicle_num, tab=name: self._handle_slider_change(vnum, tab)
                    )
                row.addWidget(value_label)
                content_layout.addLayout(row)
                self.fin_sliders[name].append(fin_slider)
            self.pop_up_tabs.addTab(content_widget, name)

        note_label = QLabel("(Arrows -> 1, Pg Up/Down -> 5)")
        note_label.setStyleSheet(f"font-weight: bold; color: {text_color};")
        layout.addWidget(note_label)
        layout.addWidget(self.pop_up_tabs)
        buttonBox = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok)
        ok_button = buttonBox.button(QDialogButtonBox.StandardButton.Ok)
        ok_button.setText("Save Changes To Params")
        buttonBox.accepted.connect(self.validate_and_accept)
        button_row = QHBoxLayout()
        button_row.addWidget(buttonBox)
        layout.addLayout(button_row)
        self.setLayout(layout)

    def _handle_slider_change(self, vehicle_num, tab_name):
        # Called whenever a slider changes for a vehicle
        if self.on_slider_change:
            # Get current values for this vehicle
            values = [slider.value() for slider in self.fin_sliders[tab_name]]
            self.on_slider_change(vehicle_num, values)

    def validate_and_accept(self): 
        self.accept()

    def get_states(self):
        """
        Returns a dict mapping each vehicle/tab name to a list of its fin slider values.
        Example: {'Vehicle 1': [val1, val2, val3], ...}
        """
        states = {}
        for tab_name, sliders in self.fin_sliders.items():
            states[tab_name[-1]] = [slider.value() for slider in sliders]
        return states