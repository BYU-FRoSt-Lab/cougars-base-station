#test
import sys 
import random, time, os
from PyQt6.QtWidgets import (QScrollArea, QApplication, QMainWindow, 
    QWidget, QPushButton, QTabWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame,
    QSizePolicy, QSplashScreen, QCheckBox, QSpacerItem, QGridLayout, 
    QStyle, QWidget, QDialog, QDialogButtonBox, QMessageBox
)
from PyQt6.QtCore import QSize, Qt, QTimer, pyqtSignal, QObject, QEvent

from PyQt6.QtGui import QColor, QPalette, QFont, QPixmap, QKeySequence, QShortcut, QCursor, QPainter

from base_station_interfaces.srv import BeaconId
from functools import partial

class MainWindow(QMainWindow):
    # Initializes GUI window with a ros node inside
    update_connections_signal = pyqtSignal(object)
    update_status_signal = pyqtSignal(object)
    update_console_signal = pyqtSignal(object, int)
    kill_confirm_signal = pyqtSignal(object)
    surface_confirm_signal = pyqtSignal(object)
    def __init__(self, ros_node, coug_dict):
        """
        Initializes GUI window with a ros node inside

        Parameters:
            ros_node (node): node passed in from ros in order to access the publisher
        """
        
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("CoUGARS_GUI")
        scale = 300  #Manually scaling the window
        self.resize(QSize(4 * scale, 3 * scale))

        self.selected_cougs = []

        for coug_data, included in coug_dict.items():
            if included: self.selected_cougs.append(int(str(coug_data[-1])))

        ###This is how the coug info gets into the GUI
        self.feedback_dict = {
            #0->negative, 1->positive, 2->waiting
            #Cougs 1-3 connections
            "Wifi_connections": {coug_num: 2 for coug_num in self.selected_cougs},
            "Radio_connections": {coug_num: 2 for coug_num in self.selected_cougs},
            "Modem_connections": {coug_num: 2 for coug_num in self.selected_cougs},

            #0->negative, 1->positive, 2->waiting
            #Cougs 1-3 sensors
            "DVL_sensors": {coug_num: 2 for coug_num in self.selected_cougs},
            "GPS_sensors": {coug_num: 2 for coug_num in self.selected_cougs},
            "IMU_sensors": {coug_num: 2 for coug_num in self.selected_cougs},
            "Leak_sensors": {coug_num: 2 for coug_num in self.selected_cougs},
            "Battery_sensors": {coug_num: 2 for coug_num in self.selected_cougs},

            #Cougs 1-3 status messages
            "Status_messages": {coug_num: "" for coug_num in self.selected_cougs},

            #Cougs 1-3 last messages
            "Last_messages": {coug_num: "" for coug_num in self.selected_cougs},

            #Cougs 1-3 message logs, lists of strings
            "Console_messages": {coug_num: [] for coug_num in self.selected_cougs},

            #Cougs 1-3 message logs, lists of strings
            "Missions": {coug_num: "" for coug_num in self.selected_cougs},    

            #Cougs 1-3 seconds since last connection, list of ints
            "Modem_seconds": {coug_num: 2 for coug_num in self.selected_cougs},    

            #Cougs 1-3 seconds since last radio connection, list of ints
            "Radio_seconds": {coug_num: 2 for coug_num in self.selected_cougs},     

            #Cougs 1-3 X Position in the DVL frame
            "XPos": {coug_num: 2 for coug_num in self.selected_cougs},     

            #Cougs 1-3 Y Position in the DVL frame
            "YPos": {coug_num: 2 for coug_num in self.selected_cougs},

            #Cougs 1-3 Depth, list of ints
            "Depth": {coug_num: 2 for coug_num in self.selected_cougs},

            #Cougs 1-3 Heading, list of ints
            "Heading": {coug_num: 2 for coug_num in self.selected_cougs},

            #Cougs 1-3 Waypoint, list of ints
            "Waypoint": {coug_num: 2 for coug_num in self.selected_cougs},

            #Cougs 1-3 Velocities, list of ints
            "DVL_vel": {coug_num: 2 for coug_num in self.selected_cougs}
        }

        #this is used to connect the feedback_dict keys to what is actually printed, in _update_status_gui
        self.key_to_text_dict = {
            "XPos": "x (meters): ",
            "YPos": "y (meters): ",
            "Depth": "Depth (meters): ",
            "Heading": "Heading (units?): ",
            "Waypoint": "Current Waypoint: ",
            "DVL_vel": "DVL Velocity (m/s): ",
            "Battery_sensors": "Battery %: "
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
        tab_names = ["General"] + [f"Coug {i}" for i in self.selected_cougs]
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

            if name.lower() != "general":
                # For Coug tabs, add specific widgets and console log
                content_layout.addWidget(self.set_specific_coug_widgets(int(name[-1])))
                content_layout.addWidget(self.make_hline())
                content_layout.addWidget(self.create_specific_coug_console_log(int(name[-1])))
            else:
                # For General tab, add general widgets
                content_layout.addWidget(content)
                self.set_general_page_widgets()

            # Set the combined layout
            content_widget.setLayout(content_layout)
            self.tab_dict[name][0] = content_widget

            # Add to tabs
            self.tabs.addTab(content_widget, name)
            self.set_background(content_widget, "cadetblue")

        # Connect tab change to scroll-to-bottom for console logs
        self.tabs.currentChanged.connect(self.scroll_console_to_bottom_on_tab)

        #Emergency exit GUI button
        self.emergency_exit_gui_button = QPushButton("Close GUI")
        self.emergency_exit_gui_button.clicked.connect(self.close_window)
        self.emergency_exit_gui_button.setStyleSheet("background-color: red; color: black;")

        #Ctrl+C shortcut to close the window
        shortcut = QShortcut(QKeySequence("Ctrl+C"), self)
        shortcut.activated.connect(self.close)

        #The overall layout is vertical
        self.main_layout = QVBoxLayout()
        #Add the tabs to the main layout
        self.main_layout.addWidget(self.tabs)
        #button confirmation label
        self.confirm_reject_label = QLabel("Confirmation/Rejection messages from command buttons will appear here")
        self.main_layout.addWidget(self.confirm_reject_label, alignment=Qt.AlignmentFlag.AlignTop)
        self.main_layout.addWidget(self.emergency_exit_gui_button)

        #create a container widget, and place the main layout inside of it
        self.container = QWidget()
        self.container.setObjectName("MyContainer")
        self.container.setLayout(self.main_layout)
        #the container with the main layout is set as teh central widget
        self.setCentralWidget(self.container)

        #creates a signal that is sent from recieve_connections, and sent to _update_connections_gui. 
        #this avoids the error of the gui not working on the main thread
        self.update_connections_signal.connect(self._update_connections_gui)
        self.update_status_signal.connect(self._update_status_gui)
        self.update_console_signal.connect(self._update_console_gui)
        self.kill_confirm_signal.connect(self._update_kill_confirmation_gui)
        self.surface_confirm_signal.connect(self._update_surf_confirmation_gui)

        # self.show()

    def scroll_console_to_bottom_on_tab(self, index):
        """
        Ensures the console log for a Coug tab is always scrolled to the bottom when the tab is selected.

        This function is connected to the QTabWidget's currentChanged signal. When the user switches
        to a Coug tab, it finds the corresponding QScrollArea for that Coug's console log and scrolls
        it to the bottom, so the latest messages are always visible.

        Parameters:
            index (int): The index of the newly selected tab.
        """
        # Get the name of the tab at the given index
        tab_name = self.tabs.tabText(index)
        # Only act if the tab is a Coug tab (e.g., "Coug 1", "Coug 2", "Coug 3")
        if tab_name.startswith("Coug"):
            # Extract the Coug number from the tab name
            coug_number = int(tab_name.split()[-1])
            # Get the scroll area for this Coug's console log
            scroll_area = getattr(self, f"coug{coug_number}_console_scroll_area", None)
            if scroll_area:
                # Process any pending events to ensure the layout is up to date
                QApplication.processEvents()
                # Scroll the vertical scrollbar to the maximum (bottom)
                scroll_area.verticalScrollBar().setValue(scroll_area.verticalScrollBar().maximum())

    #function to close the GUI window(s). Used by the keyboard interrupt signal or the exit button
    def close_window(self):
        #pop-up window
        dlg = AbortMissionsDialog("Close Window?", "Are you sure you want to close the GUI window?", self)
        #if confirm is selected
        if dlg.exec():
            self.confirm_reject_label.setText("Closing Window...")
            print("Closing the Window now...")
            self.close()  
        else:
            self.confirm_reject_label.setText("Canceling Close Window command...")
            for i in self.selected_cougs:
                self.recieve_console_update("Canceling Close Window command...", i)

    #in order to replace a label, you must know the widgets name, the parent layout, and the parent widget
    def replace_label(self, widget_name, parent_layout, parent_widget, new_label, color=""):
        """
        Replaces a label inside of the GUI

        Parameters:
            widget_name: the name of the widget to be changed
            parent_layout: the parent layout of the widget to be changed
            parent_widget: the parent widget of the widget to be changed
            new_label: the new text/icon the label will be changed to
            color: optional color to change the label to
        """

        #find the widget in reference to its parent widget
        temp_widget = parent_widget.findChild(QWidget, widget_name)
        if temp_widget:
            #the index of the widget in respect to its parent layout
            index = parent_layout.indexOf(temp_widget)
        else:
            print(f"not found. widget_name: {widget_name} : parent_widget: {parent_widget} temp_widget: {temp_widget}")
            for i in self.selected_cougs: self.recieve_console_update("GUI error. See terminal.", i)
            return

        parent_layout.removeWidget(temp_widget)
        #set parent to none so that it doesn't have any lingering consequences
        temp_widget.setParent(None)

        #The new label has the same name as the old one, so that it can be changed again
        new_label.setObjectName(widget_name)
        #set the optional color, if there wasn't a color passed, then it doesn't change anything
        new_label.setStyleSheet(f"color: {color};")
        #insert the new widget in the same index as the old one was, so the order of the text doesn't
        parent_layout.insertWidget(index, new_label)

    def get_status_message_color(self, message):
        # Returns a color and possibly modified message string based on status.
        if message.lower() == "running": message_color = "green"
        elif message.lower() == "no connection": message_color = "red"
        elif message.lower() == "waiting": message_color = "yellow"
        elif not message: 
            message_color = "orange"
            message = "No message to be read"
        else: 
            message = "Status flag unrecognized: " + message
            message_color = "blue"
        return message_color, message

    "/*Override the resizeEvent method in the sub class*/"
    def resizeEvent(self, event):
        size = self.size()
        width_px = self.width() // (len(self.selected_cougs) + 1) - 10
        self.resizeTabs(width_px)
        # Dynamically resize each console scroll area
        for i in self.selected_cougs:  # Assuming Coug 1-3
            scroll_area = getattr(self, f"coug{i}_console_scroll_area", None)
            if scroll_area:
                scroll_area.setFixedHeight(int(self.height() * 0.2))
            # Dynamically set width of column0_widget
            column0_widget = getattr(self, f"coug{i}_column0_widget", None)
            if column0_widget:
                column0_widget.setMaximumWidth(int(self.width() * 0.16))  # 16% of window width
            column01_widget = getattr(self, f"coug{i}_column01_widget", None)
            if column01_widget:
                column01_widget.setMaximumWidth(int(self.width() * 0.16))

        super().resizeEvent(event)

    "/*resize the tabs according to the width of the window*/"
    def resizeTabs(self, width_px):
        # Sets the stylesheet for tab width and appearance.
        self.tabs.setStyleSheet(f"""
        QTabBar::tab {{
            height: 30px;
            width: {width_px - 15}px;
            font-size: 12pt;
            padding: 5px;
            background: lightgray;  /* background color of non-selected tab */
            color: black;           /* font color of non-selected tab */
        }}
        QTabBar::tab:selected {{
            background: blue;       /* background color of selected tab */
            color: white;           /* font color of selected tab */
            font-weight: bold;
        }}
        """)

    #used to initialize the background of the tabs
    def set_background(self, widget, color):
        # Sets the background color of a widget.
        palette = widget.palette()
        palette.setColor(widget.backgroundRole(), QColor(color))
        widget.setAutoFillBackground(True)
        widget.setPalette(palette)

    #(NS) -> not yet connected to a signal
    def load_missions_button(self):
        # Handler for 'Load Missions' button on the general tab.
        self.confirm_reject_label.setText("Loading the missions...")
        for i in self.selected_cougs: self.recieve_console_update("Loading the missions...", i)

    #(NS) -> not yet connected to a signal
    def start_missions_button(self):
        # Handler for 'Start Missions' button on the general tab.
        self.confirm_reject_label.setText("Starting the missions...")
        for i in self.selected_cougs: self.recieve_console_update("Starting the missions...", i)

    #(NS) -> not yet connected to a signal
    def spec_load_missions_button(self, coug_number):
        # Handler for 'Load Mission' button on a specific Coug tab.
        self.confirm_reject_label.setText(f"Loading Coug {coug_number} mission...")
        self.recieve_console_update(f"Loading Coug {coug_number} mission...", coug_number)

    #(NS) -> not yet connected to a signal
    def spec_start_missions_button(self, coug_number):
        # Handler for 'Start Mission' button on a specific Coug tab.
        self.confirm_reject_label.setText(f"Starting Coug {coug_number} mission...")
        self.recieve_console_update(f"Starting Coug {coug_number} mission...", coug_number)

    #Connected to the "kill" signal
    def emergency_shutdown_button(self, coug_number):
        # Handler for 'Emergency Shutdown' button, with confirmation dialog.
        message = BeaconId.Request()
        message.beacon_id = coug_number
        dlg = AbortMissionsDialog("Emergency Shutdown?", "Are you sure you want to initiate emergency shutdown?", self)
        if dlg.exec():
            self.confirm_reject_label.setText("Starting Emergency Shutdown...")
            self.recieve_console_update(f"Starting Emergency Shutdown for Coug {coug_number}", coug_number)
            future = self.ros_node.cli.call_async(message)
            # Add callback to handle response
            future.add_done_callback(partial(self.handle_service_response, action="Emergency Shutdown", coug_number=coug_number))
            return future
        else:
            self.confirm_reject_label.setText("Canceling Emergency Shutdown command...")
            self.recieve_console_update(f"Canceling Emergency Shutdown for Coug {coug_number}", coug_number)

    #Connected to the "surface" signal
    def emergency_surface_button(self, coug_number):
        # Handler for 'Emergency Surface' button, with confirmation dialog.
        message = BeaconId.Request()
        message.beacon_id = coug_number
        dlg = AbortMissionsDialog("Emergency Surface?", "Are you sure you want to initiate emergency surface?", self)
        if dlg.exec():
            self.confirm_reject_label.setText("Starting Emergency Surface...")
            self.recieve_console_update(f"Starting Emergency Surface for Coug {coug_number}", coug_number)
            future = self.ros_node.cli2.call_async(message)
            # Add callback to handle response
            future.add_done_callback(partial(self.handle_service_response, action="Emergency Surface", coug_number=coug_number))
            return future
        else:
            self.confirm_reject_label.setText("Canceling Emergency Surface command...")
            self.recieve_console_update(f"Canceling Emergency Surface for Coug {coug_number}", coug_number)

    #used by various buttons to handle services dynamically
    def handle_service_response(self, future, action, coug_number):
        # Handles the result of an asynchronous ROS service call.
        try:
            response = future.result()
            if response.success:
                self.confirm_reject_label.setText(f"{action} Service Initiated Successfully")
                self.recieve_console_update(f"{action} Service Initiated Successfully", coug_number)
            else:
                self.confirm_reject_label.setText(f"{action} Service Initilization Failed")
                self.recieve_console_update(f"{action} Service Initilization Failed", coug_number)
        except Exception as e:
            self.confirm_reject_label.setText(f"{action} service call failed: {e}")
            if coug_number in self.selected_cougs: self.recieve_console_update(f"{action} service call failed: {e}", coug_number)
            else: print(f"{action} service call failed: {e}")

    #(NS) -> not yet connected to a signal
    def recall_cougs(self):
        # Handler for 'Recall Cougs' button on the general tab, with confirmation dialog.
        dlg = AbortMissionsDialog("Recall Cougs?", "Are you sure that you want recall the Cougs? This will abort all the missions, and cannot be undone.", self)
        if dlg.exec():
            self.confirm_reject_label.setText("Recalling the Cougs...")
            for i in self.selected_cougs: self.recieve_console_update("Recalling the Cougs...", i)
        else:
            self.confirm_reject_label.setText("Canceling Recall All Cougs Command...")
            for i in self.selected_cougs: self.recieve_console_update("Canceling Recall All Cougs Command...", i)
    
    #(NS) -> not yet connected to a signal
    def recall_spec_coug(self, coug_number):
        # Handler for 'Recall Coug' button on a specific Coug tab, with confirmation dialog.
        dlg = AbortMissionsDialog("Recall Coug?", "Are you sure that you want to recall this Coug?", self)
        if dlg.exec():
            self.confirm_reject_label.setText(f"Recalling Coug {coug_number}...")
            self.recieve_console_update(f"Recalling Coug {coug_number}...", coug_number)
        else:
            self.confirm_reject_label.setText("Canceling Recall Coug Command...")
            self.recieve_console_update(f"Canceling Recall Coug {coug_number} Command...", coug_number)

    #template to make a vertical line
    def make_vline(self):
        # Returns a vertical line QFrame for use in layouts.
        Vline = QFrame()
        Vline.setFrameShape(QFrame.Shape.VLine)
        Vline.setFrameShadow(QFrame.Shadow.Sunken)
        return Vline

    #template to make a horizontal line
    def make_hline(self):
        # Returns a horizontal line QFrame for use in layouts.
        Hline = QFrame()
        Hline.setFrameShape(QFrame.Shape.HLine)
        Hline.setFrameShadow(QFrame.Shadow.Sunken)
        return Hline

    #used to set all of the widgets on the "general" page tab
    def set_general_page_widgets(self):
        # Sets up the widgets and layouts for the General tab.
        self.general_page_layout = self.tab_dict["General"][1]

        # Create the first column (General Options)
        self.general_page_C0_widget = QWidget()
        self.general_page_C0_layout = QVBoxLayout()
        self.general_page_C0_widget.setLayout(self.general_page_C0_layout)

        # Store widgets and layouts for each Coug column
        self.general_page_coug_widgets = {}
        self.general_page_coug_layouts = {}

        # Add the first column to the layout
        self.general_page_layout.addWidget(self.general_page_C0_widget)

        # For each selected Coug, create a column and add to the layout, separated by vertical lines
        for idx, coug_number in enumerate(self.selected_cougs):
            self.general_page_layout.addWidget(self.make_vline())
            widget = QWidget()
            layout = QVBoxLayout()
            widget.setLayout(layout)
            self.general_page_layout.addWidget(widget)
            self.general_page_coug_widgets[coug_number] = widget
            self.general_page_coug_layouts[coug_number] = layout

        # Add the buttons to the first column
        self.set_general_page_C0_widgets()

        # Set the widgets for each Coug column
        for coug_number in self.selected_cougs:
            self.set_general_page_column_widgets(self.general_page_coug_layouts[coug_number], coug_number)

    #set the widgets of the first column on the general page
    def set_general_page_C0_widgets(self):

        #Create and style the label
        general_label = QLabel("General Options:")
        general_label.setFont(QFont("Arial", 17, QFont.Weight.Bold))
        general_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)

        #Load All Missions button
        self.Load_missions_button = QPushButton("Load Missions (NS)")
        self.Load_missions_button.clicked.connect(self.load_missions_button)
        self.Load_missions_button.setStyleSheet("background-color: blue; color: black;")

        #Start All Missions button
        self.Start_missions_button = QPushButton("Start Missions (NS)")
        self.Start_missions_button.clicked.connect(self.start_missions_button)
        self.Start_missions_button.setStyleSheet("background-color: blue; color: black;")

        #Recall all the cougs button
        self.recall_all_cougs = QPushButton("Recall Cougs (NS)")
        self.recall_all_cougs.clicked.connect(self.recall_cougs)
        self.recall_all_cougs.setStyleSheet("background-color: red; color: black;")

        # Add widgets to the layout
        self.general_page_C0_layout.addWidget(general_label, alignment=Qt.AlignmentFlag.AlignTop)
        self.general_page_C0_layout.addSpacing(100)
        self.general_page_C0_layout.addWidget(self.Load_missions_button, alignment=Qt.AlignmentFlag.AlignTop)
        self.general_page_C0_layout.addSpacing(100)
        self.general_page_C0_layout.addWidget(self.Start_missions_button)
        self.general_page_C0_layout.addSpacing(100)

        # Add spacer to push the rest of the buttons down
        self.general_page_C0_layout.addWidget(self.recall_all_cougs)
        
        # Add remaining buttons (red recall cougs at the bottom)
        spacer = QSpacerItem(0, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)
        self.general_page_C0_layout.addItem(spacer)
            
    #template to set the rest of widgets on the rest of the columns on the general page
    def set_general_page_column_widgets(self, layout, coug_number):
        # Sets up the widgets for each Coug column on the General tab.
        # Create and style the header label for each coug column
        title_label = QLabel(f"Coug {coug_number}:")
        title_label.setFont(QFont("Arial", 17, QFont.Weight.Bold))
        title_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        layout.addWidget(title_label, alignment=Qt.AlignmentFlag.AlignTop)
        layout.addSpacing(20)

        #section labels for each column
        section_titles = ["Connections", "Sensors", "Status", "Last Message"]
        for title in section_titles:
            layout.addWidget(QLabel(title, font=QFont("Arial", 15)), alignment=Qt.AlignmentFlag.AlignTop)
            layout.addSpacing(20)
            #repeated tab_spacing variable used throughout the file, to keep tabs consistent
            self.tab_spacing = 75

            #The connections section contains the wifi, radio, and modem connections for each Coug respectively
            if title == "Connections": 
                wifi_widget = self.create_icon_and_text("Wifi", self.icons_dict[self.feedback_dict["Wifi_connections"][coug_number]], self.tab_spacing)
                wifi_widget.setObjectName(f"Wifi_connections{coug_number}")
                layout.addWidget(wifi_widget)
                layout.addSpacing(20)

                radio_widget = self.create_icon_and_text("Radio", self.icons_dict[self.feedback_dict["Radio_connections"][coug_number]], self.tab_spacing)
                radio_widget.setObjectName(f"Radio_connections{coug_number}")
                layout.addWidget(radio_widget)
                layout.addSpacing(20)

                modem_widget = self.create_icon_and_text("Modem", self.icons_dict[self.feedback_dict["Modem_connections"][coug_number]], self.tab_spacing)
                modem_widget.setObjectName(f"Modem_connections{coug_number}")
                layout.addWidget(modem_widget)
                layout.addSpacing(40)

            #The connections section contains the DVL, GPS, and IMU sensors connections for each Coug respectively
            elif title == "Sensors":
                DVL_sensor_widget = self.create_icon_and_text("DVL", self.icons_dict[self.feedback_dict["DVL_sensors"][coug_number]], self.tab_spacing)
                DVL_sensor_widget.setObjectName(f"DVL_sensors{coug_number}")
                layout.addWidget(DVL_sensor_widget)
                layout.addSpacing(20)

                GPS_sensor_widget = self.create_icon_and_text("GPS", self.icons_dict[self.feedback_dict["GPS_sensors"][coug_number]], self.tab_spacing)
                GPS_sensor_widget.setObjectName(f"GPS_sensors{coug_number}")
                layout.addWidget(GPS_sensor_widget)
                layout.addSpacing(20)
                
                IMU_sensor_widget = self.create_icon_and_text("IMU", self.icons_dict[self.feedback_dict["IMU_sensors"][coug_number]], self.tab_spacing)
                IMU_sensor_widget.setObjectName(f"IMU_sensors{coug_number}")
                layout.addWidget(IMU_sensor_widget)
                layout.addSpacing(40)

            #The status section contains the status message for each Coug respectively
            elif title == "Status":
                status = self.feedback_dict["Status_messages"][coug_number]
                status_color, status = self.get_status_message_color(status)
                label = QLabel(f"{status}", font=QFont("Arial", 13))
                label.setObjectName(f"Status_messages{coug_number}")
                label.setStyleSheet(f"color: {status_color};")  # Change 'red' to any color you want (name, hex, rgb)
                layout.addWidget(label, alignment=Qt.AlignmentFlag.AlignTop)
                layout.addSpacing(40)

            #The Last Message section contains the last message from each Coug respectively
            elif title == "Last Message":
                last_message = self.feedback_dict["Status_messages"][coug_number]
                if last_message: 
                    last_mesage_label = QLabel(self.feedback_dict["Last_messages"][coug_number], font=QFont("Arial", 13), alignment=Qt.AlignmentFlag.AlignTop)
                else:
                    last_mesage_label = QLabel("No messages have been recieved", font=QFont("Arial", 13), alignment=Qt.AlignmentFlag.AlignTop)
                    last_mesage_label.setStyleSheet(f"color: orange;")
                last_mesage_label.setObjectName(f"Last_messages{coug_number}")
                layout.addWidget(last_mesage_label)
                layout.addSpacing(40)

        # Add spacer to push content up
        spacer = QSpacerItem(0, 0, QSizePolicy .Policy.Minimum, QSizePolicy.Policy.Expanding)
        layout.addItem(spacer)

    #This is used to set the widgets on the other tabs, namely Coug1, Coug2, Coug3, etc
    def set_specific_coug_widgets(self, coug_number):
        # Sets up the widgets for a specific Coug tab.
        #temporary container for the entire tab
        temp_container = QWidget()
        temp_container.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        temp_layout = QHBoxLayout(temp_container)
        temp_layout.setSpacing(0)
        temp_layout.setContentsMargins(0, 0, 0, 0)

        #col0 contains connections and sensor icons
        temp_layout.addWidget(self.create_specific_coug_column0(coug_number), alignment=Qt.AlignmentFlag.AlignTop)
        #col1 contains mission and coug1 status
        temp_layout.addWidget(self.create_specific_coug_column01(coug_number), alignment=Qt.AlignmentFlag.AlignTop)

        #vline
        temp_layout.addWidget(self.make_vline())
        #col2 - seconds since last connected and buttons
        temp_layout.addWidget(self.create_coug_buttons_column(coug_number))
        return temp_container

    #The scrolling log at the bottom of the specific coug tabs. 
    def create_specific_coug_console_log(self, coug_number): 
        # Creates the scrollable console log area for a specific Coug tab.
        temp_container = QWidget()
        temp_layout = QVBoxLayout(temp_container)
        setattr(self, f"coug{coug_number}_console_layout", temp_layout)
        setattr(self, f"coug{coug_number}_console_widget", temp_container)

        # First text label (bold title)
        title_text = "Console information/message log"
        title_label = QLabel(title_text)
        title_label.setWordWrap(True)
        title_label.setFont(QFont("Arial", 15, QFont.Weight.Bold))
        title_label.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        temp_layout.addWidget(title_label)

        # Second text label (wrapped long message)
        message_text = ""

        # Create a QLabel from the message_text for displaying the log
        message_label = QLabel(message_text)
        message_label.setWordWrap(True) # Enable word wrapping for readability
        message_label.setFont(QFont("Arial", 13))
        message_label.setAlignment(Qt.AlignmentFlag.AlignLeft | Qt.AlignmentFlag.AlignTop) 
        message_label.setContentsMargins(0, 0, 0, 0)
        message_label.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Preferred)
        message_label.setObjectName(f"Console_messages{coug_number}")

        # Create a QWidget to hold the message label, and a layout for it
        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setContentsMargins(0, 0, 0, 0)
        scroll_layout.addWidget(message_label)

        # Create QScrollArea and set scroll content
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setWidget(scroll_content)
        # Store the scroll area as an attribute for dynamic resizing
        setattr(self, f"coug{coug_number}_console_scroll_area", scroll_area)

        # Add scroll_area to your layout
        temp_layout.addWidget(scroll_area)

        # Add a spacer to push content up and allow for vertical expansion
        spacer = QSpacerItem(0, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)
        temp_layout.addItem(spacer)

        # Return the container widget holding the scrollable log
        return temp_container

    #used to create an icon next to text in a pre-determined fashion
    def create_icon_and_text(self, text, icon=None, temp_tab_spacing=None):
        """
        Creates a QWidget containing an icon (optional) and a text label, arranged horizontally.

        Parameters:
            text (str): The text to display next to the icon.
            icon (QStyle.StandardPixmap, optional): The standard Qt icon to display. If None, no icon is shown.
            temp_tab_spacing (int, optional): Left margin for the layout, used for tab alignment.

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
            icon_label = QLabel()
            # Set the icon pixmap (16x16 pixels)
            icon_label.setPixmap(self.style().standardIcon(icon).pixmap(16, 16))
            icon_label.setContentsMargins(0, 0, 0, 0)
            icon_label.setFixedSize(16, 16)
            temp_layout.addWidget(icon_label, alignment=Qt.AlignmentFlag.AlignVCenter)

        # Create the text label and add to the layout
        text_label = QLabel(text)
        text_label.setFont(QFont("Arial", 13))
        text_label.setContentsMargins(0, 0, 0, 0)
        temp_layout.addWidget(text_label, alignment=Qt.AlignmentFlag.AlignVCenter)

        # Return the container widget with icon and text
        return temp_container

    #used to create the title labels throughout the window
    def create_title_label(self, text):
        # Create and style the label
        temp_label = QLabel(text)
        temp_label.setFont(QFont("Arial", 15, QFont.Weight.Bold))
        temp_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        return temp_label

    #used to create the buttons column on the specific coug pages
    def create_coug_buttons_column(self, coug_number):
        """
        Creates the button column for a specific Coug tab, including mission control and emergency buttons,
        as well as labels for seconds since last connection.

        Parameters:
            coug_number (int): The vehicle number for which to create the button column.

        Returns:
            QWidget: The vertical container widget holding all buttons and labels for the Coug.
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
        setattr(self, f"coug{coug_number}_buttons_column_widget", temp_V_container)
        setattr(self, f"coug{coug_number}_buttons_column_layout", temp_V_layout)

        #load mission (blue)
        self.create_coug_button(coug_number, "load_mission", "Load Mission (NS)", "blue", lambda: self.spec_load_missions_button(coug_number))
        #start mission (blue)
        self.create_coug_button(coug_number, "start_mission", "Start Mission (NS)", "blue", lambda: self.spec_start_missions_button(coug_number))
        #system reboot (red)
        self.create_coug_button(coug_number, "emergency_surface", "Emergency Surface", "red", lambda: self.emergency_surface_button(coug_number))
        #abort mission (red)
        self.create_coug_button(coug_number, "recall", f"Recall Coug {coug_number} (NS)", "red", lambda: self.recall_spec_coug(coug_number))
        #emergency shutdown (red)
        self.create_coug_button(coug_number, "emergency_shutdown", "Emergency Shutdown", "red", lambda: self.emergency_shutdown_button(coug_number))

        temp_spacing = 50
        # Add buttons to the first and second sub-columns with spacing
        temp_layout1.addWidget(getattr(self, f"load_mission_coug{coug_number}_button"))
        temp_layout1.addSpacing(temp_spacing)
        temp_layout1.addWidget(getattr(self, f"start_mission_coug{coug_number}_button"))
        temp_layout1.addSpacing(temp_spacing)
        temp_layout1.addSpacing(temp_spacing)
        temp_layout2.addWidget(getattr(self, f"emergency_surface_coug{coug_number}_button"))
        temp_layout2.addSpacing(temp_spacing)
        temp_layout2.addWidget(getattr(self, f"recall_coug{coug_number}_button"))
        temp_layout2.addSpacing(temp_spacing)
        temp_layout2.addWidget(getattr(self, f"emergency_shutdown_coug{coug_number}_button"))

        # Add the two button columns to the main horizontal layout
        temp_layout.addWidget(temp_sub_container1)
        temp_layout.addWidget(temp_sub_container2)

        # Add a title label and connection time labels to the vertical layout
        temp_V_layout.addWidget(self.create_title_label("Seconds since last connected"))
        self.insert_label(temp_V_layout, "Radio: xxx", coug_number, 1)
        self.insert_label(temp_V_layout, "Accoustics: xxx", coug_number, 0)
        temp_V_layout.addWidget(self.make_hline())
        temp_V_layout.addWidget(temp_container)
        
        # Return the vertical container holding all buttons and labels
        return temp_V_container

    def insert_label(self, temp_layout, text, coug_number, conn_type):
        """
        Inserts a QLabel into the given layout for displaying the seconds since last connection
        for either radio or modem, and stores it as an attribute for later access.

        Parameters:
            temp_layout (QLayout): The layout to add the label to.
            text (str): The text to display in the label.
            coug_number (int): The vehicle number (Coug) this label is for.
            conn_type (int): 1 for radio, 0 for modem (used to determine label name).
        """
        text_label = QLabel(text)
        # Set the object name based on connection type for easy lookup later
        if conn_type:
            name = f"coug{coug_number}_radio_seconds_widget"
        else:
            name = f"coug{coug_number}_modem_seconds_widget"
        setattr(self, name, text_label)
        text_label.setObjectName(name)
        text_label.setFont(QFont("Arial", 13))
        text_label.setContentsMargins(0, 0, 0, 0)
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
        return text_label

    #Dynamically creates a QPushButton with the given properties and stores it as an attribute.
    def create_coug_button(self, coug_number, name, text, color, callback):
        """
        Dynamically creates a QPushButton with the given properties and stores it as an attribute.

        Parameters:
            coug_number (int): Which Coug this button is for.
            name (str): Short functional name for the button (e.g., "start_mission", "disarm_thruster").
            text (str): Text to display on the button.
            color (str): Background color for the button.
            callback (function): Function to call when the button is clicked.
        """
        button = QPushButton(text)
        button.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Fixed)
        button.clicked.connect(callback)
        button.setStyleSheet(f"background-color: {color}; color: black")
        attr_name = f"{name}_coug{coug_number}_button"
        setattr(self, attr_name, button)

    def create_specific_coug_column0(self, coug_number):
        """
        Creates the first column for a specific Coug tab, displaying connection and sensor status icons.

        Parameters:
            coug_number (int): The vehicle number for which to create the column.

        Returns:
            QWidget: The container widget holding all connection and sensor status icons for the Coug.
        """
        # Create a vertical layout for the column and store it as an attribute
        temp_layout = QVBoxLayout()
        setattr(self, f"coug{coug_number}_column0_layout", temp_layout)
        temp_layout.setContentsMargins(0, 0, 0, 0)
        temp_layout.setSpacing(0) 

        # Create the container widget for this column and store it as an attribute
        temp_container = QWidget()
        setattr(self, f"coug{coug_number}_column0_widget", temp_container)
        container_layout = QVBoxLayout(temp_container)
        container_layout.addLayout(temp_layout)

        # Add the Coug title label at the top
        temp_layout.addWidget(self.create_title_label(f"Coug {coug_number}"), alignment=Qt.AlignmentFlag.AlignTop)
        
        # Section: Connections
        temp_label = QLabel("Connections")
        temp_label.setFont(QFont("Arial", 15, QFont.Weight.Bold))
        temp_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        temp_layout.addSpacing(20)
        temp_layout.addWidget(temp_label)

        # Add connection status icons (Wifi, Radio, Modem)
        wifi_widget = self.create_icon_and_text("Wifi", self.icons_dict[self.feedback_dict["Wifi_connections"][coug_number]], 0)
        wifi_widget.setObjectName(f"Spec_Wifi_connections{coug_number}")
        temp_layout.addWidget(wifi_widget)

        radio_widget = self.create_icon_and_text("Radio", self.icons_dict[self.feedback_dict["Radio_connections"][coug_number]], 0)
        radio_widget.setObjectName(f"Spec_Radio_connections{coug_number}")
        temp_layout.addWidget(radio_widget)

        modem_widget = self.create_icon_and_text("Modem", self.icons_dict[self.feedback_dict["Modem_connections"][coug_number]], 0)
        modem_widget.setObjectName(f"Spec_Modem_connections{coug_number}")
        temp_layout.addWidget(modem_widget)
        temp_layout.addSpacing(20)

        # Section: Sensors
        temp_label = QLabel("Sensors")
        temp_label.setFont(QFont("Arial", 15, QFont.Weight.Bold))
        temp_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        temp_layout.addSpacing(20)
        temp_layout.addWidget(temp_label)

        # Add sensor status icons (DVL, GPS, IMU, Leak Detector)
        DVL_sensor_widget = self.create_icon_and_text("DVL", self.icons_dict[self.feedback_dict["DVL_sensors"][coug_number]], 0)
        DVL_sensor_widget.setObjectName(f"Spec_DVL_sensors{coug_number}")
        temp_layout.addWidget(DVL_sensor_widget)

        GPS_sensor_widget = self.create_icon_and_text("GPS", self.icons_dict[self.feedback_dict["GPS_sensors"][coug_number]], 0)
        GPS_sensor_widget.setObjectName(f"Spec_GPS_sensors{coug_number}")
        temp_layout.addWidget(GPS_sensor_widget)
        
        IMU_sensor_widget = self.create_icon_and_text("IMU", self.icons_dict[self.feedback_dict["IMU_sensors"][coug_number]], 0)
        IMU_sensor_widget.setObjectName(f"Spec_IMU_sensors{coug_number}")
        temp_layout.addWidget(IMU_sensor_widget)

        Leak_sensor_widget = self.create_icon_and_text("Leak Detector", self.icons_dict[self.feedback_dict["Leak_sensors"][coug_number]], 0)
        Leak_sensor_widget.setObjectName(f"Spec_Leak_sensors{coug_number}")
        temp_layout.addWidget(Leak_sensor_widget)

        # Return the container widget holding all status icons
        return temp_container

    #create the second sub-column in the first column of the specific cougar pages (starts with "Nodes")
    def create_specific_coug_column01(self, coug_number):
        """
        Creates the second sub-column in the first column of the specific Coug pages.
        This column displays the mission section and the status widgets for the given Coug.

        Parameters:
            coug_number (int): The vehicle number for which to create the column.

        Returns:
            QWidget: The container widget holding the mission and status widgets for the Coug.
        """
        # Create a vertical layout for the column and set margins and spacing
        temp_layout = QVBoxLayout()
        temp_layout.setContentsMargins(0, 0, 0, 0)
        temp_layout.setSpacing(0) 

        # Create the container widget for this column and store it as an attribute
        temp_container = QWidget()
        # Optionally set a maximum width for the container
        # temp_container.setMaximumWidth(220)
        setattr(self, f"coug{coug_number}_column01_layout", temp_layout)
        setattr(self, f"coug{coug_number}_column01_widget", temp_container)
        container_layout = QVBoxLayout(temp_container)
        container_layout.addLayout(temp_layout)

        # Add an (optional) title label at the top
        temp_layout.addWidget(self.create_title_label(f""), alignment=Qt.AlignmentFlag.AlignTop)

        # Mission section
        temp_label = QLabel("Mission")
        temp_label.setFont(QFont("Arial", 15, QFont.Weight.Bold))
        temp_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        temp_layout.addWidget(temp_label)        

        # Mission description label
        temp_label = QLabel(f"This is where the mission for coug #{coug_number} will go.")
        temp_label.setWordWrap(True)
        temp_label.setFont(QFont("Arial", 13))
        temp_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Expanding)
        temp_layout.addSpacing(20)
        temp_layout.addWidget(temp_label)

        # Status widgets section
        temp_layout.addSpacing(20)
        temp_layout.addWidget(self.create_title_label(f"Coug {coug_number} status"), alignment=Qt.AlignmentFlag.AlignTop)
        temp_layout.addSpacing(20)
        temp_layout.addWidget(self.create_normal_label("x (meters): x", f"XPos{coug_number}"), alignment=Qt.AlignmentFlag.AlignVCenter)
        temp_layout.addSpacing(20)
        temp_layout.addWidget(self.create_normal_label("y (meters): y", f"YPos{coug_number}"), alignment=Qt.AlignmentFlag.AlignVCenter)
        temp_layout.addSpacing(20)
        temp_layout.addWidget(self.create_normal_label("Depth (meters): d", f"Depth{coug_number}"), alignment=Qt.AlignmentFlag.AlignVCenter)
        temp_layout.addSpacing(20)
        temp_layout.addWidget(self.create_normal_label("Heading (units?): h", f"Heading{coug_number}"), alignment=Qt.AlignmentFlag.AlignVCenter)
        temp_layout.addSpacing(20)
        temp_layout.addWidget(self.create_normal_label("Current Waypoint: w", f"Waypoint{coug_number}"), alignment=Qt.AlignmentFlag.AlignVCenter)
        temp_layout.addSpacing(20)
        temp_layout.addWidget(self.create_normal_label("DVL Velocity (m/s): w", f"DVL_vel{coug_number}"), alignment=Qt.AlignmentFlag.AlignVCenter)
        temp_layout.addSpacing(20)
        # Battery status from status message
        temp_layout.addWidget(self.create_normal_label("Battery %: b", f"Battery_sensors{coug_number}"), alignment=Qt.AlignmentFlag.AlignVCenter)

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
        setattr(self, name, text_label)
        text_label.setObjectName(name) 
        text_label.setFont(QFont("Arial", 13))
        text_label.setContentsMargins(0, 0, 0, 0)
        return text_label

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
        Updates the GUI console with a confirmation or failure message for all Cougs,
        depending on the value of the kill_message.
        
        Parameters:
            kill_message: The message object received from the 'confirm_e_kill' topic (std_msgs/Bool).
        """
        value = kill_message.data if hasattr(kill_message, 'data') else kill_message
        if value: 
            for i in self.selected_cougs:
                self.recieve_console_update("Kill Command Confirmed", i)
        else: 
            for i in self.selected_cougs:
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
        Updates the GUI console with a confirmation or failure message for all Cougs,
        depending on the value of the surf_message.
        
        Parameters:
            surf_message: The message object received from the 'confirm_e_surface' topic (std_msgs/Bool).
        """
        value = surf_message.data if hasattr(surf_message, 'data') else surf_message
        if value: 
            for i in self.selected_cougs:
                self.recieve_console_update("Surface Command Confirmed", i)
        else: 
            for i in self.selected_cougs:
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
        # connection status, list of bool, representing connections of Coug1, Coug2, etc
        # bool[] connections
        # time since last ping response, representing last responses in seconds of Coug1, Coug2, etc
        # uint8[] last_ping
        self.update_connections_signal.emit(conn_message)

    def _update_connections_gui(self, conn_message):
        """
        Updates the GUI to reflect the latest connection status and ping times for each Coug.

        Parameters:
            conn_message: The Connections message object containing connection_type, connections, and last_ping.
        """
        try:
            if conn_message.connection_type:
                feedback_key = "Radio_connections"
                feedback_key_seconds = "Radio_seconds"
                conn_type = 1
            else:
                feedback_key = "Modem_connections"
                feedback_key_seconds = "Modem_seconds"
                conn_type = 0

            # Update connection status icons for each Coug
            for coug_number, data in self.feedback_dict[feedback_key].items():
                if coug_number-1 < len(conn_message.connections):
                    status = 1 if conn_message.connections[coug_number-1] else 0
                    self.feedback_dict[feedback_key][coug_number] = status
                    prefix = feedback_key.split("_")[0]
                    new_label = self.create_icon_and_text(prefix, self.icons_dict[status], self.tab_spacing)
                    layout = self.general_page_coug_layouts.get(coug_number)
                    widget = self.general_page_coug_widgets.get(coug_number)
                    self.replace_label(f"{feedback_key}{coug_number}", layout, widget, new_label)
                    new_label2 = self.create_icon_and_text(prefix, self.icons_dict[status], 0)
                    layout = getattr(self, f"coug{coug_number}_column0_layout")
                    widget = getattr(self, f"coug{coug_number}_column0_widget")
                    self.replace_label(f"Spec_{feedback_key}{coug_number}", layout, widget, new_label2)
                else:
                    # Optionally, set to a default or log a warning
                    self.feedback_dict[feedback_key][coug_number] = 2  # waiting/unknown

            # Update seconds since last ping for each Cdef recieve_console_updateoug
            ping_list = list(conn_message.last_ping)
            for coug_number, ping in enumerate(ping_list, start=1):
                if coug_number not in self.selected_cougs: continue
                self.feedback_dict[feedback_key_seconds][coug_number] = ping
                layout = getattr(self, f"coug{coug_number}_buttons_column_layout", None)
                widget = getattr(self, f"coug{coug_number}_buttons_column_widget", None)
                new_seconds_label = self.create_seconds_label(conn_type, ping)
                if conn_type:
                    old_label = f"coug{coug_number}_radio_seconds_widget"
                else:
                    old_label = f"coug{coug_number}_modem_seconds_widget"
                self.replace_label(old_label, layout, widget, new_seconds_label)

        except Exception as e:
            print("Exception in update_connections_gui:", e)
            if coug_number in self.selected_cougs: self.recieve_console_update("Exception in update_connections_gui:", coug_number)
            print()
            
    def recieve_console_update(self, console_message, coug_number):
        """
        Slot to receive a console message and emit a signal to update the GUI.

        Parameters:
            console_message: The console message object.
        """
        self.update_console_signal.emit(console_message, coug_number)
    
    def _update_console_gui(self, console_message, coug_number):
        """
        Appends a new console message to the specific Coug's console log label.
        """
        try:
            label = self.findChild(QLabel, f"Console_messages{coug_number}")
            if label:
                current_text = label.text()
                if current_text:
                    updated_text = current_text + "\n" + console_message
                else:
                    updated_text = console_message
                label.setText(updated_text)
                # Scroll to the bottom of the scroll area
                scroll_area = getattr(self, f"coug{coug_number}_console_scroll_area", None)
                if scroll_area:
                    # Defer scrolling to after the event loop processes the label update
                    QTimer.singleShot(50, lambda: scroll_area.verticalScrollBar().setValue(scroll_area.verticalScrollBar().maximum()))
            else:
                print(f"Console log label not found for Coug {coug_number}")
                self.recieve_console_update(f"Console log label not found for Coug {coug_number}", coug_number)
        except Exception as e:
            print(f"Exception in _update_console_gui: {e}")
            if coug_number in self.selected_cougs: self.recieve_console_update(f"Exception in _update_console_gui: {e}", coug_number)

    def receive_status(self, stat_message):
        """
        Slot to receive a Status message and emit a signal to update the GUI.

        Parameters:
            stat_message: The Status message object.
        """
        self.update_status_signal.emit(stat_message)

    def _update_status_gui(self, stat_message):
        """
        Updates the GUI to reflect the latest status information for a specific Coug.

        Parameters:
            stat_message: The Status message object containing vehicle_id and status fields.
        """
        # message: Status
        #   uint8 vehicle_id
        #   uint8 x
        #   uint8 y
        #   uint8 depth
        #   uint8 heading
        #   uint8 waypoint
        #   uint8 dvl_vel
        #   uint8 battery_voltage
        #   bool dvl_running
        #   bool gps_connection
        #   bool leak_detection
        try:
            coug_number = stat_message.vehicle_id
            if coug_number not in self.selected_cougs: return
            # update the feedback dict with the new status using a loop
            status_keys = [
                ("XPos", "x"),
                ("YPos", "y"),
                ("Depth", "depth"),
                ("Heading", "heading"),
                ("Waypoint", "waypoint"),
                ("DVL_vel", "dvl_vel"),
                ("Battery_sensors", "battery_voltage"),
                ("DVL_sensors", "dvl_running"),
                ("GPS_sensors", "gps_connection"),
                ("Leak_sensors", "leak_detection"),
            ]
            for key, attr in status_keys:
                value = getattr(stat_message, attr)
                if isinstance(value, bool):
                    # Store 1 for True, 0 for False in feedback_dict for sensor status
                    self.feedback_dict[key][coug_number] = 1 if value else 0
                else:
                    self.feedback_dict[key][coug_number] = value

                # Update normal labels for numeric/status fields
                if key in ["XPos", "YPos", "Depth", "Heading", "Waypoint", "DVL_vel", "Battery_sensors"]:
                    new_label = self.create_normal_label(self.key_to_text_dict[key] + str(self.feedback_dict[key][coug_number]), f"{key}{coug_number}")
                    layout = getattr(self, f"coug{coug_number}_column01_layout")
                    widget = getattr(self, f"coug{coug_number}_column01_widget")
                    self.replace_label(f"{key}{coug_number}", layout, widget, new_label)
                # Update sensor icons for DVL, GPS, Leak
                elif key in ["DVL_sensors", "GPS_sensors", "Leak_sensors"]:
                    prefix = key.split("_")[0] if key != "Leak_sensors" else "Leak Detector"
                    status = self.feedback_dict[key][coug_number]
                    new_label = self.create_icon_and_text(prefix, self.icons_dict[status], 0)
                    layout = getattr(self, f"coug{coug_number}_column0_layout")
                    widget = getattr(self, f"coug{coug_number}_column0_widget")
                    self.replace_label(f"Spec_{key}{coug_number}", layout, widget, new_label)
                    if key != "Leak_sensors":
                        new_label2 = self.create_icon_and_text(prefix, self.icons_dict[status], self.tab_spacing)
                        layout = self.general_page_coug_layouts.get(coug_number)
                        widget = self.general_page_coug_widgets.get(coug_number)
                        self.replace_label(f"{key}{coug_number}", layout, widget, new_label2)

        except Exception as e:
            print("Exception in update_connections_gui:", e)
            if coug_number in self.selected_cougs: self.recieve_console_update(f"Exception in update_connections_gui: {e}", coug_number)

#used by ros to open a window. Needed in order to start PyQt on a different thread than ros
def OpenWindow(ros_node, borders=False):
    app = QApplication(sys.argv)
    window_width, window_height = 1200, 900

    # Prepare splash image
    img_path = os.path.join(os.path.dirname(__file__), "FRoSt_Lab.png")
    pixmap = QPixmap(img_path)
    if pixmap.isNull():
        print(f"Warning: Could not load splash image '{img_path}'. Using solid color instead.")
        pixmap = QPixmap(window_width, window_height)
        pixmap.fill(QColor("#5F9EA0"))
    else:
        background = QPixmap(window_width, window_height)
        background.fill(QColor("#5F9EA0"))
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
    options = [f"Coug {i}" for i in range(1, 5)]
    dlg = ConfigurationWindow(options, parent=splash)
    dlg.setWindowModality(Qt.WindowModality.ApplicationModal)
    dlg.setWindowFlag(Qt.WindowType.WindowStaysOnTopHint)
    dlg.move(
        x + (window_width - dlg.width()) // 2,
        y + (window_height - dlg.height()) // 2
    )

    selected_cougs = None
    if dlg.exec():
        #example selected_cougs: {'Coug 1': True, 'Coug 2': True, 'Coug 3': False, 'Coug 4': True}
        selected_cougs = dlg.get_states()
    else:
        sys.exit(0)

    # Raise the splash again in case it lost focus
    splash.raise_()
    splash.showMessage("Loading main window...")

    if borders:
        app.setStyleSheet("""*{border: 1px solid red;}""")

    result = {}

    def build_main_window():
        window = MainWindow(ros_node, selected_cougs)
        window.resize(window_width, window_height)
        window.move(x, y)
        result['window'] = window
        QTimer.singleShot(3000, lambda: (
            window.show(),
            window.activateWindow(),
            splash.close()
        ))

    QTimer.singleShot(500, build_main_window)
    return app, result

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

class AbortMissionsDialog(QDialog):
    """
    Custom dialog for confirming or aborting mission-related actions (e.g., shutdown, recall).
    Presents a message and Accept/Decline buttons.

    Parameters:
        window_title (str): The title of the dialog window.
        message_text (str): The message to display in the dialog.
        parent (QWidget, optional): The parent widget.
    """
    def __init__(self, window_title, message_text, parent=None):
        super().__init__(parent)

        self.setWindowTitle(window_title)

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

class ConfigurationWindow(QDialog):
    """
    Custom configuration dialog with multiple checkboxes.
    Returns the checked states as a dictionary if accepted.
    """
    def __init__(self, options, parent=None):
        """
        Parameters:
            options (list of str): List of checkbox labels.
        """
        super().__init__(parent)
        self.setWindowTitle("Configuration")
        self.checkboxes = {}
        layout = QVBoxLayout()

        # Make the dialog not resizable
        self.setFixedSize(300, 200)  # Set to your preferred width and height

        # Create a checkbox for each option
        for opt in options:
            cb = QCheckBox(opt)
            cb.setChecked(False)  # Default to unchecked
            self.checkboxes[opt] = cb
            layout.addWidget(cb)

        # OK/Cancel buttons
        buttonBox = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok)
        buttonBox.accepted.connect(self.validate_and_accept)
        layout.addWidget(buttonBox)
        self.setLayout(layout)
    
    def validate_and_accept(self):
        valid = False
        states = self.get_states()
        for coug_data, included in states.items():
            if included: 
                self.accept()
                return
        QMessageBox.warning(self, "Selection Required", "Please select at least one Coug before continuing.")

    def get_states(self):
        """
        Returns a dict of {option: bool} for each checkbox.
        """
        return {opt: cb.isChecked() for opt, cb in self.checkboxes.items()}
