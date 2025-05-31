import sys 
import random
from PyQt6.QtWidgets import (QScrollArea, QApplication, QMainWindow, 
    QWidget, QPushButton, QTabWidget, QVBoxLayout, QHBoxLayout, QLabel, QFrame,
    QSizePolicy, QSpacerItem, QGridLayout, QStyle, QWidget, QDialog, QDialogButtonBox
)
from PyQt6.QtCore import QSize, Qt, QTimer, pyqtSignal

from PyQt6.QtGui import QColor, QPalette, QFont, QPixmap, QKeySequence, QShortcut

from base_station_interfaces.srv import BeaconId

class MainWindow(QMainWindow):
    # Initializes GUI window with a ros node inside
    update_connections_signal = pyqtSignal(object)
    def __init__(self, ros_node):
        """
        Initializes GUI window with a ros node inside

        Parameters:
            ros_node (node): node passed in from ros in order to access the publisher
        """
        
        super().__init__()
        self.ros_node = ros_node
        self.setWindowTitle("CoUGARS_GUI")
        scale = 300  # dev usage for manually scaling the window
        self.resize(QSize(4 * scale, 3 * scale))
        print(f"height: {self.height()}, width: {self.width()}")

        ###This is how the coug info gets into the GUI
        self.feedback_dict = {
            #0->negative, 1->positive, 2->waiting
            #Cougs 1-3 connections
            "Wifi_connections": {1: 0, 2: 1, 3: 2},
            "Radio_connections": {1: 0, 2: 0, 3: 0},
            "Modem_connections": {1: 0, 2: 0, 3: 0},

            #0->negative, 1->positive, 2->waiting
            #Cougs 1-3 sensors
            "Modem_sensors": {1: 0, 2: 0, 3: 0},
            "DVL_sensors": {1: 0, 2: 0, 3: 0},
            "GPS_sensors": {1: 2, 2: 1, 3: 0},
            "IMU_sensors": {1: 0, 2: 0, 3: 0},
            "Leak_sensors": {1: 0, 2: 2, 3: 1},
            "Battery_sensors": {1: 0, 2: 0, 3: 0},

            #0->negative, 1->positive, 2->waiting
            #Cougs 1-3 nodes
            "Safety_Monitoring_nodes": {1: 0, 2: 0, 3: 0},
            "Depth_Controller_nodes": {1: 0, 2: 0, 3: 0},
            "Heading_Controller_nodes": {1: 0, 2: 0, 3: 0},
            "Factor_Graph_nodes": {1: 0, 2: 0, 3: 0},
            "Modem_Timing_nodes": {1: 0, 2: 0, 3: 0},

            #Cougs 1-3 status messages
            "Status_messages": {1: "", 2: "", 3: ""},

            #Cougs 1-3 last messages
            "Last_messages": {1: "", 2: "", 3: ""},

            #Cougs 1-3 message logs, lists of strings
            "Console_messages": {1: [], 2: [], 3: []},

            #Cougs 1-3 message logs, lists of strings
            "Missions": {1: "", 2: "", 3: ""},            
            
            #Cougs 1-3 message logs, lists of strings
            "Modems": {1: "", 2: "", 3: ""},
        }

        #This is a dictionary to map the feedback_dict to the correct symbols
        #"x" symbol -> SP_MessageBoxCritical
        #"check" symbol -> SP_DialogApplyButton
        # "waiting" symbol -> SP_TitleBarContextHelpButton (for now, probably will change)
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
        self.tab_dict = {
            "General": [None, QHBoxLayout()],
            "Coug 1": [None, QHBoxLayout()],
            "Coug 2": [None, QHBoxLayout()],
            "Coug 3": [None, QHBoxLayout()]
        }

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
                content_layout.addWidget(self.set_specific_coug_widgets(int(name[-1])))
                content_layout.addWidget(self.make_hline())
                content_layout.addWidget(self.create_specific_coug_console_log(int(name[-1])))
            else:
                content_layout.addWidget(content)
                self.set_general_page_widgets()

            # Set the combined layout
            content_widget.setLayout(content_layout)
            self.tab_dict[name][0] = content_widget

            # Add to tabs
            self.tabs.addTab(content_widget, name)
            self.set_background(content_widget, "cadetblue")

        #Emergency exit GUI button
        self.emergency_exit_gui_button = QPushButton("Close GUI")
        self.emergency_exit_gui_button.clicked.connect(self.close_window)
        self.emergency_exit_gui_button.setStyleSheet("background-color: red; color: black;")

        #Random data -> for dev till we have data
        self.random_data_button = QPushButton("Random Data Button")
        self.random_data_button.clicked.connect(self.set_random_data)

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
        #add the random dev button and the exit gui button
        self.main_layout.addWidget(self.random_data_button)
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

    #function to close the GUI window(s). Used by the keyboard interrupt signal or the exit button
    def close_window(self):
        print("closing the window now...")
        self.close()  

    #Dev to see all the widgets inside a certain layout, mostly to find their names for replacement
    def print_all_widgets_in_layout(self, layout, indent=0):
        spacer = "  " * indent
        for i in range(layout.count()):
            item = layout.itemAt(i)
            
            # If the item is a widget, print its name and class
            widget = item.widget()
            if widget:
                print(f"{spacer}Widget: {widget.objectName()} ({widget.__class__.__name__})")
            
            # If the item is a layout itself, recurse into it
            child_layout = item.layout()
            if child_layout:
                print(f"{spacer}Layout:")
                print_all_widgets_in_layout(child_layout, indent + 1)

    #uses the ros node that was passed in to publish text on a topic
    def publish_from_gui(self, text):
        if text and self.ros_node:
            self.ros_node.publish_text(text)

    #dev used for testing that the data can be dynamically changed inside of the GUI
    def set_random_data(self):
        print("setting random data...")
        value = random.randint(0,2)
        value1 = random.randint(0,2)
        self.random_global_value = value
        self.publish_from_gui(f"random number: {str(self.random_global_value)}")

        # self.print_all_widgets_in_layout(self.general_page_C1_layout)

        random_messages = ["running", "no connection", "waiting", "garbage data"]

        for key, value in self.feedback_dict.items():
            if key in ["Status_messages", "Last_messages"]:
                for coug_number, data in value.items():
                    random_int = random.randint(0,3)
                    self.feedback_dict[key][coug_number] = random_messages[random_int]
                    status_color, _ = self.get_status_message_color(random_messages[random_int])
                    new_label = QLabel(random_messages[random_int])
                    layout = getattr(self, f"general_page_C{coug_number}_layout")
                    widget = getattr(self, f"general_page_C{coug_number}_widget")
                    self.replace_label(f"{key}{coug_number}", layout, widget, new_label, color=status_color)
            else:
                for coug_number, data in value.items():
                    random_int = random.randint(0,2)
                    self.feedback_dict[key][coug_number] = random_int
                    prefix = key.split("_")[0]
                    new_label = self.create_icon_and_text(prefix, self.icons_dict[self.feedback_dict[key][coug_number]], self.tab_spacing)
                    layout = getattr(self, f"general_page_C{coug_number}_layout")
                    widget = getattr(self, f"general_page_C{coug_number}_widget")
                    self.replace_label(f"{key}{coug_number}", layout, widget, new_label)
                    # replace_widget_by_name(self, root_widget, widget_name, new_widget, style=""):

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

        temp_widget = parent_widget.findChild(QWidget, widget_name)
        if temp_widget:
            #the index of the widget in respect to its parent layout
            index = parent_layout.indexOf(temp_widget)
            print(f"Replacing widget: {widget_name} at index {index}. Parent layout: {parent_layout}. Parent widget: {parent_widget}")
        else:
            print("widget_name not found")
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

    def replace_widget_by_name(self, root_widget, widget_name, new_widget, style=""):
        """
        Replaces a widget in the UI tree by its object name, without knowing its layout or container.

        Parameters:
            root_widget (QWidget): The top-level widget (e.g., main window or central widget).
            widget_name (str): Object name of the widget to replace.
            new_widget (QWidget): The new widget to insert.
            style (str): Optional CSS styling for the new widget.
        """
        old_widget = root_widget.findChild(QWidget, widget_name)
        if not old_widget:
            print(f"[replace_widget_by_name] Widget '{widget_name}' not found.")
            return

        parent = old_widget.parent()
        while parent and not isinstance(parent.layout(), QLayout):
            parent = parent.parent()

        if not parent:
            print(f"[replace_widget_by_name] Could not find layout for widget '{widget_name}'.")
            return

        layout = parent.layout()
        index = layout.indexOf(old_widget)
        if index == -1:
            print(f"[replace_widget_by_name] Widget '{widget_name}' not found in parent layout.")
            return

        layout.removeWidget(old_widget)
        old_widget.setParent(None)

        new_widget.setObjectName(widget_name)
        if style:
            new_widget.setStyleSheet(style)

        layout.insertWidget(index, new_widget)

    def find_label(self, widget_name, parent_layout, parent_widget):
        """
        Used by replace label to find a specific label in a layout

        Parameters:
            widget_name: the name of the widget to be changed
            parent_layout: the parent layout of the widget to be changed
            parent_widget: the parent widget of the widget to be changed
        """

        temp_widget = parent_widget.findChild(QWidget, widget_name)
        if temp_widget:
            index = parent_layout.indexOf(temp_widget)
            if isinstance(temp_widget, QLabel):
                label_text = temp_widget.text()
                print("Label text:", label_text)
                return label_text
            else:
                print(f"{widget_name} is not a QLabel")
        else:
            print("widget_name not found")
            return None

    def get_status_message_color(self, message):
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
        width_px = self.width() // 4
        self.resizeTabs(width_px)
        super().resizeEvent(event)  # Call the base class method after doing custom resizing

    "/*resize the tabs according to the width of the window*/"
    def resizeTabs(self, width_px):
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

    def set_background(self, widget, color):
        palette = widget.palette()
        palette.setColor(widget.backgroundRole(), QColor(color))
        widget.setAutoFillBackground(True)
        widget.setPalette(palette)

    def load_coug1_missions_button(self):
        self.confirm_reject_label.setText("Loading Coug 1 missiony...")
        self.publish_from_gui("Loading the missions...")

    def load_missions_button(self):
        #add load missions logic
        self.confirm_reject_label.setText("Loading the missions...")
        self.publish_from_gui("Loading the missions...")
        # print("Loading the missions...")

    def start_missions_button(self):
        #add start missions logic
        self.confirm_reject_label.setText("Starting the missions...")
        self.publish_from_gui("Starting the missions...")
        # print("Starting the missions...")

    def emergency_shutdown_button(self, coug_number):
        # uint8 beacon_id
        # ---
        # bool success
        message = BeaconId.Request()
        message.beacon_id = coug_number
        # print(self.ros_node.cli.call_async(message))
        return self.ros_node.cli.call_async(message)
    
    def emergency_surface_button(self, coug_number):
        # uint8 beacon_id
        # ---
        # bool success
        message = BeaconId.Request()
        message.beacon_id = coug_number
        # print(self.ros_node.cli.call_async(message))
        return self.ros_node.cli2.call_async(message)

    def recallCougs(self):
        #add recall cougs logic
        self.confirm_reject_label.setText("Recalling the Cougs...")
        self.publish_from_gui("Recalling the Cougs...")
        # print("Recalling the Cougs...")
    
    def AbortAllMissions(self):
        #add abort all missions logic
        # print("click", s)

        dlg = AbortMissionsDialog(self)
        if dlg.exec():
            self.confirm_reject_label.setText("Aborting all missions...")
            self.publish_from_gui("Aborting all missions...")
        else:
            self.confirm_reject_label.setText("Canceling abort missions command...")
            # self.publish_from_gui("Canceling abort missions command...")
        # print("You pressed the abort all missions button")
    
    #template to make a vertical line
    def make_vline(self):
        Vline = QFrame()
        Vline.setFrameShape(QFrame.Shape.VLine)
        Vline.setFrameShadow(QFrame.Shadow.Sunken)
        return Vline

    #template to make a horizontal line
    def make_hline(self):
        Hline = QFrame()
        Hline.setFrameShape(QFrame.Shape.HLine)
        Hline.setFrameShadow(QFrame.Shadow.Sunken)
        return Hline

    #used to set all of the widgets on the general page
    def set_general_page_widgets(self):
        self.general_page_layout = self.tab_dict["General"][1]

        # Create a container widget and a layout for it
        self.general_page_C0_widget = QWidget()
        self.general_page_C0_layout = QVBoxLayout()
        self.general_page_C0_widget.setLayout(self.general_page_C0_layout)
        
        self.general_page_C1_widget = QWidget()
        self.general_page_C1_layout = QVBoxLayout()
        self.general_page_C1_widget.setLayout(self.general_page_C1_layout)

        self.general_page_C2_widget = QWidget()
        self.general_page_C2_layout = QVBoxLayout()
        self.general_page_C2_widget.setLayout(self.general_page_C2_layout)

        self.general_page_C3_widget = QWidget()
        self.general_page_C3_layout = QVBoxLayout()
        self.general_page_C3_widget.setLayout(self.general_page_C3_layout)

        # Add the container widget to the main layout
        self.general_page_layout.addWidget(self.general_page_C0_widget)
        self.general_page_layout.addWidget(self.make_vline())
        self.general_page_layout.addWidget(self.general_page_C1_widget)
        self.general_page_layout.addWidget(self.make_vline())
        self.general_page_layout.addWidget(self.general_page_C2_widget)
        self.general_page_layout.addWidget(self.make_vline())
        self.general_page_layout.addWidget(self.general_page_C3_widget)

        self.set_general_page_C0_widgets()

        self.set_general_page_column_widgets(self.general_page_C1_layout, 1)
        self.set_general_page_column_widgets(self.general_page_C2_layout, 2)
        self.set_general_page_column_widgets(self.general_page_C3_layout, 3)

    #set the widgets of the first column on the general page
    def set_general_page_C0_widgets(self):
        # Create buttons
        self.Load_missions_button = QPushButton("Load Missions")
        self.Load_missions_button.clicked.connect(self.load_missions_button)
        self.Load_missions_button.setStyleSheet("background-color: blue; color: black;")

        self.Start_missions_button = QPushButton("Start Missions")
        self.Start_missions_button.clicked.connect(self.start_missions_button)
        self.Start_missions_button.setStyleSheet("background-color: blue; color: black;")

        self.Recall_cougs_button = QPushButton("Recall Cougs")
        self.Recall_cougs_button.clicked.connect(self.recallCougs)
        self.Recall_cougs_button.setStyleSheet("background-color: blue; color: black;")

        self.abort_all_missions = QPushButton("Abort All Missions")
        self.abort_all_missions.clicked.connect(self.AbortAllMissions)
        self.abort_all_missions.setStyleSheet("background-color: red; color: black;")

        # Create and style the label
        general_label = QLabel("General Options:")
        general_label.setFont(QFont("Arial", 17, QFont.Weight.Bold))
        general_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)

        # Add widgets to the layout
        self.general_page_C0_layout.addWidget(general_label, alignment=Qt.AlignmentFlag.AlignTop)
        self.general_page_C0_layout.addSpacing(100)
        self.general_page_C0_layout.addWidget(self.Load_missions_button, alignment=Qt.AlignmentFlag.AlignTop)
        self.general_page_C0_layout.addSpacing(100)

        # Add remaining buttons
        self.general_page_C0_layout.addWidget(self.Start_missions_button)
        self.general_page_C0_layout.addSpacing(100)

        self.general_page_C0_layout.addWidget(self.Recall_cougs_button)
        self.general_page_C0_layout.addSpacing(100)

        # Add spacer to push the rest of the buttons down
        spacer = QSpacerItem(0, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)
        self.general_page_C0_layout.addItem(spacer)

        self.general_page_C0_layout.addWidget(self.abort_all_missions)
            
    #template to set the rest of widgets on the rest of the columns on the general page
    def set_general_page_column_widgets(self, layout, coug_number):
        # Create and style the header label
        title_label = QLabel(f"Coug {coug_number}:")
        title_label.setFont(QFont("Arial", 17, QFont.Weight.Bold))
        title_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        layout.addWidget(title_label, alignment=Qt.AlignmentFlag.AlignTop)
        layout.addSpacing(20)

        #"x" symbol -> SP_MessageBoxCritical
        #"check" symbol -> SP_DialogApplyButton
        # "waiting" symbol -> SP_TitleBarContextHelpButton (for now, probably will change)

        #section labels
        section_titles = ["Connections", "Sensors", "Status", "Last Message"]
        for title in section_titles:
            layout.addWidget(QLabel(title, font=QFont("Arial", 15)), alignment=Qt.AlignmentFlag.AlignTop)
            layout.addSpacing(20)
            self.tab_spacing = 75

            match title:
                case "Connections": 
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

                case "Sensors":
                    modem_sensor_widget = self.create_icon_and_text("Modem", self.icons_dict[self.feedback_dict["Modem_sensors"][coug_number]], self.tab_spacing)
                    modem_sensor_widget.setObjectName(f"Modem_sensors{coug_number}")
                    layout.addWidget(modem_sensor_widget)
                    layout.addSpacing(20)

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

                case "Status":
                    status = self.feedback_dict["Status_messages"][coug_number]

                    status_color, status = self.get_status_message_color(status)

                    label = QLabel(f"{status}", font=QFont("Arial", 13))
                    label.setObjectName(f"Status_messages{coug_number}")
                    label.setStyleSheet(f"color: {status_color};")  # Change 'red' to any color you want (name, hex, rgb)
                    # label.setIndent(self.tab_spacing)
                    layout.addWidget(label, alignment=Qt.AlignmentFlag.AlignTop)
                    layout.addSpacing(40)

                case "Last Message":
                    last_message = self.feedback_dict["Status_messages"][coug_number]
                    if last_message: 
                        last_mesage_label = QLabel(self.feedback_dict["Last_messages"][coug_number], font=QFont("Arial", 13), alignment=Qt.AlignmentFlag.AlignTop)
                    else:
                        last_mesage_label = QLabel("No messages have been recieved", font=QFont("Arial", 13), alignment=Qt.AlignmentFlag.AlignTop)
                        last_mesage_label.setStyleSheet(f"color: orange;")

                    last_mesage_label.setObjectName(f"Last_messages{coug_number}")
                    
                    # last_mesage_label.setIndent(self.tab_spacing)
                    layout.addWidget(last_mesage_label)
                    layout.addSpacing(40)

        # Add spacer to push content up
        spacer = QSpacerItem(0, 0, QSizePolicy .Policy.Minimum, QSizePolicy.Policy.Expanding)
        layout.addItem(spacer)

    def set_specific_coug_widgets(self, coug_number):

        temp_container = QWidget()
        temp_layout = QHBoxLayout(temp_container)
        # temp_layout.setAlignment(Qt.AlignmentFlag.AlignTop)

        temp_layout.setSpacing(0)
        temp_layout.setContentsMargins(0, 0, 0, 0)

        #col1 - accessing feedback dictionary 
        temp_layout.addWidget(self.create_specific_coug_column0(coug_number), alignment=Qt.AlignmentFlag.AlignTop)
        temp_layout.addWidget(self.create_specific_coug_column01(coug_number), alignment=Qt.AlignmentFlag.AlignTop)

        #vline
        temp_layout.addWidget(self.make_vline())
        #col2 - buttons
        temp_layout.addWidget(self.create_coug_buttons_column(coug_number))
        return temp_container

    def create_specific_coug_console_log(self, coug_number): 
        temp_container = QWidget()
        temp_layout = QVBoxLayout(temp_container)

        # First text label (bold title)
        title_text = "Console information/message log"
        title_label = QLabel(title_text)
        title_label.setWordWrap(True)
        title_label.setFont(QFont("Arial", 15, QFont.Weight.Bold))
        title_label.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        temp_layout.addWidget(title_label)

        # Second text label (wrapped long message)
        message_text = (
            "Welcome to the advanced testing log for your GUI layout system. This message is designed to span a significant "
            "number of lines to ensure that your scrollable area handles content overflow gracefully. Each sentence contributes "
            "to building a realistic use case where a developer or end user would need to read through a large volume of text.\n\n"

            "In the world of robotics and embedded systems, message logs often include timestamps, sensor readings, diagnostic "
            "information, and user feedback. These details can accumulate quickly, especially when a system is under load or when "
            "multiple subsystems are reporting simultaneously. Properly managing how that data is displayed is essential.\n\n"

            "Let’s consider a scenario where your GUI is being used to monitor a fleet of underwater robots. Each unit might "
            "send frequent status reports including depth, heading, battery level, leak detection status, sonar returns, and "
            "environmental sensor data such as temperature, salinity, and pressure. If a fault is detected, additional logs are "
            "generated to help with troubleshooting.\n\n"

            "This message simulates such an output. We begin with a system check: All modules initialized successfully. "
            "IMU calibration completed. GPS signal locked. Radio module online. WiFi handshake successful.\n\n"

            "Next, we observe active telemetry: Heading 253.7 degrees. Depth 14.6 meters. Internal temperature: 35.4°C. "
            "Battery at 83%. Leak sensor nominal. DVL reports bottom lock achieved. Camera feed initialized.\n\n"

            "From here, diagnostics continue: Propulsion test passed. Manipulator test failed: motor controller not responding. "
            "Fallback routine engaged. Logging full diagnostic traceback for future analysis.\n\n"

            "Users should be aware that any error or warning must be scannable and clearly distinguishable from regular output. "
            "This can be done through font weight, color, or formatting styles such as indentation and bulleting. For example:\n"
            "- ERROR: Manipulator arm failed to extend\n"
            "- WARNING: Depth sensor value out of expected range\n"
            "- INFO: Waiting for surface command acknowledgment\n\n"

            "In a real interface, the user may copy this text for later use or export logs to a file. Therefore, the interface "
            "should maintain formatting consistency and avoid cutting off words or lines unexpectedly.\n\n"

            "Scrolling must be vertical only. Horizontal scrolling makes reading long logs a chore and degrades user experience. "
            "Wrapping should occur at word boundaries to enhance legibility. Margins and line spacing should be adequate but not excessive.\n\n"

            "This is line 30. Still scrolling? Good. If your vertical scrollbar is behaving, you’re seeing this message exactly as intended.\n\n"

            "Let’s push to 40 lines: Data stream stable. Reconnection attempts: 0. Command acknowledgment rate: 99.7%. "
            "Uplink stable. Downlink stable. Operator status: Connected. Console heartbeat confirmed.\n\n"

            "Final diagnostics: All systems nominal. UI test passed. Scrollbar visible. Text wrapping complete. Layout responsive. "
            "Mission control ready for deployment.\n\n"

            "End of long message."
        )

        message_label = QLabel(message_text)
        message_label.setWordWrap(True)
        message_label.setFont(QFont("Arial", 15))
        message_label.setAlignment(Qt.AlignmentFlag.AlignHCenter)
        message_label.setContentsMargins(0, 0, 0, 0)

        message_label.setFixedWidth(self.width() - 20)
        message_label.setObjectName(f"Console_messages{coug_number}")

        scroll_content = QWidget()
        scroll_layout = QVBoxLayout(scroll_content)
        scroll_layout.setContentsMargins(0, 0, 0, 0)
        scroll_layout.addWidget(message_label)

        # Create QScrollArea and set scroll content
        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_area.setWidget(scroll_content)
        scroll_area.setFixedHeight(int(self.height() * (1/5)))

        # Add scroll_area to your layout
        temp_layout.addWidget(scroll_area)

        # temp_layout.addWidget(self.make_hline())
        # temp_layout.addSpacing(10)
        temp_layout.addWidget(scroll_area)

        spacer = QSpacerItem(0, 0, QSizePolicy.Policy.Minimum, QSizePolicy.Policy.Expanding)
        temp_layout.addItem(spacer)

        # Add the temp container to the coug layout by returning it
        return temp_container

    #used to create an icon next to text in a pre-determined fashion
    def create_icon_and_text(self, text, icon=None, temp_tab_spacing=None):
        temp_container = QWidget()
        temp_layout = QHBoxLayout(temp_container)
        if temp_tab_spacing: 
            temp_layout.setContentsMargins(temp_tab_spacing, 0, 0, 0)
        temp_layout.setSpacing(20)
        if icon:
            icon_label = QLabel()
            icon_label.setPixmap(self.style().standardIcon(icon).pixmap(16, 16))
            icon_label.setContentsMargins(0, 0, 0, 0)
            icon_label.setFixedSize(16, 16)
            temp_layout.addWidget(icon_label, alignment=Qt.AlignmentFlag.AlignVCenter)
        text_label = QLabel(text)
        text_label.setFont(QFont("Arial", 13))
        text_label.setContentsMargins(0, 0, 0, 0)
        temp_layout.addWidget(text_label, alignment=Qt.AlignmentFlag.AlignVCenter)
        return temp_container

    def create_title_label(self, text, set_width=True):
        # Create and style the label
        temp_label = QLabel(text)
        if set_width: temp_label.setFixedWidth(300)
        temp_label.setFont(QFont("Arial", 17, QFont.Weight.Bold))
        temp_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        return temp_label

    def create_coug_buttons_column(self, coug_number):
        #temp container for the first column of buttons
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

        temp_V_container = QWidget()
        temp_V_layout = QVBoxLayout(temp_V_container)

        #load mission (blue)
        self.create_coug_button(coug_number, "load_mission", "Load Mission", "blue", self.load_missions_button)
        #start mission (blue)
        self.create_coug_button(coug_number, "start_mission", "Start Mission", "blue", self.start_missions_button)
        #disarm thruster (yellow)
        self.create_coug_button(coug_number, "disarm_thruster", "Disarm Thruster", "yellow", self.start_missions_button)
        #deactivate DVL (yellow)
        self.create_coug_button(coug_number, "deactivate_DVL", "Deactivate DVl", "yellow", self.start_missions_button)
        #toggle running lights (yellow)
        self.create_coug_button(coug_number, "toggle_lights", "Toggle Running Lights", "yellow", self.start_missions_button)
        #restart R-Pi (yellow)
        self.create_coug_button(coug_number, "restart_RP", "Restart R-Pi", "yellow", self.start_missions_button)
        #return to base (blue)
        self.create_coug_button(coug_number, "return_to_base", "Return to Base", "blue", self.start_missions_button)
        #system reboot (red)
        self.create_coug_button(coug_number, "emergency_surface", "Emergency Surface", "red", lambda: self.emergency_surface_button(coug_number))
        #abort mission (red)
        self.create_coug_button(coug_number, "abort_misson", "Abort Mission", "red", self.start_missions_button)
        #emergency shutdown (red)
        self.create_coug_button(coug_number, "emergency_shutdown", "Emergency Shutdown", "red", lambda: self.emergency_shutdown_button(coug_number))

        temp_spacing = 50
        #add all the buttons to the layout
        temp_layout1.addWidget(getattr(self, f"load_mission_coug{coug_number}_button"))
        temp_layout1.addSpacing(temp_spacing)
        temp_layout1.addWidget(getattr(self, f"start_mission_coug{coug_number}_button"))
        temp_layout1.addSpacing(temp_spacing)
        temp_layout1.addWidget(getattr(self, f"return_to_base_coug{coug_number}_button"))
        temp_layout1.addSpacing(temp_spacing)
        temp_layout1.addWidget(getattr(self, f"disarm_thruster_coug{coug_number}_button"))
        temp_layout1.addSpacing(temp_spacing)
        temp_layout1.addWidget(getattr(self, f"deactivate_DVL_coug{coug_number}_button"))

        temp_layout2.addWidget(getattr(self, f"toggle_lights_coug{coug_number}_button"))
        temp_layout2.addSpacing(temp_spacing)
        temp_layout2.addWidget(getattr(self, f"restart_RP_coug{coug_number}_button"))
        temp_layout2.addSpacing(temp_spacing)
        temp_layout2.addWidget(getattr(self, f"emergency_surface_coug{coug_number}_button"))
        temp_layout2.addSpacing(temp_spacing)
        temp_layout2.addWidget(getattr(self, f"abort_misson_coug{coug_number}_button"))
        temp_layout2.addSpacing(temp_spacing)
        temp_layout2.addWidget(getattr(self, f"emergency_shutdown_coug{coug_number}_button"))

        temp_layout.addWidget(temp_sub_container1)
        temp_layout.addWidget(temp_sub_container2)

        temp_V_layout.addWidget(self.create_title_label("Seconds since last connected", set_width=False))
        self.create_label(temp_V_layout, "Radio: xxx")
        self.create_label(temp_V_layout, "Accoustics: xxx")
        temp_V_layout.addWidget(self.make_hline())
        temp_V_layout.addWidget(temp_container)
        return temp_V_container

    def create_label(self, temp_layout, text):
        text_label = QLabel(text)
        text_label.setFont(QFont("Arial", 13))
        text_label.setContentsMargins(0, 0, 0, 0)
        temp_layout.addWidget(text_label, alignment=Qt.AlignmentFlag.AlignVCenter)

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
        button.setFixedWidth(300)  # or adjust the value as needed
        button.clicked.connect(callback)
        button.setStyleSheet(f"background-color: {color}; color: black")
        attr_name = f"{name}_coug{coug_number}_button"
        setattr(self, attr_name, button)

    def create_specific_coug_column0(self, coug_number):
        temp_layout = QVBoxLayout()
        temp_layout.setContentsMargins(0, 0, 0, 0)
        temp_layout.setSpacing(0) 

        # frame = QFrame()
        # frame.setFrameShape(QFrame.Shape.Box)
        # frame.setLayout(temp_layout)      

        temp_container = QWidget()
        temp_container.setMaximumWidth(180)
        container_layout = QVBoxLayout(temp_container)
        # container_layout.addWidget(frame)
        container_layout.addLayout(temp_layout)

        temp_layout.addWidget(self.create_title_label(f"Coug {coug_number}"), alignment=Qt.AlignmentFlag.AlignTop)
        
        temp_label = QLabel("Connections")
        temp_label.setFont(QFont("Arial", 15, QFont.Weight.Bold))
        temp_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        temp_layout.addSpacing(20)
        temp_layout.addWidget(temp_label)

        wifi_widget = self.create_icon_and_text("Wifi", self.icons_dict[self.feedback_dict["Wifi_connections"][coug_number]], 0)
        wifi_widget.setObjectName(f"Wifi_connections{coug_number}")
        temp_layout.addWidget(wifi_widget)

        radio_widget = self.create_icon_and_text("Radio", self.icons_dict[self.feedback_dict["Radio_connections"][coug_number]], 0)
        radio_widget.setObjectName(f"Radio_connections{coug_number}")
        temp_layout.addWidget(radio_widget)

        modem_widget = self.create_icon_and_text("Modem", self.icons_dict[self.feedback_dict["Modem_connections"][coug_number]], 0)
        modem_widget.setObjectName(f"Modem_connections{coug_number}")
        temp_layout.addWidget(modem_widget)
        temp_layout.addSpacing(20)

        temp_label = QLabel("Sensors")
        temp_label.setFont(QFont("Arial", 15, QFont.Weight.Bold))
        temp_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        temp_layout.addSpacing(20)
        temp_layout.addWidget(temp_label)

        modem_sensor_widget = self.create_icon_and_text("Modem", self.icons_dict[self.feedback_dict["Modem_sensors"][coug_number]], 0)
        modem_sensor_widget.setObjectName(f"Modem_sensors{coug_number}")
        temp_layout.addWidget(modem_sensor_widget)

        DVL_sensor_widget = self.create_icon_and_text("DVL", self.icons_dict[self.feedback_dict["DVL_sensors"][coug_number]], 0)
        DVL_sensor_widget.setObjectName(f"DVL_sensors{coug_number}")
        temp_layout.addWidget(DVL_sensor_widget)

        GPS_sensor_widget = self.create_icon_and_text("GPS", self.icons_dict[self.feedback_dict["GPS_sensors"][coug_number]], 0)
        GPS_sensor_widget.setObjectName(f"GPS_sensors{coug_number}")
        temp_layout.addWidget(GPS_sensor_widget)
        
        IMU_sensor_widget = self.create_icon_and_text("IMU", self.icons_dict[self.feedback_dict["IMU_sensors"][coug_number]], 0)
        IMU_sensor_widget.setObjectName(f"IMU_sensors{coug_number}")
        temp_layout.addWidget(IMU_sensor_widget)

        Leak_sensor_widget = self.create_icon_and_text("Leak Detector", self.icons_dict[self.feedback_dict["Leak_sensors"][coug_number]], 0)
        Leak_sensor_widget.setObjectName(f"Leak_sensors{coug_number}")
        temp_layout.addWidget(Leak_sensor_widget)

        Battery_sensor_widget = self.create_icon_and_text("Battery", self.icons_dict[self.feedback_dict["Battery_sensors"][coug_number]], 0)
        Battery_sensor_widget.setObjectName(f"Battery_sensors{coug_number}")
        temp_layout.addWidget(Battery_sensor_widget)

        return temp_container

    #create the second sub-column in the first column of the specific cougar pages (starts with "Nodes")
    def create_specific_coug_column01(self, coug_number):
        temp_layout = QVBoxLayout()
        temp_layout.setContentsMargins(0, 0, 0, 0)
        temp_layout.setSpacing(0) 

        # frame = QFrame()
        # frame.setFrameShape(QFrame.Shape.Box)
        # frame.setLayout(temp_layout)    

        temp_container = QWidget()
        temp_container.setMaximumWidth(220)
        container_layout = QVBoxLayout(temp_container)
        # container_layout.addWidget(frame)
        container_layout.addLayout(temp_layout)

        temp_layout.addWidget(self.create_title_label(f""), alignment=Qt.AlignmentFlag.AlignTop)
        
        temp_label = QLabel("Nodes")
        temp_label.setFont(QFont("Arial", 15, QFont.Weight.Bold))
        temp_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        temp_layout.addSpacing(20)
        temp_layout.addWidget(temp_label)

        Safety_Monitoring_widget = self.create_icon_and_text("Safety Monitoring", self.icons_dict[self.feedback_dict["Safety_Monitoring_nodes"][coug_number]], 0)
        Safety_Monitoring_widget.setObjectName(f"Safety_Monitoring_nodes{coug_number}")
        temp_layout.addWidget(Safety_Monitoring_widget)

        Depth_Controller_widget = self.create_icon_and_text("Depth Controller", self.icons_dict[self.feedback_dict["Depth_Controller_nodes"][coug_number]], 0)
        Depth_Controller_widget.setObjectName(f"Depth_Controller_nodes{coug_number}")
        temp_layout.addWidget(Depth_Controller_widget)
        
        Heading_Controller_widget = self.create_icon_and_text("Heading Controller", self.icons_dict[self.feedback_dict["Heading_Controller_nodes"][coug_number]], 0)
        Heading_Controller_widget.setObjectName(f"Heading_Controller_nodes{coug_number}")
        temp_layout.addWidget(Heading_Controller_widget)

        Factor_Graph_widget = self.create_icon_and_text("Factor Graph", self.icons_dict[self.feedback_dict["Factor_Graph_nodes"][coug_number]], 0)
        Factor_Graph_widget.setObjectName(f"Factor_Graph_nodes{coug_number}")
        temp_layout.addWidget(Factor_Graph_widget)

        Modem_Timing_widget = self.create_icon_and_text("Modem Timing", self.icons_dict[self.feedback_dict["Modem_Timing_nodes"][coug_number]], 0)
        Modem_Timing_widget.setObjectName(f"Modem_Timing_nodes{coug_number}")
        temp_layout.addWidget(Modem_Timing_widget)

        temp_label = QLabel("Mission")

        temp_label.setFont(QFont("Arial", 15, QFont.Weight.Bold))
        temp_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        temp_layout.addSpacing(20)
        temp_layout.addWidget(temp_label)        
        
        temp_label = QLabel(f"This is where the mission for coug #{coug_number} will go.")
        # temp_label = QLabel(f"")
        temp_label.setWordWrap(True)
        temp_label.setFont(QFont("Arial", 13))
        temp_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Fixed)
        temp_layout.addSpacing(20)
        temp_layout.addWidget(temp_label)

        temp_label = QLabel("Modem")
        temp_label.setFont(QFont("Arial", 15, QFont.Weight.Bold))
        temp_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        temp_layout.addSpacing(20)
        temp_layout.addWidget(temp_label)
   
        temp_label = QLabel(f"This is where the messages from the modem for coug #{coug_number} will go.")
        # temp_label = QLabel(f"")
        temp_label.setWordWrap(True)
        temp_label.setFont(QFont("Arial", 13))
        temp_label.setSizePolicy(QSizePolicy.Policy.Preferred, QSizePolicy.Policy.Preferred)
        temp_label.setMinimumHeight(temp_label.sizeHint().height())
        temp_layout.addSpacing(20)
        temp_layout.addWidget(temp_label)

        return temp_container

    #currently used as a proof of concept of receiving subscriptions
    def recieve_message(self, message): 
        self.confirm_reject_label.setText(message.data)

    #message: Connections
        #std_msgs/Header header

        # 0 for acoustic modem, 1 for radio
        # uint8 connection_type

        # connection status, list of bool, representing connections of Coug1, Coug2, etc
        # bool[] connections

        # time since last ping response, representing last responses in seconds of Coug1, Coug2, etc
        # uint8[] last_ping
    def recieve_connections(self, conn_message):
        self.update_connections_signal.emit(conn_message)

    def _update_connections_gui(self, conn_message):
        try:
            if conn_message.connection_type:
                feedback_key = "Radio_connections"
            else:
                feedback_key = "Modem_connections"

            for coug_number, data in self.feedback_dict[feedback_key].items():
                status = 1 if conn_message.connections[coug_number-1] else 0
                self.feedback_dict[feedback_key][coug_number] = status
                prefix = feedback_key.split("_")[0]
                new_label = self.create_icon_and_text(prefix, self.icons_dict[status], self.tab_spacing)
                layout = getattr(self, f"general_page_C{coug_number}_layout")
                widget = getattr(self, f"general_page_C{coug_number}_widget")
                self.replace_label(f"{feedback_key}{coug_number}", layout, widget, new_label)
        except Exception as e:
            print("Exception in update_gui:", e)
            
#used by ros to open a window. Needed in order to start PyQt on a different thread than ros
def OpenWindow(ros_node, borders=False):
    app = QApplication(sys.argv)
    if borders: app.setStyleSheet("""*{border: 1px solid red;}""")
    window = MainWindow(ros_node)
    window.show()
    return app, window  # Return both

class AbortMissionsDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)

        self.setWindowTitle("Abort Missions?")

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
        message = QLabel("Are you sure that you want to abort all missions?")
        layout.addWidget(message)
        layout.addWidget(self.buttonBox)
        self.setLayout(layout)