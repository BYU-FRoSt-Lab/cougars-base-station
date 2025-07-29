import sys
import json
from PyQt6.QtWidgets import (
    QDialog, QApplication, QVBoxLayout, QRadioButton, QLineEdit, QSpinBox, QCheckBox, QTextEdit, QHBoxLayout, QLabel, QMainWindow, QWidget, QPushButton
)

import subprocess
from PyQt6.QtCore import Qt

class PostProcessingWindow(QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Post Processing")
        self.resize(800, 600)  # Set a larger initial window size

        # QWidget wrapper is needed in QMainWindow
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Layout and label
        layout = QVBoxLayout()
        label = QLabel("Welcome to the Post Processing window!")
        layout.addWidget(label, alignment=Qt.AlignmentFlag.AlignHCenter)

        # Buttons
        self.plot_btn = QPushButton("Plot Data")
        self.plot_btn.clicked.connect(self.show_plot_config)

        self.view_btn = QPushButton("View Plots")
        layout.addWidget(self.plot_btn)
        layout.addWidget(self.view_btn)

        central_widget.setLayout(layout)

        # Apply naval blue theme
        self.setStyleSheet("""
            QMainWindow, QWidget {
                background-color: #001f3f;
            }
            QLabel {
                color: #e0eaf3;
                font-size: 17px;
            }
            QPushButton {
                background-color: #005792;
                color: #fff;
                font-size: 15px;
                border-radius: 5px;
                padding: 8px 16px;
            }
            QPushButton:hover {
                background-color: #0074d9;
            }
        """)

    def center_on_screen(self):
        screen = QApplication.primaryScreen().availableGeometry()
        size = self.geometry()
        x = (screen.width() - size.width()) // 2
        y = (screen.height() - size.height()) // 2
        self.move(x, y)
    
    def show_plot_config(self):
        dialog = PlotConfigDialog(self)
        if dialog.exec():
            cfg = dialog.get_config()
            sys_args = get_sys_args_from_config(cfg)
            # print("Sys Args:", sys_args)
            cmd = ['python3', '/home/frostlab/base_station/postprocessing/postmissionprocessor.py'] + sys_args

            try:
                # Run the script and print its output/errors in the console
                result = subprocess.run(cmd, capture_output=True, text=True)
                print("Script output:", result.stdout)
                print("Script errors:", result.stderr)
            except Exception as e:
                print(f"Failed to run script: {e}")
            # You can now use sys_args to call your plotting script!


def get_sys_args_from_config(cfg):
    # Defaults: Assume False unless plot type or config is present
    plot_types = {"dead_reckoning": False, "static_beacon": False, "gps_pings": False}
    plot_covariance = False
    run_live = False
    plot_separate = False
    modem_positions = []
    central_modem = 0

    for plot in cfg.get("plots", []):
        if plot["type"] == "dead_reckoning":
            plot_types["dead_reckoning"] = True
            plot_covariance = plot.get("plot_covariance", False)
        if plot["type"] == "static_beacon":
            plot_types["static_beacon"] = True
            run_live = plot.get("run_live", False)
            plot_separate = plot.get("plot_separate", False)
            modem_positions = plot.get("modem_positions", [])
            central_modem = plot.get("central_modem", 0)
        if plot["type"] == "gps_pings":
            plot_types["gps_pings"] = True

    # Arguments in specified order:
    sys_args = [
        str(plot_types["dead_reckoning"]),
        str(plot_types["static_beacon"]),
        str(plot_types["gps_pings"]),
        str(plot_covariance),
        str(run_live),
        str(plot_separate),
        json.dumps(modem_positions),  # Proper JSON string for modem positions
        str(central_modem)
    ]
    return sys_args


class PlotConfigDialog(QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Plot Data Options")
        self.setMinimumSize(800, 600)
        self.setStyleSheet("""
    QDialog, QWidget {
        background-color: #001f3f;
    }
    QLabel, QRadioButton, QCheckBox {
        color: #e0eaf3;
        font-size: 15px;
    }
    QLineEdit, QTextEdit {
        background-color: #003366;
        color: #e0eaf3;
        border: 1px solid #0074d9;
    }
    QSpinBox {
        background-color: #003366;
        color: #e0eaf3;
        border: 1px solid #0074d9;
    }
    QPushButton {
        background-color: #005792;
        color: #fff;
        font-size: 15px;
        border-radius: 5px;
        padding: 8px 16px;
    }
    QPushButton:hover {
        background-color: #0074d9;
    }
""")

        layout = QVBoxLayout(self)

        # Bagpath input
        layout.addWidget(QLabel("Bagpath:"))
        self.bagpath_input = QLineEdit()
        layout.addWidget(self.bagpath_input)

        # Plot type selection
        self.cb_dead_reckoning = QCheckBox("Plot Dead Reckoning")
        self.cb_static_beacon = QCheckBox("Plot Static Beacon")
        self.cb_gps_pings = QCheckBox("Plot GPS Pings")
        self.cb_dead_reckoning.setChecked(True)
        layout.addWidget(self.cb_dead_reckoning)
        layout.addWidget(self.cb_static_beacon)
        layout.addWidget(self.cb_gps_pings)

        # Dead reckoning configs
        self.dr_config = QWidget()
        dr_layout = QVBoxLayout(self.dr_config)
        self.plot_cov_cb = QCheckBox("Plot Covariance")
        self.plot_dir_cb = QCheckBox("Plot Direction Line")
        dr_layout.addWidget(self.plot_cov_cb)
        dr_layout.addWidget(self.plot_dir_cb)
        self.plot_cov_cb.setChecked(True)
        self.plot_dir_cb.setChecked(True)
        layout.addWidget(self.dr_config)

        # Static beacon configs
        self.sb_config = QWidget()
        sb_layout = QVBoxLayout(self.sb_config)
        self.live_cb = QCheckBox("Run Live")
        self.separate_cb = QCheckBox("Plot Separate")
        sb_layout.addWidget(self.live_cb)
        sb_layout.addWidget(self.separate_cb)
        sb_layout.addWidget(QLabel("Modem Positions (ID, Lat, Lon, Depth per line):"))
        self.modem_txt = QTextEdit()
        sb_layout.addWidget(self.modem_txt)
        sb_layout.addWidget(QLabel("Central Modem (int):"))
        self.central_spin = QSpinBox()
        self.central_spin.setRange(0, 9999)
        sb_layout.addWidget(self.central_spin)
        layout.addWidget(self.sb_config)

        # GPS Pings configs (placeholder)
        self.gps_config = QWidget()
        gps_layout = QVBoxLayout(self.gps_config)
        gps_layout.addWidget(QLabel("No extra configuration for GPS Pings."))
        layout.addWidget(self.gps_config)

        # Show config sections depending on selection
        self.cb_dead_reckoning.toggled.connect(self._update_visible)
        self.cb_static_beacon.toggled.connect(self._update_visible)
        self.cb_gps_pings.toggled.connect(self._update_visible)
        self._update_visible()

        # Ok/Cancel buttons
        btn_layout = QHBoxLayout()
        ok_btn = QPushButton("OK")
        cancel_btn = QPushButton("Cancel")
        btn_layout.addWidget(ok_btn)
        btn_layout.addWidget(cancel_btn)
        layout.addLayout(btn_layout)
        ok_btn.clicked.connect(self.accept)
        cancel_btn.clicked.connect(self.reject)

    def _update_visible(self):
        self.dr_config.setVisible(self.cb_dead_reckoning.isChecked())
        self.sb_config.setVisible(self.cb_static_beacon.isChecked())
        self.gps_config.setVisible(self.cb_gps_pings.isChecked())

    def get_config(self):
        out = {"bagpath": self.bagpath_input.text(), "plots": []}
        if self.cb_dead_reckoning.isChecked():
            dr_cfg = {
                "type": "dead_reckoning",
                "plot_covariance": self.plot_cov_cb.isChecked(),
                "plot_direction_line": self.plot_dir_cb.isChecked()
            }
            out["plots"].append(dr_cfg)
        if self.cb_static_beacon.isChecked():
            sb_cfg = {
                "type": "static_beacon",
                "run_live": self.live_cb.isChecked(),
                "plot_separate": self.separate_cb.isChecked(),
                "modem_positions": [
                    dict(zip(["id","lat","lon","depth"], map(str.strip, line.split(","))))
                    for line in self.modem_txt.toPlainText().splitlines() if line.count(",") == 3
                ],
                "central_modem": self.central_spin.value()
            }
            out["plots"].append(sb_cfg)
        if self.cb_gps_pings.isChecked():
            gps_cfg = {"type": "gps_pings"}
            out["plots"].append(gps_cfg)
        return out





if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = PostProcessingWindow()
    window.center_on_screen()  # Center the window on the screen
    window.show()
    sys.exit(app.exec())

