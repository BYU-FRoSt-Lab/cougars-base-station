import sys
from PyQt6.QtWidgets import QApplication, QWidget, QLabel, QPushButton
from PyQt6.QtGui import QPixmap, QFont, QCloseEvent
from PyQt6.QtCore import Qt, QThread
import graphviz
from base_station_gui.modemSubscriber import ModemStatus, ModemSubscriber, CougModemInfo
import logging
from base_station_gui.loggerInit import loggerInit

import rclpy

# Set up the logger

log = logging.getLogger("MainWindow")
loggerInit(log)
log.setLevel(logging.DEBUG)

'''------------------------------

Temporary Testing Info

------------------------------'''

Cougs: list[tuple[str, int]] = [
    ("Coug 1", 1),
    ("Coug 2", 2),
    ("Coug 3", 3),
]


'''------------------------------

Temporary Testing Info

------------------------------'''

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()

        self.initializeUI()

    def initializeUI(self) -> None:
        self.setGeometry(100, 200, 1000, 1000)
        self.setWindowTitle("GUI Example")
        self.setUpMainWindow()
        self.show()

    def setUpMainWindow(self) -> None:
        self.timingLabel = QLabel(self)

        # Set up the modem subscriber
        self.modemSub = ModemSubscriber(Cougs)

        # Make the modem subscriber work in a thread
        self.modemSubThread = QThread()
        self.modemSub.moveToThread(self.modemSubThread)

        # Generate an inital diagram before the thread even starts
        self.generateModemDiagram()

        # Connect the signal generated by the subscriber to the call back function
        self.modemSub.sig_ModemSubscriberUpdate.connect(self.generateModemDiagram)

        # Start running the thread
        self.modemSubThread.started.connect(self.modemSub.spin)
        self.modemSubThread.start()

    def generateModemDiagram(self) -> None:
        log.info("Generating the new diagram")

        cougModemInfos: list[CougModemInfo] = self.modemSub.getModemInfo()

        log.debug("Retrieved the statuses of %d modems", len(cougModemInfos))

        diagram = graphviz.Digraph("Modem_Timing", format="png")
        diagram.graph_attr["size"] = "6, 2!"
        diagram.edge_attr["constraint"] = "false"

        # Generate all the nodes
        for currModem in cougModemInfos:
            # Determine the attributes to plug into the node
            match currModem.status:
                case ModemStatus.RESPONDING:
                    shape = "octagon"
                    fillcolor = "#AAAAAA"
                    label=f"{currModem.label}\n✓"
                case ModemStatus.WARNING:
                    shape = "octagon"
                    fillcolor = "#FFFF22"
                    label=f"{currModem.label}\n! {currModem.numFailedSinceSuccess}"
                case ModemStatus.UNRESPONSIVE:
                    shape = "octagon"
                    fillcolor = "#FF0000"
                    label=f"{currModem.label}\nX"
                case ModemStatus.TURN_WAITING:
                    shape = "doubleoctagon"
                    fillcolor = "#FFFFFF"
                    label=f"{currModem.label}\n?"
                case ModemStatus.TURN_RESPONDED:
                    shape = "tripleoctagon"
                    fillcolor = "#00AA00"
                    label=f"{currModem.label}\n✓"
                case _:
                    shape = "octagon"
                    fillcolor = "#4444FF"
                    label=f"{currModem.label}\nInit"

            diagram.attr("node", shape=shape, style="filled", fillcolor=fillcolor, label=label)
            diagram.node(currModem.label)
        #-- End for loop --#

        # Generate all the edges
        for i in range(len(cougModemInfos) - 1):

            edge1 = cougModemInfos[i].label
            edge2 = cougModemInfos[(i+1) % len(cougModemInfos)].label
            diagram.edge(edge1, edge2)

        image = diagram.pipe(format="png")
        pixmap = QPixmap()
        pixmap.loadFromData(image, format="png")
        pixmap.scaled(10, 10)
        self.timingLabel.setPixmap(pixmap)
        self.repaint()

        log.debug("Set the new pixmap")

    #-- End generateModemDiagram --#

    def closeEvent(self, event: QCloseEvent):
        log.info("Shutting down")
        if (self.modemSubThread.isRunning()):
            log.debug("Deactivating the modemsub thread")
            self.modemSub.end()
            self.modemSubThread.quit()
            self.modemSubThread.wait()

        event.accept()


    
def main(args=None) -> None:
    rclpy.init(args=args)
    log.info("Launching the window")
    app = QApplication([])
    window = MainWindow()
    app.exec()
    log.info("Closing")
    rclpy.shutdown()

if __name__ == "__main__":
    main()
