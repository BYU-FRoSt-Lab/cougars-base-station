from PyQt6.QtCore import QObject, pyqtSignal, QMutex, QTimer
from enum import Enum
import logging
from base_station_gui.loggerInit import loggerInit
from seatrac_interfaces.msg import ModemCmdUpdate, ModemRec, ModemSend
import rclpy
from rclpy.node import Node
from threading import Lock

# These are just here for testing sake
from time import sleep
import random 

NUM_FAILS_FOR_UNRESPONSIVE = 3

# This is just here for testing sake
PROB_OF_RESPONSE = [0, 95, 70, 10, 25, 50]

log = logging.getLogger("ModemSubscriber")
loggerInit(log)
log.setLevel(logging.DEBUG)



class ModemStatus(Enum):
    INIT = 0
    RESPONDING = 1
    WARNING = 2
    UNRESPONSIVE = 3
    TURN_WAITING = 4
    TURN_RESPONDED = 5

class CougModemInfo():
    def __init__(self, label: str, id: id) -> None:
        self.numPings = 0
        self.numFailedResponses = 0
        self.numFailedSinceSuccess = 0
        self.label = label
        self.id = id
        self.status = ModemStatus.INIT

dataLock = Lock()
modemData: list[CougModemInfo] = []
dataUpdated = False

def findModemIndex(id: int) -> int:
    dataLock.acquire()

    retVal = -1

    for i in range(len(modemData)):
        if modemData[i].id == id:
            retVal = i

    dataLock.release()   
    return retVal

class SubNode(Node):
    def __init__(self):
        super().__init__("ModemSubscriberNode")
        
        # Create each of the subscriptions
        self.sendSubscription = self.create_subscription(
            ModemSend,
            "modem_send",
            self.sendHandler,
            10
        )

        self.cmdUpdateSubscription = self.create_subscription(
            ModemCmdUpdate,
            "modem_cmd_update",
            self.cmdUpdateHandler,
            10
        )

        self.recSubscription = self.create_subscription(
            ModemRec,
            "modem_rec",
            self.recHandler,
            10
        )
    
    #-- __init__ --##

    def sendHandler(self, sendData: ModemSend):
        if (type(sendData) != ModemSend):
            log.error("Received wrong message type")
            return

        log.debug("Pinging modem %d\n\tCID: %X", sendData.dest_id, sendData.msg_id)
        return
    
    def cmdUpdateHandler(self, updateData: ModemCmdUpdate):
        if (type(updateData) != ModemCmdUpdate):
            log.error("Received wrong message type")
            return
        
        log.debug("Received command update about modem %d,\n\tCID: %X", updateData.target_id, updateData.msg_id)

        index = findModemIndex(updateData.target_id)
        if (index == -1):
            log.error("Unable to find data on modem %d", updateData.target_id)
            return

        currentModem: CougModemInfo = modemData[index]

        dataLock.acquire()

        # Check for the status
        if updateData.msg_id == 0x60:
            # The modem was just pinged      
            currentModem.status = ModemStatus.TURN_WAITING
            currentModem.numPings += 1
        elif updateData.msg_id == 0x63:
            # The modem missed a response
            currentModem.numFailedResponses += 1
            currentModem.numFailedSinceSuccess += 1

            if (currentModem.numFailedResponses >= NUM_FAILS_FOR_UNRESPONSIVE):
                currentModem.status = ModemStatus.UNRESPONSIVE
            else:
                currentModem.status = ModemStatus.WARNING
        else:
            # Any other status
            log.warning("Cast for CID %X is not handled", updateData.msg_id)

        # Set all other modems to not their turn
        for i in range(len(modemData)):
            if i == index:
                continue
            if modemData[i].status == ModemStatus.TURN_RESPONDED:
                modemData[i].status = ModemStatus.RESPONDING

        dataLock.release()

        global dataUpdated
        dataUpdated = True
        return
    
    def recHandler(self, recData: ModemRec):
        if (type(recData) != ModemRec):
            log.error("Received wrong message type")
            return

        log.debug("Received response.\n\tSource id: %d\n\tDest id: %d\n\tCID: %X", recData.src_id, recData.dest_id, recData.msg_id)

        index = findModemIndex(recData.src_id)
        if (index == -1):
            log.error("Unable to find data on modem %d", recData.src_id)
            return

        currentModem: CougModemInfo = modemData[index]
        dataLock.acquire()
        currentModem.numFailedSinceSuccess = 0
        currentModem.status = ModemStatus.TURN_RESPONDED
        dataLock.release()

        global dataUpdated
        dataUpdated = True
        return


class ModemSubscriber(QObject):
    sig_ModemSubscriberUpdate = pyqtSignal()

    def __init__(self, modemInfo: list[tuple[str, int]]):
        super().__init__()

        log.debug("Initalizing the Modem Subscriber with %d modems", len(modemInfo))

        # Set up the class information
        self.currentModemIndex: int = 0

        # Load in all of the modem info
        dataLock.acquire()
        for currModem in modemInfo:
            modemData.append(CougModemInfo(currModem[0], currModem[1]))
        dataLock.release()

        self.keepRunning = True

        self.subNode = SubNode()

    def spin(self) -> None:
        log.info("Beginning to spin")
        while self.keepRunning and rclpy.ok():
            rclpy.spin_once(self.subNode)
            global dataUpdated
            if (dataUpdated):
                self.sig_ModemSubscriberUpdate.emit()
                log.info("Sending signal to update the diagram")
                dataUpdated = False
        # End while loop

        log.info("Shutting down")

    def getModemInfo(self) -> list[CougModemInfo]:
        dataLock.acquire()
        output = list(modemData)
        dataLock.release()
        return output

    def end(self) -> None:
        log.debug("Ending loop")
        self.keepRunning = False
        

        




        


    

