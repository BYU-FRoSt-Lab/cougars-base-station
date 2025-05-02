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

def findModemIndex(id: int) -> int:
    dataLock.acquire()
    log.debug("Scanning the modem to find modem %d", id)
    log.debug("Length of the modems is: %d", len(modemData))

    retVal = -1

    for i in range(len(modemData)):
        log.debug("\tid at index %d is: %d", i, modemData[i].id)
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

        index = findModemIndex(sendData.dest_id)
        if (index == -1):
            log.error("Unable to find data on modem %d", sendData.dest_id)
            return
        

        return
    
    def cmdUpdateHandler(self, updateData: ModemCmdUpdate):
        if (type(updateData) != ModemCmdUpdate):
            log.error("Received wrong message type")
            return
        
        log.debug("Received command update about modem %d,\n\tCID: %X", updateData.target_id, updateData.msg_id)
        return
    
    def recHandler(self, recData: ModemRec):
        if (type(recData) != ModemRec):
            log.error("Received wrong message type")
            return

        log.debug("Received response.\n\tSource id: %d\n\tDest id: %d\n\tCID: %X", recData.src_id, recData.dest_id, recData.msg_id)
        return


class ModemSubscriber(QObject):
    sig_ModemSubscriberUpdate = pyqtSignal()

    def __init__(self, modemInfo: list[tuple[str, int]]):
        super().__init__()

        log.debug("Initalizing the Modem Subscriber with %d modems", len(modemInfo))

        # Set up the class information
        modemData: list[CougModemInfo] = []
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
            
        log.info("Shutting down")

    def nextTurn(self) -> bool:

        log.debug("Simulating the next modem's turn")

        # Reset the information for the current modem
        dataLock.acquire()
        currModem = modemData[self.currentModemIndex]

        currModem.numPings += 1

        currModemState = currModem.status

        if (currModemState == ModemStatus.TURN_RESPONDED):
            nextState = ModemStatus.RESPONDING
            currModem.numFailedSinceSuccess = 0
        elif (currModemState == ModemStatus.TURN_WAITING):
            currModem.numFailedResponses += 1
            currModem.numFailedSinceSuccess += 1
            if (currModem.numFailedSinceSuccess >= NUM_FAILS_FOR_UNRESPONSIVE):
                nextState = ModemStatus.UNRESPONSIVE
                log.warning("Modem %d has been unresponsive for %d cycles", currModem.id, currModem.numFailedSinceSuccess)
            else:
                nextState = ModemStatus.WARNING
        else:
            nextState = ModemStatus.UNRESPONSIVE

        currModem.status = nextState
        log.debug("Modem %d is set to %s", currModem.id, nextState)

        # Set the values for the next modem
        self.currentModemIndex += 1
        self.currentModemIndex %= len(modemData)
        currModem = modemData[self.currentModemIndex]
        currModem.status = ModemStatus.TURN_WAITING

        # Determine if the modem will respond or not
        probabilityOfResponse = PROB_OF_RESPONSE[currModem.id]
        rolledVal = random.randint(1, 100)
        doesRespond = rolledVal <= probabilityOfResponse
        log.debug("Modem %d has a %d%% chance of response.  Rolled %d.  Modem will respond: %s",
                  currModem.id, PROB_OF_RESPONSE[currModem.id], rolledVal, doesRespond)

        dataLock.release()

        self.sig_ModemSubscriberUpdate.emit()

        return doesRespond

    def processResponse(self) -> None:
        dataLock.acquire()
        currModem = modemData[self.currentModemIndex]
        log.debug("Processing response for modem %d", currModem.id)
        currModem.status = ModemStatus.TURN_RESPONDED
        self.sig_ModemSubscriberUpdate.emit()
        dataLock.release()

    def getModemInfo(self) -> list[CougModemInfo]:
        dataLock.acquire()
        output = list(modemData)
        dataLock.release()
        return output

    def end(self) -> None:
        log.debug("Ending loop")
        self.keepRunning = False
        

        




        


    

