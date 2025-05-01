from PyQt6.QtCore import QObject, pyqtSignal, QMutex, QTimer
from enum import Enum
import logging
from base_station_gui.loggerInit import loggerInit

# These are just here for testing sake
from time import sleep
import random 

NUM_FAILS_FOR_UNRESPONSIVE = 3

# This is just here for testing sake
PROB_OF_RESPONSE = [0, 95, 70, 10, 25, 50]

log = logging.getLogger("ModemSubscriber")
loggerInit(log)
log.setLevel(logging.INFO)

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

class ModemSubscriber(QObject):
    sig_ModemSubscriberUpdate = pyqtSignal()

    def __init__(self, modemInfo: list[tuple[str, int]]):
        super().__init__()

        log.debug("Initalizing the Modem Subscriber with %d modems", len(modemInfo))

        # Set up the class information
        self.modems: list[CougModemInfo] = []
        self.mutex: QMutex = QMutex()
        self.currentModemIndex: int = 0

        # Load in all of the modem info
        self.mutex.lock()
        for currModem in modemInfo:
            self.modems.append(CougModemInfo(currModem[0], currModem[1]))
        self.mutex.unlock()

        self.keepRunning = True

    def spin(self) -> None:
        log.info("Beginning to spin")
        while self.keepRunning:
            responds = self.nextTurn()
            sleep(1)
            if (responds):
                self.processResponse()
            sleep(1)

        log.info("Shutting down")

    def nextTurn(self) -> bool:

        log.debug("Simulating the next modem's turn")

        # Reset the information for the current modem
        self.mutex.lock()
        currModem = self.modems[self.currentModemIndex]

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
        self.currentModemIndex %= len(self.modems)
        currModem = self.modems[self.currentModemIndex]
        currModem.status = ModemStatus.TURN_WAITING

        # Determine if the modem will respond or not
        probabilityOfResponse = PROB_OF_RESPONSE[currModem.id]
        rolledVal = random.randint(1, 100)
        doesRespond = rolledVal <= probabilityOfResponse
        log.debug("Modem %d has a %d%% chance of response.  Rolled %d.  Modem will respond: %s",
                  currModem.id, PROB_OF_RESPONSE[currModem.id], rolledVal, doesRespond)

        self.mutex.unlock()

        self.sig_ModemSubscriberUpdate.emit()

        return doesRespond

    def processResponse(self) -> None:
        self.mutex.lock()
        currModem = self.modems[self.currentModemIndex]
        log.debug("Processing response for modem %d", currModem.id)
        currModem.status = ModemStatus.TURN_RESPONDED
        self.sig_ModemSubscriberUpdate.emit()
        self.mutex.unlock()

    def getModemInfo(self) -> list[CougModemInfo]:
        self.mutex.lock()
        output = list(self.modems)
        self.mutex.unlock()
        return output

    def end(self) -> None:
        log.debug("Ending loop")
        self.keepRunning = False
        

        




        


    

