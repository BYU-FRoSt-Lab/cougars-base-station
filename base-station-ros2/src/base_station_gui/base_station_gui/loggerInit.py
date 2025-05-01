import logging

LEVEL_COLORS = {
    logging.DEBUG:      "\033[36m",
    logging.INFO:       "\033[32m",
    logging.WARNING:    "\033[33m",
    logging.ERROR:      "\033[31m",
    logging.CRITICAL:   "\033[41m"
}

FORMAT_STRING: str = "[%(asctime)s] [%(levelname)s] %(name)s: %(message)s"

class ColorFormatter(logging.Formatter):
    def format(self, record):
        level_color = LEVEL_COLORS.get(record.levelno, "\033[0m")
        message = super().format(record)
        return f"{level_color}{message}\033[0m"
    
consoleHandler = logging.StreamHandler()
consoleHandler.setLevel(logging.DEBUG)
formatter = ColorFormatter(FORMAT_STRING)
consoleHandler.setFormatter(formatter)

def loggerInit(logger : logging.Logger) -> None:
    if not logger.hasHandlers():
        logger.addHandler(consoleHandler)