import logging
import logging.handlers
import os

scriptDir = os.path.dirname(os.path.abspath(__file__))

# CURRENT_PATH = pkg_resources.resource_filename("online_algorithms", "online_aggregation")

def initLogger(level, console=False):
    logfile = os.path.join(scriptDir, "../log/log.log")
    # logfile = f"{CURRENT_PATH}/log/log.log"
    logger = logging.getLogger(__name__);
    logger.setLevel(level)
    formatter = logging.Formatter("[%(asctime)-15s] %(levelname)s %(name)s : %(message)s")

    if console:
        ch = logging.StreamHandler()
        ch.setFormatter(formatter)
        logger.addHandler(ch)

    logDir = os.path.dirname(logfile)
    if not os.path.isdir(logDir):
        os.makedirs(logDir)
    handler = logging.handlers.TimedRotatingFileHandler(logfile, when="D", backupCount=30)
    handler.setFormatter(formatter)
    logger.addHandler(handler)

    return logger


# logger = initLogger(logging.INFO, console=True)
logger = initLogger(logging.INFO, console=False)




if __name__ == "__main__":
    # logfile = os.path.join(scriptDir, "../log/log.log")
    logger = initLogger(logging.INFO, console=True)
    logger.debug("debug test!")
    logger.info("info test!")
    logger.warning("warning test!")
    logger.error("error test!")
    logger.critical("critical test!")