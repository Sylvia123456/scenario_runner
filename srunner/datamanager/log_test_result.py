import logging
import logging.handlers
import os
from datetime import datetime

scriptDir = os.path.dirname(os.path.abspath(__file__))

# CURRENT_PATH = pkg_resources.resource_filename("online_algorithms", "online_aggregation")

def initLogger(level, console=False):
    file_path = '../test_result/' + datetime.now().strftime("%Y-%m-%d_%H-%M-%S") + '.log'
    logfile = os.path.join(scriptDir, file_path)
    logdir = os.path.dirname(logfile)
    if not os.path.isdir(logdir):
        os.makedirs(logdir)

    logger = logging.getLogger(__name__)
    logger.setLevel(level)
    formatter = logging.Formatter("[%(asctime)-15s] %(levelname)s %(name)s : %(message)s")

    if console:
        ch = logging.StreamHandler()
        ch.setFormatter(formatter)
        logger.addHandler(ch)

    handler = logging.handlers.TimedRotatingFileHandler(logfile, when="D", backupCount=30)
    handler.setFormatter(formatter)
    logger.addHandler(handler)

    return logger


# logger = initLogger(logging.INFO, console=True)
logger = initLogger(logging.INFO, console=False)