import sys
import logging
from logging.handlers import RotatingFileHandler


def setupLogger(logName, level=logging.DEBUG):
    logger = logging.getLogger(logName)
    hdlr = RotatingFileHandler(
        "{0}.log".format(logName),
        mode="a",
        maxBytes=1 * 1024 * 1024,
        backupCount=2,
        encoding=None,
        delay=0,
    )
    formatter = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
    hdlr.setFormatter(formatter)
    logger.addHandler(hdlr)
    # TODO: set level based on level passed to fn
    logger.setLevel(level)
    return logger


def stdoutLogger(hdlr_name=__name__, level=logging.DEBUG):
    logger = logging.getLogger(hdlr_name)
    hdlr = logging.StreamHandler(sys.stdout)
    hdlr.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(message)s"))
    hdlr.setLevel(logging.INFO)
    logger.addHandler(hdlr)
    logger.setLevel(level)
    return logger


def wrap360(angle):
    if angle > 360 or angle < 0:
        angle = angle % 360.0
    return angle
