import os
import logging

from tornado.log import LogFormatter
from tornado.options import define, options

from modules.base.util import functions


class MavLogging(object):
    def __init__(self):
        self.archive_directory = "archive"
        self.archive_path = os.path.join(options.logdir, self.archive_directory)
        self.logs = []
        self.log_fmt_prefix = "[%(levelname)1.1s %(asctime)s %(pathname)s:%(lineno)d]\n"
        self.date_fmt = "%Y-%m-%d,%H:%M:%S"
        self.create_logdirs()
        self.setup_logging()

    def create_logdirs(self):
        functions.mkdirs(options.logdir)
        functions.mkdirs(self.archive_path)

    def setup_logging(self):
        self.access_log = logging.getLogger("tornado.access")
        acc_handler = logging.handlers.RotatingFileHandler(
            os.path.join(options.logdir, "access.log"),
            mode="a",
            maxBytes=1 * 1024 * 1024,
            backupCount=2,
            encoding=None,
            delay=0,
        )
        acc_handler.setFormatter(
            LogFormatter(
                color=False,
                fmt=self.log_fmt_prefix + "%(message)s",
                datefmt=self.date_fmt,
            )
        )
        acc_handler.namer = self.namer
        self.access_log.addHandler(acc_handler)
        self.logs.append(self.access_log)

        self.application_log = logging.getLogger("tornado.application")
        channel = logging.StreamHandler()
        channel.setFormatter(
            LogFormatter(
                color=True,
                fmt="%(color)s" + self.log_fmt_prefix + "%(end_color)s%(message)s",
                datefmt=self.date_fmt,
                colors={10: 4, 20: 2, 30: 3, 40: 1},
            )
        )
        self.application_log.addHandler(channel)
        app_handler = logging.handlers.RotatingFileHandler(
            os.path.join(options.logdir, "api.log"),
            mode="a",
            maxBytes=1 * 1024 * 1024,
            backupCount=2,
            encoding=None,
            delay=0,
        )
        app_handler.setFormatter(
            LogFormatter(
                color=False,
                fmt=self.log_fmt_prefix + "%(message)s",
                datefmt=self.date_fmt,
            )
        )
        app_handler.namer = self.namer
        self.application_log.addHandler(app_handler)
        self.logs.append(self.application_log)

        self.general_log = logging.getLogger("tornado.general")
        # Log tornado.general into tornado.application
        self.general_log.addHandler(app_handler)
        self.logs.append(self.general_log)
        
        if options.debug == True:
            # Only setup the websocket logging if logging level is set to debug
            self.websocket_log = logging.getLogger("tornado.websocket")
            websoc_handler = logging.handlers.RotatingFileHandler(
                os.path.join(options.logdir, "websocket.log"),
                mode="a",
                maxBytes=1 * 1024,
                backupCount=2,
                encoding=None,
                delay=0,
            )
            websoc_handler.setFormatter(
                LogFormatter(
                    color=False,
                    fmt="[%(levelname)1.1s %(asctime)s] %(message)s",
                    datefmt=self.date_fmt,
                )
            )
            websoc_handler.namer = self.namer
            self.websocket_log.addHandler(websoc_handler)
            self.logs.append(self.websocket_log)

        # Update log level depending on 'debug' config value
        if options.debug == True:
            for log in self.logs:
                log.setLevel(logging.DEBUG)

        else:
            for log in self.logs:
                log.setLevel(logging.INFO)
        self.application_log.info("Loaded config options")

    def namer(self, log_name):
        """Implement python log naming function to send old log files to archive directory"""
        if log_name.split(".")[-1].isdigit():
            return os.path.join(self.archive_path, os.path.split(log_name)[-1])
        else:
            return log_name
