import os
import logging

from tornado.log import LogFormatter
from tornado.options import define, options

from modules.base.util import functions

class MavLogging(object):
    def __init__(self):
        self.create_logdirs()
        self.setup_logging()

    def create_logdirs(self):
        functions.mkdirs(options.logdir)

    def setup_logging(self):
        access_log = logging.getLogger('tornado.access')
        acc_handler = logging.handlers.RotatingFileHandler(
                                    os.path.join(options.logdir, 'access.log'),
                                    mode="a",
                                    maxBytes=1 * 1024 * 1024,
                                    backupCount=2,
                                    encoding=None,
                                    delay=0,
                                    )
        acc_handler.setFormatter(LogFormatter(color=False, fmt='%(asctime)s.%(msecs)03d %(levelname)s %(message)s', datefmt='%Y-%m-%d,%H:%M:%S'))
        access_log.addHandler(acc_handler)

        application_log = logging.getLogger('tornado.application')
        channel = logging.StreamHandler()
        channel.setFormatter(LogFormatter(color=True, fmt='%(asctime)s.%(msecs)03d %(levelname)s %(message)s', datefmt='%Y-%m-%d,%H:%M:%S'))
        application_log.addHandler(channel)
        app_handler = logging.handlers.RotatingFileHandler(
                                                    os.path.join(options.logdir, 'api.log'),
                                                    mode="a",
                                                    maxBytes=1 * 1024 * 1024,
                                                    backupCount=2,
                                                    encoding=None,
                                                    delay=0,
                                                    )
        app_handler.setFormatter(LogFormatter(color=False, fmt='%(asctime)s.%(msecs)03d %(levelname)s %(message)s', datefmt='%Y-%m-%d,%H:%M:%S'))
        application_log.addHandler(app_handler)
        
        general_log = logging.getLogger('tornado.general')
        gen_handler = logging.handlers.RotatingFileHandler(
                                            os.path.join(options.logdir, 'general.log'),
                                            mode="a",
                                            maxBytes=1 * 1024 * 1024,
                                            backupCount=2,
                                            encoding=None,
                                            delay=0,
                                            )
        gen_handler.setFormatter(LogFormatter(color=False, fmt='%(asctime)s.%(msecs)03d %(levelname)s %(message)s', datefmt='%Y-%m-%d,%H:%M:%S'))
        general_log.addHandler(gen_handler)

        # Update log level depending on 'debug' value
        if options.debug == True:
            logging.getLogger().setLevel(logging.DEBUG)
        else:
            logging.getLogger().setLevel(logging.INFO)
        logging.info("Loaded config options")
