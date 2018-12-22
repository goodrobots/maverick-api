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
        options.log_file_prefix = os.path.join(options.logdir, 'maverick-api.log')

        access_log = logging.getLogger('tornado.access')
        acc_handler = logging.FileHandler(os.path.join(options.logdir, 'access.log'))
        access_log.addHandler(acc_handler)

        app_log = logging.getLogger('tornado.application')
        channel = logging.StreamHandler()
        channel.setFormatter(LogFormatter(color=True, fmt='%(asctime)s.%(msecs)03d', datefmt='%Y-%m-%d,%H:%M:%S'))
        app_log.addHandler(channel)
        app_handler = logging.FileHandler(options.log_file_prefix)
        app_handler.setFormatter(LogFormatter(color=False, fmt='%(asctime)s.%(msecs)03d', datefmt='%Y-%m-%d,%H:%M:%S'))
        app_log.addHandler(app_handler)

        # Update log level depending on 'debug' value
        if options.debug == True:
            logging.getLogger().setLevel(logging.DEBUG)
        else:
            logging.getLogger().setLevel(logging.INFO)
        logging.info("Loaded config options")
