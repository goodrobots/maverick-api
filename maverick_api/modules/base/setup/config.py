import os
import sys
import functools
import logging

from tornado import ioloop
from tornado.options import define, options

class MavConfig(object):
    def __init__(self, config_file):
        self.config_file = config_file
        self.define_options()
        self.modify_time = None
        self.load_options()
        self.autoreload_config_file()

    # Define, config options
    def define_options(self):
        define('app_prefix', default='maverick-api/', type=str, help='URL prefix')
        define('app_secretkey', default='super_s3cret', type=str, help='Secret Key')
        define('basedir', default=None, type=str, help='Base Directory, defaults to directory of maverick_api.py script if not set')
        define('debug', default=False, type=bool, help='Turn on debug mode')
        define('logdir', default="logs/", type=str, help='Log directory')
        define('server_interface', default='0.0.0.0', type=str, help='Interface to listen on: 0.0.0.0 represents all interfaces')
        define('server_port', default=6795, type=int, help='Port to listen on')

    # Parse and load config options
    def load_options(self):
        try:
            options.parse_config_file(self.config_file)
        except FileNotFoundError as e:
            print("Error, config file {} not found".format(self.config_file))
            sys.exit(1)
        #tornado.options.parse_command_line()

    def autoreload_config_file(self):
        logging.debug("Starting autoreload_config_file")
        def reload_options_on_update(config_file):
            modified = os.stat(config_file).st_mtime
            if not self.modify_time:
                self.modify_time = modified
                return
            if self.modify_time != modified:
                logging.info('Modified config file, reloading options')
                self.modify_time = modified
                self.load_options()
        config_callback = functools.partial(reload_options_on_update, self.config_file)
        ioloop.PeriodicCallback(config_callback, 1000).start()
