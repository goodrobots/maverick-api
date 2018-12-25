#!/usr/bin/python3

"""
Tornado server for maverick-api
Samuel Dudley
Feb 2018
https://github.com/goodrobots/maverick-api
"""
__version__ = "0.2"

# TODO: setup tests and flake8

# std lib imports
import logging
import os
import sys
import signal
import threading
from pathlib import Path, PurePath

# tornado imports
import tornado.ioloop
from tornado.options import options

# module imports
from modules.base.setup.config import MavConfig
from modules.base.setup.logging import MavLogging
from modules.base.tornadoql.tornadoql import TornadoQL

#import modules  # noqa E402
from modules.api.mavros import MAVROSConnection
from modules.api.status import StatusModule
from modules.api import module_schema

class Server(object):
    def __init__(self):
        self.exit = False
        self.server = None
        self.mavros_connection = None
        self.mavlink_connection = None

        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)
        
    def initialize(self):
        # setup the connection to ROS
        loop = tornado.ioloop.IOLoop.current()
        self.mavros_connection = MAVROSConnection(loop, module_schema)
        self.mavros_thread = threading.Thread(target=self.mavros_connection.run)
        self.mavros_thread.daemon = True
        self.mavros_thread.start()
        self.status_module = StatusModule(loop, module_schema)
        
        application = TornadoQL()
        self.server = tornado.httpserver.HTTPServer(application)
        self.server.listen(
            port=options.server_port,
            address=options.server_interface
        )
        logging.debug("Starting Maverick API server: {0}:{1}/{2}".format(options.server_interface, options.server_port, options.app_prefix))
        
    def serve(self):
        tornado.ioloop.IOLoop.current().start()
        # this function blocks at this point until the server
        # is asked to exit via request_stop()
        logging.debug("Tornado finished")
    
    def request_stop(self):
        # TODO: close all websocket connections (required?)
        ioloop = tornado.ioloop.IOLoop.current()
        ioloop.add_callback(ioloop.stop)
        logging.debug("Asked Tornado to exit")

    def exit_gracefully(self, signum, frame):
        """called on sigterm"""
        self.exit = True
        if self.mavros_connection:
            self.mavros_connection.shutdown()
        if self.mavlink_connection:
            self.mavlink_connection.shutdown()
        self.request_stop()


if __name__ == "__main__":
    # Obtain basedir path (must be done from main script)
    basedir = Path(__file__).resolve().parent
    
    # Setup config
    MavConfig(PurePath(basedir).joinpath('config', 'maverick-api.conf'))
    
    # Define basedir in options
    options.basedir = str(basedir)
    
    # Setup logging
    MavLogging()

    # Instantiate and start api server
    api = Server()
    api.initialize()
    api.serve()
