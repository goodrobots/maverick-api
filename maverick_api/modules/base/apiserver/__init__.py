# std lib imports
import logging
import os
import sys
import signal
import threading

# tornado imports
import tornado.ioloop
from tornado.options import options

# module imports
from modules.base.tornadoql.tornadoql import TornadoQL

#import modules  # noqa E402
from modules.api.mavros import MAVROSConnection
from modules.api.status import StatusModule
from modules.api import module_schema

class ApiServer(object):
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
