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
import traceback
import os
import json
import sys
import signal
import time
import threading
from pathlib import Path, PurePath

# tornado imports
import tornado.web
import tornado.ioloop
from tornado.options import options

# tornadoql imports
from modules.base.tornadoql.graphql_handler import GQLHandler
from modules.base.tornadoql.subscription_handler import GQLSubscriptionHandler

# graphql imports
from graphql import graphql
from graphql import get_introspection_query

# database imports
import motor

# module imports
from modules.base.setup.config import MavConfig
from modules.base.setup.logging import MavLogging

#import modules  # noqa E402
from modules.api.maverick_mavros import MAVROSConnection
from modules.api.maverick_status import StatusModule
from modules.api import module_schema, api_schema

from tornado.log import app_log

"""
# setup mongo database
db_client = motor.motor_tornado.MotorClient("localhost", 27017)
"""

class GraphQLHandler(GQLHandler):
    def initialize(self):
        super(GQLHandler, self).initialize()

    @property
    def schema(self):
        return TornadoQL.schema


class GraphQLSubscriptionHandler(GQLSubscriptionHandler):
    def initialize(self):
        super(GraphQLSubscriptionHandler, self).initialize()
        self.handler_sockets = []
        self.handler_subscriptions = {}

    @property
    def schema(self):
        return TornadoQL.schema

    @property
    def sockets(self):
        return self.handler_sockets

    @property
    def subscriptions(self):
        return self.handler_subscriptions.get(self, {})

    @subscriptions.setter
    def subscriptions(self, subscriptions):
        self.handler_subscriptions[self] = subscriptions


class GraphiQLHandler(tornado.web.RequestHandler):
    def get(self):
        self.render(os.path.join(self.application.settings.get("static_path"), "graphiql.html"))


class SchemaHandler(tornado.web.RequestHandler):
    """introspection of the api schema"""

    async def get(self):
        query = get_introspection_query(descriptions=True)
        introspection_query_result = await graphql(TornadoQL.schema, query)
        introspection_dict = introspection_query_result.data
        self.write(json.dumps(introspection_dict, indent=4, sort_keys=True))
        self.finish()


class TornadoQL(tornado.web.Application):
    def __init__(self):
        handlers = [
            (r"/subscriptions", GraphQLSubscriptionHandler),
            (r"/graphql", GraphQLHandler),
            (r"/graphiql", GraphiQLHandler),
            (r"/schema", SchemaHandler),
        ]

        settings = dict(
            debug = options.debug,
            cookie_secret = options.app_secretkey,
            static_path = os.path.join(options.basedir, "data", "static"),
            xsrf_cookies=False,
        )

        TornadoQL.schema = api_schema
        super(TornadoQL, self).__init__(handlers, **settings)


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
        app_log.debug("Starting Maverick API server: {0}:{1}/{2}".format(options.server_interface, options.server_port, options.app_prefix))
        
    def serve(self):
        tornado.ioloop.IOLoop.current().start()
        # this function blocks at this point until the server
        # is asked to exit via request_stop()
        app_log.debug("Tornado finished")
    
    def request_stop(self):
        # TODO: close all websocket connections (required?)
        ioloop = tornado.ioloop.IOLoop.current()
        ioloop.add_callback(ioloop.stop)
        app_log.debug("Asked Tornado to exit")

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
