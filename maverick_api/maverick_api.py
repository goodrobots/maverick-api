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
#from logging.handlers import RotatingFileHandler
import os
import json
import sys
import signal
import time
import threading
from pathlib import Path

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


"""
# setup logging
handler = RotatingFileHandler(
    "tornado.log",
    mode="a",
    maxBytes=1 * 1024 * 1024,
    backupCount=2,
    encoding=None,
    delay=0,
)
enable_pretty_logging()
formatter = logging.Formatter("%(asctime)s %(levelname)s %(message)s")
handler.setFormatter(formatter)
access_log = logging.getLogger("tornado.access")
app_log = logging.getLogger("tornado.application")
gen_log = logging.getLogger("tornado.general")
app_log.addHandler(handler)
gen_log.addHandler(handler)
access_log.addHandler(handler)
"""

# def exception_handler(type, value, tb):
#     """Setup exception handler"""
#     app_log.exception(
#         "Uncaught exception: {0} - {1}".format(str(value), traceback.print_tb(tb))
#     )

# Install exception handler
# sys.excepthook = exception_handler

# TODO: log level set by config
"""
app_log.setLevel(logging.INFO)
gen_log.setLevel(logging.DEBUG)
access_log.setLevel(logging.DEBUG)
"""

"""
# setup mongo database
db_client = motor.motor_tornado.MotorClient("localhost", 27017)

APP_ROOT = os.path.dirname(os.path.abspath(__file__))
APP_STATIC = os.path.join(APP_ROOT, "static")
# TODO: group the settings and make configurable
SETTINGS = {
    "static_path": APP_STATIC,
    "sockets": [],
    "subscriptions": {},
    "db_client": db_client,
}
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
        self.render(os.path.join(APP_STATIC, "graphiql.html"))


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


def start_server():
    application = TornadoQL()
    server = tornado.httpserver.HTTPServer(application)
    server.listen(
        port=options.server_port,
        address=options.server_interface
    )
    logging.debug("Starting Wholesale-API server: {0}:{1}/{2}".format(options.server_interface, options.server_port, options.app_prefix))
    return server


def main():
    server = start_server()
    tornado.ioloop.IOLoop.current().start()
    # this function blocks at this point until the server
    # is asked to exit via request_server_stop()
    logging.debug("Tornado finished")
    server.stop()


def request_server_stop():
    # TODO: close all websocket connections (required?)
    ioloop = tornado.ioloop.IOLoop.current()
    ioloop.add_callback(ioloop.stop)
    logging.debug("Asked Tornado to exit")


class Server(object):
    def __init__(self):
        self.exit = False
        self.server_thread = None
        self.mavros_connection = None
        self.mavlink_connection = None

        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)

        # setup the connection to ROS
        loop = tornado.ioloop.IOLoop.current()
        self.mavros_connection = MAVROSConnection(loop, module_schema)
        self.mavros_thread = threading.Thread(target=self.mavros_connection.run)
        self.mavros_thread.daemon = True
        self.mavros_thread.start()
        self.status_module = StatusModule(loop, module_schema)
        main()

    def exit_gracefully(self, signum, frame):
        """called on sigterm"""
        self.exit = True
        if self.mavros_connection:
            self.mavros_connection.shutdown()
        if self.mavlink_connection:
            self.mavlink_connection.shutdown()
        if self.server_thread:
            # attempt to shutdown the tornado server
            request_server_stop()
            self.server_thread.join(timeout=4)
        else:
            request_server_stop()


if __name__ == "__main__":
    # Setup config
    MavConfig('config/maverick-api.conf')

    # Define basedir in options, must be done from main script
    options.basedir = str(Path(__file__).resolve().parent)

    # Setup logging
    MavLogging()

    # Instantiate and start api server
    Server()
