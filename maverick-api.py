#!/usr/bin/env python

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
from logging.handlers import RotatingFileHandler
import os
import json
import sys
import signal
import time
import threading

# tornado imports
import tornado.web
import tornado.ioloop
from tornado.log import enable_pretty_logging

# tornadoql imports
from tornadoql.graphql_handler import GQLHandler
from tornadoql.subscription_handler import GQLSubscriptionHandler

# graphql imports
from graphql import graphql
from graphql import get_introspection_query

# database imports
import motor

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


# def exception_handler(type, value, tb):
#     """Setup exception handler"""
#     app_log.exception(
#         "Uncaught exception: {0} - {1}".format(str(value), traceback.print_tb(tb))
#     )

# Install exception handler
# sys.excepthook = exception_handler

# TODO: log level set by config
app_log.setLevel(logging.INFO)
gen_log.setLevel(logging.DEBUG)
access_log.setLevel(logging.DEBUG)

# module imports
import modules # noqa E402
from modules.maverick_mavros import MAVROSConnection

# setup mongo database
db_client = motor.motor_tornado.MotorClient("localhost", 27017)

APP_ROOT = os.path.dirname(os.path.abspath(__file__))
APP_STATIC = os.path.join(APP_ROOT, "static")
# TODO: group the settings and make configurable
SETTINGS = {"static_path": APP_STATIC, "sockets": [], "subscriptions": {}, "db_client":db_client}


class GraphQLHandler(GQLHandler):
    def initialize(self, opts):
        super(GQLHandler, self).initialize()
        self.opts = opts
    
    @property
    def schema(self):
        return TornadoQL.schema


class GraphQLSubscriptionHandler(GQLSubscriptionHandler):
    def initialize(self, opts):
        super(GraphQLSubscriptionHandler, self).initialize()
        self.opts = opts

    @property
    def schema(self):
        return TornadoQL.schema

    @property
    def sockets(self):
        return self.opts["sockets"]

    @property
    def subscriptions(self):
        return self.opts["subscriptions"].get(self, {})

    @subscriptions.setter
    def subscriptions(self, subscriptions):
        self.opts["subscriptions"][self] = subscriptions


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
    def __init__(self, config):
        # TODO: roll settings into config
        args = dict(opts=SETTINGS)
        handlers = [
            (r"/subscriptions", GraphQLSubscriptionHandler, args),
            (r"/graphql", GraphQLHandler, args),
            (r"/graphiql", GraphiQLHandler),
            (r"/schema", SchemaHandler),
        ]
        
        if config["APP_DEBUG"]:
            debug = True
        else:
            debug = False
            
        settings = dict(
            debug = debug,
            cookie_secret=config["APP_SECRET_KEY"],
            static_path=APP_STATIC,
            xsrf_cookies=False,
        )
        TornadoQL.schema = modules.api_schema
        super(TornadoQL, self).__init__(handlers, **settings)

def start_server(config):
    application = TornadoQL(config)
    server = tornado.httpserver.HTTPServer(application)
    server.listen(
        port=int(config["SERVER_PORT"]), address=str(config["SERVER_INTERFACE"])
    )
    if config["APP_DEBUG"]:
        app_log.debug(
            "Starting Wholesale-API server: {0}:{1}/{2}".format(
                config["SERVER_INTERFACE"], config["SERVER_PORT"], config["APP_PREFIX"]
            )
        )
    return server


def main(config):
    server = start_server(config)
    tornado.ioloop.IOLoop.current().start()
    # this function blocks at this point until the server
    # is asked to exit via request_server_stop()
    app_log.debug("Tornado finished")
    server.stop()

def request_server_stop(config):
    # TODO: close all websocket connections (required?)
    ioloop = tornado.ioloop.IOLoop.current()
    ioloop.add_callback(ioloop.stop)
    app_log.debug("Asked Tornado to exit")


class Server(object):
    def __init__(self, optsargs):
        self.exit = False
        (self.opts, self.args) = optsargs
        self.server_thread = None
        self.mavros_connection = None
        self.mavlink_connection = None

        # TODO: fix this config mess...
        self.config = Configuration(self.opts.configuration)
        self.config = self.config.get_config()
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)
        
        # setup the connection to ROS
        loop = tornado.ioloop.IOLoop.current()
        self.mavros_connection = MAVROSConnection(self.config, loop, modules.module_schema)
        self.mavros_thread = threading.Thread(target=self.mavros_connection.run)
        self.mavros_thread.daemon = True
        self.mavros_thread.start()
        main(self.config)

    def exit_gracefully(self, signum, frame):
        """called on sigterm"""
        self.exit = True
        if self.mavros_connection:
            self.mavros_connection.shutdown()
        if self.mavlink_connection:
            self.mavlink_connection.shutdown()
        if self.server_thread:
            # attempt to shutdown the tornado server
            request_server_stop(self.config)
            self.server_thread.join(timeout=4)
        else:
            request_server_stop(self.config)


if __name__ == "__main__":
    from optparse import OptionParser
    from config import Configuration

    parser = OptionParser("maverick-api [options]")

    parser.add_option(
        "--configuration",
        dest="configuration",
        type="str",
        help="configuration file name",
        default="config.json",
    )
    optsargs = parser.parse_args()
    (opts, args) = optsargs
    Server(optsargs)
