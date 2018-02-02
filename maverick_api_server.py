'''
Tornado server for maverick-api
Samuel Dudley
Feb 2018
https://github.com/goodrobots/maverick-api
'''
VERSION = "v0.1"

# TODO: setup tests and flake8
import future
import tornado.web
import logging

import os, json, sys, select, signal, threading
import time
from uuid import uuid4 # used to generate unique IDs

# zeroconf placeholder
# from zeroconf import ServiceInfo, Zeroconf # advertise available websockets to browser

from tornadoql.graphql_handler import GQLHandler
from tornadoql.subscription_handler import GQLSubscriptionHandler

from api.schema import schema

APP_ROOT = os.path.dirname(os.path.abspath(__file__))
APP_STATIC = os.path.join(APP_ROOT, 'static')
# TODO: group the settings and make configurable
SETTINGS = {
    'static_path': APP_STATIC,
    'sockets': [],
    'subscriptions': {}
}

# Taken from https://github.com/IlyaRadinsky/tornadoql under MIT license
class GraphQLHandler(GQLHandler):
    @property
    def schema(self):
        return TornadoQL.schema
    
# Taken from https://github.com/IlyaRadinsky/tornadoql under MIT license
class GraphQLSubscriptionHandler(GQLSubscriptionHandler):

    def initialize(self, opts):
        super(GraphQLSubscriptionHandler, self).initialize()
        self.opts = opts

    @property
    def schema(self):
        return TornadoQL.schema

    @property
    def sockets(self):
        return self.opts['sockets']

    @property
    def subscriptions(self):
        return self.opts['subscriptions'].get(self, {})

    @subscriptions.setter
    def subscriptions(self, subscriptions):
        self.opts['subscriptions'][self] = subscriptions
        
# Taken from https://github.com/IlyaRadinsky/tornadoql under MIT license
class GraphiQLHandler(tornado.web.RequestHandler):
    def get(self):
        self.render(os.path.join(APP_STATIC, 'graphiql.html'))

# Adapted from https://github.com/IlyaRadinsky/tornadoql under MIT license
class TornadoQL(tornado.web.Application):
    def __init__(self, config):
        # TODO: roll settings into config
        args = dict(opts=SETTINGS)
        handlers = [
            (r'/{0}subscriptions'.format(config['APP_PREFIX']) , GraphQLSubscriptionHandler, args),
            (r'/{0}graphql'.format(config['APP_PREFIX']), GraphQLHandler),
            (r'/{0}graphiql'.format(config['APP_PREFIX']), GraphiQLHandler)
        ]
        
        settings = dict(
            cookie_secret = config['APP_SECRET_KEY'],
            static_path = APP_STATIC,
            static_url_prefix = '/{0}static/'.format(config['APP_PREFIX']),
            xsrf_cookies = False,
        )
        TornadoQL.schema = schema
        super(TornadoQL, self).__init__(handlers, **settings)

def start_app(config):
    logging.getLogger("tornado").setLevel(logging.WARNING)
    application = TornadoQL(config)
    server = tornado.httpserver.HTTPServer(application)
    server.listen(port = int(config['SERVER_PORT']), address = str(config['SERVER_INTERFACE']))
    if config['APP_DEBUG']:
        print("Starting Maverick-API server: {0}:{1}/{2}".format(config['SERVER_INTERFACE'], config['SERVER_PORT'], config['APP_PREFIX']))
    return server

def stop_tornado(config):
    # TODO: close all websocket connections (required?)
    ioloop = tornado.ioloop.IOLoop.current()
    ioloop.add_callback(ioloop.stop)
    if config['APP_DEBUG']:
        print("Asked Tornado to exit")

def main(config):
    server = start_app(config=config)
    tornado.ioloop.IOLoop.current().start()
    if config['APP_DEBUG']:
        print("Tornado finished")
    server.stop()
    
     
class Server(object):
    def __init__(self, optsargs):
        self.exit = False
        (self.opts, self.args) = optsargs
        self.server_thread = None
        # TODO: fix this config mess... 
        self.config = Configuration(self.opts.configuration)
        self.config = self.config.get_config()
        signal.signal(signal.SIGINT, self.exit_gracefully)
        signal.signal(signal.SIGTERM, self.exit_gracefully)
        
        self.server_thread = threading.Thread(target=main, args = (self.config,))
        self.server_thread.daemon = True
        self.server_thread.start()
        self.main_loop()

    def main_loop(self):
        '''main loop of the api server'''
        while not self.exit:
            self.process_connection_in() # any down time occurs here (ideally we can block on connection read)
        print('Server finished')
        
    def process_connection_in(self):
        '''process data connection(s)'''
        data = None
        # TODO: Replace with abstracted mavros interface
        data = {"data_type":"mavlink", "version":2.0, "system_id":123, "payload":str(uuid4())}
        time.sleep(0.05) # TODO: remove this when we are actually reading something! Ideally callback driven
        
    def exit_gracefully(self, signum, frame):
        '''called on sigterm'''
        self.exit = True
        if self.server_thread:
            # attempt to shutdown the tornado server
            stop_tornado(self.config)
            self.server_thread.join(timeout=10)
        
if __name__ == '__main__':
    from optparse import OptionParser
    from config import Configuration
    parser = OptionParser('maverick_api_server.py [options]')
    
    parser.add_option("--configuration", dest="configuration", type='str',
                      help="configuration file name", default="config.json")
    optsargs = parser.parse_args()
    (opts,args) = optsargs
    
    Server(optsargs)