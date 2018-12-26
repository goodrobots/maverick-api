import os

from modules.api import api_schema
from modules.base.tornadoql.dev_graphiql_handler import GraphiQLHandler
from modules.base.tornadoql.graphql_handler import GraphQLHandler
from modules.base.tornadoql.subscription_handler import GraphQLSubscriptionHandler
from modules.base.tornadoql.schema_handler import SchemaHandler

import tornado.web
from tornado.options import options


class TornadoQL(tornado.web.Application):
    def __init__(self):
        handlers = [
            (r"/subscriptions", GraphQLSubscriptionHandler),
            (r"/graphql", GraphQLHandler),
            (r"/graphiql", GraphiQLHandler),
            (r"/schema", SchemaHandler),
        ]

        settings = dict(
            debug=options.debug,
            cookie_secret=options.app_secretkey,
            static_path=os.path.join(options.datadir, "static"),
            xsrf_cookies=False,
        )

        TornadoQL.schema = api_schema
        super(TornadoQL, self).__init__(handlers, **settings)
