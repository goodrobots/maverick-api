import os

from maverick_api.modules import get_api_schema
from maverick_api.modules.base.tornadoql.dev_graphiql_handler import GraphiQLHandler
from maverick_api.modules.base.tornadoql.graphql_handler import GraphQLHandler
from maverick_api.modules.base.tornadoql.subscription_handler import (
    GraphQLSubscriptionHandler,
)
from maverick_api.modules.base.tornadoql.schema_handler import SchemaHandler

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
            compress_response=True,
            websocket_ping_interval=10,
            cookie_secret=options.app_secretkey,
            static_path=os.path.join(options.basedir, "data", "static"),
            xsrf_cookies=False,
        )
        TornadoQL.schema = get_api_schema()
        super(TornadoQL, self).__init__(handlers, **settings)
