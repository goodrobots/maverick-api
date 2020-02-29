import json

# graphql imports
from graphql import graphql
from graphql import get_introspection_query

from maverick_api.modules import get_api_schema

import tornado.web


class SchemaHandler(tornado.web.RequestHandler):
    """introspection of the api schema"""

    def set_default_headers(self):
        self.set_header("Access-Control-Allow-Origin", "*")
        self.set_header("Access-Control-Allow-Credentials", "true")
        self.set_header(
            "Access-Control-Allow-Headers",
            "Origin, X-Requested-With, Content-Type, Accept, Authorization",
        )
        self.set_header("Access-Control-Allow-Methods", "GET, OPTIONS")

    def options(self):
        self.set_status(204)

    async def get(self):
        query = get_introspection_query(descriptions=True)
        introspection_query_result = await graphql(get_api_schema(), query)
        introspection_dict = introspection_query_result.data
        self.write(json.dumps(introspection_dict, indent=4, sort_keys=True))
        self.finish()
