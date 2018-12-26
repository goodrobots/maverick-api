# graphql imports
from graphql import graphql
from graphql import get_introspection_query

from modules.api import module_schema, api_schema

import tornado.web


class SchemaHandler(tornado.web.RequestHandler):
    """introspection of the api schema"""

    async def get(self):
        query = get_introspection_query(descriptions=True)
        introspection_query_result = await graphql(api_schema, query)
        introspection_dict = introspection_query_result.data
        self.write(json.dumps(introspection_dict, indent=4, sort_keys=True))
        self.finish()
