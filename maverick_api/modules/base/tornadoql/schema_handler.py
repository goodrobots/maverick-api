import json

# graphql imports
from graphql import graphql
from graphql import get_introspection_query

from maverick_api.modules import generate_schema

import tornado.web


class SchemaHandler(tornado.web.RequestHandler):
    """introspection of the api schema"""

    async def get(self):
        query = get_introspection_query(descriptions=True)
        (api_schema,_)=generate_schema()
        introspection_query_result = await graphql(api_schema, query)
        introspection_dict = introspection_query_result.data
        self.write(json.dumps(introspection_dict, indent=4, sort_keys=True))
        self.finish()
