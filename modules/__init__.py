# based off https://www.bnmetrics.com/blog/dynamic-import-in-python3
import sys
import inspect
import pkgutil
from pathlib import Path
from importlib import import_module
from graphql import (
    GraphQLField,
    GraphQLObjectType,
    GraphQLSchema
    )
from graphql.pyutils.event_emitter import EventEmitter

class schemaBase(object):
    def __init__(self,):
        self.q = {} 
        self.m = {}
        self.s = {}
        self.subscriptions = EventEmitter()
        
    def get_mutation_args(self, GraphQLObject):
        return {field: GraphQLObject.fields[field].type for field in GraphQLObject.fields}

def check_schema_class(attribute):
    return (inspect.isclass(attribute) and
    issubclass(attribute, schemaBase) and
    (attribute.__name__ != schemaBase.__name__))

def check_schema_attribute(instance):
    if isinstance(instance, dict):
        errors = 0
        for k in instance:
            # TODO: Check to ensure we dont already have a handler
            #     by this name (using if key in q)
            if not isinstance(instance[k], GraphQLField):
                # Ensure we are adding the correct types to the
                #     application_schema
                errors += 1
        if not errors:
            return True
    return False

def extend_application_schema(name, q, m, s):
    schema_target_attributes = ["q", "m", "s"]
    for schema_target_attribute in schema_target_attributes:
        schema_attribute = getattr(module_schema[name], schema_target_attribute)
        if check_schema_attribute(schema_attribute):
            # Extend the application schema with class schema
            if schema_target_attribute == "m":
                m = {**m, **schema_attribute}
            elif schema_target_attribute == "q":
                q = {**q, **schema_attribute}
            elif schema_target_attribute == "s":
                s = {**s, **schema_attribute}
    return (q, m, s)

module_schema = {}
m = dict()
q = dict()
s = dict()

# iterate over all available modules under this folder
for (_, name, _) in pkgutil.iter_modules([Path(__file__).parent]):
    # import the module
    imported_module = import_module(f"{__name__}.{name}")
    # search for the schema class
    for i in dir(imported_module):
        attribute = getattr(imported_module, i)
        if check_schema_class(attribute):
            # create an instance of the schema class
            module_schema[name] = attribute()
            # add the class schema to the application schema
            (q, m, s)  = extend_application_schema(name, q, m, s)

api_schema = GraphQLSchema(
    query=GraphQLObjectType("query", lambda: q),
    mutation=GraphQLObjectType("mutation", lambda: m),
    subscription=GraphQLObjectType("subscription", lambda: s),
) 