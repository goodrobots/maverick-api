import logging
import inspect
import pkgutil
from pathlib import Path
from importlib import import_module

from graphql import GraphQLField, GraphQLObjectType, GraphQLSchema
from graphql.pyutils.event_emitter import EventEmitter
from tornado.options import options

application_log = logging.getLogger("tornado.application")

api_schema = None
module_schema = None


class moduleBase(object):
    def __init__(self, loop, module, **kwargs):
        # Attributes
        self.loop = loop
        self.module = module


class schemaBase(object):
    def __init__(self, **kwargs):
        self.q = {}
        self.m = {}
        self.s = {}
        self.subscriptions = EventEmitter()

    def get_mutation_args(self, GraphQLObject):
        return {
            field: GraphQLObject.fields[field].type for field in GraphQLObject.fields
        }


def api_callback(loop, func, **kwargs):
    loop.add_callback(func, None, None, **kwargs)


def check_schema_class(attribute):
    return (
        inspect.isclass(attribute)
        and issubclass(attribute, schemaBase)
        and (attribute.__name__ != schemaBase.__name__)
    )


def check_schema_attribute(instance):
    if isinstance(instance, dict):
        errors = 0
        for k in instance:
            # TODO: Check to ensure we don't already have a handler
            #     by this name (using if key in q)
            if not isinstance(instance[k], GraphQLField):
                # Ensure we are adding the correct types to the
                #     application_schema
                errors += 1
        if not errors:
            return True
    return False


def extend_application_schema(module_schema, ref_name, q, m, s):
    schema_target_attributes = ["q", "m", "s"]
    for schema_target_attribute in schema_target_attributes:
        schema_attribute = getattr(module_schema[ref_name], schema_target_attribute)
        if check_schema_attribute(schema_attribute):
            # Extend the application schema with class schema
            if schema_target_attribute == "m":
                m = {**m, **schema_attribute}
            elif schema_target_attribute == "q":
                q = {**q, **schema_attribute}
            elif schema_target_attribute == "s":
                s = {**s, **schema_attribute}
    return (q, m, s)


def generate_schema():
    global module_schema
    global api_schema
    module_schema = {}
    m = dict()
    q = dict()
    s = dict()

    base_module_path = Path(__file__).resolve().parent
    api_module_path = base_module_path.joinpath("api").resolve()
    custom_module_path = base_module_path.joinpath("custom").resolve()
    module_paths = [api_module_path, custom_module_path]
    # iterate over all available modules under this folder
    for (module_folder_finder, module_name, _) in pkgutil.iter_modules(module_paths):
        module_folder_name = module_folder_finder.path.name
        # check to see if the module name is in the module deny list to prevent imports
        if options.module_allow_list and module_name in options.module_allow_list:
            application_log.warning(
                f"Module allow list match. Loading module: {module_name}"
            )
        elif module_name in options.module_deny_list:
            # don't import or append to the global schema
            application_log.warning(
                f"Module deny list match. Not loading module: {module_name}"
            )
            continue
        # otherwise, import the module
        imported_module = import_module(
            f"{__name__}.{module_folder_name}.{module_name}"
        )
        application_log.info(f"Loading module: {module_folder_name}.{module_name}")
        # search for the schema class
        for i in dir(imported_module):
            attribute = getattr(imported_module, i)
            if check_schema_class(attribute):
                # create an instance of the schema class
                ref_name = f"{__name__}.{module_folder_name}.{module_name}.{attribute.__name__}"
                # print(f"{ref_name}", attribute)
                module_schema[ref_name] = attribute()
                # add the class schema to the application schema
                (q, m, s) = extend_application_schema(module_schema, ref_name, q, m, s)
    api_schema = GraphQLSchema(
        query=GraphQLObjectType("query", lambda: q),
        mutation=GraphQLObjectType("mutation", lambda: m),
        subscription=GraphQLObjectType("subscription", lambda: s),
    )
    # return (api_schema, module_schema)


def get_api_schema():
    return api_schema


def get_module_schema():
    return module_schema
