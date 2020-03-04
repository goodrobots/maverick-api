import logging
import inspect
import pkgutil
from pathlib import Path
from importlib import import_module
import time

from graphql import GraphQLField, GraphQLObjectType, GraphQLSchema
from graphql.pyutils.event_emitter import EventEmitter
from tornado.options import options
import tornado.ioloop

application_log = logging.getLogger("tornado.application")

api_schema = None
module_schema = None
module_base = None
schema_timestamp = None


class moduleBase(object):
    def __init__(self, loop=tornado.ioloop.IOLoop.current(), module=None, **kwargs):
        self.loop = loop
        self._module = module

    @property
    def module(self):
        if self._module:
            return self._module
        return get_module_schema()

    def start(self):
        pass

    def shutdown(self):
        pass


class schemaBase(object):
    def __init__(self, child, **kwargs):
        self.subscription_string = f"{child.__module__}.{child.__class__.__name__}"
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


def check_class(attribute, class_to_check):
    return (
        inspect.isclass(attribute)
        and issubclass(attribute, class_to_check)
        and (attribute.__name__ != class_to_check.__name__)
    )


def check_schema_attribute(instance):
    if isinstance(instance, dict):
        errors = 0
        for k in instance:
            # TODO: Check to ensure we don't already have a handler
            #   by this name (using if key in q)
            if not isinstance(instance[k], GraphQLField):
                # Ensure we are adding the correct types to the
                #   application_schema
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
    global schema_timestamp
    global module_base
    _module_schema = dict()
    _module_base = dict()
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
            if check_class(attribute, schemaBase):
                # create an instance of the schema class
                ref_name = f"{__name__}.{module_folder_name}.{module_name}.{attribute.__name__}"
                application_log.info(f"Appending schema from {ref_name}")
                _module_schema[ref_name] = attribute()
                # add the class schema to the application schema
                (q, m, s) = extend_application_schema(_module_schema, ref_name, q, m, s)
        # search for the module class
        for i in dir(imported_module):
            attribute = getattr(imported_module, i)
            if check_class(attribute, moduleBase):
                # create an instance of the module class
                ref_name = f"{__name__}.{module_folder_name}.{module_name}.{attribute.__name__}"
                application_log.info(f"Creating module instance: {ref_name}")
                _module_base[ref_name] = attribute()

    # TODO: is a lock needed here to avoid a race when get_module_schema()
    #   or get_api_schema() is accessed from an external thread?
    api_schema = GraphQLSchema(
        query=GraphQLObjectType("query", lambda: q),
        mutation=GraphQLObjectType("mutation", lambda: m),
        subscription=GraphQLObjectType("subscription", lambda: s),
    )
    module_schema = _module_schema
    schema_timestamp = int(time.time())
    module_base = _module_base


def start_all_modules():
    if not module_base:
        return False
    for module in module_base:
        application_log.info(f"Starting module instance: {module}")
        module_base[module].start()
    return True


def stop_all_modules():
    if not module_base:
        return False
    for module in module_base:
        module_base[module].shutdown()
    return True


def get_api_schema():
    return api_schema


def get_module_schema():
    return module_schema


def get_modules():
    return module_base


def get_schema_timestamp():
    return schema_timestamp
