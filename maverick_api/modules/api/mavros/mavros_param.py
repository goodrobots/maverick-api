# This file interacts with the mavros param system to expose the following
# query params
# query param meta data
# update param data
# similar to the mavros mission we can pass a list of params via graphql


import functools
import logging
import time
import numbers
from decimal import Decimal
import ast

from modules.api import api_callback, moduleBase, schemaBase

from tornado.options import options

import rospy
from mavros_msgs.msg import Param  # callback msg on param change
from mavros.param import param_ret_value
from modules.base.param.parse_param_xml import get_param_meta
from mavros.param import param_get_all

# graphql imports
from graphql import (
    GraphQLScalarType,
    GraphQLArgument,
    GraphQLEnumType,
    GraphQLEnumValue,
    GraphQLField,
    GraphQLInterfaceType,
    GraphQLList,
    GraphQLNonNull,
    GraphQLObjectType,
    GraphQLSchema,
    GraphQLString,
    GraphQLBoolean,
    GraphQLInt,
    GraphQLFloat,
    GraphQLInputObjectType,
    GraphQLInputField,
)
from graphql.error import INVALID
from graphql.pyutils.event_emitter import EventEmitter, EventEmitterAsyncIterator

application_log = logging.getLogger("tornado.application")

MAX_INT = 2_147_483_647
MIN_INT = -2_147_483_648


def serialize_param_value(value):
    if isinstance(value, numbers.Integral):
        return value
    else:
        try:
            value = round(value, 6)
            return ast.literal_eval(format(Decimal(str(value)), "f"))
        except Exception as e:
            application_log.error(f"serialize failed {value}:{type(value)} {e}")
            return INVALID


def coerce_param_value(value):
    if not isinstance(value, numbers.Integral):
        return value
    if not MIN_INT <= value <= MAX_INT:
        raise TypeError(f"Int cannot represent non 32-bit signed integer value")
    return value


def parse_param_value_literal(value_ast, _variables=None):
    if isinstance(value_ast, (ast.StringValue, ast.IntValue, ast.FloatValue)):
        return value_ast.value
    return INVALID


class ParamSchema(schemaBase):
    def __init__(self):
        super().__init__()

        self.parameter_data = {}
        self.parameter_meta = {}

        # the trick with parameters is that they are either floats or ints
        # a custom type is required...
        GraphQLParamValue = GraphQLScalarType(
            name="ParamValue",
            description="",
            serialize=serialize_param_value,
            parse_value=coerce_param_value,
            parse_literal=parse_param_value_literal,
        )

        self.parameter_type = GraphQLObjectType(
            "Parameter",
            lambda: {
                "id": GraphQLField(
                    GraphQLString, description="The id of the parameter"
                ),
                "value": GraphQLField(
                    GraphQLParamValue, description="The value of the parameter"
                ),
            },
            description="Parameter item",
        )

        self.parameter_list_type = GraphQLObjectType(
            "ParameterList",
            lambda: {"parameters": GraphQLField(GraphQLList(self.parameter_type))},
        )

        # TODO: needed?
        self.parameter_input_type = GraphQLInputObjectType(
            "ParameterInput",
            {
                "id": GraphQLInputField(GraphQLNonNull(GraphQLString)),
                "value": GraphQLInputField(GraphQLNonNull(GraphQLParamValue)),
            },
        )

        self.q = {
            "Parameter": GraphQLField(
                self.parameter_type,
                args={
                    "id": GraphQLArgument(
                        GraphQLNonNull(GraphQLString),
                        description="The id of the desired parameter",
                    )
                },
                resolve=self.get_parameter,
            ),
            "ParameterList": GraphQLField(
                self.parameter_list_type,
                args={
                    "query": GraphQLArgument(
                        GraphQLNonNull(GraphQLString),
                        description="The query used to match desired parameters",
                    )
                },
                resolve=self.get_parameter_list,
            ),
        }
        self.m = {
            "Parameter": GraphQLField(
                self.parameter_type,
                args=self.get_mutation_args(self.parameter_type),
                resolve=self.update_parameter,
            ),
            "ParameterList": GraphQLField(
                self.parameter_list_type,
                args={
                    "parameters": GraphQLArgument(
                        GraphQLNonNull(GraphQLList(self.parameter_input_type))
                    )
                },
                resolve=self.update_parameter_list,
            ),
        }

        self.s = {
            "Parameter": GraphQLField(
                self.parameter_type, subscribe=self.sub_parameter, resolve=None
            ),
            "ParameterList": GraphQLField(
                self.parameter_list_type,
                subscribe=self.sub_parameter_list,
                resolve=None,
            ),
        }

    def get_parameter(self, root, info, **kwargs):
        application_log.debug(f"Parameter query handler {kwargs}")
        param_id = kwargs["id"]
        param_value = self.parameter_data.get(param_id, None)
        # TODO: handle none error...
        return {"id": param_id, "value": param_value}

    def update_parameter(self, root, info, **kwargs):
        application_log.debug(f"Parameter mutation handler {kwargs}")
        parameter_id = kwargs.get("id")
        parameter_value = kwargs.get("value")
        self.parameter_data[parameter_id] = parameter_value

    def sub_parameter(self, root, info):
        application_log.debug(f"Parameter subscription handler")
        return EventEmitterAsyncIterator(
            self.subscriptions, "modules.api.mavros.ParamSchema" + "Parameter"
        )

    def get_parameter_list(self, root, info, **kwargs):
        application_log.debug(f"Parameter list query handler {kwargs}")
        tmp = []
        for k in self.parameter_data.keys():
            tmp.append({"id": k, "value": self.parameter_data[k]})
        tmp.sort(key=lambda i: i["id"])
        return {"parameters": tmp}

    def update_parameter_list(self, root, info, **kwargs):
        application_log.debug(f"Parameter list mutation handler {kwargs}")
        pass

    def sub_parameter_list(self, root, info):
        application_log.debug(f"Parameter list subscription handler")
        return EventEmitterAsyncIterator(
            self.subscriptions, "modules.api.mavros.ParamSchema" + "ParameterList"
        )


class ParamInterface(moduleBase):
    def __init__(self, loop, module):
        super().__init__(loop, module)

        rospy.Subscriber("/mavros/param_value", Param, self.param_callback)
        self.params = {}
        self.param_meta = {}

    def param_callback(self, data):
        """Called every time mavros receives a parameter from the vehicle"""
        # application_log.debug('param callback data: {0}  value type: {1} '.format(data, type(data.value)))
        kwargs = {"id": data.param_id, "value": data.param_value}
        api_callback(
            self.loop,
            self.module["modules.api.mavros.ParamSchema"].update_parameter,
            **kwargs,
        )

    # The following decorator is only useful if we are supporting more than one vehicle per API instance
    #   Rather than reaching out to calc the meta again we return the cached value for the meta_string
    @functools.lru_cache(maxsize=10)  # cache the param meta for each vehicle
    def vehicle_params(self, meta_string=None):
        """Called once on API start up"""
        # TODO: make vehicle dynamic and chosse between px4 and ardupilot
        # TODO: If this fails try to run it some time in the future

        param_received, param_list = param_get_all(True)
        # TODO move this to a background thread or make it an async call (when ROS supports it)
        application_log.debug(f"Parameters received: {param_received}")

        param_meta_vehicle = {}
        for param in param_list:
            kwargs = {"id": param.param_id, "value": param.param_value}
            self.param_callback(param)

            param_meta_vehicle[param.param_id] = {
                "group": param.param_id.split("_")[0].strip().rstrip("_").upper()
            }
            application_log.debug(
                f"param get {param.param_id}:{param.param_value}  {type(param.param_value)}"
            )

        # TODO: handle IOError when mavlink-router is not connected to the AP but mavros is running

        start_time = time.time()
        application_log.debug("starting parameter meta fetch")
        param_meta_server = get_param_meta(meta_string)
        application_log.debug("finished parameter meta fetch")
        application_log.debug(f"parameter meta fetch took {time.time() - start_time}s")
        self.param_meta = {**param_meta_vehicle, **param_meta_server}
        # application_log.info(
        #     f"self.param_meta {self.param_meta}"
        # )
