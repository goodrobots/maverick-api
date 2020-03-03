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
import json
import re, fnmatch
from enum import Enum

from maverick_api.modules import api_callback, moduleBase, schemaBase

from tornado.options import options

import rospy
from mavros_msgs.msg import Param  # callback msg on param change

# from mavros.param import param_ret_value
from maverick_api.modules.base.param.parse_param_xml import get_param_meta
from mavros.param import param_get_all
from mavros.param import param_set
from mavros.param import param_get

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
from graphql.language.ast import FloatValueNode, IntValueNode, StringValueNode
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
    if isinstance(value_ast, (StringValueNode, IntValueNode, FloatValueNode)):
        return float(value_ast.value)
    return INVALID


class ParamSchema(schemaBase):
    def __init__(self):
        super(ParamSchema, self).__init__()

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

        self.parameter_meta_input = GraphQLInputObjectType(
            "MetaInput",
            lambda: {
                "humanName": GraphQLInputField(GraphQLString),
                "humanGroup": GraphQLInputField(GraphQLString),
                "documentation": GraphQLInputField(GraphQLString),
                "group": GraphQLInputField(GraphQLString),
                "increment": GraphQLInputField(GraphQLParamValue),
                "min": GraphQLInputField(GraphQLParamValue),
                "max": GraphQLInputField(GraphQLParamValue),
                "decimal": GraphQLInputField(GraphQLParamValue),
                "rebootRequired": GraphQLInputField(GraphQLBoolean),
                "unitText": GraphQLInputField(GraphQLString),
                "units": GraphQLInputField(GraphQLString),
                "bitmask": GraphQLInputField(GraphQLString),
                "values": GraphQLInputField(GraphQLString),
                "type": GraphQLInputField(GraphQLString),
            },
        )
        self.parameter_meta_type = GraphQLObjectType(
            "Meta",
            lambda: {
                "humanName": GraphQLField(GraphQLString, description=""),
                "humanGroup": GraphQLField(GraphQLString, description=""),
                "documentation": GraphQLField(GraphQLString, description=""),
                "group": GraphQLField(GraphQLString, description=""),
                "increment": GraphQLField(GraphQLParamValue, description=""),
                "min": GraphQLField(GraphQLParamValue, description=""),
                "max": GraphQLField(GraphQLParamValue, description=""),
                "decimal": GraphQLField(GraphQLParamValue, description=""),
                "rebootRequired": GraphQLField(GraphQLBoolean, description=""),
                "unitText": GraphQLField(GraphQLString, description=""),
                "units": GraphQLField(GraphQLString, description=""),
                "bitmask": GraphQLField(GraphQLString, description=""),
                "values": GraphQLField(GraphQLString, description=""),
                "type": GraphQLField(GraphQLString, description=""),
            },
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
                "meta": GraphQLField(self.parameter_meta_type),
            },
            description="Parameter item",
        )

        self.parameter_list_type = GraphQLObjectType(
            "ParameterList",
            lambda: {"parameters": GraphQLField(GraphQLList(self.parameter_type))},
        )

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
                        description="The id of the desired parameter.",
                    )
                },
                resolve=self.get_parameter,
            ),
            "ParameterList": GraphQLField(
                self.parameter_list_type,
                args={
                    "query": GraphQLArgument(
                        GraphQLNonNull(GraphQLString),
                        description="The query used to match desired parameters. * can be used as a wildcard.",
                    )
                },
                resolve=self.get_parameter_list,
            ),
        }

        self.m = {
            "Parameter": GraphQLField(
                self.parameter_type,
                args={
                    "id": GraphQLArgument(GraphQLNonNull(GraphQLString)),
                    "value": GraphQLArgument(GraphQLNonNull(GraphQLParamValue)),
                    "meta": GraphQLArgument(self.parameter_meta_input),
                },
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
            )
        }

    def get_parameter(self, root, info, **kwargs):
        application_log.debug(f"Parameter query handler {kwargs}")
        parameter_id = kwargs.get("id")
        param_value = self.parameter_data.get(parameter_id, None)
        # TODO: handle none error...
        return {
            "id": parameter_id,
            "value": param_value,
            "meta": self.parameter_meta.get(parameter_id, None),
        }

    def update_parameter(self, root, info, **kwargs):
        parameter_id = kwargs.get("id")
        parameter_value = kwargs.get("value")
        parameter_meta = self.parameter_meta.get(parameter_id, None)
        parameter_type = kwargs.get("type", None)
        if parameter_type and parameter_meta:
            self.parameter_meta[parameter_id]["type"] = parameter_type
            parameter_meta = self.parameter_meta.get(parameter_id, None)

        application_log.debug(
            f"Parameter mutation handler {kwargs} root:{root} info:{info} meta:{parameter_meta}"
        )
        if (root is None) and (info is None):
            # The call came from the api, don't action as a param change request
            self.parameter_data[parameter_id] = parameter_value
            self.subscriptions.emit(
                "modules.api.mavros.ParamSchema" + "Parameter",
                {
                    "Parameter": {
                        "id": parameter_id,
                        "value": parameter_value,
                        "meta": parameter_meta,
                    }
                },
            )
        else:
            ret = param_set(parameter_id, float(parameter_value))
            if ret == parameter_value:
                # param set worked
                pass
            else:
                # param set failed
                pass

            # check to see if the set value matches the provided value
            ret_param = param_get(parameter_id)
            application_log.debug(
                "param set {0}:{1}  {2}".format(parameter_id, parameter_value, ret)
            )
            application_log.debug("{0}".format(ret_param))

            return {"id": parameter_id, "value": parameter_value}

    def sub_parameter(self, root, info):
        application_log.debug(f"Parameter subscription handler")
        return EventEmitterAsyncIterator(
            self.subscriptions, "modules.api.mavros.ParamSchema" + "Parameter"
        )

    def get_parameter_list(self, root, info, **kwargs):
        application_log.debug(f"Parameter list query handler {kwargs}")
        q = kwargs.get("query", "*")
        tmp = []
        if q == "*":
            # we want all the parameters
            for k in self.parameter_data.keys():
                tmp.append(
                    {
                        "id": k,
                        "value": self.parameter_data[k],
                        "meta": self.parameter_meta.get(k, None),
                    }
                )
        elif "*" in q:
            # the query contains at least one wildcard
            # create a regular expression from a unix style wildcard pattern
            regex = fnmatch.translate(q.upper())
            # compile the regular expression for speed
            reobj = re.compile(regex)
            for k in self.parameter_data.keys():
                # check to see if pattern is present and the matched parameter is not already in our param_list
                if reobj.match(k) and self.parameter_data[k] not in tmp:
                    tmp.append(
                        {
                            "id": k,
                            "value": self.parameter_data[k],
                            "meta": self.parameter_meta.get(k, None),
                        }
                    )
        else:
            # try to match the supplied id against the parameter list
            # if it cant be found return the query id with a null value
            value = self.parameter_data.get(q.upper(), None)
            meta = self.parameter_meta.get(q.upper(), None)
            if value is not None:
                tmp.append({"id": q.upper(), "value": value, "meta": meta})
        if tmp:
            tmp.sort(key=lambda i: i["id"])
        return {"parameters": tmp}

    def update_parameter_list(self, root, info, **kwargs):
        application_log.debug(f"Parameter list mutation handler {kwargs}")
        # TODO iterate over the list and set all the values
        params = kwargs.get("parameters", [])
        ret_param_list = []
        for param in params:
            parameter_id = param.get("id")
            parameter_value = float(param.get("value"))
            ret = param_set(parameter_id, parameter_value)
            if ret == parameter_value:
                # param set worked
                pass
            else:
                # param set failed
                pass

            # check to see if the set value matches the provided value
            ret_param = param_get(parameter_id)
            application_log.debug(
                f"param set {parameter_id}:{parameter_value}->{ret_param}  {ret}"
            )
            if ret:
                ret_param_list.append(
                    {
                        "id": parameter_id,
                        "value": ret_param,
                        "meta": self.parameter_meta.get(parameter_id, None),
                    }
                )

        return {"parameters": ret_param_list}

    def update_parameter_meta(self, root, info, **kwargs):
        self.parameter_meta = kwargs
        # application_log.info(f"self.parameter_meta {self.parameter_meta}")


class ParamInterface(moduleBase):
    def __init__(self, loop, module, **kwargs):
        super(ParamInterface, self).__init__(loop, module, **kwargs)
        # we call the vehicle params first to gather the meta before pushing
        #  changes automatically via the mavros subscription
        self.vehicle_params(meta_string=kwargs.get("meta_string", None))
        rospy.Subscriber("/mavros/param/param_value", Param, self.param_callback)

    def param_callback(self, data):
        """Called every time mavros receives a parameter from the vehicle"""
        val_int = data.value.integer
        val_float = data.value.real
        param_value = val_int
        param_type = "INTEGER"
        if val_float:
            param_value = val_float
            param_type = "FLOAT"
        elif val_int:
            param_value = val_int
            param_type = "INTEGER"
        kwargs = {"id": data.param_id, "value": param_value, "type": param_type}
        self.param_callback_final(**kwargs)

    def param_callback_final(self, **kwargs):
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
        # TODO: make vehicle dynamic and choose between px4 and ardupilot
        # TODO: If this fails try to run it some time in the future

        param_received, param_list = param_get_all(True)
        # TODO move this to a background thread or make it an async call (when ROS supports it)
        application_log.debug(f"Parameters received: {param_received}")

        param_meta_vehicle = {}
        for param in param_list:
            param_meta_vehicle[param.param_id] = {
                "group": param.param_id.split("_")[0].strip().rstrip("_").upper()
            }

        # TODO: handle IOError when mavlink-router is not connected to the AP but mavros is running

        start_time = time.time()
        application_log.debug("starting parameter meta fetch")
        param_meta_server = get_param_meta(meta_string)
        application_log.debug("finished parameter meta fetch")
        application_log.debug(f"parameter meta fetch took {time.time() - start_time}s")
        param_meta = {**param_meta_vehicle, **param_meta_server}
        # sort the bitmask and values by the keys
        # at the same time turn the sorted dict into a string
        for param_key in param_meta:
            param_meta[param_key]["type"] = "NUMBER"
            values = param_meta[param_key].get("values", None)
            if values:
                sorted_values = {values[k]: k for k in sorted(values)}
                param_meta[param_key]["values"] = json.dumps(sorted_values)
            bitmask = param_meta[param_key].get("bitmask", None)
            if bitmask:
                sorted_bitmask = {bitmask[k]: k for k in sorted(bitmask)}
                param_meta[param_key]["bitmask"] = json.dumps(sorted_bitmask)
                param_meta[param_key]["type"] = "BITMASK"

        # application_log.info(f"self.param_meta {self.param_meta}")
        api_callback(
            self.loop,
            self.module["modules.api.mavros.ParamSchema"].update_parameter_meta,
            **param_meta,
        )

        for param in param_list:
            kwargs = {"id": param.param_id, "value": param.param_value}
            self.param_callback_final(**kwargs)
