# This file interacts with the mavros param system to expose the following
# query params
# query param meta data
# update param data
# similar to the mavros mission we can pass a list of params via graphql



import functools
import logging
import time

from modules.api import api_callback, moduleBase, schemaBase

from tornado.options import options

import rospy
from mavros_msgs.msg import Param  # callback msg on param change
from mavros.param import param_ret_value
from modules.base.param.parse_param_xml import get_param_meta
from mavros.param import param_get_all

# graphql imports
from graphql import (
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
from graphql.pyutils.event_emitter import EventEmitter, EventEmitterAsyncIterator

application_log = logging.getLogger("tornado.application")


class ParamSchema(schemaBase):
    def __init__(self):
        super().__init__()

        self.q = {}
        self.m = {}
        self.s = {}


class ParamInterface(moduleBase):
    def __init__(self, loop, module):
        super().__init__(loop, module)

        rospy.Subscriber("/mavros/param_value", Param, self.param_callback)
        self.params = {}
        self.param_meta = {}

    def param_callback(self, data):
        """Called every time mavros receives a parameter from the vehicle"""
        # application_log.debug('param callback data: {0}  value type: {1} '.format(data, type(data.value)))
        kwargs = {"id": data.param_id, "value": param_ret_value(data)}
        api_callback(
            self.loop,
            self.module["modules.api.mavros.ParamSchema"].update_param,
            **kwargs,
        )
        # print(kwargs)

    # The following decorator is only useful if we are supporting more than one vehicle per API instance
    #   Rather than reaching out to calc the meta again we return the cached value for the meta_string
    @functools.lru_cache(maxsize=10)  # cache the param meta for each vehicle
    def vehicle_params(self, meta_string=None):
        """Called once on API start up"""
        # TODO: make vehicle dynamic and chosse between px4 and ardupilot
        # TODO: If this fails try to run it some time in the future

        param_received, param_list = param_get_all(False)
        # TODO move this to a background thread or make it an async call (when ROS supports it)
        application_log.debug(f"Parameters received: {param_received}")

        param_meta_vehicle = {}
        for param in param_list:
            kwargs = {"id": param.param_id, "value": param.param_value}
            # add_ioloop_callback(UpdateParameter().mutate, **kwargs)

            param_meta_vehicle[param.param_id] = {
                "group": param.param_id.split("_")[0].strip().rstrip("_").upper()
            }
            # application_log.debug(
            #     "param get {0}:{1}  {2}".format(
            #         param.param_id, param.param_value, type(param.param_value)
            #     )
            # )

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
