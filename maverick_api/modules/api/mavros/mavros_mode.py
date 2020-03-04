import logging
import time

import rospy
import mavros
from mavros_msgs.srv import SetMode
from mavros_msgs.msg import State
from maverick_api.modules import api_callback, moduleBase, schemaBase

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
)
from graphql.pyutils.event_emitter import EventEmitter, EventEmitterAsyncIterator

application_log = logging.getLogger("tornado.application")


class ModeSchema(schemaBase):
    def __init__(self):
        super().__init__()
        self.mode_data = {}
        mavros.set_namespace("mavros")
        # register mode control
        self.mode_control = rospy.ServiceProxy(mavros.get_topic("set_mode"), SetMode)

        self.mode_control_type = GraphQLObjectType(
            "Mode",
            lambda: {
                "uuid": GraphQLField(
                    GraphQLString, description="The uuid of the vehicle"
                ),
                "modeString": GraphQLField(GraphQLString, description=""),
                "baseMode": GraphQLField(GraphQLInt, description=""),
                "customMode": GraphQLField(GraphQLInt, description=""),
                "updateTime": GraphQLField(GraphQLInt, description=""),
                "returncode": GraphQLField(GraphQLInt, description=""),
            },
            description="Mode control",
        )

        self.q = {
            "Mode": GraphQLField(
                self.mode_control_type,
                args={
                    "uuid": GraphQLArgument(
                        GraphQLNonNull(GraphQLString),
                        description="The uuid of the target vehicle",
                    )
                },
                resolve=self.get_mode,
            )
        }

        self.m = {
            "Mode": GraphQLField(
                self.mode_control_type,
                args=self.get_mutation_args(self.mode_control_type),
                resolve=self.update_mode,
            )
        }

        self.s = {
            "Mode": GraphQLField(
                self.mode_control_type, subscribe=self.sub_mode, resolve=None
            )
        }

    def get_mode(self, root, info, **kwargs):
        """Mode query handler"""
        vehicle_uuid = kwargs.get("uuid")  # UUID is required
        # FIXME: remove me
        vehicle_uuid = list(self.mode_data.keys())[0]
        mode_info = self.mode_data.get(vehicle_uuid, {})
        return mode_info

    def update_mode(self, root, info, **kwargs):
        """Mode mutation handler"""
        if (root is None) and (info is None):
            # The call came from the api, dont action as a mode change request
            self.mode_data["test"] = kwargs
            self.subscriptions.emit(
                "maverick_api.modules.api.mavros.ModeSchema" + "Mode", {"Mode": kwargs}
            )
        else:
            # The web has asked for a mode change
            # TODO: handle / enforce mode
            # TODO: require uuid
            base_mode_input = None
            custom_mode_input = None
            base_mode = kwargs.get("baseMode", None)
            custom_mode = kwargs.get("customMode", None)
            mode_string = kwargs.get("modeString", None)

            if not mode_string:
                base_mode_input = base_mode
                custom_mode_input = kwargs.get("customMode", None)
            else:
                base_mode_input = 0
                custom_mode_input = mode_string.strip().upper()

            try:
                ret = self.mode_control(
                    base_mode=base_mode_input, custom_mode=custom_mode_input
                )
                mode = {
                    "uuid": "test",
                    "modeString": mode_string,
                    "baseMode": base_mode,
                    "customMode": custom_mode,
                    "updateTime": int(time.time()),
                    "returncode": ret.mode_sent,
                }

                self.subscriptions.emit(
                    "maverick_api.modules.api.mavros.ModeSchema" + "Mode", {"Mode": mode}
                )
                return mode

            except rospy.ServiceException as ex:
                application_log.error(
                    f"An error occurred while attempting to set vehicle mode via ROS: {ex}"
                )

    def sub_mode(self, root, info):
        """mode subscription handler"""
        return EventEmitterAsyncIterator(
            self.subscriptions, "maverick_api.modules.api.mavros.ModeSchema" + "Mode"
        )


class ModeInterface(moduleBase):
    def __init__(self, loop, module):
        super().__init__(loop, module)

        rospy.Subscriber("/mavros/state", State, self.mode_state_callback)

    def mode_state_callback(self, data):
        kwargs = {
            "uuid": "test",
            "modeString": data.mode,
            "updateTime": int(time.time()),
        }
        api_callback(
            self.loop,
            self.module["maverick_api.modules.api.mavros.ModeSchema"].update_mode,
            **kwargs,
        )
