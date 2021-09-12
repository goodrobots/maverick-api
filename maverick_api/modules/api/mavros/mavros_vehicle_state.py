import logging

from maverick_api.modules import api_callback, moduleBase, schemaBase

import rospy
from mavros_msgs.msg import State

# graphql imports
from graphql import (
    GraphQLField,
    GraphQLObjectType,
    GraphQLString,
    GraphQLBoolean,
    GraphQLInt,
)
from graphql.pyutils.simple_pub_sub import SimplePubSubIterator

application_log = logging.getLogger("tornado.application")


class VehicleStateSchema(schemaBase):
    def __init__(self):
        super().__init__(self)

        self.state_data = {"uuid": "test"}

        self.state_message_type = GraphQLObjectType(
            "VehicleState",
            lambda: {
                "uuid": GraphQLField(
                    GraphQLString, description="The uuid of the vehicle."
                ),
                "seq": GraphQLField(
                    GraphQLInt, description="The sequence number of the message."
                ),
                "secs": GraphQLField(GraphQLInt, description=""),
                "nsecs": GraphQLField(GraphQLInt, description=""),
                "frameId": GraphQLField(GraphQLString, description=""),
                "connected": GraphQLField(GraphQLBoolean, description=""),
                "armed": GraphQLField(GraphQLBoolean, description=""),
                "guided": GraphQLField(GraphQLBoolean, description=""),
                "mode": GraphQLField(GraphQLString, description=""),
                "systemStatus": GraphQLField(GraphQLInt, description=""),
            },
            description="MAVROS StateMessage",
        )

        self.q = {
            "VehicleState": GraphQLField(
                self.state_message_type, resolve=self.get_state_message
            )
        }

        self.m = {
            "VehicleState": GraphQLField(
                self.state_message_type,
                args=self.get_mutation_args(self.state_message_type),
                resolve=self.set_state_message,
            )
        }

        self.s = {
            "VehicleState": GraphQLField(
                self.state_message_type, subscribe=self.sub_state_message, resolve=None
            )
        }

    def get_state_message(self, root, info):
        """StateMessage query handler"""
        return self.state_data

    def set_state_message(self, root, info, **kwargs):
        """StateMessage mutation handler"""
        updated_dict = {**self.state_data, **kwargs}
        self.subscriptions.emit(
            "maverick_api.modules.api.mavros.VehicleStateSchema" + "VehicleState",
            {"VehicleState": updated_dict},
        )
        self.state_data = updated_dict
        return updated_dict

    def sub_state_message(self, root, info):
        """StateMessage subscription handler"""
        return SimplePubSubIterator(
            self.subscriptions,
            "maverick_api.modules.api.mavros.VehicleStateSchema" + "VehicleState",
        )


class VehicleStateInterface(moduleBase):
    def __init__(self, loop, module):
        super().__init__(loop, module)

        rospy.Subscriber("/mavros/state", State, self.state_callback)

    def state_callback(self, data):
        kwargs = {
            "seq": data.header.seq,
            "frameId": data.header.frame_id,
            "guided": data.guided,
            "nsecs": data.header.stamp.nsecs,
            "systemStatus": data.system_status,
            "secs": data.header.stamp.secs,
            "connected": data.connected,
            "mode": data.mode,
            "armed": data.armed,
        }
        api_callback(
            self.loop,
            self.module[
                "maverick_api.modules.api.mavros.VehicleStateSchema"
            ].set_state_message,
            **kwargs,
        )
