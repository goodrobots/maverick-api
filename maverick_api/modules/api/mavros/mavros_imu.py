import logging

from maverick_api.modules import api_callback, moduleBase, schemaBase

import rospy
from sensor_msgs.msg import Imu

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


class ImuSchema(schemaBase):
    def __init__(self):
        super().__init__()

        self.imu_data = {"uuid": "test"}

        self.imu_message_type = GraphQLObjectType(
            "Imu",
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
                "orientationX": GraphQLField(GraphQLFloat, description=""),
                "orientationY": GraphQLField(GraphQLFloat, description=""),
                "orientationZ": GraphQLField(GraphQLFloat, description=""),
                "orientationW": GraphQLField(GraphQLFloat, description=""),
                "angularVelocityX": GraphQLField(GraphQLFloat, description=""),
                "angularVelocityY": GraphQLField(GraphQLFloat, description=""),
                "angularVelocityZ": GraphQLField(GraphQLFloat, description=""),
                "linearAccelerationX": GraphQLField(GraphQLFloat, description=""),
                "linearAccelerationY": GraphQLField(GraphQLFloat, description=""),
                "linearAccelerationZ": GraphQLField(GraphQLFloat, description=""),
            },
            description="MAVROS ImuMessage",
        )

        self.q = {
            "Imu": GraphQLField(self.imu_message_type, resolve=self.get_imu_message)
        }

        self.m = {
            "Imu": GraphQLField(
                self.imu_message_type,
                args=self.get_mutation_args(self.imu_message_type),
                resolve=self.set_imu_message,
            )
        }

        self.s = {
            "Imu": GraphQLField(
                self.imu_message_type, subscribe=self.sub_imu_message, resolve=None
            )
        }

    def get_imu_message(self, root, info):
        """ImuMessage query handler"""
        return self.imu_data

    def set_imu_message(self, root, info, **kwargs):
        """ImuMessage mutation handler"""
        updated_dict = {**self.imu_data, **kwargs}
        self.subscriptions.emit(
            "maverick_api.modules.api.mavros.ImuSchema" + "Imu", {"Imu": updated_dict}
        )
        self.imu_data = updated_dict
        return updated_dict

    def sub_imu_message(self, root, info):
        """ImuMessage subscription handler"""
        return EventEmitterAsyncIterator(
            self.subscriptions, "maverick_api.modules.api.mavros.ImuSchema" + "Imu"
        )


class ImuInterface(moduleBase):
    def __init__(self, loop, module):
        super().__init__(loop, module)

        rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)

    def imu_callback(self, data):
        kwargs = {
            "seq": data.header.seq,
            "secs": data.header.stamp.secs,
            "nsecs": data.header.stamp.nsecs,
            "frameId": data.header.frame_id,
            "orientationX": data.orientation.x,
            "orientationY": data.orientation.y,
            "orientationZ": data.orientation.z,
            "orientationW": data.orientation.w,
            "angularVelocityX": data.angular_velocity.x,
            "angularVelocityY": data.angular_velocity.y,
            "angularVelocityZ": data.angular_velocity.z,
            "linearAccelerationX": data.linear_acceleration.x,
            "linearAccelerationY": data.linear_acceleration.y,
            "linearAccelerationZ": data.linear_acceleration.z,
        }
        api_callback(
            self.loop,
            self.module["maverick_api.modules.api.mavros.ImuSchema"].set_imu_message,
            **kwargs,
        )
