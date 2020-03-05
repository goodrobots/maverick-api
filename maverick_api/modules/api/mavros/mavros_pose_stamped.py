import logging

from maverick_api.modules import api_callback, moduleBase, schemaBase

import rospy
from geometry_msgs.msg import PoseStamped

# graphql imports
from graphql import (
    GraphQLField,
    GraphQLObjectType,
    GraphQLString,
    GraphQLInt,
    GraphQLFloat,
)
from graphql.pyutils.event_emitter import EventEmitterAsyncIterator

application_log = logging.getLogger("tornado.application")


class PoseStampedSchema(schemaBase):
    def __init__(self):
        super().__init__(self)
        self.pose_data = {"uuid": "test"}

        self.pose_stamped_message_type = GraphQLObjectType(
            "PoseStamped",
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
                "posePositionX": GraphQLField(GraphQLFloat, description=""),
                "posePositionY": GraphQLField(GraphQLFloat, description=""),
                "posePositionZ": GraphQLField(GraphQLFloat, description=""),
                "poseOrientationX": GraphQLField(GraphQLFloat, description=""),
                "poseOrientationY": GraphQLField(GraphQLFloat, description=""),
                "poseOrientationZ": GraphQLField(GraphQLFloat, description=""),
                "poseOrientationW": GraphQLField(GraphQLFloat, description=""),
            },
            description="MAVROS PoseStampedMessage",
        )

        self.q = {
            "PoseStamped": GraphQLField(
                self.pose_stamped_message_type, resolve=self.get_pose_stamped_message
            )
        }

        self.m = {
            "PoseStamped": GraphQLField(
                self.pose_stamped_message_type,
                args=self.get_mutation_args(self.pose_stamped_message_type),
                resolve=self.set_pose_stamped_message,
            )
        }

        self.s = {
            "PoseStamped": GraphQLField(
                self.pose_stamped_message_type,
                subscribe=self.sub_pose_stamped_message,
                resolve=None,
            )
        }

    def get_pose_stamped_message(self, root, info):
        """PoseStampedMessage query handler"""
        return self.pose_data

    def set_pose_stamped_message(self, root, info, **kwargs):
        """PoseStampedMessage mutation handler"""
        updated_dict = {**self.pose_data, **kwargs}
        self.subscriptions.emit(
            "maverick_api.modules.api.mavros.PoseStampedSchema" + "PoseStamped",
            {"PoseStamped": updated_dict},
        )
        self.pose_data = updated_dict
        return updated_dict

    def sub_pose_stamped_message(self, root, info):
        """PoseStampedMessage subscription handler"""
        return EventEmitterAsyncIterator(
            self.subscriptions,
            "maverick_api.modules.api.mavros.PoseStampedSchema" + "PoseStamped",
        )


class PoseStampedInterface(moduleBase):
    def __init__(self, loop, module):
        super().__init__(loop, module)

        rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.pose_stamped_callback
        )

    def pose_stamped_callback(self, data):
        kwargs = {
            "seq": data.header.seq,
            "secs": data.header.stamp.secs,
            "nsecs": data.header.stamp.nsecs,
            "frameId": data.header.frame_id,
            "posePositionX": data.pose.position.x,
            "posePositionY": data.pose.position.y,
            "posePositionZ": data.pose.position.z,
            "poseOrientationX": data.pose.orientation.x,
            "poseOrientationY": data.pose.orientation.y,
            "poseOrientationZ": data.pose.orientation.z,
            "poseOrientationW": data.pose.orientation.w,
        }
        api_callback(
            self.loop,
            self.module[
                "maverick_api.modules.api.mavros.PoseStampedSchema"
            ].set_pose_stamped_message,
            **kwargs,
        )
