import logging

from maverick_api.modules import api_callback, moduleBase, schemaBase

import rospy
from mavros_msgs.msg import VFR_HUD
from std_msgs.msg import Float64

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


class VfrHudSchema(schemaBase):
    def __init__(self):
        super().__init__(self)
        self.vfr_hud_data = {"uuid": "test"}

        self.vfr_hud_message_type = GraphQLObjectType(
            "VfrHud",
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
                "airspeed": GraphQLField(GraphQLFloat, description=""),
                "groundspeed": GraphQLField(GraphQLFloat, description=""),
                "heading": GraphQLField(GraphQLInt, description=""),
                "throttle": GraphQLField(GraphQLFloat, description=""),
                "altitude": GraphQLField(GraphQLFloat, description=""),
                "relativeAltitude": GraphQLField(
                    GraphQLFloat,
                    description="Current altitude relative to origin altitude",
                ),
                "climb": GraphQLField(GraphQLFloat, description=""),
            },
            description="MAVROS VfrHudMessage",
        )

        self.q = {
            "VfrHud": GraphQLField(
                self.vfr_hud_message_type, resolve=self.get_vfr_hud_message
            )
        }

        self.m = {
            "VfrHud": GraphQLField(
                self.vfr_hud_message_type,
                args=self.get_mutation_args(self.vfr_hud_message_type),
                resolve=self.set_vfr_hud_message,
            )
        }

        self.s = {
            "VfrHud": GraphQLField(
                self.vfr_hud_message_type,
                subscribe=self.sub_vfr_hud_message,
                resolve=None,
            )
        }

    def get_vfr_hud_message(self, root, info):
        """VfrHudMessage query handler"""
        return self.vfr_hud_data

    def set_vfr_hud_message(self, root, info, **kwargs):
        """VfrHudMessage mutation handler"""
        updated_dict = {**self.vfr_hud_data, **kwargs}
        self.subscriptions.emit(
            "maverick_api.modules.api.mavros.VfrHudSchema" + "VfrHud",
            {"VfrHud": updated_dict},
        )
        self.vfr_hud_data = updated_dict
        return updated_dict

    def sub_vfr_hud_message(self, root, info):
        """VfrHudMessage subscription handler"""
        return EventEmitterAsyncIterator(
            self.subscriptions,
            "maverick_api.modules.api.mavros.VfrHudSchema" + "VfrHud",
        )


class VfrHudInterface(moduleBase):
    def __init__(self, loop, module):
        super().__init__(loop, module)

        rospy.Subscriber("/mavros/vfr_hud", VFR_HUD, self.vfr_hud_callback)
        rospy.Subscriber(
            "/mavros/global_position/rel_alt", Float64, self.rel_alt_callback
        )

    def vfr_hud_callback(self, data):
        kwargs = {
            "seq": data.header.seq,
            "secs": data.header.stamp.secs,
            "nsecs": data.header.stamp.nsecs,
            "frameId": data.header.frame_id,
            "airspeed": data.airspeed,
            "groundspeed": data.groundspeed,
            "heading": data.heading,
            "throttle": data.throttle,
            "altitude": data.altitude,
            "climb": data.climb,
        }
        api_callback(
            self.loop,
            self.module[
                "maverick_api.modules.api.mavros.VfrHudSchema"
            ].set_vfr_hud_message,
            **kwargs,
        )

    def rel_alt_callback(self, data):
        kwargs = {"relativeAltitude": data.data}
        api_callback(
            self.loop,
            self.module[
                "maverick_api.modules.api.mavros.VfrHudSchema"
            ].set_vfr_hud_message,
            **kwargs,
        )
