import logging

from maverick_api.modules import api_callback, moduleBase, schemaBase

import rospy
from sensor_msgs.msg import NavSatFix

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


class NavSatFixSchema(schemaBase):
    def __init__(self):
        super().__init__(self)
        self.nav_sat_fix_data = {"uuid": "test"}

        self.nav_sat_fix_message_type = GraphQLObjectType(
            "NavSatFix",
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
                "statusStatus": GraphQLField(GraphQLInt, description=""),
                "statusService": GraphQLField(GraphQLInt, description=""),
                "latitude": GraphQLField(GraphQLFloat, description=""),
                "longitude": GraphQLField(GraphQLFloat, description=""),
                "altitude": GraphQLField(GraphQLFloat, description=""),
                # TODO: position_covariance array
                "positionCovarianceType": GraphQLField(GraphQLInt, description=""),
            },
            description="MAVROS NavSatFixMessage",
        )

        self.q = {
            "NavSatFix": GraphQLField(
                self.nav_sat_fix_message_type, resolve=self.get_nav_sat_fix_message
            )
        }

        self.m = {
            "NavSatFix": GraphQLField(
                self.nav_sat_fix_message_type,
                args=self.get_mutation_args(self.nav_sat_fix_message_type),
                resolve=self.set_nav_sat_fix_message,
            )
        }

        self.s = {
            "NavSatFix": GraphQLField(
                self.nav_sat_fix_message_type,
                subscribe=self.sub_nav_sat_fix_message,
                resolve=None,
            )
        }

    # @GraphQLSession.authenticated(RBAC="FIXME")
    def get_nav_sat_fix_message(self, root, info):
        """NavSatFixMessage query handler"""
        return self.nav_sat_fix_data

    def set_nav_sat_fix_message(self, root, info, **kwargs):
        """NavSatFixMessage mutation handler"""
        updated_dict = {**self.nav_sat_fix_data, **kwargs}
        self.subscriptions.emit(
            "maverick_api.modules.api.mavros.NavSatFixSchema" + "NavSatFix",
            {"NavSatFix": updated_dict},
        )
        self.nav_sat_fix_data = updated_dict
        return updated_dict

    def sub_nav_sat_fix_message(self, root, info):
        """NavSatFixMessage subscription handler"""
        return EventEmitterAsyncIterator(
            self.subscriptions,
            "maverick_api.modules.api.mavros.NavSatFixSchema" + "NavSatFix",
        )


class NavSatFixInterface(moduleBase):
    def __init__(self, loop, module):
        super().__init__(loop, module)

        rospy.Subscriber(
            "/mavros/global_position/global", NavSatFix, self.nav_sat_fix_callback
        )

    def nav_sat_fix_callback(self, data):
        kwargs = {
            "seq": data.header.seq,
            "secs": data.header.stamp.secs,
            "nsecs": data.header.stamp.nsecs,
            "frameId": data.header.frame_id,
            "statusStatus": data.status.status,
            "statusService": data.status.service,
            "latitude": data.latitude,
            "longitude": data.longitude,
            "altitude": data.altitude,
            "positionCovarianceType": data.position_covariance_type,
        }
        api_callback(
            self.loop,
            self.module[
                "maverick_api.modules.api.mavros.NavSatFixSchema"
            ].set_nav_sat_fix_message,
            **kwargs
        )
