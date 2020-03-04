import logging

from maverick_api.modules import api_callback, moduleBase, schemaBase

import rospy
from rosgraph_msgs.msg import Log

# graphql imports
from graphql import GraphQLField, GraphQLObjectType, GraphQLString, GraphQLInt
from graphql.pyutils.event_emitter import EventEmitterAsyncIterator

application_log = logging.getLogger("tornado.application")


class StatusTextSchema(schemaBase):
    def __init__(self):
        super().__init__()
        self.status_text_data = {"uuid": "test"}

        self.status_text_message_type = GraphQLObjectType(
            "StatusText",
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
                "level": GraphQLField(GraphQLInt, description=""),
                "message": GraphQLField(GraphQLString, description=""),
            },
            description="MAVROS StatusTextMessage",
        )

        self.q = {
            "StatusText": GraphQLField(
                self.status_text_message_type, resolve=self.get_status_text_message
            )
        }

        self.m = {
            "StatusText": GraphQLField(
                self.status_text_message_type,
                args=self.get_mutation_args(self.status_text_message_type),
                resolve=self.set_status_text_message,
            )
        }

        self.s = {
            "StatusText": GraphQLField(
                self.status_text_message_type,
                subscribe=self.sub_status_text_message,
                resolve=None,
            )
        }

    def get_status_text_message(self, root, info):
        """StatusTextMessage query handler"""
        return self.status_text_data

    def set_status_text_message(self, root, info, **kwargs):
        """StatusTextMessage mutation handler"""
        updated_dict = {**self.status_text_data, **kwargs}
        self.subscriptions.emit(
            "maverick_api.modules.api.mavros.StatusTextSchema" + "StatusText",
            {"StatusText": updated_dict},
        )
        self.status_text_data = updated_dict
        return updated_dict

    def sub_status_text_message(self, root, info):
        """StatusTextMessage subscription handler"""
        return EventEmitterAsyncIterator(
            self.subscriptions,
            "maverick_api.modules.api.mavros.StatusTextSchema" + "StatusText",
        )


class StatusTextInterface(moduleBase):
    def __init__(self, loop, module):
        super().__init__(loop, module)

        rospy.Subscriber("/rosout", Log, self.statustext_callback)

    def statustext_callback(self, data):
        if data.name == "/mavros":
            # application_log.debug("statustext: {0}:{1}".format(data.level, data.msg))
            kwargs = {
                "seq": data.header.seq,
                "secs": data.header.stamp.secs,
                "nsecs": data.header.stamp.nsecs,
                "frameId": data.header.frame_id,
                "level": data.level,
                "message": data.msg,
            }
            api_callback(
                self.loop,
                self.module[
                    "maverick_api.modules.api.mavros.StatusTextSchema"
                ].set_status_text_message,
                **kwargs,
            )
