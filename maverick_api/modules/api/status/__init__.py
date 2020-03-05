import logging
from uuid import uuid4
import os
import functools
import time

import tornado.ioloop

from graphql import (
    GraphQLField,
    GraphQLNonNull,
    GraphQLObjectType,
    GraphQLString,
    GraphQLInt,
)
from graphql.pyutils.event_emitter import EventEmitterAsyncIterator

from maverick_api.modules import moduleBase
from maverick_api.modules import schemaBase
from maverick_api.modules import get_schema_timestamp

application_log = logging.getLogger("tornado.application")

api_instance_uuid = str(uuid4())

#################### TODO: REMOVE ME
def get_script():
    status_messages = []
    with open(
        os.path.join(os.path.dirname(os.path.abspath(__file__)), "script.txt"), "r+"
    ) as fid:
        for line in fid:
            if not line.isspace():
                status_messages.append(line.rstrip("\n"))
    return status_messages


def get_status(status_messages, status_count):
    try:
        msg = status_messages[status_count]
        status_count += 1
    except:
        msg = "..."
        status_count = 0
    return (msg, status_count)


#################### TODO: REMOVE ME


class StatusSchema(schemaBase):
    def __init__(self):
        super().__init__(self)
        self.status_data = {
            "id": api_instance_uuid,
            "currentStatus": "...",
            "currentTime": time.time(),
            "schemaGenerationTime": get_schema_timestamp(),
            "_status_messages": get_script(),
            "_status_count": 0,
        }
        self.report_type = GraphQLObjectType(
            "Status",
            lambda: {
                "id": GraphQLField(
                    GraphQLNonNull(GraphQLString),
                    description="The id of the API instance.",
                ),
                "currentStatus": GraphQLField(
                    GraphQLString, description="The API status."
                ),
                "currentTime": GraphQLField(
                    GraphQLInt, description="The current API time."
                ),
                "schemaGenerationTime": GraphQLField(
                    GraphQLInt,
                    description="The time when the graphql schema was last generated.",
                ),
            },
        )

        self.q = {"Status": GraphQLField(self.report_type, resolve=self.get_report)}

        self.s = {
            "Status": GraphQLField(
                self.report_type, subscribe=self.sub_report, resolve=None
            )
        }

    def get_report(self, root, info):
        """API status report query handler"""
        # application_log.info(f"API status report query handler {info}")
        self.status_data["currentTime"] = int(time.time())
        self.status_data["schemaGenerationTime"] = get_schema_timestamp()
        (
            self.status_data["currentStatus"],
            self.status_data["_status_count"],
        ) = get_status(
            self.status_data["_status_messages"], self.status_data["_status_count"]
        )
        self.subscriptions.emit(
            "maverick_api.modules.api.status.StatusSchema" + "Status",
            {"Status": self.status_data},
        )
        return self.status_data

    def sub_report(self, root, info, **kwargs):
        """API status report subscription handler"""
        application_log.info(f"API status report subscription handler {info}")
        return EventEmitterAsyncIterator(
            self.subscriptions,
            "maverick_api.modules.api.status.StatusSchema" + "Status",
        )


class StatusModule(moduleBase):
    def __init__(self):
        super().__init__()
        self.periodic_callbacks = []

    def start(self):
        self.install_periodic_callbacks()
        self.start_periodic_callbacks()

    def install_periodic_callbacks(self):
        callback = functools.partial(
            self.module["maverick_api.modules.api.status.StatusSchema"].get_report,
            None,
            None,
        )
        self.periodic_callbacks.append(
            tornado.ioloop.PeriodicCallback(callback, callback_time=1000, jitter=0.1)
        )

    def start_periodic_callbacks(self):
        for periodic_callback in self.periodic_callbacks:
            periodic_callback.start()
