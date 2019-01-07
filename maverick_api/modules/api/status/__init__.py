import logging
import ctypes
from uuid import uuid4
import os
import functools

import tornado.ioloop

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
)
from graphql.pyutils.event_emitter import EventEmitter, EventEmitterAsyncIterator

from modules.api import moduleBase
from modules.api import schemaBase

application_log = logging.getLogger("tornado.application")

# Setup monotonic clock
CLOCK_MONOTONIC_RAW = 4


class timespec(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_nsec", ctypes.c_long)]


librt = ctypes.CDLL("librt.so.1", use_errno=True)
clock_gettime = librt.clock_gettime
clock_gettime.argtypes = [ctypes.c_int, ctypes.POINTER(timespec)]


def monotonic_time():
    t = timespec()
    if clock_gettime(CLOCK_MONOTONIC_RAW, ctypes.pointer(t)) != 0:
        errno_ = ctypes.get_errno()
        raise OSError(errno_, os.strerror(errno_))
    return (t.tv_sec * 1e9) + t.tv_nsec


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

status_data = dict(
    id=uuid4().__str__(),
    currentStatus="...",
    currentTime=str(monotonic_time()),
    status_messages=get_script(),
    status_count=0,
)


class StatusSchema(schemaBase):
    def __init__(self):
        super().__init__()
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
                    GraphQLString, description="The current API time."
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
        status_data["currentTime"] = str(monotonic_time())
        (status_data["currentStatus"], status_data["status_count"]) = get_status(
            status_data["status_messages"], status_data["status_count"]
        )
        self.subscriptions.emit(
            "modules.api.status.StatusSchema" + "Status", {"Status": status_data}
        )
        return status_data

    def sub_report(self, root, info, **kwargs):
        """API status report subscription handler"""
        application_log.info(f"API status report subscription handler {info}")
        return EventEmitterAsyncIterator(
            self.subscriptions, "modules.api.status.StatusSchema" + "Status"
        )


class StatusModule(moduleBase):
    def __init__(self, loop, module):
        super().__init__(loop, module)
        self.periodic_callbacks = []
        self.install_periodic_callbacks()
        self.start_periodic_callbacks()

    def install_periodic_callbacks(self):
        callback = functools.partial(
            self.module["modules.api.status.StatusSchema"].get_report, None, None
        )
        self.periodic_callbacks.append(
            tornado.ioloop.PeriodicCallback(callback, callback_time=1000, jitter=0.1)
        )

    def start_periodic_callbacks(self):
        for periodic_callback in self.periodic_callbacks:
            periodic_callback.start()
