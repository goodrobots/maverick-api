import logging
import time

from mavros import mission
from modules.api import api_callback, moduleBase, schemaBase

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


class MissionSchema(schemaBase):
    def __init__(self):
        super().__init__()
        self.mission_data = {}
        self.mission_meta = {"meta": {"total": 0, "updateTime": int(time.time())}}

        self.mission_type = GraphQLObjectType(
            "Mission",
            lambda: {
                "seq": GraphQLField(
                    GraphQLString,
                    description="The sequence number of the mission item.",
                ),
                "isCurrent": GraphQLField(
                    GraphQLBoolean,
                    description="True if this mission item is the active target",
                ),
                "autocontinue": GraphQLField(
                    GraphQLBoolean,
                    description="Continue mission after this mission item",
                ),
                "frame": GraphQLField(GraphQLInt, description=""),
                "command": GraphQLField(GraphQLInt, description=""),
                "param1": GraphQLField(GraphQLFloat, description=""),
                "param2": GraphQLField(GraphQLFloat, description=""),
                "param3": GraphQLField(GraphQLFloat, description=""),
                "param4": GraphQLField(GraphQLFloat, description=""),
                "latitude": GraphQLField(GraphQLFloat, description=""),
                "longitude": GraphQLField(GraphQLFloat, description=""),
                "altitude": GraphQLField(GraphQLFloat, description=""),
                "updateTime": GraphQLField(GraphQLInt, description=""),
                "total": GraphQLField(
                    GraphQLInt, description="Total number of mission items"
                ),
            },
            description="Mission item",
        )

        self.mission_list_type = GraphQLObjectType(
            "MissionList",
            lambda: {
                "id": GraphQLField(GraphQLString, description="The id of the mission."),
                "mission": GraphQLField(GraphQLList(self.mission_type)),
                "total": GraphQLField(
                    GraphQLInt, description="Total number of mission items"
                ),
                "updateTime": GraphQLField(GraphQLInt, description=""),
            },
        )

        self.q = {
            "Mission": GraphQLField(
                self.mission_type,
                args={
                    "seq": GraphQLArgument(
                        GraphQLNonNull(GraphQLString),
                        description="The sequence number of desired mission item",
                    )
                },
                resolve=self.get_mission,
            ),
            "MissionList": GraphQLField(
                self.mission_list_type,
                args={
                    "id": GraphQLArgument(
                        GraphQLNonNull(GraphQLString),
                        description="The id of the desired mission",
                    )
                },
                resolve=self.get_mission_list,
            ),
        }

        self.m = {
            "Mission": GraphQLField(
                self.mission_type,
                args=self.get_mutation_args(self.mission_type),
                resolve=self.update_mission,
            )
        }

        self.s = {
            "Mission": GraphQLField(
                self.mission_type, subscribe=self.sub_mission, resolve=None
            ),
            "MissionList": GraphQLField(
                self.mission_list_type, subscribe=self.sub_mission_list, resolve=None
            ),
        }

    def get_mission(self, root, info, **kwargs):
        """Mission query handler"""
        application_log.debug(f"Mission query handler {kwargs}")
        sequence = kwargs.get("seq", "meta")
        if sequence == "meta":
            mission_item = self.mission_meta[sequence]
        else:
            mission_item = self.mission_data.get(sequence, {})
        return mission_item

    def update_mission(self, root, info, **kwargs):
        """Mission mutation handler"""
        data = kwargs.get("data")
        total = len(data.waypoints)
        update_time = int(time.time())
        self.mission_meta = {"meta": {"total": total, "updateTime": update_time}}
        mission_data = {}
        for seq, waypoint in enumerate(data.waypoints):
            mission_item = {
                "seq": str(seq),
                "frame": waypoint.frame,
                "command": waypoint.command,
                "isCurrent": waypoint.is_current,
                "autocontinue": waypoint.autocontinue,
                "param1": waypoint.param1,
                "param2": waypoint.param2,
                "param3": waypoint.param3,
                "param4": waypoint.param4,
                "latitude": waypoint.x_lat,
                "longitude": waypoint.y_long,
                "altitude": waypoint.z_alt,
                "updateTime": update_time,
                "total": total,
            }
            mission_data[mission_item["seq"]] = mission_item
            self.subscriptions.emit(
                "modules.api.mavros.MissionSchema" + "Mission",
                {"Mission": mission_item},
            )
        self.mission_data = mission_data
        self.subscriptions.emit(
            "modules.api.mavros.MissionSchema" + "MissionList",
            {"MissionList": self.get_mission_list(None, None, id="loaded")},
        )
        application_log.debug(f"Mission mutation handler {self.mission_data}")

    def sub_mission(self, root, info):
        """Mission subscription handler"""
        return EventEmitterAsyncIterator(
            self.subscriptions, "modules.api.mavros.MissionSchema" + "Mission"
        )

    def get_mission_list(self, root, info, **kwargs):
        """Mission list query handler"""
        application_log.debug(f"Mission list query handler {kwargs}")
        mission_id = kwargs.get("id")
        mission_list = []

        mission_id = "loaded"  # FIXME: remove this line to make mission_id dynamic
        if mission_id == "loaded":
            mission_list = [self.mission_data[x] for x in self.mission_data.keys()]
        else:
            # TODO: load the mission from file / database based on mission_id
            # mission_list = ...
            pass
        application_log.debug(f"Mission list query handler {mission_list}")
        try:
            update_time = max([x["updateTime"] for x in mission_list])
        except ValueError as e:
            update_time = None
        return {
            "id": mission_id,
            "mission": mission_list,
            "total": len(mission_list),
            "updateTime": update_time,
        }

    def sub_mission_list(self, root, info):
        """Mission list subscription handler"""
        return EventEmitterAsyncIterator(
            self.subscriptions, "modules.api.mavros.MissionSchema" + "MissionList"
        )


class MissionInterface(moduleBase):
    def __init__(self, loop, module):
        super().__init__(loop, module)
        self.waypoints = None

    def mission_waypoints(self):
        self.waypoints = mission.pull()
        application_log.warn(f"waypoints: {self.waypoints}")
        mission.subscribe_waypoints(self.mission_callback)

    def mission_callback(self, data):
        # application_log.debug(f"mission_callback: {data}")
        api_callback(
            self.loop,
            self.module["modules.api.mavros.MissionSchema"].update_mission,
            data=data,
        )
