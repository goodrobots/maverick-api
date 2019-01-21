import logging
import time
import os
import glob
import json
from pathlib import Path

from mavros import mission

from modules.api import api_callback, moduleBase, schemaBase
from modules.base.util.functions import mkdirs

from tornado.options import options

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
        # create a mission database dir
        self.mission_database_dir = Path(options.datadir).joinpath("missions")
        # make the path if it does not exist
        mkdirs(self.mission_database_dir)
        
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
            description="Mission",
        )
        
        self.mission_database_type = GraphQLObjectType(
            "MissionDatabase",
            lambda: {
                "id": GraphQLField(GraphQLString, description="The id of the database use the wildcard '*' to access all avalable databases."),
                "missions": GraphQLField(GraphQLList(self.mission_list_type)),
                "total": GraphQLField(
                    GraphQLInt, description="Total number of missions in database"
                ),
                "updateTime": GraphQLField(GraphQLInt, description=""),
            },
            description="Collection of missions",
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
            "MissionDatabase": GraphQLField(
                self.mission_database_type,
                args={
                    "id": GraphQLArgument(
                        GraphQLNonNull(GraphQLString),
                        description="The id of the database",
                    )
                },
                resolve=self.get_mission_database,
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
            "MissionDatabase": GraphQLField(
                self.mission_database_type, subscribe=self.sub_mission_database, resolve=None
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
        
        ret = {
            "id": mission_id,
            "mission": mission_list,
            "total": len(mission_list),
            "updateTime": update_time,
        }
        
        # FIXME: write mission to database, use file system for now
        with open(os.path.join(self.mission_database_dir, ret["id"]+".mission"), "w+") as fid:
            fid.write(json.dumps(ret, indent=4))
        return ret

    def sub_mission_list(self, root, info):
        """Mission list subscription handler"""
        return EventEmitterAsyncIterator(
            self.subscriptions, "modules.api.mavros.MissionSchema" + "MissionList"
        )
        
    def get_mission_database(self, root, info, **kwargs):
        """Mission database query handler"""
        application_log.debug(f"Mission list query handler {kwargs}")
        database_id = kwargs.get("id")
        missions = []
        database_id = "*"  # FIXME: remove this line to make database_id dynamic
        if database_id == "*":
            # look at all databases we have access to
            # open the database (async)
            # load all missions from database into responce (async)
            for mission_file in self.mission_database_dir.glob('*.mission'):
                with open(mission_file, "r+") as fid:
                    mission.append(json.load(fid))
            # missions = [json.load(open(x)) for x in self.mission_database_dir.glob('*.mission')]
        else:
            # TODO: add database lookup search
            # missions = ...
            pass
        return {
            "id": database_id,
            "missions": missions,
            "total": len(missions),
            "updateTime": int(time.time()),
        }

    def sub_mission_database(self, root, info):
        """Mission database subscription handler"""
        return EventEmitterAsyncIterator(
            self.subscriptions, "modules.api.mavros.MissionSchema" + "MissionDatabase"
        )


class MissionInterface(moduleBase):
    def __init__(self, loop, module):
        super().__init__(loop, module)
        self.waypoints = None

    def mission_waypoints(self):
        self.waypoints = mission.pull()
        application_log.debug(f"waypoints: {self.waypoints}")
        mission.subscribe_waypoints(self.mission_callback)

    def mission_callback(self, data):
        # application_log.debug(f"mission_callback: {data}")
        api_callback(
            self.loop,
            self.module["modules.api.mavros.MissionSchema"].update_mission,
            data=data,
        )
