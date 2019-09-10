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
    GraphQLInputObjectType,
    GraphQLInputField,
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
                    GraphQLInt, description="The sequence number of the mission item."
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
            },
            description="Mission item",
        )

        self.mission_input_type = GraphQLInputObjectType(
            "MissionInput",
            {
                "seq": GraphQLInputField(
                    GraphQLNonNull(GraphQLInt),
                    description="The sequence number of the mission item.",
                ),
                "isCurrent": GraphQLInputField(
                    GraphQLNonNull(GraphQLBoolean),
                    description="True if this mission item is the active target",
                ),
                "autocontinue": GraphQLInputField(
                    GraphQLNonNull(GraphQLBoolean),
                    description="Continue mission after this mission item",
                ),
                "frame": GraphQLInputField(GraphQLNonNull(GraphQLInt), description=""),
                "command": GraphQLInputField(
                    GraphQLNonNull(GraphQLInt), description=""
                ),
                "param1": GraphQLInputField(
                    GraphQLNonNull(GraphQLFloat), description=""
                ),
                "param2": GraphQLInputField(
                    GraphQLNonNull(GraphQLFloat), description=""
                ),
                "param3": GraphQLInputField(
                    GraphQLNonNull(GraphQLFloat), description=""
                ),
                "param4": GraphQLInputField(
                    GraphQLNonNull(GraphQLFloat), description=""
                ),
                "latitude": GraphQLInputField(
                    GraphQLNonNull(GraphQLFloat), description=""
                ),
                "longitude": GraphQLInputField(
                    GraphQLNonNull(GraphQLFloat), description=""
                ),
                "altitude": GraphQLInputField(
                    GraphQLNonNull(GraphQLFloat), description=""
                ),
            },
        )

        self.mission_list_type = GraphQLObjectType(
            "MissionList",
            lambda: {
                "id": GraphQLField(GraphQLString, description="The id of the mission."),
                "name": GraphQLField(GraphQLString, description="Short name of the mission."),
                "description": GraphQLField(GraphQLString, description="Description of the mission."),
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
                "id": GraphQLField(
                    GraphQLString,
                    description="The id of the database use the wildcard '*' to access all avalable databases.",
                ),
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
                        GraphQLNonNull(GraphQLInt),
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
            ),
            "MissionList": GraphQLField(
                self.mission_list_type,
                args={
                    "id": GraphQLArgument(GraphQLNonNull(GraphQLString)),
                    "mission": GraphQLArgument(
                        GraphQLNonNull(GraphQLList(self.mission_input_type))
                    ),
                    "name": GraphQLArgument(GraphQLString),
                    "description": GraphQLArgument(GraphQLString),
                },
                resolve=self.update_mission_list,
            ),
        }

        self.s = {
            "Mission": GraphQLField(
                self.mission_type, subscribe=self.sub_mission, resolve=None
            ),
            "MissionList": GraphQLField(
                self.mission_list_type, subscribe=self.sub_mission_list, resolve=None
            ),
            "MissionDatabase": GraphQLField(
                self.mission_database_type,
                subscribe=self.sub_mission_database,
                resolve=None,
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
                "seq": seq,
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
        # application_log.debug(f"Mission mutation handler {self.mission_data}")

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
        update_time = int(time.time())
        ret = {}

        if mission_id == "loaded":
            # TODO: if mission data is empty peform a mission pull from the FC
            # mission.pull()
            mission_list = [self.mission_data[x] for x in self.mission_data.keys()]
            # application_log.debug(
            #     f"Mission list query handler for {mission_id}: {mission_list}"
            # )
            # TODO support name and description in loaded mission
            ret = {
                "id": mission_id,
                "mission": mission_list,
                "total": len(mission_list),
                "updateTime": update_time,
                "name": "",
                "description": "",
            }

        else:
            try:
                # attempt to load the mission from a database
                mission_file = self.mission_database_dir.joinpath(f"{mission_id}.mission")
                # TODO: handle missing files
                with open(mission_file, "r+") as fid:
                    mission_list = json.load(fid)
                application_log.debug(
                    f"Mission list query handler for {mission_id}: {mission_list}"
                )
                ret = {
                    "id": mission_list["id"],
                    "mission": mission_list["mission"],
                    "total": len(mission_list["mission"]),
                    "updateTime": mission_list["updateTime"],
                    "name": mission_list.get("name", ""),
                    "description": mission_list.get("description", ""),
                }
            except Exception as e:
                application_log.warning(e)
                return {}
        return ret

    def update_mission_list(self, root, info, **kwargs):
        # The following kwargs are enforced by the schema
        mission_id = kwargs.get("id")
        mission_list = kwargs.get("mission")
        name = kwargs.get("name", "")
        description = kwargs.get("description", "")

        if mission_id == "loaded":
            mission_loaded_filepath = os.path.join(
                self.mission_database_dir, "loaded.csv"
            )
            # requesting update to waypoints on flight controller
            application_log.debug(f"Writing mission to autopilot: {mission_list}")
            # Need to save the mission into a csv format and read it back
            # First save the mission to a csv file
            # generate the string we want to write
            # write the string to disk
            # load the string and push to the autopilot
            # TODO: FIXME: This is a bit of a hack for now...
            #   we should make a wps object and load that without the save + load steps
            mission_string = "QGC WPL 120\r\n"
            for idx, mission_action in enumerate(mission_list):
                mission_line = f"{mission_action['seq']}\t\
                    {int(mission_action['isCurrent'])}\t\
                    {mission_action['frame']}\t\
                    {mission_action['command']}\t\
                    {mission_action['param1']}\t\
                    {mission_action['param2']}\t\
                    {mission_action['param3']}\t\
                    {mission_action['param4']}\t\
                    {mission_action['latitude']}\t\
                    {mission_action['longitude']}\t\
                    {mission_action['altitude']}\t\
                    {int(mission_action['autocontinue'])}\r\n"
                mission_string += mission_line
            with open(mission_loaded_filepath, "w+") as fid:
                fid.write(mission_string)
            with open(mission_loaded_filepath, "r+") as fid:
                wps = [w for w in mission.QGroundControlWP().read(fid)]
            ret = mission.push(start_index=0, waypoints=wps)
        else:
            # write the mission to the database
            application_log.debug(
                f"Writing mission to database: {mission_id}:{mission_list}"
            )
            update_time = int(time.time())
            total = len(mission_list)
            for mission_val in mission_list:
                mission_val["total"] = total
                mission_val["updateTime"] = update_time

            ret = {
                "id": mission_id,
                "mission": mission_list,
                "total": total,
                "updateTime": update_time,
                "name": name,
                "description": description,
            }
            # FIXME: write mission to database, use file system for now
            with open(
                os.path.join(self.mission_database_dir, mission_id + ".mission"), "w+"
            ) as fid:
                fid.write(json.dumps(ret, indent=4))
            # TODO: return success?
            

    def sub_mission_list(self, root, info):
        """Mission list subscription handler"""
        return EventEmitterAsyncIterator(
            self.subscriptions, "modules.api.mavros.MissionSchema" + "MissionList"
        )

    def get_mission_database(self, root, info, **kwargs):
        """Mission database query handler"""
        application_log.debug(f"Mission database query handler {kwargs}")
        database_id = kwargs.get("id", None)
        missions = []
        if database_id in ["", " ", "*", None]:
            # look at all databases we have access to
            # open the database (async)
            # load all missions from database into responce (async)
            for mission_file in self.mission_database_dir.glob("*.mission"):
                with open(mission_file, "r+") as fid:
                    try:
                        loaded_mission = json.load(fid)
                        # TODO: ensure the mission has correct fields
                        #   for now just make sure its the correct length
                        if len(loaded_mission) == 6:
                            missions.append(loaded_mission)
                        else:
                            application_log.info(f'Failed to parse the data: {fid}\nfrom the file: {mission_file}')
                    except json.decoder.JSONDecodeError as e:
                        application_log.info(f'Failed to parse the data: {fid}\nfrom the file: {mission_file}')
                    
            # missions = [json.load(open(x)) for x in self.mission_database_dir.glob('*.mission')]
        else:
            # TODO: add database lookup search, for now just grab the mission with the correct name
            with open(
                os.path.join(self.mission_database_dir, database_id + ".mission"), "r+"
            ) as fid:
                missions.append(json.load(fid))
        return {
            "id": database_id,
            "missions": missions,
            "total": len(missions),
            "updateTime": int(time.time()),
        }

    def update_mission_database(self, root, info, **kwargs):
        pass

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
        # application_log.debug(f"waypoints: {self.waypoints}")
        mission.subscribe_waypoints(self.mission_callback)

    def mission_callback(self, data):
        # application_log.debug(f"mission_callback: {data}")
        api_callback(
            self.loop,
            self.module["modules.api.mavros.MissionSchema"].update_mission,
            data=data,
        )
