import logging
import time
from uuid import uuid4

from modules.base.util.mavlink import get_vehicle_strings
from mavros_msgs.srv import VehicleInfoGet
from modules.api import api_callback, moduleBase, schemaBase

# mavros & ros imports
import rospy
import mavros

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


class VehicleInfoSchema(schemaBase):
    def __init__(self):
        super().__init__()
        self.vehicle_info_data = {}
        self.vehicle_info_meta = {"meta": {"total": 0, "updateTime": int(time.time())}}

        self.vehicle_info_type = GraphQLObjectType(
            "VehicleInfo",
            lambda: {
                "uuid": GraphQLField(
                    GraphQLString, description="The UUID of the vehicle."
                ),
                "sysid": GraphQLField(GraphQLInt, description="System ID"),
                "compid": GraphQLField(GraphQLInt, description="Component ID"),
                "autopilot": GraphQLField(
                    GraphQLInt,
                    description="Autopilot ID (maps to value defined in mavlink)",
                ),
                "autopilotString": GraphQLField(
                    GraphQLString, description="Autopilot string from mavlink mapping"
                ),
                "type": GraphQLField(
                    GraphQLInt, description="Type ID (maps to value defined in mavlink)"
                ),
                "typeString": GraphQLField(
                    GraphQLString, description="Type string from mavlink mapping"
                ),
                "capabilities": GraphQLField(GraphQLInt, description="Bit field"),
                "flightSoftwareVersion": GraphQLField(GraphQLInt, description=""),
                "middlewareSoftwareVersion": GraphQLField(GraphQLInt, description=""),
                "osSoftwareVersion": GraphQLField(GraphQLInt, description=""),
                "boardVersion": GraphQLField(GraphQLInt, description=""),
                "vendorId": GraphQLField(GraphQLInt, description=""),
                "productId": GraphQLField(GraphQLInt, description=""),
                "uid": GraphQLField(GraphQLInt, description=""),
                "updateTime": GraphQLField(GraphQLInt, description=""),
            },
            description="Vehicle info",
        )

        self.vehicle_info_list_type = GraphQLObjectType(
            "VehicleInfoList",
            lambda: {
                "info": GraphQLField(GraphQLList(self.vehicle_info_type)),
                "total": GraphQLField(
                    GraphQLInt, description="Total number of info entries"
                ),
                "updateTime": GraphQLField(GraphQLInt, description=""),
            },
        )

        self.q = {
            "VehicleInfo": GraphQLField(
                self.vehicle_info_type,
                args={
                    "uuid": GraphQLArgument(
                        GraphQLNonNull(GraphQLString),
                        description="The UUID of the vehicle",
                    )
                },
                resolve=self.get_vehicle_info,
            ),
            "VehicleInfoList": GraphQLField(
                self.vehicle_info_list_type, resolve=self.get_vehicle_info_list
            ),
        }

        self.m = {
            "VehicleInfo": GraphQLField(
                self.vehicle_info_type,
                args=self.get_mutation_args(self.vehicle_info_type),
                resolve=self.update_vehicle_info,
            )
        }

        self.s = {
            "VehicleInfo": GraphQLField(
                self.vehicle_info_type, subscribe=self.sub_vehicle_info, resolve=None
            ),
            "VehicleInfoList": GraphQLField(
                self.vehicle_info_list_type,
                subscribe=self.sub_vehicle_info_list,
                resolve=None,
            ),
        }

    def get_vehicle_info(self, root, info, **kwargs):
        """Vehicle info query handler"""
        vehicle_uuid = kwargs.get("uuid")  # UUID is required
        # application_log.debug(f"Vehicle info query handler: {vehicle_uuid}")
        # FIXME: remove me
        vehicle_uuid = list(self.vehicle_info_data.keys())[0]
        vehicle_info = self.vehicle_info_data.get(vehicle_uuid, {})
        return vehicle_info

    def update_vehicle_info(self, root, info, **kwargs):
        """Vehicle info mutation handler"""
        data = kwargs.get("data")
        info = data["info"]

        vehicle_info = {
            "uuid": data["uuid"],
            "sysid": info.sysid,
            "compid": info.sysid,
            "autopilot": info.autopilot,
            "autopilotString": data["autopilot_string"],
            "type": info.type,
            "typeString": data["type_string"],
            "capabilities": info.capabilities,
            "flightSoftwareVersion": info.flight_sw_version,
            "middlewareSoftwareVersion": info.middleware_sw_version,
            "osSoftwareVersion": info.os_sw_version,
            "boardVersion": info.board_version,
            "vendorId": info.vendor_id,
            "productId": info.product_id,
            "uid": info.uid,
            "updateTime": data["update_time"],
        }
        self.vehicle_info_data[vehicle_info["uuid"]] = vehicle_info
        self.subscriptions.emit(
            "modules.api.mavros.VehicleInfoSchema" + "VehicleInfo",
            {"VehicleInfo": vehicle_info},
        )
        self.subscriptions.emit(
            "modules.api.mavros.VehicleInfoSchema" + "VehicleInfoList",
            {"VehicleInfoList": self.get_vehicle_info_list(None, None)},
        )

    def sub_vehicle_info(self, root, info):
        """Vehicle info subscription handler"""
        return EventEmitterAsyncIterator(
            self.subscriptions, "modules.api.mavros.VehicleInfoSchema" + "VehicleInfo"
        )

    def get_vehicle_info_list(self, root, info, **kwargs):
        """Vehicle info list query handler"""
        vehicle_info_list = [
            self.vehicle_info_data[x] for x in self.vehicle_info_data.keys()
        ]
        application_log.debug(f"Vehicle info list query handler {vehicle_info_list}")
        try:
            update_time = max([x["updateTime"] for x in vehicle_info_list])
        except ValueError as e:
            update_time = None
        return {
            "info": vehicle_info_list,
            "total": len(vehicle_info_list),
            "updateTime": update_time,
        }

    def sub_vehicle_info_list(self, root, info):
        """Vehicle info list subscription handler"""
        return EventEmitterAsyncIterator(
            self.subscriptions,
            "modules.api.mavros.VehicleInfoSchema" + "VehicleInfoList",
        )


class VehicleInfoInterface(moduleBase):
    def __init__(self, loop, module):
        super().__init__(loop, module)
        self.vehicle_info = {}
        self.parameter_string = None
        # Create ROS service definition for VehicleInfo
        self.get_ros_vehicle_info = rospy.ServiceProxy(
            mavros.get_topic("vehicle_info_get"), VehicleInfoGet
        )
        # FIXME: currently this is run only once at startup
        # TODO: run this in a loop every X Hz to detect changes
        #   Need to make sure we dont block this thread waiting for the service
        #   Note that ros2 supports async calls to services (nice!)
        self.run()

    def run(self):
        try:
            vehicle_info = self.get_ros_vehicle_info()
            application_log.info(
                f"Obtained info from {len(vehicle_info.vehicles)} vehicles"
            )
            update_time = int(time.time())
            for vehicle in vehicle_info.vehicles:
                # application_log.debug(f"{self.vehicle_info}")
                (autopilot_string, type_string, parameter_string) = get_vehicle_strings(
                    vehicle
                )
                application_log.debug(
                    f"{autopilot_string} {type_string}\n{vehicle_info}"
                )

                vehicle_uuid = self.get_vehicle_uuid()

                data = {
                    "uuid": vehicle_uuid,
                    "update_time": update_time,
                    "info": vehicle,
                    "autopilot_string": autopilot_string,
                    "type_string": type_string,
                    "parameter_string": parameter_string,
                }

                api_callback(
                    self.loop,
                    self.module[
                        "modules.api.mavros.VehicleInfoSchema"
                    ].update_vehicle_info,
                    data=data,
                )
                self.vehicle_info[data["uuid"]] = data
        except rospy.ServiceException as ex:
            application_log.error(
                f"An error occurred while retrieving vehicle info via ROS: {ex}"
            )

    def get_vehicle_uuid(self):
        # TODO: generate a uuid based off the hardware or system id?
        return str(uuid4())

    def get_meta_string(self):
        parameter_string = None
        for vehicle in self.vehicle_info:
            # TODO return more than one string and handle correctly
            #   for now just return one
            parameter_string = self.vehicle_info[vehicle]["parameter_string"]
        return parameter_string
