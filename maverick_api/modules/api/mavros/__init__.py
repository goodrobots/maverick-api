import logging
import time

from modules.api import moduleBase
from modules.api import schemaBase
from modules.api import api_callback

from modules.base.tornadoql.session_control import GraphQLSession
from modules.base.util.mavlink import get_meta_string

# mavros & ros imports
import rospy
import mavros
import mavros.utils
from std_msgs.msg import String, Float64
from rosgraph_msgs.msg import Log
from mavros_msgs.msg import State, VFR_HUD
from mavros_msgs.srv import StreamRate, StreamRateRequest
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from mavros_maverick.msg import Param  # callback msg on param change
from mavros_maverick.srv import VehicleInfo
from mavros.param import param_ret_value

from modules.api.mavros.mavros_mission import MissionSchema, MissionInterface
from modules.api.mavros.mavros_nav_sat_fix import NavSatFixSchema, NavSatFixInterface

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


class MAVROSSchema(schemaBase):
    def __init__(self):
        super().__init__()

        self.imu_data = {"uuid": "test"}
        self.state_data = {"uuid": "test"}
        self.pose_data = {"uuid": "test"}
        self.vfr_hud_data = {"uuid": "test"}
        self.status_text_data = {"uuid": "test"}

        self.imu_message_type = GraphQLObjectType(
            "Imu",
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
                "orientationX": GraphQLField(GraphQLFloat, description=""),
                "orientationY": GraphQLField(GraphQLFloat, description=""),
                "orientationZ": GraphQLField(GraphQLFloat, description=""),
                "orientationW": GraphQLField(GraphQLFloat, description=""),
                "angularVelocityX": GraphQLField(GraphQLFloat, description=""),
                "angularVelocityY": GraphQLField(GraphQLFloat, description=""),
                "angularVelocityZ": GraphQLField(GraphQLFloat, description=""),
                "linearAccelerationX": GraphQLField(GraphQLFloat, description=""),
                "linearAccelerationY": GraphQLField(GraphQLFloat, description=""),
                "linearAccelerationZ": GraphQLField(GraphQLFloat, description=""),
            },
            description="MAVROS ImuMessage",
        )

        self.state_message_type = GraphQLObjectType(
            "VehicleState",
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
                "connected": GraphQLField(GraphQLBoolean, description=""),
                "armed": GraphQLField(GraphQLBoolean, description=""),
                "guided": GraphQLField(GraphQLBoolean, description=""),
                "mode": GraphQLField(GraphQLString, description=""),
                "systemStatus": GraphQLField(GraphQLInt, description=""),
            },
            description="MAVROS StateMessage",
        )

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
            "Imu": GraphQLField(self.imu_message_type, resolve=self.get_imu_message),
            "VehicleState": GraphQLField(
                self.state_message_type, resolve=self.get_state_message
            ),
            "PoseStamped": GraphQLField(
                self.pose_stamped_message_type, resolve=self.get_pose_stamped_message
            ),
            "VfrHud": GraphQLField(
                self.vfr_hud_message_type, resolve=self.get_vfr_hud_message
            ),
            "StatusText": GraphQLField(
                self.status_text_message_type, resolve=self.get_status_text_message
            ),
        }

        self.m = {
            "Imu": GraphQLField(
                self.imu_message_type,
                args=self.get_mutation_args(self.imu_message_type),
                resolve=self.set_imu_message,
            ),
            "VehicleState": GraphQLField(
                self.state_message_type,
                args=self.get_mutation_args(self.state_message_type),
                resolve=self.set_state_message,
            ),
            "PoseStamped": GraphQLField(
                self.state_message_type,
                args=self.get_mutation_args(self.pose_stamped_message_type),
                resolve=self.set_pose_stamped_message,
            ),
            "VfrHud": GraphQLField(
                self.vfr_hud_message_type,
                args=self.get_mutation_args(self.vfr_hud_message_type),
                resolve=self.set_vfr_hud_message,
            ),
            "StatusText": GraphQLField(
                self.status_text_message_type,
                args=self.get_mutation_args(self.status_text_message_type),
                resolve=self.set_status_text_message,
            ),
        }

        self.s = {
            "Imu": GraphQLField(
                self.imu_message_type, subscribe=self.sub_imu_message, resolve=None
            ),
            "VehicleState": GraphQLField(
                self.state_message_type, subscribe=self.sub_state_message, resolve=None
            ),
            "PoseStamped": GraphQLField(
                self.pose_stamped_message_type,
                subscribe=self.sub_pose_stamped_message,
                resolve=None,
            ),
            "VfrHud": GraphQLField(
                self.vfr_hud_message_type,
                subscribe=self.sub_vfr_hud_message,
                resolve=None,
            ),
            "StatusText": GraphQLField(
                self.status_text_message_type,
                subscribe=self.sub_imu_message,
                resolve=None,
            ),
        }

    def get_imu_message(self, root, info):
        """ImuMessage query handler"""
        application_log.info(f"ImuMessage query handler {info}")
        return self.imu_data

    def set_imu_message(self, root, info, **kwargs):
        """ImuMessage mutation handler"""
        updated_dict = {**self.imu_data, **kwargs}
        self.subscriptions.emit(
            "modules.api.mavros.MAVROSSchema" + "Imu", {"Imu": updated_dict}
        )
        self.imu_data = updated_dict
        return updated_dict

    def sub_imu_message(self, root, info):
        """ImuMessage subscription handler"""
        application_log.info(f"ImuMessage subscription handler {info}")
        return EventEmitterAsyncIterator(
            self.subscriptions, "modules.api.mavros.MAVROSSchema" + "Imu"
        )

    def get_state_message(self, root, info):
        """StateMessage query handler"""
        return self.state_data

    def set_state_message(self, root, info, **kwargs):
        """StateMessage mutation handler"""
        updated_dict = {**self.state_data, **kwargs}
        self.subscriptions.emit(
            "modules.api.mavros.MAVROSSchema" + "VehicleState",
            {"VehicleState": updated_dict},
        )
        self.state_data = updated_dict
        return updated_dict

    def sub_state_message(self, root, info):
        """StateMessage subscription handler"""
        return EventEmitterAsyncIterator(
            self.subscriptions, "modules.api.mavros.MAVROSSchema" + "VehicleState"
        )

    def get_pose_stamped_message(self, root, info):
        """PoseStampedMessage query handler"""
        return self.pose_data

    def set_pose_stamped_message(self, root, info, **kwargs):
        """PoseStampedMessage mutation handler"""
        updated_dict = {**self.pose_data, **kwargs}
        self.subscriptions.emit(
            "modules.api.mavros.MAVROSSchema" + "PoseStamped",
            {"PoseStamped": updated_dict},
        )
        self.pose_data = updated_dict
        return updated_dict

    def sub_pose_stamped_message(self, root, info):
        """PoseStampedMessage subscription handler"""
        return EventEmitterAsyncIterator(
            self.subscriptions, "modules.api.mavros.MAVROSSchema" + "PoseStamped"
        )

    def get_vfr_hud_message(self, root, info):
        """VfrHudMessage query handler"""
        return self.vfr_hud_data

    def set_vfr_hud_message(self, root, info, **kwargs):
        """VfrHudMessage mutation handler"""
        updated_dict = {**self.vfr_hud_data, **kwargs}
        self.subscriptions.emit(
            "modules.api.mavros.MAVROSSchema" + "VfrHud", {"VfrHud": updated_dict}
        )
        self.vfr_hud_data = updated_dict
        return updated_dict

    def sub_vfr_hud_message(self, root, info):
        """VfrHudMessage subscription handler"""
        return EventEmitterAsyncIterator(
            self.subscriptions, "modules.api.mavros.MAVROSSchema" + "VfrHud"
        )

    def get_status_text_message(self, root, info):
        """StatusTextMessage query handler"""
        return self.status_text_data

    def set_status_text_message(self, root, info, **kwargs):
        """StatusTextMessage mutation handler"""
        updated_dict = {**self.status_text_data, **kwargs}
        self.subscriptions.emit(
            "modules.api.mavros.MAVROSSchema" + "StatusText",
            {"StatusText": updated_dict},
        )
        self.status_text_data = updated_dict
        return updated_dict

    def sub_status_text_message(self, root, info):
        """StatusTextMessage subscription handler"""
        return EventEmitterAsyncIterator(
            self.subscriptions, "modules.api.mavros.MAVROSSchema" + "StatusText"
        )


class MAVROSConnection(moduleBase):
    def __init__(self, loop, module):
        super().__init__(loop, module)
        # Attributes
        self.info = None
        self.meta_string = ""  # meta string used to load the correct params
        self.mission_interface = MissionInterface(self.loop, self.module)

    def run(self):
        self.connect()
        # self.topics()
        self.streams()
        self.vehicle_info()  # work out the autopilot and vehicle type
        self.params(
            meta_string=self.meta_string
        )  # this is called from the IOloop once we know the vehicle type
        self.mission_interface.mission_waypoints()
        self.nav_sat_fix_interface = NavSatFixInterface(self.loop, self.module)
        self.listener()

    def connect(self):
        rospy.init_node("listener", anonymous=True, disable_signals=True)
        mavros.set_namespace("mavros")

    def shutdown(self):
        rospy.signal_shutdown("API Shutdown")

    def streams(self):
        # Create ROS service definition for StreamRate
        set_rate = rospy.ServiceProxy(mavros.get_topic("set_stream_rate"), StreamRate)
        # Define function to do the stream request
        def do_set_rate(stream_rate, stream_id):
            if stream_rate is not None:
                try:
                    set_rate(
                        stream_id=stream_id,
                        message_rate=stream_rate,
                        on_off=(stream_rate != 0),
                    )
                    logging.debug(
                        "Set stream {} rate {}".format(stream_id, stream_rate)
                    )
                except rospy.ServiceException as ex:
                    logging.error(
                        "An error occurred while setting vehicle stream rates via ROS: {0}".format(
                            ex
                        )
                    )

        # Request streams
        """
        stream_rates = {
            'STREAM_RAW_SENSORS': 5,
            'STREAM_EXTENDED_STATUS': 5,
            'STREAM_RC_CHANNELS': 5,
            'STREAM_RAW_CONTROLLER': 5,
            'STREAM_POSITION': 10, # POSE
            'STREAM_EXTRA1': 10, # IMU
            'STREAM_EXTRA2': 5, # VFR_HUD
            'STREAM_EXTRA3': 1
        }
        """
        stream_rates = {
            "STREAM_RAW_SENSORS": 1,
            "STREAM_EXTENDED_STATUS": 1,
            "STREAM_RC_CHANNELS": 1,
            "STREAM_RAW_CONTROLLER": 1,
            "STREAM_POSITION": 1,  # POSE
            "STREAM_EXTRA1": 1,  # IMU
            "STREAM_EXTRA2": 1,  # VFR_HUD
            "STREAM_EXTRA3": 1,
        }
        for stream, rate in stream_rates.items():
            do_set_rate(rate, getattr(StreamRateRequest, stream))

    def vehicle_info(self):
        # Create ROS service definition for VehicleInfo
        get_vehicle_info = rospy.ServiceProxy(
            mavros.get_topic("get_vehicle_info"), VehicleInfo
        )
        try:
            self.info = get_vehicle_info()
            application_log.debug(f"{self.info}")
            self.meta_string = get_meta_string(self.info)
            logging.debug(self.info)
            logging.info(self.meta_string)
        except rospy.ServiceException as ex:
            logging.error(
                "An error occurred while retrieving vehicle info via ROS: {0}".format(
                    ex
                )
            )

    def listener(self):
        rospy.Subscriber("/mavros/state", State, self.state_callback)
        rospy.Subscriber("/mavros/vfr_hud", VFR_HUD, self.vfr_hud_callback)
        rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, self.pose_stamped_callback
        )
        rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
        rospy.Subscriber("/mavros/param_value", Param, self.param_callback)
        rospy.Subscriber("/rosout", Log, self.statustext_callback)
        rospy.Subscriber(
            "/mavros/global_position/rel_alt", Float64, self.rel_alt_callback
        )

    def topics(self):
        topics = rospy.get_published_topics()
        for topic in topics:
            logging.info(topic)

    def params(self, meta_string="ArduCopter"):
        # TODO: make vehicle dynamic and chosse between px4 and ardupilot
        from modules.base.param.parse_param_xml import get_param_meta

        # from api.schema import Parameters
        from mavros.param import param_get_all

        # TODO: UPDATE CALLBACK
        # Parameters.callback = self.param_set_callback
        param_received, param_list = param_get_all(False)
        logging.debug("Parameters received: {0}".format(param_received))

        param_meta_vehicle = {}
        for param in param_list:
            kwargs = {"id": param.param_id, "value": param.param_value}
            # add_ioloop_callback(UpdateParameter().mutate, **kwargs)

            param_meta_vehicle[param.param_id] = {
                "group": param.param_id.split("_")[0].strip().rstrip("_").upper()
            }
            logging.debug(
                "param get {0}:{1}  {2}".format(
                    param.param_id, param.param_value, type(param.param_value)
                )
            )

        # TODO: handle IOError when mavlink-router is not connected to the AP but mavros is running

        if options.debug:
            start_time = time.time()
        logging.debug("starting parameter meta fetch")
        param_meta_server = get_param_meta(meta_string)
        logging.debug("finished parameter meta fetch")
        if options.debug:
            logging.debug(
                "parameter meta fetch took {0}s".format(time.time() - start_time)
            )
        # Parameters.meta = merge_two_dicts(param_meta_vehicle, param_meta_server)

    def param_set_callback(self, param_data):
        # from api.schema import Parameters
        # TODO:
        # write unit test
        # clean up logic
        from mavros.param import param_set

        mavros.set_namespace("mavros")
        ret = param_set(
            param_data["id"].encode("ascii", "ignore"), float(param_data["value"])
        )
        # if ret == param_value:
        #     # param set worked
        #     pass
        # else:
        #     # param set failed
        #     pass

        # # check to see if the set value matches the provided value
        # ret_param = param_get(param_data['id'])
        logging.debug(
            "param set {0}:{1}  {2}".format(param_data["id"], param_data["value"], ret)
        )
        # logging.debug('{0}'.format(ret_param))
        return ret

    def state_callback(self, data):
        kwargs = {
            "seq": data.header.seq,
            "frameId": data.header.frame_id,
            "guided": data.guided,
            "nsecs": data.header.stamp.nsecs,
            "systemStatus": data.system_status,
            "secs": data.header.stamp.secs,
            "connected": data.connected,
            "mode": data.mode,
            "armed": data.armed,
        }
        api_callback(
            self.loop,
            self.module["modules.api.mavros.MAVROSSchema"].set_state_message,
            **kwargs,
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
            self.module["modules.api.mavros.MAVROSSchema"].set_vfr_hud_message,
            **kwargs,
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
            self.module["modules.api.mavros.MAVROSSchema"].set_pose_stamped_message,
            **kwargs,
        )

    def imu_callback(self, data):
        kwargs = {
            "seq": data.header.seq,
            "secs": data.header.stamp.secs,
            "nsecs": data.header.stamp.nsecs,
            "frameId": data.header.frame_id,
            "orientationX": data.orientation.x,
            "orientationY": data.orientation.y,
            "orientationZ": data.orientation.z,
            "orientationW": data.orientation.w,
            "angularVelocityX": data.angular_velocity.x,
            "angularVelocityY": data.angular_velocity.y,
            "angularVelocityZ": data.angular_velocity.z,
            "linearAccelerationX": data.linear_acceleration.x,
            "linearAccelerationY": data.linear_acceleration.y,
            "linearAccelerationZ": data.linear_acceleration.z,
        }
        api_callback(
            self.loop,
            self.module["modules.api.mavros.MAVROSSchema"].set_imu_message,
            **kwargs,
        )

    def param_callback(self, data):
        # logging.debug('param callback data: {0}  value type: {1} '.format(data, type(data.value)))
        kwargs = {"id": data.param_id, "value": param_ret_value(data)}
        # api_callback(self.loop, self.module[__name__].set_param, **kwargs)
        # print(kwargs)

    def statustext_callback(self, data):
        if data.name == "/mavros":
            logging.debug("statustext: {0}:{1}".format(data.level, data.msg))
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
                self.module["modules.api.mavros.MAVROSSchema"].set_status_text_message,
                **kwargs,
            )

    def rel_alt_callback(self, data):
        kwargs = {"relativeAltitude": data.data}
        api_callback(
            self.loop,
            self.module["modules.api.mavros.MAVROSSchema"].set_vfr_hud_message,
            **kwargs,
        )
