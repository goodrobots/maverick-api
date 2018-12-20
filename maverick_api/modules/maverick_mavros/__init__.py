import logging
import time

from modules import moduleBase
from modules import schemaBase
from modules import api_callback

from util.mavlink import get_meta_string

# mavros & ros imports
import rospy
import mavros
import mavros.utils
from std_msgs.msg import String
from rosgraph_msgs.msg import Log
from mavros_msgs.msg import State, VFR_HUD
from mavros_msgs.srv import StreamRate, StreamRateRequest
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import PoseStamped
from mavros_maverick.msg import Param  # callback msg on param change
from mavros_maverick.srv import VehicleInfo
from mavros.param import param_ret_value
from mavros import mission

app_log = logging.getLogger("tornado.application")

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

nav_sat_fix_data = {}


class MAVROSSchema(schemaBase):
    def __init__(self):
        super().__init__()

        self.telem_message = GraphQLInterfaceType(
            "TelemMessage",
            lambda: {
                "id": GraphQLField(
                    GraphQLNonNull(GraphQLString), description="The id of the message."
                ),
                "seq": GraphQLField(GraphQLInt, description="The sequence number of the message."),
                "secs": GraphQLField(GraphQLInt, description=""),
                "nsecs": GraphQLField(GraphQLInt, description=""),
                "frame_id": GraphQLField(GraphQLString, description=""),
            },
        )

        self.nav_sat_fix_message_type = GraphQLObjectType(
            "NavSatFixMessage",
            lambda: {
                "status_status": GraphQLField(GraphQLInt, description=""),
                "status_service": GraphQLField(GraphQLInt, description=""),
                "latitude": GraphQLField(GraphQLFloat, description=""),
                "longitude": GraphQLField(GraphQLFloat, description=""),
                "altitude": GraphQLField(GraphQLFloat, description=""),
                # TODO: position_covariance array
                "position_covariance_type": GraphQLField(GraphQLInt, description=""),
            },
            interfaces=[self.telem_message],
            description="MAVROS NavSatFixMessage",
        )

        self.q = {
            "NavSatFixMessage": GraphQLField(
                self.nav_sat_fix_message_type, args={}, resolve=self.get_nav_sat_fix_message
            )
        }

        self.m = {
            "NavSatFixMessage": GraphQLField(
                self.nav_sat_fix_message_type,
                args=self.get_mutation_args(self.nav_sat_fix_message_type),
                resolve=self.set_nav_sat_fix_message,
            )
        }

        self.s = {
            "Authentication": GraphQLField(
                self.nav_sat_fix_message_type, subscribe=self.sub_nav_sat_fix_message, resolve=None
            )
        }

    def get_nav_sat_fix_message(self, root, info, **kwargs):
        """NavSatFixMessage query handler"""
        return nav_sat_fix_data

    def set_nav_sat_fix_message(self, root, info, **kwargs):
        """NavSatFixMessage mutation handler"""
        updated_dict = {**nav_sat_fix_data, **kwargs}
        self.subscriptions.emit(__name__, {"NavSatFixMessage": updated_dict})
        nav_sat_fix_data = updated_dict
        return updated_dict

    def sub_nav_sat_fix_message(self, root, info, **kwargs):
        """NavSatFixMessage subscription handler"""
        return EventEmitterAsyncIterator(self.subscriptions, __name__)


class MAVROSConnection(moduleBase):
    def __init__(self, config, loop, modules):
        super().__init__(config, loop, modules)
        # Attributes
        self.info = None
        self.meta_string = ""  # meta string used to load the correct params
        self.waypoints = None

    def run(self):
        self.connect()
        # self.topics()
        self.streams()
        self.vehicle_info()  # work out the autopilot and vehicle type
        self.params(
            meta_string=self.meta_string
        )  # this is called from the IOloop once we know the vehicle type
        self.mission_waypoints()
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
                        stream_id=stream_id, message_rate=stream_rate, on_off=(stream_rate != 0)
                    )
                    app_log.debug("Set stream {} rate {}".format(stream_id, stream_rate))
                except rospy.ServiceException as ex:
                    app_log.error(
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
        get_vehicle_info = rospy.ServiceProxy(mavros.get_topic("get_vehicle_info"), VehicleInfo)
        try:
            self.info = get_vehicle_info()
            print(self.info)
            self.meta_string = get_meta_string(self.info)
            app_log.debug(self.info)
            app_log.info(self.meta_string)
        except rospy.ServiceException as ex:
            app_log.error("An error occurred while retrieving vehicle info via ROS: {0}".format(ex))

    def mission_waypoints(self):
        self.waypoints = mission.pull()
        mission.subscribe_waypoints(self.mission_callback)

    def listener(self):
        rospy.Subscriber("/mavros/state", State, self.state_callback)
        rospy.Subscriber("/mavros/vfr_hud", VFR_HUD, self.vfr_hud_callback)
        rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.nav_sat_fix_callback)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_stamped_callback)
        rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)
        rospy.Subscriber("/mavros/param_value", Param, self.param_callback)
        rospy.Subscriber("/rosout", Log, self.statustext_callback)

    def topics(self):
        topics = rospy.get_published_topics()
        for topic in topics:
            app_log.info(topic)

    def params(self, meta_string="ArduCopter"):
        # TODO: make vehicle dynamic and chosse between px4 and ardupilot
        from param.parse_param_xml import get_param_meta

        # from api.schema import Parameters
        from mavros.param import param_get_all

        # TODO: UPDATE CALLBACK
        # Parameters.callback = self.param_set_callback
        param_received, param_list = param_get_all(False)
        app_log.debug("Parameters received: {0}".format(param_received))

        param_meta_vehicle = {}
        for param in param_list:
            kwargs = {"id": param.param_id, "value": param.param_value}
            # add_ioloop_callback(UpdateParameter().mutate, **kwargs)

            param_meta_vehicle[param.param_id] = {
                "group": param.param_id.split("_")[0].strip().rstrip("_").upper()
            }
            app_log.debug(
                "param get {0}:{1}  {2}".format(
                    param.param_id, param.param_value, type(param.param_value)
                )
            )

        # TODO: handle IOError when mavlink-router is not connected to the AP but mavros is running

        if self.config["APP_DEBUG"]:
            start_time = time.time()
        app_log.debug("starting parameter meta fetch")
        param_meta_server = get_param_meta(meta_string)
        app_log.debug("finished parameter meta fetch")
        if self.config["APP_DEBUG"]:
            app_log.debug("parameter meta fetch took {0}s".format(time.time() - start_time))
        # Parameters.meta = merge_two_dicts(param_meta_vehicle, param_meta_server)

    def param_set_callback(self, param_data):
        # from api.schema import Parameters
        # TODO:
        # write unit test
        # clean up logic
        from mavros.param import param_set

        mavros.set_namespace("mavros")
        ret = param_set(param_data["id"].encode("ascii", "ignore"), float(param_data["value"]))
        # if ret == param_value:
        #     # param set worked
        #     pass
        # else:
        #     # param set failed
        #     pass

        # # check to see if the set value matches the provided value
        # ret_param = param_get(param_data['id'])
        app_log.debug("param set {0}:{1}  {2}".format(param_data["id"], param_data["value"], ret))
        # app_log.debug('{0}'.format(ret_param))
        return ret

    def state_callback(self, data):
        kwargs = {
            "seq": data.header.seq,
            "frame_id": data.header.frame_id,
            "guided": data.guided,
            "nsecs": data.header.stamp.nsecs,
            "system_status": data.system_status,
            "secs": data.header.stamp.secs,
            "connected": data.connected,
            "mode": data.mode,
            "armed": data.armed,
        }
        # api_callback(self.loop, self.modules[__name__].set_state, **kwargs)

    def vfr_hud_callback(self, data):
        kwargs = {
            "seq": data.header.seq,
            "secs": data.header.stamp.secs,
            "nsecs": data.header.stamp.nsecs,
            "frame_id": data.header.frame_id,
            "airspeed": data.airspeed,
            "groundspeed": data.groundspeed,
            "heading": data.heading,
            "throttle": data.throttle,
            "altitude": data.altitude,
            "climb": data.climb,
        }
        # api_callback(self.loop, self.modules[__name__].set_vfr_hud, **kwargs)

    def nav_sat_fix_callback(self, data):
        kwargs = {
            "seq": data.header.seq,
            "secs": data.header.stamp.secs,
            "nsecs": data.header.stamp.nsecs,
            "frame_id": data.header.frame_id,
            "status_status": data.status.status,
            "status_service": data.status.service,
            "latitude": data.latitude,
            "longitude": data.longitude,
            "altitude": data.altitude,
            "position_covariance_type": data.position_covariance_type,
        }
        # api_callback(self.loop, self.modules[__name__].set_nav_sat_fix, **kwargs)

    def pose_stamped_callback(self, data):
        kwargs = {
            "seq": data.header.seq,
            "secs": data.header.stamp.secs,
            "nsecs": data.header.stamp.nsecs,
            "frame_id": data.header.frame_id,
            "pose_position_x": data.pose.position.x,
            "pose_position_y": data.pose.position.y,
            "pose_position_z": data.pose.position.z,
            "pose_orientation_x": data.pose.orientation.x,
            "pose_orientation_y": data.pose.orientation.y,
            "pose_orientation_z": data.pose.orientation.z,
            "pose_orientation_w": data.pose.orientation.w,
        }
        # api_callback(self.loop, self.modules[__name__].set_pose_stamped, **kwargs)
        # print(kwargs)

    def imu_callback(self, data):
        kwargs = {
            "seq": data.header.seq,
            "secs": data.header.stamp.secs,
            "nsecs": data.header.stamp.nsecs,
            "frame_id": data.header.frame_id,
            "orientation_x": data.orientation.x,
            "orientation_y": data.orientation.y,
            "orientation_z": data.orientation.z,
            "orientation_w": data.orientation.w,
            "angular_velocity_x": data.angular_velocity.x,
            "angular_velocity_y": data.angular_velocity.y,
            "angular_velocity_z": data.angular_velocity.z,
            "linear_acceleration_x": data.linear_acceleration.x,
            "linear_acceleration_y": data.linear_acceleration.y,
            "linear_acceleration_z": data.linear_acceleration.z,
        }
        # api_callback(self.loop, self.modules[__name__].set_imu, **kwargs)
        user2 = dict(id="2", userName="ben", password="password132")
        api_callback(self.loop, self.modules["modules.maverick_authentication"].set_auth, **user2)

    def param_callback(self, data):
        # app_log.debug('param callback data: {0}  value type: {1} '.format(data, type(data.value)))
        kwargs = {"id": data.param_id, "value": param_ret_value(data)}
        # api_callback(self.loop, self.modules[__name__].set_param, **kwargs)
        # print(kwargs)

    def mission_callback(self, data):
        for seq, waypoint in enumerate(data.waypoints):
            kwargs = {
                "seq": seq,
                "frame": waypoint.frame,
                "command": waypoint.command,
                "is_current": waypoint.is_current,
                "autocontinue": waypoint.autocontinue,
                "param1": waypoint.param1,
                "param2": waypoint.param2,
                "param3": waypoint.param3,
                "param4": waypoint.param4,
                "latitude": waypoint.x_lat,
                "longitude": waypoint.y_long,
                "altitude": waypoint.z_alt,
            }
            # api_callback(self.loop, self.modules[__name__].set_mission, **kwargs)
            # print(kwargs)

    def statustext_callback(self, data):
        if data.name == "/mavros":
            app_log.debug("statustext: {0}:{1}".format(data.level, data.msg))
            kwargs = {
                "seq": data.header.seq,
                "secs": data.header.stamp.secs,
                "nsecs": data.header.stamp.nsecs,
                "frame_id": data.header.frame_id,
                "level": data.level,
                "message": data.msg,
            }
            # api_callback(self.loop, self.modules[__name__].set_statustext, **kwargs)
            user2 = dict(id="2", userName="ned", password="password132")
            api_callback(
                self.loop, self.modules["modules.maverick_authentication"].set_auth, **user2
            )

