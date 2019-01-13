import functools
import logging
import time

from modules.api import moduleBase
from modules.api import schemaBase
from modules.api import api_callback

from modules.base.tornadoql.session_control import GraphQLSession

# mavros & ros imports
import rospy
import mavros
import mavros.utils
from mavros_msgs.srv import StreamRate, StreamRateRequest
from mavros_msgs.msg import Param  # callback msg on param change
from mavros.param import param_ret_value

from modules.api.mavros.mavros_mission import (
    MissionSchema,
    MissionInterface,
)
from modules.api.mavros.mavros_nav_sat_fix import (
    NavSatFixSchema,
    NavSatFixInterface,
)
from modules.api.mavros.mavros_vehicle_info import (
    VehicleInfoSchema,
    VehicleInfoInterface,
)
from modules.api.mavros.mavros_imu import (
    ImuSchema,
    ImuInterface,
)
from modules.api.mavros.mavros_vehicle_state import (
    VehicleStateSchema,
    VehicleStateInterface,
)
from modules.api.mavros.mavros_pose_stamped import (
    PoseStampedSchema,
    PoseStampedInterface,
)
from modules.api.mavros.mavros_vfr_hud import (
    VfrHudSchema,
    VfrHudInterface,
)
from modules.api.mavros.mavros_status_text import (
    StatusTextSchema,
    StatusTextInterface,
)

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

        self.q = {}

        self.m = {}

        self.s = {}


class MAVROSConnection(moduleBase):
    def __init__(self, loop, module):
        super().__init__(loop, module)
        # Attributes
        self.mission_interface = MissionInterface(self.loop, self.module)

    def run(self):
        self.connect()
        # self.topics()
        self.streams()
        self.vehicle_info_interface = VehicleInfoInterface(self.loop, self.module)
        self.mission_interface.mission_waypoints()
        self.nav_sat_fix_interface = NavSatFixInterface(self.loop, self.module)
        self.vehicle_state_interface = VehicleStateInterface(self.loop, self.module) # <-- causing errors
        self.status_text_interface = StatusTextInterface(self.loop, self.module)
        self.vfr_hud_interface = VfrHudInterface(self.loop, self.module)
        self.pose_stamped_interface = PoseStampedInterface(self.loop, self.module)
        self.imu_interface = ImuInterface(self.loop, self.module) # <-- causing errors 
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
                    application_log.debug(
                        "Set stream {} rate {}".format(stream_id, stream_rate)
                    )
                except rospy.ServiceException as ex:
                    application_log.error(
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

    def listener(self):
        rospy.Subscriber("/mavros/param_value", Param, self.param_callback)

    def topics(self):
        topics = rospy.get_published_topics()
        for topic in topics:
            application_log.info(topic)

    @functools.lru_cache(maxsize=10)  # cache the param meta for each vehicle
    def params(self, meta_string="ArduCopter"):
        # TODO: make vehicle dynamic and chosse between px4 and ardupilot
        from modules.base.param.parse_param_xml import get_param_meta
        from mavros.param import param_get_all

        param_received, param_list = param_get_all(False)
        application_log.debug("Parameters received: {0}".format(param_received))

        param_meta_vehicle = {}
        for param in param_list:
            kwargs = {"id": param.param_id, "value": param.param_value}
            # add_ioloop_callback(UpdateParameter().mutate, **kwargs)

            param_meta_vehicle[param.param_id] = {
                "group": param.param_id.split("_")[0].strip().rstrip("_").upper()
            }
            application_log.debug(
                "param get {0}:{1}  {2}".format(
                    param.param_id, param.param_value, type(param.param_value)
                )
            )

        # TODO: handle IOError when mavlink-router is not connected to the AP but mavros is running

        if options.debug:
            start_time = time.time()
        application_log.debug("starting parameter meta fetch")
        param_meta_server = get_param_meta(meta_string)
        application_log.debug("finished parameter meta fetch")
        if options.debug:
            application_log.debug(
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
        application_log.debug(
            "param set {0}:{1}  {2}".format(param_data["id"], param_data["value"], ret)
        )
        # application_log.debug('{0}'.format(ret_param))
        return ret

    def param_callback(self, data):
        # application_log.debug('param callback data: {0}  value type: {1} '.format(data, type(data.value)))
        kwargs = {"id": data.param_id, "value": param_ret_value(data)}
        # api_callback(self.loop, self.module[__name__].set_param, **kwargs)
        # print(kwargs)
