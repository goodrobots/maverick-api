import logging
from threading import Thread

from maverick_api.modules import moduleBase

# mavros & ros imports
import rospy
import mavros
import mavros.utils
from mavros_msgs.srv import StreamRate, StreamRateRequest

from maverick_api.modules.api.mavros.mavros_mission import MissionInterface
from maverick_api.modules.api.mavros.mavros_nav_sat_fix import NavSatFixInterface
from maverick_api.modules.api.mavros.mavros_vehicle_info import VehicleInfoInterface
from maverick_api.modules.api.mavros.mavros_imu import ImuInterface
from maverick_api.modules.api.mavros.mavros_vehicle_state import VehicleStateInterface
from maverick_api.modules.api.mavros.mavros_pose_stamped import PoseStampedInterface
from maverick_api.modules.api.mavros.mavros_vfr_hud import VfrHudInterface
from maverick_api.modules.api.mavros.mavros_status_text import StatusTextInterface
from maverick_api.modules.api.mavros.mavros_mode import ModeInterface
from maverick_api.modules.api.mavros.mavros_param import ParamInterface

application_log = logging.getLogger("tornado.application")


class MAVROSConnection(moduleBase):
    def __init__(self):
        super().__init__()
        # Attributes
        self.mission_interface = MissionInterface(self.loop, self.module)
        self._thread = None

    def start(self):
        self._thread = Thread(target=self.run)
        self._thread.daemon = True
        self._thread.start()

    def run(self):
        self.connect()
        self.topics()
        self.streams()
        self.vehicle_info_interface = VehicleInfoInterface(self.loop, self.module)
        self.param_interface = ParamInterface(
            self.loop,
            self.module,
            meta_string=self.vehicle_info_interface.get_meta_string(),
        )
        self.mission_interface.mission_waypoints()
        self.nav_sat_fix_interface = NavSatFixInterface(self.loop, self.module)
        self.vehicle_state_interface = VehicleStateInterface(self.loop, self.module)
        self.status_text_interface = StatusTextInterface(self.loop, self.module)
        self.vfr_hud_interface = VfrHudInterface(self.loop, self.module)
        self.pose_stamped_interface = PoseStampedInterface(self.loop, self.module)
        self.imu_interface = ImuInterface(self.loop, self.module)
        self.mode_interface = ModeInterface(self.loop, self.module)

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

    def topics(self):
        topics = rospy.get_published_topics()
        for topic in topics:
            application_log.info(topic)
