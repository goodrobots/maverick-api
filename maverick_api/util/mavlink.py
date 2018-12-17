# MAVLink utils for maverick
from pymavlink import mavutil

def get_meta_string(msg):
    if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
        # we are connected to a PX4
        # load the PX4 params
        return 'PX4'
        
    elif msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
        # we are connected to a pixhawk or similar
        # work out the vehicle type
        if msg.type in [mavutil.mavlink.MAV_TYPE_QUADROTOR, mavutil.mavlink.MAV_TYPE_HEXAROTOR,
                        mavutil.mavlink.MAV_TYPE_OCTOROTOR, mavutil.mavlink.MAV_TYPE_TRICOPTER,
                        mavutil.mavlink.MAV_TYPE_COAXIAL, mavutil.mavlink.MAV_TYPE_HELICOPTER,
                        mavutil.mavlink.MAV_TYPE_DODECAROTOR]:
            # ArduCopter
            return 'ArduCopter'
        elif msg.type in [mavutil.mavlink.MAV_TYPE_FIXED_WING, mavutil.mavlink.MAV_TYPE_VTOL_TILTROTOR,
                          mavutil.mavlink.MAV_TYPE_VTOL_DUOROTOR, mavutil.mavlink.MAV_TYPE_VTOL_QUADROTOR]:
            # get the ArduPlane params
            # ArduPlane
            return 'ArduPlane'
        elif msg.type in [mavutil.mavlink.MAV_TYPE_GROUND_ROVER, mavutil.mavlink.MAV_TYPE_SURFACE_BOAT]:
            # APMrover2
            return 'APMrover2'
        elif msg.type in [mavutil.mavlink.MAV_TYPE_SUBMARINE]:
            # ArduSub
            return 'ArduSub'
        elif msg.type in [mavutil.mavlink.MAV_TYPE_ANTENNA_TRACKER]:
            # AntennaTracker
            return 'AntennaTracker'
        else:
            # no known param meta...
            return ''