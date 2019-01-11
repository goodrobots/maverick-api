# MAVLink utils for maverick
from pymavlink import mavutil


def get_vehicle_strings(msg):
    # TODO: Use the mavlink defines to generate the string
    autopilot_string = ""
    type_string = ""
    parameter_string = ""

    # Work out the autopilot type
    if msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
        # we are connected to a PX4
        # load the PX4 params
        autopilot_string = "PX4"

    elif msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
        # we are connected to a pixhawk or similar
        autopilot_string = "ArduPilot"

    else:
        autopilot_string = "Unknown"

    # Work out the vehicle type
    if msg.type in [
        mavutil.mavlink.MAV_TYPE_QUADROTOR,
        mavutil.mavlink.MAV_TYPE_HEXAROTOR,
        mavutil.mavlink.MAV_TYPE_OCTOROTOR,
        mavutil.mavlink.MAV_TYPE_TRICOPTER,
        mavutil.mavlink.MAV_TYPE_COAXIAL,
        mavutil.mavlink.MAV_TYPE_HELICOPTER,
        mavutil.mavlink.MAV_TYPE_DODECAROTOR,
    ]:
        type_string = "Copter"

    elif msg.type in [
        mavutil.mavlink.MAV_TYPE_FIXED_WING,
        mavutil.mavlink.MAV_TYPE_VTOL_TILTROTOR,
        mavutil.mavlink.MAV_TYPE_VTOL_DUOROTOR,
        mavutil.mavlink.MAV_TYPE_VTOL_QUADROTOR,
    ]:
        type_string = "Plane"

    elif msg.type in [
        mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
        mavutil.mavlink.MAV_TYPE_SURFACE_BOAT,
    ]:
        type_string = "Rover"

    elif msg.type in [mavutil.mavlink.MAV_TYPE_SUBMARINE]:
        type_string = "ArduSub"

    elif msg.type in [mavutil.mavlink.MAV_TYPE_ANTENNA_TRACKER]:
        type_string = "Tracker"
    else:
        # Unknown type
        type_string = "Unknown"

    return (autopilot_string, type_string, parameter_string)
