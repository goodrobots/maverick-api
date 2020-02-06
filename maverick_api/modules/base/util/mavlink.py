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
        parameter_string = "PX4"

    elif msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA:
        # we are connected to a pixhawk or similar
        autopilot_string = "ArduPilot"

    else:
        autopilot_string = "Unknown"
        parameter_string = None

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
        if msg.type == mavutil.mavlink.MAV_TYPE_QUADROTOR:
            type_string = "Quadrotor"
        elif msg.type == mavutil.mavlink.MAV_TYPE_HEXAROTOR:
            type_string = "Hexarotor"
        elif msg.type == mavutil.mavlink.MAV_TYPE_OCTOROTOR:
            type_string = "Octorotor"
        elif msg.type == mavutil.mavlink.MAV_TYPE_COAXIAL:
            type_string = "Coaxial"
        elif msg.type == mavutil.mavlink.MAV_TYPE_HELICOPTER:
            type_string = "Helicopter"
        elif msg.type == mavutil.mavlink.MAV_TYPE_DODECAROTOR:
            type_string = "Dodecarotor"
        else:
            type_string = "Copter"

        if autopilot_is_ardupilot(autopilot_string):
            parameter_string = "ArduCopter"

    elif msg.type in [
        mavutil.mavlink.MAV_TYPE_FIXED_WING,
        mavutil.mavlink.MAV_TYPE_VTOL_TILTROTOR,
        mavutil.mavlink.MAV_TYPE_VTOL_DUOROTOR,
        mavutil.mavlink.MAV_TYPE_VTOL_QUADROTOR,
    ]:
        if msg.type == mavutil.mavlink.MAV_TYPE_FIXED_WING:
            type_string = "Plane"
        elif msg.type == mavutil.mavlink.MAV_TYPE_VTOL_TILTROTOR:
            type_string = "VTOL Tiltrotor"
        elif msg.type == mavutil.mavlink.MAV_TYPE_VTOL_DUOROTOR:
            type_string = "VTOL Duorotor"
        elif msg.type == mavutil.mavlink.MAV_TYPE_VTOL_QUADROTOR:
            type_string = "VTOL Quadrotor"
        else:
            type_string = "Plane"

        if autopilot_is_ardupilot(autopilot_string):
            parameter_string = "ArduPlane"

    elif msg.type in [
        mavutil.mavlink.MAV_TYPE_GROUND_ROVER,
        mavutil.mavlink.MAV_TYPE_SURFACE_BOAT,
    ]:
        if msg.type == mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            type_string = "Rover"
        elif msg.type == mavutil.mavlink.MAV_TYPE_SURFACE_BOAT:
            type_string = "Boat"
        else:
            type_string = "Rover"

        if autopilot_is_ardupilot(autopilot_string):
            parameter_string = "APMrover2"

    elif msg.type in [mavutil.mavlink.MAV_TYPE_SUBMARINE]:
        type_string = "Submarine"
        if autopilot_is_ardupilot(autopilot_string):
            parameter_string = "ArduSub"

    elif msg.type in [mavutil.mavlink.MAV_TYPE_ANTENNA_TRACKER]:
        type_string = "Tracker"
        if autopilot_is_ardupilot(autopilot_string):
            parameter_string = "AntennaTracker"
    else:
        # Unknown type
        type_string = "Unknown"

    return (autopilot_string, type_string, parameter_string)


def autopilot_is_ardupilot(autopilot_string):
    if autopilot_string == "ArduPilot":
        return True
    else:
        return False
