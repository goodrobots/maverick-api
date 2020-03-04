# Samuel Dudley
# September 2018

import os, inspect
from lib import navpy
from util import common, file_tools, mavlink_meta

# avoid SRTM imput for now
# import SRTM
from mavros import mission as ros_mission
import numpy as np

from shapely.geometry import LineString, Point
from shapely import affinity

# Mission generation tool for mavlink enabled vehicles

# generic mission object
class BaseMission(object):
    def __init__(
        self,
        missionID,
        takeoffAlt=5,
        takeoffLoiterTime=5,
        takeoffLoiterAlt=15,
        outputDir=os.path.join(os.path.dirname(os.path.realpath(__file__)), "missions"),
        logName="mission",
    ):
        # setup logger
        self.logger = common.setupLogger(logName)
        # set ouput directory
        self.outputDir = outputDir
        # TODO: Check to make sure the dir exists
        self.latRef = None
        self.lonRef = None
        self.altRef = None
        self.frame = "ned"
        self.points = []
        # setup altitude types
        self.altitudeTypes = {"relative": 3, "terrain": 10}
        self.availableFrames = ["ned", "lla"]
        self.missionID = missionID
        self.takeoffAlt = takeoffAlt
        self.takeoffLoiterTime = takeoffLoiterTime
        self.takeoffLoiterAlt = takeoffLoiterAlt
        self.filePath = os.path.join(self.outputDir, self.missionID + ".txt")
        self.fid = None
        self.missionLine = 0
        self.autoContinue = 1
        self.srtmGrid = None
        self.mavlinkEnums = mavlink_meta.getMavlinkEnums()
        ros_mission.subscribe_waypoints(self.mission_callback)

    def mission_callback(self, data):
        validActions = [
            self.mavlinkEnums["MAV_CMD"]["NAV_WAYPOINT"]["value"],
            self.mavlinkEnums["MAV_CMD"]["NAV_SPLINE_WAYPOINT"]["value"],
        ]
        # self.logger.error(str(data.waypoints))
        for seq, waypoint in enumerate(data.waypoints):
            if waypoint.command in validActions:
                if waypoint.frame == 3:
                    alt = self.altRef + waypoint.z_alt
                else:
                    alt = waypoint.z_alt
                ned = self.getPointsNED([waypoint.x_lat, waypoint.y_long, alt])
                self.logger.error("! seq: {0}, ned: {1}".format(seq, ned))

    def writeWaypointFile(self, actions):
        file_tools.makePath(self.outputDir)
        with open(self.filePath, "w+") as self.fid:
            for action in actions:
                if inspect.ismethod(action):
                    action()
                else:
                    self.writeGenericAction(action)
        return self.filePath

    def writeWaypointLine(self, line, newLine=True, replaceSpace=True):
        if replaceSpace:
            line = line.replace(" ", "\t")
        if newLine:
            line += "\r\n"
        if not self.fid.closed:
            self.fid.write(line)
            self.missionLine += 1
        else:
            # The waypoint file is closed
            self.logger.error("Failed to write to waypoint file")

    def writeHomeLLA(self):
        line = "{0} 0 0 {1} 0.0 0.0 0.0 0.0 {2} {3} {4} {5}".format(
            self.missionLine,
            self.mavlinkEnums["MAV_CMD"]["NAV_WAYPOINT"]["value"],
            self.latRef,
            self.lonRef,
            self.altRef,
            self.autoContinue,
        )
        self.writeWaypointLine(line)

    def writeTakeoffLLA(self):
        line = "{0} 0 {1} {2} 0.0 0.0 0.0 0.0 {3} {4} {5} {6}".format(
            self.missionLine,
            self.mavlinkEnums["MAV_FRAME"]["GLOBAL_RELATIVE_ALT"]["value"],
            self.mavlinkEnums["MAV_CMD"]["NAV_TAKEOFF"]["value"],
            self.latRef,
            self.lonRef,
            self.takeoffAlt,
            self.autoContinue,
        )
        self.writeWaypointLine(line)

    def writeLoiterTime(self, delay=None, lla=None):
        if delay is None:
            delay = self.takeoffLoiterTime
        if delay < 0:
            command = self.mavlinkEnums["MAV_CMD"]["NAV_LOITER_UNLIM"]["value"]
            delay = 0.0
        else:
            command = self.mavlinkEnums["MAV_CMD"]["NAV_LOITER_TIME"]["value"]
        if lla is None:
            msgLat = self.latRef
            msgLon = self.lonRef
            msgAlt = self.takeoffLoiterAlt
        else:
            msgLat = lla[0]
            msgLon = lla[1]
            msgAlt = lla[2]

        line = "{0} 0 {1} {2} {3} 0.0 0.0 0.0 {4} {5} {6} {7}".format(
            self.missionLine,
            self.mavlinkEnums["MAV_FRAME"]["GLOBAL_RELATIVE_ALT"]["value"],
            command,
            delay,
            msgLat,
            msgLon,
            msgAlt,
            self.autoContinue,
        )
        self.writeWaypointLine(line)

    def writeReturnToLaunch(self):
        line = "{0} 0 {1} {2} 0.0 0.0 0.0 0.0 0.0 0.0 0.0 {3}".format(
            self.missionLine,
            self.mavlinkEnums["MAV_FRAME"]["GLOBAL_RELATIVE_ALT"]["value"],
            self.mavlinkEnums["MAV_CMD"]["NAV_RETURN_TO_LAUNCH"]["value"],
            self.autoContinue,
        )
        self.writeWaypointLine(line)

    def writeWaypointLLA(self, lla, holdTime=0, spline=False):
        if spline:
            wp_cmd_val = self.mavlinkEnums["MAV_CMD"]["NAV_SPLINE_WAYPOINT"]["value"]
        else:
            wp_cmd_val = self.mavlinkEnums["MAV_CMD"]["NAV_WAYPOINT"]["value"]
        line = "{0} 0 {1} {2} {3} 0.0 0.0 0.0 {4} {5} {6} {7}".format(
            self.missionLine,
            self.mavlinkEnums["MAV_FRAME"]["GLOBAL_RELATIVE_ALT"]["value"],
            wp_cmd_val,
            holdTime,
            lla[0],
            lla[1],
            lla[2],
            self.autoContinue,
        )
        self.writeWaypointLine(line)

    def writeROILLA(self, lla, relAlt=False, heightOffset=0, endROI=False):
        # NOTE: This interface is DEPRECATED but the new interface is not yet supported by Ardu*
        # NOTE: Alt in this command is m above home point, but we typically deal with GPS / SRTM alt
        # NOTE: Height offset is NED frame and therfore up is -ve
        msgLat = lla[0]
        msgLon = lla[1]
        if endROI:
            msgAlt = 0
            msgLat = 0
            msgLon = 0
        elif relAlt:
            # height relitive to home alt
            msgAlt = lla[2] - heightOffset
        else:
            msgAlt = self.altRef - lla[2] - heightOffset
        line = "{0} 0 0 {1} 0.0 0.0 0.0 0.0 {2} {3} {4} {5}".format(
            self.missionLine,
            self.mavlinkEnums["MAV_CMD"]["DO_SET_ROI"]["value"],
            msgLat,
            msgLon,
            msgAlt,
            self.autoContinue,
        )
        self.writeWaypointLine(line)

    def writeSetYaw(self, desYaw, relYaw=False, CW=True):
        if relYaw:
            msgRelAbs = 1  # Rel rotation mode
            if CW:
                # clockwise
                msgRotDir = 1
            else:
                # anti-clockwise
                msgRotDir = -1
        else:
            msgRelAbs = 0  # Abs rotation mode
            msgRotDir = 0  # only applicable for rel rotations
        desYaw = common.wrap360(desYaw)
        line = "{0} 0 0 {1} {2} 0.0 {3} {4} 0.0 0.0 0.0 {5}".format(
            self.missionLine,
            self.mavlinkEnums["MAV_CMD"]["CONDITION_YAW"]["value"],
            desYaw,
            msgRotDir,
            msgRelAbs,
            self.autoContinue,
        )
        self.writeWaypointLine(line)

    # def writeSetDelay(self, delay, hourUTC= -1, minuteUTC = -1, secondUTC = -1):
    #     # NOTE: all *UTC vars are 24h format, UTC time -1 to ignore
    #     # TODO: check values for either delay = -1 or all others -1
    #     line = "{0} 0 0 {1} {2} {3} {4} {5} 0.0 0.0 0.0 {6}".format(self.missionLine,
    #                                                                 self.mavlinkEnums['MAV_CMD']['NAV_DELAY']['value'],
    #                                                                 delay, hourUTC, minuteUTC, secondUTC, self.autoContinue)
    #     self.writeWaypointLine(line)

    def writeSetDelay(self, delay):
        line = "{0} 0 0 {1} {2} 0.0 0.0 0.0 0.0 0.0 0.0 {3}".format(
            self.missionLine,
            self.mavlinkEnums["MAV_CMD"]["CONDITION_DELAY"]["value"],
            delay,
            self.autoContinue,
        )
        self.writeWaypointLine(line)

    def writePreamble(self):
        self.writeWaypointLine("QGC WPL 120", replaceSpace=False)
        self.missionLine = 0

    def writeGenericAction(self, action):
        line = "{0} {1}".format(self.missionLine, action)
        self.writeWaypointLine(line)

    def checkFrame(self):
        if self.frame.lower() in self.availableFrames:
            return True
        else:
            return False

    def setReferenceLLA(self, LLA=[]):
        # TODO: check LLA length
        self.latRef = LLA[0]
        self.lonRef = LLA[1]
        self.altRef = LLA[2]
        # TODO: when we need the SRTM data we will activate it...
        # self.srtmGrid = SRTM.NEDGround(lla_ref = LLA , width_m = 10000 , height_m = 10000 , step_m = 30, logger = self.logger)

    def setReferenceLatitude(self, lat):
        self.latRef = lat

    def setReferenceLongitude(self, lon):
        self.lonRef = lon

    def setReferenceAltitude(self, alt):
        self.altRef = alt

    def getPointsNED(self, lla):
        ned = navpy.lla2ned(
            lla[0],
            lla[1],
            lla[2],
            lat_ref=self.latRef,
            lon_ref=self.lonRef,
            alt_ref=self.altRef,
        )
        return list(ned)

    def getPointsLLA(self, ned, lla=None):
        if lla is None:
            lat_ref = self.latRef
            lon_ref = self.lonRef
            alt_ref = self.altRef
        else:
            lat_ref = lla[0]
            lon_ref = lla[1]
            alt_ref = lla[2]

        lla = navpy.ned2lla(ned, lat_ref=lat_ref, lon_ref=lon_ref, alt_ref=alt_ref)
        return list(lla)


class GridMission(BaseMission):
    def __init__(
        self,
        missionID,
        takeoffAlt=10,
        takeoffLoiterTime=5,
        takeoffLoiterAlt=15,
        append=False,
    ):
        super(GridMission, self).__init__(
            missionID, takeoffAlt, takeoffLoiterTime, takeoffLoiterAlt
        )
        self.logger.debug(missionID)
        self.out = None
        self.right = None
        self.offset = None
        self.yaw = None
        self.alt = None
        self.lla = None

    def generateGrid(self, out=100, right=200, yaw=45, alt=25, offset=100):
        # TODO: dynamically calculate lane width from sensor FoV and alt
        if self.out is None:
            self.out = out
        if self.right is None:
            self.right = right
        if self.yaw is None:
            self.yaw = common.wrap360(yaw)
        if self.offset is None:
            self.offset = offset
        if self.alt is None:
            self.alt = alt
        laneWidth = (
            self.alt
        )  # set to alt for now, until a value can be calculated from the sensor
        points = []
        if self.right < 0:
            laneWidth = -laneWidth

        for k in range(0, 50, 2):
            points.append((0, k * laneWidth))
            points.append((self.out, k * laneWidth))
            points.append((self.out, (k + 1) * laneWidth))
            points.append((0, (k + 1) * laneWidth))
            if abs(laneWidth * (k + 1)) > abs(self.right):
                break

        line = LineString(points)
        line = affinity.translate(line, xoff=self.offset, yoff=0.0, zoff=0.0)
        line = affinity.rotate(line, angle=self.yaw, origin=(0, 0), use_radians=False)
        llas = [
            self.getPointsLLA([point[0], point[1], 0], lla=self.lla)
            for point in list(line.coords)
        ]
        for lla in llas:
            self.writeWaypointLLA([lla[0], lla[1], self.alt])


class LineMission(BaseMission):
    def __init__(
        self,
        missionID,
        takeoffAlt=10,
        takeoffLoiterTime=5,
        takeoffLoiterAlt=15,
        append=False,
    ):
        super(LineMission, self).__init__(
            missionID, takeoffAlt, takeoffLoiterTime, takeoffLoiterAlt
        )
        self.logger.debug(missionID)
        self.out = None
        self.yaw = None
        self.offset = None
        self.alt = None
        self.lla = None

    def generateLine(self, out=1500, yaw=45, offset=1000, alt=25):
        if self.out is None:
            self.out = out
        if self.yaw is None:
            self.yaw = common.wrap360(yaw)
        if self.offset is None:
            self.offset = offset
        if self.alt is None:
            self.alt = alt
        angleStep = 15
        alpha = np.radians(self.yaw)
        # Calculate near point
        nearPoint = [np.cos(alpha) * self.offset, np.sin(alpha) * self.offset]
        # Calculate far point
        farPoint = [
            np.cos(alpha) * (self.out + self.offset),
            np.sin(alpha) * (self.out + self.offset),
        ]
        nearlla = self.getPointsLLA([nearPoint[0], nearPoint[1], 0], lla=self.lla)
        farlla = self.getPointsLLA([farPoint[0], farPoint[1], 0], lla=self.lla)

        self.writeWaypointLLA([nearlla[0], nearlla[1], self.alt], holdTime=1.0)

        for angleOffset in np.linspace(
            0.0, -90.0, num=int(90.0 / 15) + 1, endpoint=True
        ):
            # for angleOffset in np.linspace(180, 270, num=int(90.0/15)+1, endpoint=True):
            self.writeSetYaw(self.yaw + angleOffset)
            self.writeLoiterTime(delay=1, lla=[nearlla[0], nearlla[1], self.alt])
        self.writeSetYaw(self.yaw - 90)

        self.writeWaypointLLA([farlla[0], farlla[1], self.alt], holdTime=1.0)

        for angleOffset in np.linspace(
            -90.0, 90.0, num=int(180.0 / 15) + 1, endpoint=True
        ):
            self.writeSetYaw(self.yaw + angleOffset)
            self.writeLoiterTime(delay=1, lla=[farlla[0], farlla[1], self.alt])
        self.writeSetYaw(self.yaw + 90)

        self.writeWaypointLLA([nearlla[0], nearlla[1], self.alt], holdTime=1.0)

        # for angleOffset in np.linspace(90.0, 180.0, num=int(90.0/15)+1, endpoint=True):
        for angleOffset in np.linspace(
            90.0, 180.0, num=int(90.0 / 15) + 1, endpoint=True
        ):
            self.writeSetYaw(self.yaw + angleOffset)
            self.writeLoiterTime(delay=1, lla=[nearlla[0], nearlla[1], self.alt])
        self.writeLoiterTime(delay=1, lla=[nearlla[0], nearlla[1], self.alt])

        # # DEBUG
        # tmp = self.getPointsLLA([250, 250, 0])
        # self.writeWaypointLLA([tmp[0], tmp[1], alt])


class CircleMission(BaseMission):
    def __init__(
        self,
        missionID,
        takeoffAlt=10,
        takeoffLoiterTime=5,
        takeoffLoiterAlt=15,
        ROI=[0, 0, None],
    ):
        super(CircleMission, self).__init__(
            missionID, takeoffAlt, takeoffLoiterTime, takeoffLoiterAlt
        )
        self.logger.debug(missionID)
        self.out = None
        self.radius = None
        self.segments = None
        self.spline = False
        self.yaw = None
        self.offset = None
        self.alt = None
        self.lla = None

    def generateCircle(
        self,
        radius=300,
        segments=10,
        alt=50,
        heightOffset=0,
        yaw=45,
        offset=150,
        offset_ref_center=False,
    ):
        if self.radius is None:
            self.radius = radius
        if self.segments is None:
            self.segments = segments
        if self.yaw is None:
            self.yaw = yaw
        if self.offset is None:
            self.offset = offset
        if not offset_ref_center:
            # this is default behaviour
            self.offset += self.radius
        if self.alt is None:
            self.alt = alt

        angles = np.linspace(0.0, 2.0 * np.pi, num=self.segments, endpoint=True)
        points = [
            (np.cos(alpha) * self.radius, np.sin(alpha) * self.radius)
            for alpha in angles
        ]
        points.append((0, 0))
        line = LineString(points)
        # rotate the circle 180 degrees so that the entry point is at the bottom of the pattern
        line = affinity.rotate(line, angle=180, origin=(0, 0), use_radians=False)
        line = affinity.translate(line, xoff=self.offset, yoff=0.0, zoff=0.0)
        line = affinity.rotate(line, angle=self.yaw, origin=(0, 0), use_radians=False)
        llas = [
            self.getPointsLLA([point[0], point[1], 0], lla=self.lla)
            for point in list(line.coords)
        ]
        # The last point is the ROI
        roilla = llas[-1]
        self.logger.warning("roi lla: {0}".format(roilla))
        # start ROI mode
        self.writeROILLA(
            [roilla[0], roilla[1], 0], relAlt=True, heightOffset=0, endROI=False
        )
        # write the path waypoints
        # make the fist waypoint non-spline for now
        self.writeWaypointLLA(
            [llas[0][0], llas[0][1], self.alt], holdTime=1.0, spline=False
        )
        for lla in llas[1:-2]:
            self.writeWaypointLLA([lla[0], lla[1], self.alt], spline=self.spline)
        self.writeWaypointLLA(
            [llas[-2][0], llas[-2][1], self.alt], holdTime=1.0, spline=True
        )
        # finish ROI mode
        self.writeROILLA([0, 0, 0], endROI=True)

        # # DEBUG
        # tmp = self.getPointsLLA([250, 250, 0])
        # self.writeWaypointLLA([tmp[0], tmp[1], alt])


class TakeoffMission(BaseMission):
    """A mission that takesoff and loiters for a set time"""

    def __init__(
        self,
        missionID,
        takeoffAlt=10,
        takeoffLoiterTime=2,
        takeoffLoiterAlt=15,
        append=False,
    ):
        super(TakeoffMission, self).__init__(
            missionID, takeoffAlt, takeoffLoiterTime, takeoffLoiterAlt
        )
        self.logger.debug(missionID)
        self.yaw = None

    def generateTakeoff(self, yaw=90):
        if self.yaw is None:
            self.yaw = yaw
        self.logger.warning("takeoff yaw: {0}".format(self.yaw))
        self.writePreamble()
        self.writeHomeLLA()
        self.writeTakeoffLLA()
        loiterPoint = Point(
            0.0, 0.0
        )  # generate a point directly above the takeoff location
        # loiterPoint = Point(self.takeoffLoiterAlt, 0.0) # generate a point just as far out as it is high
        # loiterPoint = affinity.rotate(loiterPoint, angle=self.yaw, origin=(0,0), use_radians=False)
        coords = list(loiterPoint.coords)[0]
        lla = self.getPointsLLA([coords[0], coords[1], 0])
        self.writeSetYaw(self.yaw)
        self.writeLoiterTime(
            delay=self.takeoffLoiterTime, lla=[lla[0], lla[1], self.takeoffLoiterAlt]
        )


if __name__ == "__main__":
    mission = GridMission("grid_test")
    mission.setReferenceLLA([-35.3615074158, 149.163650513, 500])
    actions = [
        mission.writePreamble,
        mission.writeHomeLLA,
        mission.writeTakeoffLLA,
        mission.writeLoiterTime,
        mission.generateGrid,
        mission.writeReturnToLaunch,
    ]
    mission.writeWaypointFile(actions)
