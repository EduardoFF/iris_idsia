#!/usr/bin/env python


NAME = 'gotowaypoint_server'
import rospy
import roslib

from iris_test.srv import *
from mavros.srv import WaypointGOTO
from mavros.msg import Waypoint


def transform_coordinate(src_epsg, dst_epsg, x, y):
    import ogr, osr

    pointX = -11705274.6374 
    pointY = 4826473.6922
    
    # Spatial Reference System
    inputEPSG = src_epsg
    outputEPSG = dst_epsg

    # create a geometry from coordinates
    point = ogr.Geometry(ogr.wkbPoint)
    point.AddPoint(x, y)

    # create coordinate transformation
    inSpatialRef = osr.SpatialReference()
    inSpatialRef.ImportFromEPSG(inputEPSG)

    outSpatialRef = osr.SpatialReference()
    outSpatialRef.ImportFromEPSG(outputEPSG)

    coordTransform = osr.CoordinateTransformation(inSpatialRef, outSpatialRef)

    # transform point
    point.Transform(coordTransform)

    # print point in EPSG 4326
    return point.GetX(), point.GetY()

def goto_waypoint(req):
    lon,lat = transform_coordinate(21781, 4326, req.x, req.y)
    print "goto lon ",lon, " lat ",lat
#    rospy.wait_for_service('/mavros/mission/goto')
    try:
        goto = rospy.ServiceProxy('/mavros/mission/goto',WaypointGOTO )
        wp = Waypoint()
        """
        uint8 FRAME_GLOBAL = 0
        uint8 FRAME_LOCAL_NED = 1
        uint8 FRAME_MISSION = 2
        uint8 FRAME_GLOBAL_REL_ALT = 3
        uint8 FRAME_LOCAL_ENU = 4
        """

        wp.frame = 3
        """
        uint16 NAV_WAYPOINT = 16
        uint16 NAV_LOITER_UNLIM = 17
        uint16 NAV_LOITER_TURNS = 18
        uint16 NAV_LOITER_TIME = 19
        uint16 NAV_RETURN_TO_LAUNCH = 20
        uint16 NAV_LAND = 21
        uint16 NAV_TAKEOFF = 22
        """
        wp.command = 16
        
        wp.is_current = True
        wp.autocontinue = False
        wp.param1 = 0
        wp.param2 = 0
        wp.param3 = 0
        wp.param4 = 0
        wp.x_lat = lat
        wp.y_long = lon
        wp.z_alt = 5

        print "sending goto waypoint ",wp
        ret = goto(wp)
        print "ret_value ",ret
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
    
    
    return GoToWaypointResponse(0)

def add_two_ints_server():
    rospy.init_node(NAME)
    s = rospy.Service('gotowaypoint', GoToWaypoint, goto_waypoint)

    # spin() keeps Python from exiting until node is shutdown
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
