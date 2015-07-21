#!/usr/bin/env python


NAME = 'gotowaypoint_server'
import rospy
import roslib

from iris_test.srv import *

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
    print transform_coordinate(21781, 4326, req.x, req.y)
    
    
    return GoToWaypointResponse(0)

def add_two_ints_server():
    rospy.init_node(NAME)
    s = rospy.Service('gotowaypoint', GoToWaypoint, goto_waypoint)

    # spin() keeps Python from exiting until node is shutdown
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()