#!/usr/bin/env python3
#
#   showtouchandgo.py
#
#   Show the "Touch and Go" cradles and obstacle.
#
#   This generates visualization messages, which rviz can render.
#
#   Publish:    /visualization_marker_array     visualization_msgs/MarkerArray
import rospy

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


#
#  Basic Marker
#
def BasicMarker():
    # Configure the basics/shared parameters of the markers.
    marker = Marker()
    marker.header.frame_id    = "world"
    marker.header.stamp       = rospy.Time.now()
    marker.ns                 = "environment"
    marker.action             = Marker.ADD
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    marker.color.a            = 1.0     # Make non-transparent!
    return marker

#
#  Add a Cube
#
def AddCube(markers, position, scale, color):
    # Add a single cube
    markers.append(BasicMarker())
    markers[-1].id   = len(markers)
    markers[-1].type = Marker.CUBE
    markers[-1].pose.position.x = position[0]
    markers[-1].pose.position.y = position[1]
    markers[-1].pose.position.z = position[2]
    markers[-1].scale.x         = scale[0]
    markers[-1].scale.y         = scale[1]
    markers[-1].scale.z         = scale[2]
    markers[-1].color.r         = color[0]
    markers[-1].color.g         = color[1]
    markers[-1].color.b         = color[2]

#
#  Add a Cradle
#
def AddCradle(markers, p, colors):
    # Set the dimensions.
    d = 0.1     # Diameter of cube
    l = 0.06    # Length of rail
    w = 0.02    # Width of rail
    s = (d+w)/2 # Shift from center of cube to center of rail

    # Pull out the colors.
    (cbase, cposx, cnegx, cposy, cnegy) = colors

    # Add the elements.
    AddCube(markers, (p[0],   p[1],   d/2),   (d, d, d), cbase)
    AddCube(markers, (p[0]+s, p[1],   d/2+s), (w, l, w), cposx)
    AddCube(markers, (p[0]-s, p[1],   d/2+s), (w, l, w), cnegx)
    AddCube(markers, (p[0],   p[1]+s, d/2+s), (l, w, w), cposy)
    AddCube(markers, (p[0],   p[1]-s, d/2+s), (l, w, w), cnegy)


#
#  Main Code
#
if __name__ == "__main__":
    # Prepare the node.
    rospy.init_node('showtouchandgo')

    # Prepare a publisher (latching so new subscribers also get the msg).
    pub = rospy.Publisher("/visualization_marker_array", MarkerArray,
                          queue_size=1, latch=True)

    # Wait for '/rviz' to subscribe.
    rospy.loginfo("Waiting for rviz to subscribe...")
    while not rospy.is_shutdown() and not pub.impl.has_connection('/rviz'):
        print(pub.get_num_connections(), end="..", flush=True)
        rospy.sleep(0.1)
    print()

    # Create an (so far empty) marker list.
    array = MarkerArray()
    id    = 0

    # Add the left side.
    AddCradle(array.markers, (0.3, 0.5),
              ((0.0, 0.0,  1.0),
               (1.0, 0.65, 0.0),
               (1.0, 0.0,  0.0),
               (0.0, 1.0,  0.0),
               (1.0, 1.0,  0.0)))

    # Add the right side.
    AddCradle(array.markers, (-0.3, 0.5),
              ((1.0, 1.0,  0.0),
               (1.0, 0.65, 0.0),
               (1.0, 0.0,  0.0),
               (0.0, 0.0,  1.0),
               (1.0, 1.0,  1.0)))

    # Add the fence.
    AddCube(array.markers, (0.0, 0.5, 0.25), (0.02, 0.6, 0.5), (1.0, 0.0, 1.0))
            
    # Publish.
    rospy.loginfo("Showing the markers...")
    pub.publish(array)

    # No need to spin, as we know rviz was subscribing.
    # Spin: Sit here doing nothing (other than responding to ROS events)
    # rospy.loginfo("Hit Ctrl-c to kill...")
    # rospy.spin()

