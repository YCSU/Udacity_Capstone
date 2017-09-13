#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from tensorflow.transformations import euler_from_quaternion
import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 20 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        rospy.spin()

    def pose_cb(self, msg):
        # TODO: Implement
        pose = msg.pose
        frame_id = msg.header.frame_id
        if self.base_waypoints != None:
            # find closet waypoint ahead
            nearest_idx = self.find_closest_waypoint_idx(pose)
            target_speed = 4.4
            next_waypoints = self.waypoints[nearest_idx:nearest_idx+LOOKAHEAD_WPS]
            
            for waypoint in next_waypoints:
                waypoint.twist.twist.linear.x = target_speed

            lane = self.create_lane(frame_id, next_waypoints)
            self.final_waypoints_pub.publish(lane)

    def waypoints_cb(self, data):
        self.base_waypoints = data.waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def cum_distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def distance(self, waypoint, pose):
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        return dl(waypoint.pose.pose.position, pose.position)

    def find_closest_waypoint_idx(self, pose):
        """
        find the closest waypoint wrt car's current position
        ======
        Args:
            pose: a pose object

        Returns:
            int: index of the closet waypoint
        """
        dist = float('inf') # a very large number
        for idx,waypoint in enumerate(self.base_waypoints):
            temp_dist = self.distance(waypoint, pose)
            if dist > temp_dist:
                dist = temp_dist
                nearest_idx = idx

            if self.is_behind(idx, pose):
                nearest_idx += 1

        return nearest_idx

    def is_behind(self, idx, pose):
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        wp_pos = self.base_waypoints[idx].pose.pose.position
        dx = wp_pos.x - pose.position.x
        dy = wp_pos.y - pose.position.y
        
        dx_car = dx * cos(raw) + dy * sin(yaw)
        if dx_car > 0:
            return False
        return True 

    def create_lane(self, frame_id, waypoints):
        lane = Lane()
        lane.header.frame_id  =frame_id
        lane.waypoints = waypoints
        lane.header.stamp = rospy.Time.now()
        return lane


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
