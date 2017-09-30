#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
SPEED = 10
END_INDEX = 10

class WaypointUpdater(object):
    def __init__(self):
		rospy.init_node('waypoint_updater')

		rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
		rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
		rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
		

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
		self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)
		self.car_position_pub = rospy.Publisher('car_position', Int32, queue_size=1)

        # TODO: Add other member variables you need below
		self.pose = None
		self.waypoints = None
		self.tf_light_index = None

		self.loop()

    def loop(self):
		rate = rospy.Rate(50) # 50Hz
        
		while not rospy.is_shutdown():
			rate.sleep()
			lane = Lane()

			if self.waypoints is not None and self.pose is not None:
				closet_waypoint_index = self.get_closest_waypoint_index(self.waypoints.waypoints, self.pose)
				print("Car Way point index-->" + str(closet_waypoint_index))
				print("Car X,Y Position-->" + str(self.pose.position.x) + "  "  + str(self.pose.position.y))
				print("Closest Waypoint X,Y Position-->" + str(self.waypoints.waypoints[closet_waypoint_index].pose.pose.position.x) + "  "  + str(self.waypoints.waypoints[closet_waypoint_index].pose.pose.position.y))
				end_index = None
				start_index = None
				if self.tf_light_index != -1 and self.tf_light_index >= closet_waypoint_index and self.tf_light_index <=closet_waypoint_index+LOOKAHEAD_WPS:
					print("Traffic light is Red, preparing to stop")
					end_index = self.tf_light_index - closet_waypoint_index
					start_index = end_index - END_INDEX
					if start_index < 0:
						start_index = 0
				
				lane.waypoints = self.waypoints.waypoints[closet_waypoint_index:closet_waypoint_index+LOOKAHEAD_WPS]

				for waypoint in lane.waypoints:
					waypoint.twist.twist.linear.x = SPEED
				
				if end_index is not None:
					for waypoint in lane.waypoints[start_index:end_index]:
						waypoint.twist.twist.linear.x = 0.0
				
				self.final_waypoints_pub.publish(lane)
				self.car_position_pub.publish(closet_waypoint_index)
			
			
		
    def pose_cb(self, msg):
        # TODO: Implement
		self.pose = msg.pose
	

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.tf_light_index = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def get_closest_waypoint_index(self, waypoints, pose):
		dist = float('inf')
		index = 0
		selected_index = None
		dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
		for waypoint in waypoints:
			temp_dist = dl(waypoint.pose.pose.position, pose.position)
			if temp_dist < dist:
				dist = temp_dist
				selected_index = index
			index = index + 1
			
		return selected_index
		
if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
