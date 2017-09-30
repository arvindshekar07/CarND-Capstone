#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3
LOOKAHEAD_WPS = 20

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.car_position = None

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        sub9 = rospy.Subscriber('/car_position', Int32, self.car_position_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.spin()
		

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def car_position_cb(self, msg):
        self.car_position = msg.data
		
    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
		
        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()
 
        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, pose, waypoints):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
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


    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get transform between pose of camera and world frame
        trans = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link",
                  "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link",
                  "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        #TODO Use tranform and rotation to calculate 2D position of light in image

        x = 0
        y = 0

        return (x, y)

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #x, y = self.project_to_image_plane(light.pose.pose.position)

        #TODO use light location to zoom in on traffic light in image

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
        #car_position = None
        light_index = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        
        if(self.pose is not None and self.lights is not None and self.waypoints is not None):
            light_index = self.closest_traffic_light(self.pose.pose, self.lights)

        #TODO find the closest visible traffic light (if one exists)
        if light_index is not None:
            light = self.lights[light_index]
            stop_line_position = stop_line_positions[light_index]
            light_wp = self.closest_stop_line_position(stop_line_position)
            #car_position = self.get_closest_waypoint(self.pose.pose, self.waypoints)			
            #car_position = self.car_position

            #print(str(light_wp) + " " + str(self.car_position))						
   
        if light and self.car_position is not None and light_wp >= self.car_position and light_wp <=self.car_position+LOOKAHEAD_WPS:
            state = self.get_light_state(light)
            #print(state)
            #state = light.state
            return light_wp, state
        return -1, TrafficLight.UNKNOWN

    def closest_stop_line_position(self, stop_line_position):
        dist = float('inf')
        index = 0
        selected_index = None
        dl = lambda a, b: math.sqrt((a.x-b[0])**2 + (a.y-b[1])**2)
        for waypoint in self.waypoints:
            temp_dist = dl(waypoint.pose.pose.position, stop_line_position)
            if temp_dist < dist:
               dist = temp_dist
               selected_index = index
            index = index + 1

         #   if self.waypoints[selected_index].pose.pose.position.x < stop_line_position[0]:
         #      selected_index = selected_index + 1
  
        return selected_index		

    def closest_traffic_light(self, pose, lights):
        dist = float('inf')
        index = 0
        selected_index = None
        #dl = lambda a, b: a.x-b.x
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2)
        for light in lights:
            temp_dist = dl(light.pose.pose.position, pose.position)
            if temp_dist < dist:
               dist = temp_dist
               selected_index = index
            index = index + 1   
        return selected_index	
		
		
		
if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
