from styx_msgs.msg import TrafficLight
import cv2
import numpy as np
import rospy

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """

        def convert_color_space(image):
            return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        def apply_color_mask(HSV_image, low, high):
            mask =  cv2.inRange(HSV_image, low, high)
            return mask, cv2.bitwise_and(image, image, mask=mask)

        def get_area(mask):
            return cv2.countNonZero(mask)

        environment = rospy.get_param('~environment', "sim")
		
        # simulator
        if environment == "sim":
           red_low = np.array([150, 150, 40])
           red_high = np.array([180, 255, 255])

           yellow_low = np.array([20, 150, 100])
           yellow_high = np.array([40, 255, 255])

           green_low = np.array([50, 100, 100])
           green_high = np.array([70, 255, 255])
        else :
        # environment
   
           red_low = np.array([18, 120, 40])
           red_high = np.array([22, 255, 255])

           yellow_low = np.array([28, 120, 100])
           yellow_high = np.array([35, 255, 255])

           green_low = np.array([70, 95, 100])
           green_high = np.array([90, 255, 255])
   

        # Convert RBG to HSV
        HSV_image = convert_color_space(image)

        # Threshold the HSV image to get only red, yellow and green colors
        # Bitwise-AND mask
        red_mask, red_image = apply_color_mask(HSV_image, red_low, red_high)
        red_area = get_area(red_mask)

        yellow_mask, yellow_image = apply_color_mask(HSV_image, yellow_low, yellow_high)
        yellow_area = get_area(yellow_mask)

        green_mask, green_image = apply_color_mask(HSV_image, green_low, green_high)
        green_area = get_area(green_mask)

        #print(red_area, yellow_area, green_area)

        if red_area > 180:
            print ('Approaching red light')
            return TrafficLight.RED

        if yellow_area > 180:
            #print ('yellow')
            return TrafficLight.YELLOW

        if green_area > 180:
            #print ('green')
            return TrafficLight.GREEN

        return TrafficLight.UNKNOWN
