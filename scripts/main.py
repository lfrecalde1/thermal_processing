#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import matplotlib.pyplot as plt

class ImageRepublisher:
    def __init__(self):
        rospy.init_node('image_republisher', anonymous=True)
        self.image_sub = rospy.Subscriber('/openmv_cam/image/raw', Image, self.callback)
        self.image_pub = rospy.Publisher('/openmv_cam/image/filter', Image, queue_size=10)
        self.image_pub_temp = rospy.Publisher('/openmv_cam/image/temp', Image, queue_size=10)
        self.bridge = CvBridge()
        self.max_temp_in_celsius = 150                             # Change to set the maximun value
        self.min_temp_in_celsius = 0.0                             # Change to set the minimun value
        self. threshold_celsius = 40                               # Threshold  temperature 
        self.threshold_byte = ((self.threshold_celsius - self.min_temp_in_celsius)*255)/(self.max_temp_in_celsius - self.min_temp_in_celsius)
        print(self.threshold_byte)

    def callback(self, data):
        try:
            # Convert the ROS image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "mono8")
            normalized_image = cv2.normalize(cv_image, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8U)

            # Apply a colormap to the image
            # OpenCV provides several colormaps such as COLORMAP_JET, COLORMAP_HOT, COLORMAP_INFERNO, etc.
            colored_image = cv2.applyColorMap(normalized_image, cv2.COLORMAP_JET)

        except Exception as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return
        # Check for temperature
        contoured_image = self.find_temperatures(cv_image= cv_image)

        # Display the original and processed images
        cv2.imshow('Original Image', cv_image)
        cv2.imshow('White Objects Detected', contoured_image)
        cv2.waitKey(1)  # Wait for a key press for 1 millisecond

        try:
            # Republish the same image - conversion back to ROS image message
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(colored_image, "bgr8"))
            self.image_pub_temp.publish(self.bridge.cv2_to_imgmsg(contoured_image, "bgr8"))
        except Exception as e:
            rospy.logerr("Publish Error: {0}".format(e))

    def map_g_to_temp(self, g):
        aux = ((g * (self.max_temp_in_celsius - self.min_temp_in_celsius)) / 255.0) + self.min_temp_in_celsius
        
        return aux*0.8

    def find_temperatures(self, cv_image):
        # Set the threshold value to segment white objects; values might need adjustment
        threshold_value = self.threshold_byte # Values above this will be considered white

        # Threshold the image to isolate white objects
        _, thresholded_image = cv2.threshold(cv_image, threshold_value, 255, cv2.THRESH_BINARY)

        # Find contours of the white objects
        contours, _ = cv2.findContours(thresholded_image, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
             # Find the largest contour by area
            largest_contour = max(contours, key=cv2.contourArea)

            # Create a mask for the largest contour
            mask = np.zeros_like(cv_image)  # Create a mask that is the same size as the image
            cv2.drawContours(mask, [largest_contour], -1, 255, thickness=cv2.FILLED)  # Fill the contour in the mask

            # Calculate the average pixel value within the contour
            mean_val = cv2.mean(cv_image, mask=mask)[0]
            max_val = np.max(cv_image[mask == 255])
            celcius = self.map_g_to_temp(max_val)

             # Find the bounding rectangle of the largest contour
            x, y, w, h = cv2.boundingRect(largest_contour)
    
            # Calculate the position for the text (just above the contour)
            text_position = (x, y + 10)  # Adjust the y-coordinate to place the text above the contour

            # Draw the largest contour and the mean value on the original image (converted to color)
            contoured_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)  # Convert to BGR to draw colored contours and text
            cv2.drawContours(contoured_image, [largest_contour], -1, (255, 0, 0), 2)
            cv2.putText(contoured_image, f"Temp: {celcius:.2f}", text_position, cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)
        else:
            # If no contours were found, just convert the original image
            contoured_image = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)

        return contoured_image

if __name__ == '__main__':
    try:
        ImageRepublisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        cv2.destroyAllWindows()  # Make sure to destroy the OpenCV windows on shutdown
        pass