#!/usr/bin/env python
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import matplotlib.pyplot as plt
from scipy.io import savemat

data_total_x = []
data_total_y = []

data_total_x_drone = []
data_total_y_drone = []
data_total_z_drone = []

class ImageRepublisher:
    def __init__(self):
        rospy.init_node('image_republisher_simulation', anonymous=True)
        self.image_sub = rospy.Subscriber('/sim/color/image_raw', Image, self.callback)
        self.image_pub = rospy.Publisher('/sim/color/image_raw/filter', Image, queue_size=10)
        self.odometry_subscriber = rospy.Subscriber('/dji_sdk/odometry', Odometry, self.odometry_call_back)
        self.marker_publisher = rospy.Publisher("/marker/array", Marker, queue_size=10)
        self.bridge = CvBridge()
        self.mesh_marker_msg = Marker()
        # Global Variables Odometry
        self.xd = 0.0
        self.yd = 0.0
        self.zd = 0.0
        self.vxd = 0.0
        self.vyd = 0.0
        self.vzd = 0.0

        # Angular velocities
        self.qx = 0.0000
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0
        self.wxd = 0.0
        self.wyd = 0.0
        self.wzd = 0.0


        # Average Values
        self.x_data = []
        self.y_data = []

        self.x_average = 0
        self.y_average = 0
    def odometry_call_back(self, odom_msg):

        self.xd = odom_msg.pose.pose.position.x
        self.yd = odom_msg.pose.pose.position.y
        self.zd = odom_msg.pose.pose.position.z 

        self.vxd = odom_msg.twist.twist.linear.x
        self.vyd = odom_msg.twist.twist.linear.y
        self.vzd = odom_msg.twist.twist.linear.z


        self.qx = odom_msg.pose.pose.orientation.x
        self.qy = odom_msg.pose.pose.orientation.y
        self.qz = odom_msg.pose.pose.orientation.z
        self.qw = odom_msg.pose.pose.orientation.w

        self.wxd = odom_msg.twist.twist.angular.x
        self.wyd = odom_msg.twist.twist.angular.y
        self.wzd = odom_msg.twist.twist.angular.z

        return None

    def callback(self, data):
        try:
            # Convert the ROS image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            # Apply a colormap to the image
            # OpenCV provides several colormaps such as COLORMAP_JET, COLORMAP_HOT, COLORMAP_INFERNO, etc.

        except Exception as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return
        # Check for temperature
        flag, filter_image = self.filter_red_color(image= cv_image)

        # Display the original and processed images
        cv2.imshow('Original Image', cv_image)
        cv2.imshow('Red Object in the Image Frame', filter_image)
        cv2.waitKey(1)  # Wait for a key press for 1 millisecond
        if flag:
            print("Red Color Detected")
            self.x_average = sum(self.x_data)/len(self.x_data)
            self.y_average = sum(self.y_data)/len(self.y_data)
            data_total_x.append(self.x_average)
            data_total_y.append(self.y_average)
            self.set_marker()
        else:
            print("No Red Color Detected")


        try:
            # Republish the same image - conversion back to ROS image message
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(filter_image, "bgr8"))
        except Exception as e:
            rospy.logerr("Publish Error: {0}".format(e))

    def filter_red_color(self, image):
        # Convert the image from BGR to HSV color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        image_copy = image.copy()

        # Define the lower and upper bounds of the red color in HSV at the lower end
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])

        # Define the lower and upper bounds of the red color in HSV at the upper end
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # Create masks for the specified red color ranges
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

        # Combine the masks
        mask = cv2.bitwise_or(mask1, mask2)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.x_data = []
        self.y_data = []

        self.x_average = 0
        self.y_average = 0
        # Draw rectangles around each contour
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)  # Get the bounding rectangle for each contour

            # Calculate the midle point
            x_midle = x + w/2
            y_midle = y + h/2

            # Store the average
            self.x_data.append(self.xd)
            self.y_data.append(self.yd)

            center = (int(x_midle), int(y_midle))  # Calculate the center of the rectangle
            cv2.circle(image_copy, center, 1, (255, 0, 0), 2)  # Green circle with thickness of 2
            cv2.rectangle(image_copy, (x, y), (x+w, y+h), (0, 255, 0), 2)  # Draw the rectangle on the original image

        # Check if there is any red detected in the image
        if len(contours) > 0:
            # Return True and the image with rectangles if red is detected
            return True, image_copy
        else:
            # Return False and original image if no red is detected
            return False, image_copy

    def set_marker(self):
        self.mesh_marker_msg.header.stamp = rospy.Time.now()
        self.mesh_marker_msg.header.frame_id = "world"  # Set the reference frame for the marker
        self.mesh_marker_msg.type = Marker.POINTS  # Define marker type as points
        self.mesh_marker_msg.action = Marker.ADD  # Specify the action as add/create
        self.mesh_marker_msg.scale.x = 0.05  # Size of the points
        self.mesh_marker_msg.scale.y = 0.05  # Size of the points
        self.mesh_marker_msg.color.a = 1.0  # Set the opacity of the marker
        self.mesh_marker_msg.color.r = 1.0  # Set the color of the marker to red
        aux_point = Point(self.x_average, self.y_average, 0.0)  # Create a point with the given coordinates
        self.mesh_marker_msg.points.append(aux_point)  # Add the point to the marker's points list

        self.marker_publisher.publish(self.mesh_marker_msg)
        return None
if __name__ == '__main__':
    try:
        ImageRepublisher()
        rospy.spin()
    except(rospy.ROSInterruptException, KeyboardInterrupt):
        cv2.destroyAllWindows()  # Make sure to destroy the OpenCV windows on shutdown
        print("Error System")
        pass
    else:
        Data_matrix_panel = np.array([data_total_x, data_total_y])
        # Save information of the entire system
        mdic_h = {"Data_matrix": Data_matrix_panel, "label": "points"}
        savemat("Data_"+ str(1) + ".mat", mdic_h)
        print("Complete Execution")
        pass