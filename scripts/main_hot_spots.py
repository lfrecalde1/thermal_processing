#!/usr/bin/env python
import cv2
import rosbag
from cv_bridge import CvBridge
import numpy as np
from nav_msgs.msg import Odometry
from scipy.io import savemat

# Path to your ROS bag file
bag_path = '/home/fer/Rosbag_thermal/2024-02-29-01-26-35.bag'

# The topic where the image data is published
image_topic = '/sim/color/image_raw'
odometry_topic = '/dji_sdk/odometry'

bridge = CvBridge()
odom_msg = Odometry()
time = []

odom_msgs = []
odom_timestamps = []
odom_msg_hot_spots_x = []
odom_msg_hot_spots_y = []
odom_msg_hot_spots_z = []

# First, read all odometry messages and their timestamps
with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[odometry_topic]):
        odom_msgs.append(msg)
        odom_timestamps.append(msg.header.stamp.to_sec())  # Convert to seconds

# Function to find the closest odometry message by timestamp
def find_closest_odom(timestamp_sec, odom_timestamps, odom_msgs):
    index = np.argmin(np.abs(np.array(odom_timestamps) - timestamp_sec))
    return odom_msgs[index]


def filter_red_color(image):
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

    # Draw rectangles around each contour
    for cnt in contours:
        x, y, w, h = cv2.boundingRect(cnt)  # Get the bounding rectangle for each contour

        # Calculate the midle point
        x_midle = x + w/2
        y_midle = y + h/2

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

with rosbag.Bag(bag_path, 'r') as bag:
    for topic, msg, t in bag.read_messages(topics=[image_topic]):
        # Convert the ROS image message to an OpenCV image
        try:
            cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            flag, filter_image = filter_red_color(cv_image)

            if flag:
                image_timestamp_sec = msg.header.stamp.to_sec()
                closest_odom_msg = find_closest_odom(image_timestamp_sec, odom_timestamps, odom_msgs)
                odom_msg_hot_spots_x.append(closest_odom_msg.pose.pose.position.x)
                odom_msg_hot_spots_y.append(closest_odom_msg.pose.pose.position.y)
                odom_msg_hot_spots_z.append(0)
            else:
                None
        except Exception as e:
            print(e)
            continue
    # Save Information in a matrix
    Data_matrix_panel = np.array([odom_msg_hot_spots_x, odom_msg_hot_spots_y, odom_msg_hot_spots_z])
    mdic_h = {"Data_matrix": Data_matrix_panel, "label": "points"}
    savemat("Data_"+ str(1) + ".mat", mdic_h)
    print("Complete Execution")