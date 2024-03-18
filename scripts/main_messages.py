#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, PointCloud2
import rospy
from threading import Timer

flag_image = 0
flag_lidar = 0
timer_camera = None
timer_lidar = None
timeout_seconds = 5  # Define timeout duration as needed

def reset_flag_camera():
    global flag_image
    flag_image = 0

def reset_flag_lidar():
    global flag_lidar
    flag_lidar = 0

def callback_camera(data):
    global flag_image, timer_camera
    flag_image = 1
    # Cancel the previous timer and start a new one upon receiving a new message
    if timer_camera is not None:
        timer_camera.cancel()
    timer_camera = Timer(timeout_seconds, reset_flag_camera)
    timer_camera.start()

def callback_lidar(data):
    global flag_lidar, timer_lidar
    flag_lidar = 1
    # Cancel the previous timer and start a new one upon receiving a new message
    if timer_lidar is not None:
        timer_lidar.cancel()
    timer_lidar = Timer(timeout_seconds, reset_flag_lidar)
    timer_lidar.start()

def main():
    # Sample Time Defintion
    sample_time = 0.1

    # Frequency of the simulation
    hz = int(1/sample_time)
    loop_rate = rospy.Rate(hz)

    # Message Ros
    rospy.loginfo_once("Checking Messages")

    # Simulation Loop
    while not rospy.is_shutdown():
        tic = rospy.get_time()
        print(flag_image, flag_lidar)

        # Print Time Verification
        loop_rate.sleep()
        rospy.loginfo(str(flag_image))
    return None

if __name__ == '__main__':
    try:
        rospy.init_node('image_republisher', anonymous=True)
        image_sub = rospy.Subscriber('/openmv_cam/image/raw', Image, callback_camera)
        lidar_sub = rospy.Subscriber('/livox/lidar', PointCloud2, callback_lidar)
        main()
    except rospy.ROSInterruptException:
        pass