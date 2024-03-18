#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import Odometry
import rospy
from threading import Timer

flag_image = 0
flag_lidar = 0
flag_odometry = 0
flag_image_rgbd = 0

timer_camera = None
timer_lidar = None
timer_odometry = None
timer_camera_rgbd = None
timeout_seconds = 5  # Define timeout duration as needed

def reset_flag_camera():
    global flag_image
    flag_image = 0

def reset_flag_camera_rgbd():
    global flag_image_rgbd
    flag_image_rgbd = 0

def reset_flag_lidar():
    global flag_lidar
    flag_lidar = 0

def reset_flag_odometry():
    global flag_odometry
    flag_odometry = 0

def callback_camera(data):
    global flag_image, timer_camera
    flag_image = 1
    # Cancel the previous timer and start a new one upon receiving a new message
    if timer_camera is not None:
        timer_camera.cancel()
    timer_camera = Timer(timeout_seconds, reset_flag_camera)
    timer_camera.start()

def callback_camera_rgbd(data):
    global flag_image_rgbd, timer_camera_rgbd
    flag_image_rgbd = 1
    # Cancel the previous timer and start a new one upon receiving a new message
    if timer_camera_rgbd is not None:
        timer_camera_rgbd.cancel()
    timer_camera_rgbd = Timer(timeout_seconds, reset_flag_camera_rgbd)
    timer_camera_rgbd.start()

def callback_lidar(data):
    global flag_lidar, timer_lidar
    flag_lidar = 1
    # Cancel the previous timer and start a new one upon receiving a new message
    if timer_lidar is not None:
        timer_lidar.cancel()
    timer_lidar = Timer(timeout_seconds, reset_flag_lidar)
    timer_lidar.start()

def callback_odometry(data):
    global flag_odometry, timer_odometry
    flag_odometry = 1
    # Cancel the previous timer and start a new one upon receiving a new message
    if timer_odometry is not None:
        timer_odometry.cancel()
    timer_odometry = Timer(timeout_seconds, reset_flag_odometry)
    timer_odometry.start()

def main():
    # Sample Time Defintion
    sample_time = 0.05

    # Frequency of the simulation
    hz = int(1/sample_time)
    loop_rate = rospy.Rate(hz)

    # Message Ros
    rospy.loginfo_once("Checking Messages")

    # Simulation Loop
    while not rospy.is_shutdown():
        tic = rospy.get_time()
        #print(flag_image, flag_lidar)

        # Print Time Verification
        loop_rate.sleep()
        rospy.loginfo("Thermal:"+str(flag_image) + " " + "Lidar:" + str(flag_lidar) + " " + "Odom:"+str(flag_odometry) + " " + "RGBD:"+str(flag_image_rgbd))
    return None

if __name__ == '__main__':
    try:
        rospy.init_node('image_republisher', anonymous=True)
        image_sub = rospy.Subscriber('/openmv_cam/image/raw', Image, callback_camera)
        image_rgbd_sub = rospy.Subscriber('/camera/color/image_raw', Image, callback_camera_rgbd)
        lidar_sub = rospy.Subscriber('/livox/lidar', PointCloud2, callback_lidar)
        odom_sub = rospy.Subscriber('/dji_sdk/odometry', Odometry, callback_odometry)
        main()
    except rospy.ROSInterruptException:
        pass