#!/usr/bin/env python3

"""Publish StepCount msg of Adafruit LSM6DS33 6-DoF IMU Breakout for the Promen-Aid Obstacle Avoidance System"""

import board
from adafruit_lsm6ds.lsm6ds33 import LSM6DS33
from adafruit_lsm6ds import Rate, AccelRange
import rospy
from orientation.msg import StepCount

def pedometer():
    i2c = board.I2C()  # uses board.SCL and board.SDA
    sensor = LSM6DS33(i2c)

    # enable accelerometer sensor @ 2G and 26 Hz
    sensor.accelerometer_range = AccelRange.RANGE_2G
    sensor.accelerometer_data_rate = Rate.RATE_26_HZ

    # enable the pedometer
    sensor.pedometer_enable = True

    rospy.loginfo("Initializing pedometer publisher")
    ped_pub = rospy.Publisher('/ped', StepCount, queue_size=5)
    rospy.loginfo("Publishing StepCount at: " + ped_pub.resolved_name)
    rospy.init_node('pedometer_node')

    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        msg = StepCount()
        msg.total = sensor.pedometer_steps
        ped_pub.publish(msg)
        rospy.loginfo(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        pedometer()
    except rospy.ROSInterruptException:
        pass
