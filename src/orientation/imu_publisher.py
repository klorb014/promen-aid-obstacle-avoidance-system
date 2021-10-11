#!/usr/bin/env python3

"""Publish IMU msg of Adafruit LSM6DS33 6-DoF IMU Breakout for the Promen-Aid Obstacle Avoidance System"""

import board
from adafruit_lsm6ds.lsm6ds33 import LSM6DS33
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3


def create_vector3(xdata, ydata, zdata):
    v = Vector3()
    v.x = xdata
    v.y = ydata
    v.z = zdata
    return v

def imu():
    i2c = board.I2C()  # uses board.SCL and board.SDA
    sensor = LSM6DS33(i2c)

    rospy.loginfo("Initializing imu publisher")
    imu_pub = rospy.Publisher('/imu', Imu, queue_size=5)
    rospy.loginfo("Publishing Imu at: " + imu_pub.resolved_name)
    rospy.init_node('imu_node')

    rate = rospy.Rate(1) # 50hz
    while not rospy.is_shutdown():
        rospy.loginfo("Acceleration: X:%.2f, Y: %.2f, Z: %.2f m/s^2" % (sensor.acceleration))
        rospy.loginfo("Gyro X:%.2f, Y: %.2f, Z: %.2f radians/s" % (sensor.gyro))
        rospy.loginfo("")

        imu = Imu()
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = 'LSM6DS33 6-DoF IMU'
        imu.orientation_covariance[0] = -1
        imu.angular_velocity = create_vector3(sensor.gyro[0], sensor.gyro[1], sensor.gyro[2])
        imu.linear_acceleration_covariance[0] = -1
        imu.linear_acceleration = create_vector3(sensor.acceleration[0], sensor.acceleration[1], sensor.acceleration[2])
        imu.angular_velocity_covariance[0] = -1
        imu_pub.publish(imu)
        rate.sleep()

if __name__ == '__main__':
    try:
        imu()
    except rospy.ROSInterruptException:
        pass
