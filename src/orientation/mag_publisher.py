#!/usr/bin/env python3

"""Publish MagneticFeild msg of Adafruit LIS3MDL Triple-axis Magnetometer Breakout for the Promen-Aid Obstacle Avoidance System"""

import board
import adafruit_lis3mdl
import rospy
from sensor_msgs.msg import MagneticField
from geometry_msgs.msg import Vector3

def calibrate(x,y,z):
    hard_iron_offset = (-25.13, -16.98, -12.37)
    x, y, z = x - hard_iron_offset[0], y - hard_iron_offset[1], z - hard_iron_offset[2]
    return x, y

def create_vector3(xdata, ydata, zdata):
    v = Vector3()
    v.x = xdata
    v.y = ydata
    v.z = zdata
    return v

def magnetometer():
    i2c = board.I2C()  # uses board.SCL and board.SDA
    sensor = adafruit_lis3mdl.LIS3MDL(i2c)

    rospy.loginfo("Initializing mag publisher")
    mag_pub = rospy.Publisher('/mag', MagneticField, queue_size=5)
    rospy.loginfo("Publishing MagneticField at: " + mag_pub.resolved_name)
    rospy.init_node('mag_node')

    rate = rospy.Rate(10) # 50hz
    while not rospy.is_shutdown():
        mag_x, mag_y, mag_z = sensor.magnetic
        mag_x, mag_y, mag_z = calibrate(mag_x, mag_y, mag_z)
        rospy.loginfo('X:{0:10.2f}, Y:{1:10.2f}, Z:{2:10.2f} uT'.format(mag_x, mag_y, mag_z))
        rospy.loginfo("")

        mag = MagneticField()
        mag.header.stamp = rospy.Time.now()
        mag.header.frame_id = 'LIS3MDL Triple-axis Magnetometer'
        mag.magnetic_field_covariance = 0
        mag.magnetic_field = create_vector3(mag_x, mag_y, mag_z)
        mag_pub.publish(mag)
        rate.sleep()

if __name__ == '__main__':
    try:
        magnetometer()
    except rospy.ROSInterruptException:
        pass
