#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
import numpy as np
import csv

#Program to handle fall detection and user triggered emergency alerts

def total_acceleration(linear_accel):
    accel_x = linear_accel.x
    accel_y = linear_accel.y
    accel_z = linear_accel.z
    accel = np.array([accel_x, accel_y, accel_z])
    return round(np.sqrt(np.sum(np.square(accel))),2)

def data_logger(imu_data):
    linear_acceleration = imu_data.linear_acceleration
    total_a = total_acceleration(linear_acceleration)
    rospy.loginfo(total_a)

    with open('acceleration_logger.csv', 'a', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([str(total_a)])



if __name__ == '__main__':
    try:
        rospy.init_node('emergency',anonymous=True)
        rospy.Subscriber("/imu", Imu, callback=data_logger, queue_size=1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass