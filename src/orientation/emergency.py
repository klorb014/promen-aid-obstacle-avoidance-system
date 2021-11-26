#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, NavSatFix
import numpy as np
import csv
import RPi.GPIO as GPIO
import time
import datetime
from alert import SMSAlert

#Program to handle fall detection and user triggered emergency alerts

class EmergencyController:
    def __init__(self,distress_button_pin, fall_dectection_threshold=16):
        self.button_pin = distress_button_pin
        self.fall_dectection_threshold = fall_dectection_threshold
        self.alert = SMSAlert()
        self.location = None

        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(distress_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        GPIO.add_event_detect(distress_button_pin,GPIO.RISING,callback=self.button_callback)

        rospy.init_node('emergency',anonymous=True)
        rospy.Subscriber("/imu", Imu, callback=self.fall_detector, queue_size=1)
        rospy.Subscriber("/gps", NavSatFix, callback=self.set_location, queue_size=1)


    def total_acceleration(self, linear_accel):
        accel_x = linear_accel.x
        accel_y = linear_accel.y
        accel_z = linear_accel.z
        accel = np.array([accel_x, accel_y, accel_z])
        return round(np.sqrt(np.sum(np.square(accel))),2)

    def data_logger(self, imu_data):
        linear_acceleration = imu_data.linear_acceleration
        total_a = self.total_acceleration(linear_acceleration)
        rospy.loginfo(total_a)

        with open('acceleration_logger.csv', 'a', encoding='UTF8', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([str(total_a)])

    def fall_detector(self, imu_data):
        linear_acceleration = imu_data.linear_acceleration
        total_a = self.total_acceleration(linear_acceleration)
        if total_a > self.fall_dectection_threshold:
            timestamp = str(datetime.datetime.now()).split('.')[0]
            message = "FALL DETECTED AT: {0}".format(timestamp)
            if self.location is not None:
                self.alert.send_alert(message, location=self.location)
            else:
                message = "FALL DETECTED AT: {0} \n LOCATION DATA COULD NOT BE ACCUIRED".format(timestamp)
                self.alert.send_alert(message)

    def set_location(self, location):
        latitude = location.latitude
        longitude = location.longitude
        self.location = (latitude,longitude)

    def button_callback(self, channel):
        start_time = time.time()
        while GPIO.input(16) == 1: # Wait for the button up
            pass
        buttonTime = time.time() - start_time
        if 5 <= buttonTime < 10:
            timestamp = str(datetime.datetime.now()).split('.')[0]
            message = "ASSISTANCE REQUIRED AT: {0}".format(timestamp)
            if self.location is not None:
                self.alert.send_alert(message, location=self.location)
            else:
                message = "ASSISTANCE REQUIRED AT: {0} \n LOCATION DATA COULD NOT BE ACCUIRED".format(timestamp)
                self.alert.send_alert(message)


if __name__ == '__main__':
    distress_button_pin, fall_dectection_threshold = 16, 16
    e = EmergencyController(distress_button_pin, fall_dectection_threshold)

    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()
