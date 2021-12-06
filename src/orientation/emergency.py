#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu, NavSatFix
import numpy as np
import csv
import time
import datetime
import argparse
import numpy as np
from matplotlib.lines import Line2D
import matplotlib.pyplot as plt
import matplotlib.animation as animation

#if running on PI import GPIO
try:
    from alert import SMSAlert
    import RPi.GPIO as GPIO
except ImportError:
    pass


#Program to handle fall detection and user triggered emergency alerts

class EmergencyController:
    def __init__(self,distress_button_pin, fall_dectection_threshold=16, display=None):
        self.button_pin = distress_button_pin
        self.fall_dectection_threshold = fall_dectection_threshold

        self.location = None
        self.a_total = None

        rospy.init_node('emergency',anonymous=True)
        rospy.Subscriber("/imu", Imu, callback=self.fall_detector, queue_size=1)
        rospy.Subscriber("/gps", NavSatFix, callback=self.set_location, queue_size=1)

        if display is not None:
            self.display = True
            self.ax = ax
            self.dt = 0.02
            self.maxt = 20
            self.tdata = [0]
            self.ydata = [0]
            self.line = Line2D(self.tdata, self.ydata)
            self.ax.add_line(self.line)
            self.ax.set_ylim(5, 25)
            self.ax.set_xlim(0, self.maxt)
            self.ax.axhline(self.fall_dectection_threshold, linestyle='--', color='red')
            self.ax.set_xlabel("Time")
            self.ax.set_ylabel("Total Acceleration")
        else:
            self.display = False
            self.alert = SMSAlert()
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BOARD)
            GPIO.setup(distress_button_pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
            GPIO.add_event_detect(distress_button_pin,GPIO.RISING,callback=self.button_callback)

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
        self.a_total = self.total_acceleration(linear_acceleration)
        if self.a_total > self.fall_dectection_threshold:
            timestamp = str(datetime.datetime.now()).split('.')[0]
            message = "FALL DETECTED AT: {0}".format(timestamp)
            if not self.display:
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

    def get_acceleration(self):
        if self.a_total is not None:
            yield self.a_total
        else:
            yield 10 # ~gravity

    def init():
        ax.set_xlim(0, 2*np.pi)
        ax.set_ylim(-1, 1)
        return ln,

    def update(self, y):
        lastt = self.tdata[-1]
        if lastt > self.tdata[0] + self.maxt:  # reset the arrays
            self.tdata = [self.tdata[-1]]
            self.ydata = [self.ydata[-1]]
            self.ax.set_xlim(self.tdata[0], self.tdata[0] + self.maxt)
            self.ax.figure.canvas.draw()

        t = self.tdata[-1] + self.dt
        self.tdata.append(t)
        self.ydata.append(y)
        self.line.set_data(self.tdata, self.ydata)
        return self.line,


if __name__ == '__main__':
    distress_button_pin, fall_dectection_threshold = 16, 20

    parser = argparse.ArgumentParser()
    parser.add_argument("--display", help="display video feed", action="store_true")
    args = parser.parse_args()
    if args.display:
        fig, ax = plt.subplots()
        e = EmergencyController(distress_button_pin, fall_dectection_threshold, display=ax)

        # pass a generator in "emitter" to produce data for the update func
        ani = animation.FuncAnimation(fig, e.update, frames=e.get_acceleration, interval=50,blit=True)
        plt.show()
    else:
        e = EmergencyController(distress_button_pin, fall_dectection_threshold)
        rospy.spin()



    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()
