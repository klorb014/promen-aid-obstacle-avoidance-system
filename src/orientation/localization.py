#!/usr/bin/env python
import rospy
from math import atan2, degrees, sqrt
from sensor_msgs.msg import MagneticField, Imu, NavSatFix
from orientation.msg import StepCount


class Localizer:
    def __init__(self):
        rospy.init_node('localizer',anonymous=True)
        rospy.Subscriber('/gps', NavSatFix, callback=self.gps_callback, queue_size=1)
        rospy.Subscriber('/pedometer', StepCount, callback=self.step_callback, queue_size=1)
        rospy.Subscriber('/mag', MagneticField, callback=self.compass_callback, queue_size=1)


    def vector_2_degrees(self, x, y):
        angle = degrees(atan2(y, x))
        print(angle)
        if angle < 0:
            angle += 360
        return angle


    def unit_vector(self,x,y):
        return x/sqrt(x**2 + y**2), y/sqrt(x**2 + y**2)

    def steps_axes(self, magnet_x, magnet_y, steps):
        x_unit, y_unit = self.unit_vector(magnet_x, magnet_y)
        return round(steps*x_unit,2), round(steps*y_unit,2)
    
    def displacement(self):
        pass

    def compass_callback(self, data):
        self.mag = data.magnetic_field
        self.heading = self.vector_2_degrees(self.mag.x, self.mag.y)
        