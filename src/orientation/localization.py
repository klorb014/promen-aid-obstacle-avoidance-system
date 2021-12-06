#!/usr/bin/env python
import rospy
from math import atan2, degrees, sqrt, cos, pi
from sensor_msgs.msg import MagneticField, Imu, NavSatFix
from orientation.msg import StepCount
import argparse
import time
import turtle


class Localizer:
    def __init__(self,display=False):
        rospy.init_node('localizer',anonymous=True)
        rospy.Subscriber('/gps', NavSatFix, callback=self.gps_callback, queue_size=1)
        rospy.Subscriber('/ped', StepCount, callback=self.step_callback, queue_size=1)
        rospy.Subscriber('/mag', MagneticField, callback=self.compass_callback, queue_size=1)

        self.gps_stamp = None
        self.shift = 0
        self.steps_prev = None
        self.displacement_x = 0
        self.displacement_y = 0
        self.displacement = False
        self.window_size = 20
        self.heading_MA = []
        self.gps_timeout = 5

    def vector_2_degrees(self, x, y):
        angle = degrees(atan2(y, x))
        if angle < 0:
            angle += 360
        return angle

    def get_displacement(self):
        return self.displacement_x, self.displacement_y

    def unit_vector(self,x,y):
        return x/sqrt(x**2 + y**2), y/sqrt(x**2 + y**2)

    def steps_axes(self, magnet_x, magnet_y, steps):
        x_unit, y_unit = self.unit_vector(magnet_x, magnet_y)
        return round(steps*x_unit,2), round(steps*y_unit,2)
        return x_unit, y_unit

    def smooth_mag(self):
        smooth_mag_x = 0
        smooth_mag_y = 0
        for raw_data in self.heading_MA:
            smooth_mag_x +=  raw_data.z
            smooth_mag_y +=  raw_data.x
        return smooth_mag_x/self.window_size, smooth_mag_y/self.window_size

    def gps_callback(self, gps_data):
        self.gps_coord = (gps_data.latitude,gps_data.longitude)
        self.gps_stamp = gps_data.header.stamp.secs

    def lost_fix(self):
        #if no new gps data is received in last 5 seconds assume loss of fix
        now = round(time.time())
        if self.gps_stamp is not None:
            if abs(now - self.gps_stamp) > self.gps_timeout:
                return True
                print("LOST FIX")
            else:
                return False
        else:
            print("WAITING FOR INITIAL GPS FIX")
            return False

    def correct_gps(self):

        #Use approximation that 111,111m -> 1 degree
        m_per_degree = 111111
        lat_radians = self.gps_coord[0] * (pi/180)
        lat_correction = self.displacement_y / m_per_degree
        long_correction = self.displacement_x / (m_per_degree*cos(lat_radians))
        return self.gps_coord[0] + lat_correction, self.gps_coord[1] + long_correction

    def reset(self):
        self.shift = 0
        self.steps_prev = None
        self.displacement_x = 0
        self.displacement_y = 0
        self.displacement = False

    def compass_callback(self, data):
        mag = data.magnetic_field
        self.heading_MA.append(mag)
        # MOVING AVERAGE WINDOW
        if len(self.heading_MA) > self.window_size:
            self.heading_MA.pop(0)
        self.mag_x, self.mag_y = self.smooth_mag();
        self.heading = round(self.vector_2_degrees(self.mag_x, self.mag_y))

    def step_callback(self, data):
        if self.lost_fix():
            steps = data.total

            if self.steps_prev is None:
                self.shift = steps
                self.steps_prev = 0

            steps -= self.shift

            if steps != self.steps_prev and steps >= 0:
                self.displacement = True
                steps_diff = steps-self.steps_prev
                self.steps_prev = steps
                x_steps, y_steps = self.steps_axes(self.mag_x, self.mag_y,steps_diff)
                self.displacement_x += y_steps
                self.displacement_y += x_steps
                corrected_gps = self.correct_gps()
                msg = "==============================\nSteps: {0}\nDisplacement:\nHeading: {1} degrees, X: {2} steps, Y: {3} steps\nGPS Coordinate: {4}\nCorrected GPS Coordinate: {5}".format(steps, self.heading, self.displacement_x, self.displacement_y,self.gps_coord,corrected_gps)
                rospy.loginfo(msg)
            else:
                self.displacement = False
                corrected_gps = self.correct_gps()
                msg = "==============================\nSteps: {0}\nDisplacement:\nHeading: {1} degrees, X: {2} steps, Y: {3} steps\nGPS Coordinate: {4}\nCorrected GPS Coordinate: {5}".format(steps, self.heading, self.displacement_x, self.displacement_y,self.gps_coord,corrected_gps)
                rospy.loginfo(msg)
        else:
            self.reset()

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("--display", help="display XY displacement feed", action="store_true")
    args = parser.parse_args()
    if args.display:
        try:
            screen = turtle.getscreen()
            turtle.title("Promenaid: Pedestrian Dead Reckoning")
            t = turtle.Turtle()

            localizer = Localizer(display=True)
            rate = rospy.Rate(5) # 6hz
            while not rospy.is_shutdown():
                if localizer.displacement:
                    x, y = localizer.get_displacement()
                    t.goto(10*x,10*y)
                rate.sleep()
        except rospy.ROSInterruptException:
            turtle.exitonclick()
    else:
        localizer = Localizer()
        rospy.spin()
