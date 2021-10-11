#!/usr/bin/env python3

"""Publish NavSatFix msg of Adafruit Mini GPS PA1010D Breakout for the Promen-Aid Obstacle Avoidance System"""


import board
import adafruit_gps
import rospy
from sensor_msgs.msg import NavSatFix

def gps():

    # If using I2C, we'll create an I2C interface to talk to using default pins
    i2c = board.I2C()

    # Create a GPS module instance.
    gps = adafruit_gps.GPS_GtopI2C(i2c, debug=False)  # Use I2C interface

    # Turn on the basic GGA and RMC info (what you typically want)
    gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")

    # Set update rate to once a second (1hz)
    gps.send_command(b"PMTK220,1000")

    rospy.loginfo("Initializing gps publisher")
    gps_pub = rospy.Publisher('/gps', NavSatFix, queue_size=5)
    rospy.loginfo("Publishing NavSatFix at: " + gps_pub.resolved_name)
    rospy.init_node('gps_node')


    rate = rospy.Rate(1) # 50hz
    while not rospy.is_shutdown():
        gps.update()
          
        if gps.has_fix:
            
            nav = NavSatFix()
            nav.header.stamp = rospy.Time.now()
            nav.header.frame_id = 'Adafruit Mini GPS PA1010D'
            nav.latitude = gps.latitude
            nav.longitude = gps.longitude
            nav.altitude = gps.altitude_m

            rospy.loginfo("=" * 40)
            rospy.loginfo("Latitude: {0:.6f} degrees".format(gps.latitude))
            rospy.loginfo("Longitude: {0:.6f} degrees".format(gps.longitude))
            rospy.loginfo("Fix quality: {}".format(gps.fix_quality))
            # Some attributes beyond latitude, longitude and timestamp are optional
            # and might not be present.  Check if they're None before trying to use!
            if gps.satellites is not None:
                rospy.loginfo("# satellites: {}".format(gps.satellites))
            if gps.altitude_m is not None:
                rospy.loginfo("Altitude: {} meters".format(gps.altitude_m))
            if gps.speed_knots is not None:
                rospy.loginfo("Speed: {} knots".format(gps.speed_knots))
            if gps.track_angle_deg is not None:
                rospy.loginfo("Track angle: {} degrees".format(gps.track_angle_deg))
            if gps.horizontal_dilution is not None:
                rospy.loginfo("Horizontal dilution: {}".format(gps.horizontal_dilution))
            if gps.height_geoid is not None:
                rospy.loginfo("Height geoid: {} meters".format(gps.height_geoid))

            gps_pub.publish(nav)
        else:
            # Try again if we don't have a fix yet.
            rospy.loginfo("Waiting for fix...")
        rate.sleep()

if __name__ == '__main__':
    try:
        gps()
    except rospy.ROSInterruptException:
        pass