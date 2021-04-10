#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
from tf.transformations import euler_from_quaternion

import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt
import math
import tf2_ros
import geometry_msgs.msg
from math import sin, cos



def find_nearest_obstacle(img, target):
    nonzero = cv.findNonZero(img)
    distances = np.sqrt((nonzero[:,:,0] - target[0]) ** 2 + (nonzero[:,:,1] - target[1]) ** 2)
    nearest_index = np.argmin(distances)
    return nonzero[nearest_index]

def map_callback(data):

    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.info)
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", (data.data))
    
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            trans = tfBuffer.lookup_transform('odom', 'camera_link', rospy.Time()).transform
            x = trans.translation.x
            y = trans.translation.y
            quaternion = (trans.rotation.x,trans.rotation.y,trans.rotation.z,trans.rotation.w)
            rospy.loginfo(rospy.get_caller_id() + "I heard %s",tfBuffer.lookup_transform('odom', 'camera_link', rospy.Time()))
            break
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            continue

    euler = euler_from_quaternion(quaternion)

   
    yaw = euler[2]
    new_y = sin(yaw)
    new_x = cos(yaw)
   

    w = data.info.width
    h = data.info.height

    origin_x =  data.info.origin.position.x
    origin_y =  data.info.origin.position.y

    grid_x = (x - origin_x)
    grid_y = (y - origin_y)

    img = np.absolute(np.array(list(data.data)))
    img = img.astype('uint8')
    img = np.reshape(img, (h,w))
    
 
    resolution = data.info.resolution

    pos_y = int(round(grid_y/resolution))
    pos_x = int(round(grid_x/resolution))

    ret,thresh = cv.threshold(img,90,255,cv.THRESH_BINARY)

    nearest_obstacle = find_nearest_obstacle(thresh, (pos_x, pos_y))[0]    

    
    x = [pos_x, nearest_obstacle[0]]
    y = [pos_y, nearest_obstacle[1]]
   

    dist = math.sqrt((x[0]-x[1])**2 + (y[0]-y[1])**2)*resolution
    
    thresh[pos_y][pos_x] = 255 

    obs_vector = [x[1]-x[0], y[1]-y[0]]/np.linalg.norm([x[1]-x[0], y[1]-y[0]])
    dir_vector = [new_x, new_y]
    dot_product = np.dot(obs_vector, dir_vector)
    angle = np.arccos(dot_product) * 57.2958 #Convert to degrees
    angle = angle * -1 if y[0] > y[1] else angle
    print("Obstacle: "+ str(round(dist,2))+"m @" + str(round(angle,2)) + "degrees")
    #print(vector)
   

    
    #thresh = np.rot90(thresh, k=1)
    #thresh = np.fliplr(thresh)

    #plt.imshow(thresh,'gray',vmin=0,vmax=255)
    #plt.xticks([]),plt.yticks([])

    #plt.plot(x, y, color="white", linewidth=3)

    #plt.quiver(pos_x, pos_y, new_x, new_y, color='white')

    #plt.show()
    


def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("/rtabmap/proj_map", OccupancyGrid, map_callback)
    #rospy.Subscriber("/rtabmap/proj_map", Odometry, odom_callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    print("listening for /rtabmap/proj_map")
    listener()

