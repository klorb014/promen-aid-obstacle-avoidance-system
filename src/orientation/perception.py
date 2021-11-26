#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from orientation.msg import HapticMsg
import numpy as np
import cv2
import argparse

class ObstacleDetector:

    FOV_H = 48

    def __init__(self, depth_topic, slices=3, image_topic=None):

        rospy.init_node('obstacle_detector',anonymous=True)

        self.depth_topic = depth_topic
        self.image_topic = image_topic
        self.slices = slices
        self.obstacle_distances = np.zeros(slices, dtype=float)
        self.obstacle_locations = np.zeros([slices, 2], dtype=int)
        self.pub = rospy.Publisher('haptic_feedback', HapticMsg, queue_size=1)

        rospy.Subscriber(depth_topic, Image,callback=self.convert_depth_image, queue_size=1)
        if image_topic is not None:
            rospy.Subscriber(image_topic, Image,callback=self.image_callback, queue_size=1)

    def publish_obstacle_data(self):
        msg = HapticMsg()
        msg.obstacle_distances = self.obstacle_distances
        self.pub.publish(msg)
        rospy.loginfo(msg)

    def nearest_obstacles(self, depth_array):
        depth_array[depth_array == 0] = np.nan
        image_width = depth_array.shape[1]
        slice_width = image_width/self.slices

        for slice_index in range(self.slices):
            slice = int(round(slice_width * slice_index))
            argmin = np.unravel_index(np.nanargmin(depth_array[:,slice:slice+slice_width]), depth_array[:,slice:slice+slice_width].shape)
            argmin_distance = (argmin[0],argmin[1]+slice)
            min_distance = round(depth_array[argmin_distance]/1000,2)
            self.obstacle_distances[slice_index] = min_distance
            if self.depth_topic is not None:
                self.obstacle_locations[slice_index][0] = argmin_distance[1]
                self.obstacle_locations[slice_index][1] = argmin_distance[0]


    def convert_depth_image(self, ros_image):
        bridge = CvBridge()
         # Use cv_bridge() to convert the ROS image to OpenCV format
        try:
         #Convert the depth image using the default passthrough encoding
            depth_image = bridge.imgmsg_to_cv2(ros_image, desired_encoding="passthrough")
            depth_array = np.array(depth_image, dtype=np.float32)
            self.nearest_obstacles(depth_array)
        except CvBridgeError as e:
            print(e)

    def image_callback(self, img_msg):
        # log some info about the image topic
        rospy.loginfo(img_msg.header)
        bridge = CvBridge()
        # Try to convert the ROS Image message to a CV2 Image
        try:
            cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
        except CvBridgeError, e:
            rospy.logerr("CvBridge Error: {0}".format(e))

        image_width = cv_image.shape[1]
        slice_width = image_width/self.slices

        for slice_index in range(1,self.slices):
            slice = int(round(slice_width * slice_index))
            color = (255, 0, 0)
            # Line thickness of 9 px
            thickness = 2
            print(cv_image.shape)
            start_point = (slice,0)
            end_point = (slice,720)
            # Using cv2.line() method
            # Draw a diagonal green line with thickness of 9 px
            image = cv2.line(cv_image, start_point, end_point, color, thickness)

        obstacles = self.obstacle_distances.shape[0]
        for obstacle_index in range(obstacles):
            coordinate = self.obstacle_locations[obstacle_index]
            x, y = coordinate[0], coordinate[1]
            dist = self.obstacle_distances[obstacle_index]
            cv_image = cv2.circle(cv_image, (x,y), radius=5, color=(0, 0, 255), thickness=-1)
            cv2.putText(cv_image, "{0}m".format(str(dist)), (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36,255,12), 2)
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(3)

    def run(self):
        rate = rospy.Rate(6) # 6hz
        while not rospy.is_shutdown():
            self.publish_obstacle_data()
            rate.sleep()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--display", help="display video feed", action="store_true")
    args = parser.parse_args()
    if args.display:
        obstacle_detector = ObstacleDetector("/camera/aligned_depth_to_color/image_raw", 3, "/camera/color/image_raw")
    else:
        obstacle_detector = ObstacleDetector("/camera/aligned_depth_to_color/image_raw")
    obstacle_detector.run()
