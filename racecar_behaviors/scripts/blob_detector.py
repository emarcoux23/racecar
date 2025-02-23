#!/usr/bin/env python

import rospy
import cv2
import numpy as np
import message_filters
import tf
import tf2_ros
from racecar_behaviors.cfg import BlobDetectorConfig
from dynamic_reconfigure.server import Server
from std_msgs.msg import String, ColorRGBA
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, Quaternion
from cv_bridge import CvBridge, CvBridgeError
from racecar_behaviors.srv import goal, goalRequest, goalResponse
from tf.transformations import euler_from_quaternion
from libbehaviors import *
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def is_in_range(point, point_list, range_value):
    for p in point_list:
        if all(abs(p[i] - point[i]) <= range_value for i in range(len(point))):
            return True
    return False

def quaternion_to_yaw(w, x, y, z):
    # Calculate yaw (rotation around the vertical axis)
    yaw_rad = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    yaw_deg = np.degrees(yaw_rad)
    return yaw_deg

class BlobDetector:
    def __init__(self):
        self.blob_array: list = [(0,0)]
        self.blob_pos_x = 0.0
        self.blob_pos_y = 0.0
        self.distance = 0.0
        self.angle = 0.0
        self.flagSent = False
        self.robotPosition = Odometry()
        
        self.f = open("/home/racecar/catkin_ws/src/racecar/coordonnees.txt", "w")

        self.bridge = CvBridge()
        self.map_frame_id = rospy.get_param('~map_frame_id', 'map')
        self.frame_id = rospy.get_param('~frame_id', 'base_link')
        self.object_frame_id = rospy.get_param('~object_frame_id', 'object')
        self.color_hue = rospy.get_param('~color_hue', 120) # 160=purple, 100=blue, 10=Orange
        self.color_range = rospy.get_param('~color_range', 10) 
        self.color_saturation = rospy.get_param('~color_saturation', 50) 
        self.color_value = rospy.get_param('~color_value', 50) 
        self.border = rospy.get_param('~border', 10) 
        self.config_srv = Server(BlobDetectorConfig, self.config_callback)
        
        params = cv2.SimpleBlobDetector_Params()
        # see https://www.geeksforgeeks.org/find-circles-and-ellipses-in-an-image-using-opencv-python/
        #     https://docs.opencv.org/3.4/d0/d7a/classcv_1_1SimpleBlobDetector.html
        
        params.thresholdStep = 10;
        params.minThreshold = 50;
        params.maxThreshold = 220;
        params.minRepeatability = 2;
        params.minDistBetweenBlobs = 10;
        
        # Set Color filtering parameters 
        params.filterByColor = False
        params.blobColor = 255
        
        # Set Area filtering parameters 
        params.filterByArea = True
        params.minArea = 10
        params.maxArea = 5000000000
          
        # Set Circularity filtering parameters 
        params.filterByCircularity = True 
        params.minCircularity = 0.3
          
        # Set Convexity filtering parameters 
        params.filterByConvexity = False
        params.minConvexity = 0.95
              
        # Set inertia filtering parameters 
        params.filterByInertia = False
        params.minInertiaRatio = 0.1

        self.detector = cv2.SimpleBlobDetector_create(params)
        
        self.br = tf.TransformBroadcaster()
        self.listener = tf.TransformListener()
        
        self.image_pub = rospy.Publisher('image_detections', Image, queue_size=1)
        self.object_pub = rospy.Publisher('object_detected', String, queue_size=1)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.subOdom = rospy.Subscriber('/racecar/odometry/corrected', Odometry, self.odomCallback, queue_size=1)

        self.image_sub = message_filters.Subscriber('image', Image)
        self.depth_sub = message_filters.Subscriber('depth', Image)
        self.info_sub = message_filters.Subscriber('camera_info', CameraInfo)
        self.ts = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub, self.info_sub], 10)
        self.ts.registerCallback(self.image_callback)
    
    def odomCallback(self, odom: Odometry):
        self.robotPosition = odom

    def config_callback(self, config, level):
        rospy.loginfo("""Reconfigure Request: {color_hue}, {color_saturation}, {color_value}, {color_range}, {border}""".format(**config))
        self.color_hue = config.color_hue
        self.color_range = config.color_range
        self.color_saturation = config.color_saturation
        self.color_value = config.color_value
        self.border = config.border
        return config
  
    def image_callback(self, image, depth, info):
            
        try:
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except CvBridgeError as e:
            print(e)
            
        try:
            cv_depth = self.bridge.imgmsg_to_cv2(depth, "32FC1")
        except CvBridgeError as e:
            print(e)
        
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        mask = cv2.inRange(hsv, np.array([self.color_hue-self.color_range,self.color_saturation,self.color_value]), np.array([self.color_hue+self.color_range,255,255]))
        keypoints = self.detector.detect(mask) 
        
        closestObject = [0,0,0] # Object pose (x,y,z) in camera frame (x->right, y->down, z->forward)
        if len(keypoints) > 0:
            cv_image = cv2.drawKeypoints(cv_image, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
            
            for i in range(0, len(keypoints)):
                if info.K[0] > 0 and keypoints[i].pt[0] >= self.border and keypoints[i].pt[0] < cv_image.shape[1]-self.border:
                    pts_uv = np.array([[[keypoints[i].pt[0], keypoints[i].pt[1]]]], dtype=np.float32)
                    info_K = np.array(info.K).reshape([3, 3])
                    info_D = np.array(info.D)
                    info_P = np.array(info.P).reshape([3, 4])
                    pts_uv = cv2.undistortPoints(pts_uv, info_K, info_D, info_P)
                    self.angle = np.arcsin(-pts_uv[0][0][0]) # negative to get angle from forward x axis
                    x = pts_uv[0][0][0]
                    y = pts_uv[0][0][1]
                    #rospy.loginfo("(%d/%d) %f %f -> %f %f angle=%f deg", i+1, len(keypoints), keypoints[i].pt[0], keypoints[i].pt[1], x, y, angle*180/np.pi)
                    
                    # Get depth.
                    u = int(x * info.P[0] + info.P[2])
                    v = int(y * info.P[5] + info.P[6])
                    depth = -1
                    if u >= 0 and u < cv_depth.shape[1]:
                        for j in range(0, cv_depth.shape[0]):
                            if cv_depth[j, u] > 0:
                                depth = cv_depth[j, u]
                                break
                                # is the depth contained in the blob?
                                if abs(j-v) < keypoints[i].size/2:
                                    depth = cv_depth[j, u]
                                    break
                                
                    if depth > 0 and (closestObject[2]==0 or depth<closestObject[2]):
                        closestObject[0] = x*depth
                        closestObject[1] = 0 # to be same height than camera
                        closestObject[2] = depth

        # We process only the closest object detected
        if closestObject[2] > 0:
            # assuming the object is circular, use center of the object as position
            transObj = (closestObject[0], closestObject[1], closestObject[2])
            rotObj = tf.transformations.quaternion_from_euler(0, np.pi/2, -np.pi/2)
            self.br.sendTransform(transObj, rotObj,
                    image.header.stamp,
                    self.object_frame_id,
                    image.header.frame_id)  
            msg = String()
            msg.data = self.object_frame_id
            # rospy.loginfo("allo je suis ici")
            self.object_pub.publish(msg) # signal that an object has been detected
            
            # Compute object pose in map frame
            try:
                self.listener.waitForTransform(self.map_frame_id, image.header.frame_id, image.header.stamp, rospy.Duration(0.5))
                (transMap,rotMap) = self.listener.lookupTransform(self.map_frame_id, image.header.frame_id, image.header.stamp)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException) as e:
                print(e)
                return
            (transMap, rotMap) = multiply_transforms(transMap, rotMap, transObj, rotObj)
            
            # Compute object pose in base frame
            try:
                self.listener.waitForTransform(self.frame_id, image.header.frame_id, image.header.stamp, rospy.Duration(0.5))
                (transBase,rotBase) = self.listener.lookupTransform(self.frame_id, image.header.frame_id, image.header.stamp)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf2_ros.TransformException) as e:
                print(e)
                return
            (transBase, rotBase) = multiply_transforms(transBase, rotBase, transObj, rotObj)
            
            self.distance = np.linalg.norm(transBase[0:2])
            self.angle = np.arcsin(transBase[1]/transBase[0])
            
            # rospy.loginfo("Object detected at [%f,%f] in %s frame! Distance and direction from robot: %fm %fdeg.", transMap[0], transMap[1], self.map_frame_id, self.distance, self.angle*180.0/np.pi)
            self.object_pub.publish(str(self.distance) + ", " + str(self.angle*180.0/np.pi))

            self.blob_pos_x = transMap[0]
            self.blob_pos_y = transMap[1]

        distanceToStop = 1.5
        distanceMove = 3

        if (self.blob_pos_x != None and self.blob_pos_y != None and self.distance != None and self.angle != None):
            if not is_in_range((self.blob_pos_x, self.blob_pos_y), self.blob_array, 2.0):
                if self.distance > distanceMove:
                    if not self.flagSent:
                        rospy.wait_for_service('/pertiglance/goal')

                        try:
                            rospy.logwarn("Object detected sending goal")
                            srv = rospy.ServiceProxy('/pertiglance/goal', goal)
                            posRobotFrameX = self.robotPosition.pose.pose.position.x
                            posRobotFrameY = self.robotPosition.pose.pose.position.y

                            angleRobotFrame = quaternion_to_yaw(self.robotPosition.pose.pose.orientation.w,
                                                                self.robotPosition.pose.pose.orientation.x,
                                                                self.robotPosition.pose.pose.orientation.y,
                                                                self.robotPosition.pose.pose.orientation.z)

                            # angleFrame = np.atan(self.blob_pos_y/self.blob_pos_x) - self.angle
                            srv(posX= self.blob_pos_x, posY= self.blob_pos_y, theta_deg= angleRobotFrame - np.degrees(self.angle), type= 2)
                            self.flagSent = True
                            
                            self.filename = "Photo_debris_" + self.blob_array.size() + "_trajet_1.png"
                            self.f.write("Blob_" + self.blob_array.size() + " at: (" + self.blob_pos_x + ":" + self.blob_pos_y + "). Saved under: " + self.filename)
                            
                        except e:
                            rospy.logerr(e)

                elif self.distance > distanceToStop:
                    twist = Twist()
                    twist.angular.z = 2*self.angle
                    twist.linear.x = 0.25
                    # rospy.logwarn(twist)
                    self.cmd_vel_pub.publish(twist)

                elif self.distance < distanceToStop:
                    time = rospy.get_time()
                    while rospy.get_time() - time < 5:
                        self.cmd_vel_pub.publish(Twist())
                    
                    self.blob_array.append((self.blob_pos_x, self.blob_pos_y))
                    self.flagSent = False
                    self.cmd_vel_pub.publish(Twist())
                    rospy.wait_for_service('/pertiglance/goal')
                    self.cmd_vel_pub.publish(Twist())
                    
                    try:
                        self.cmd_vel_pub.publish(Twist())
                        srv = rospy.ServiceProxy('/pertiglance/goal', goal)
                        srv(posX= 0, posY= 0, theta_deg= 0, type= 4)
                    except :
                        rospy.logerr("Some erreur")
                        
                    try:
                        img: Image = rospy.wait_for_message("/racecar/raspicam_node/image", Image, rospy.Duration(1))
                        cv_image = self.bridge.imgmsg_to_cv2(img, "bgr8")
                        cv2.imwrite("/home/racecar/catkin_ws/src/racecar/" + self.filename, cv_image)
                        
                    except:
                        rospy.logerr("Some erreur :'(")

        # debugging topic
        if self.image_pub.get_num_connections()>0:
            cv_image = cv2.bitwise_and(cv_image, cv_image, mask=mask)
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                print(e)

def main():
    rospy.init_node('blob_detector')
    blobDetector = BlobDetector()
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
