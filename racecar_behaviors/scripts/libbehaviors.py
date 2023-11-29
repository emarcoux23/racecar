#!/usr/bin/env python

import rospy
import tf
import numpy as np
from tf.transformations import euler_from_quaternion

def quaternion_to_yaw(quat):
    # Uses TF transforms to convert a quaternion to a rotation angle around Z.
    # Usage with an Odometry message: 
    #   yaw = quaternion_to_yaw(msg.pose.pose.orientation)
    (roll, pitch, yaw) = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
    return yaw
    
def multiply_transforms(trans1, rot1, trans2, rot2):
    trans1_mat = tf.transformations.translation_matrix(trans1)
    rot1_mat   = tf.transformations.quaternion_matrix(rot1)
    mat1 = np.dot(trans1_mat, rot1_mat)

    trans2_mat = tf.transformations.translation_matrix(trans2)
    rot2_mat    = tf.transformations.quaternion_matrix(rot2)
    mat2 = np.dot(trans2_mat, rot2_mat)

    mat3 = np.dot(mat1, mat2)
    trans3 = tf.transformations.translation_from_matrix(mat3)
    rot3 = tf.transformations.quaternion_from_matrix(mat3)
    
    return (trans3, rot3)

def brushfire(occupancyGrid):
    mapOfWorld = np.zeros(occupancyGrid.shape, dtype=int)
    
    mapOfWorld[occupancyGrid==100] = -1 # obstacles
    mapOfWorld[occupancyGrid==-1] = -2  # unknowns
    
    a = -1
    changed = True
    # do brushfire algorithm here
    while(changed):
        changed = False
        for y in range(1, mapOfWorld.shape[0]-1):
            for x in range(1, mapOfWorld.shape[1]-1):
                if mapOfWorld[y, x] == 0:
                    if mapOfWorld[y-1, x] == a or mapOfWorld[y+1, x] == a or mapOfWorld[y, x-1] == a or mapOfWorld[y, x+1] == a:
                        if a == -1:
                            mapOfWorld[y, x] = 1
                        else:
                            mapOfWorld[y, x] = a + 1
                        changed = True

        if a == -1:
            a = 1
        else:
            a += 1

    # brushfire: -1 = obstacle or unknown, safer cells have higher value)
    return mapOfWorld
