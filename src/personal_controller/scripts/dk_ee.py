#!/usr/bin/env python3

import rospy
import numpy as np
import math
from geometry_msgs.msg import PoseStamped, PointStamped
from sensor_msgs.msg import JointState
import tf
from nav_msgs.msg import Path
import csv

trajectory = []

def get_dh(joints):
   # DH parameters Craig's convention (from doc)
   # a,  d,  alpha, q
   dh_table=np.array([[0,        0.333, 0,          joints[0]],
                      [0,        0,     -math.pi/2, joints[1]],
                      [0,        0.316,  math.pi/2, joints[2]],
                      [0.0825,   0,      math.pi/2, joints[3]],
                      [-0.0825,  0.384, -math.pi/2, joints[4]],
                      [0,        0,      math.pi/2, joints[5]],
                      [0.088,    0,      math.pi/2, joints[6]],
                      [0,        0.107,  0,         0],
                      [0,        0.1034,  0,        -math.pi/4]])

   return dh_table

def TF(i, dh_pars):
 
 a      = dh_pars[i][0]
 d      = dh_pars[i][1]
 alpha  = dh_pars[i][2]
 q      = dh_pars[i][3]
 
 T_matrix = np.array([[np.cos(q),-np.sin(q), 0, a],
                    [np.sin(q)*np.cos(alpha), np.cos(q)*np.cos(alpha), -np.sin(alpha), -np.sin(alpha)*d],
                    [np.sin(q)*np.sin(alpha), np.cos(q)*np.sin(alpha),  np.cos(alpha),  np.cos(alpha)*d],
                    [   0,  0,  0,  1]])

 return T_matrix

def jointCallBack(msg: JointState):

 joints=[]

 for i in range(0,7):
   joints.append(msg.position[i])

 dh_pars=get_dh(joints)

 T_01 = TF(0, dh_pars)
 T_12 = TF(1, dh_pars)
 T_23 = TF(2, dh_pars)
 T_34 = TF(3, dh_pars)
 T_45 = TF(4, dh_pars)
 T_56 = TF(5, dh_pars)
 T_67 = TF(6, dh_pars)
 T_78 = TF(7, dh_pars)  # Flange
 T_8G = TF(8, dh_pars)  # Gripper

 T_0ee = T_01@T_12@T_23@T_34@T_45@T_56@T_67@T_78@T_8G

 trans = tf.transformations.translation_from_matrix(T_0ee)  
 quat = tf.transformations.quaternion_from_matrix(T_0ee)

 trajectory.append([rospy.Time.now().to_sec(), trans[0], trans[1], trans[2],quat[0],quat[1],quat[2],quat[3]])

 # Preparing and publishing PoseStamped message
 pose_stamped = PoseStamped()
 pose_stamped.header.stamp = rospy.Time.now()
 pose_stamped.header.frame_id = "panda_link0"
 pose_stamped.pose.position.x = trans[0]
 pose_stamped.pose.position.y = trans[1]
 pose_stamped.pose.position.z = trans[2]
 pose_stamped.pose.orientation.x = quat[0]
 pose_stamped.pose.orientation.y = quat[1]
 pose_stamped.pose.orientation.z = quat[2]
 pose_stamped.pose.orientation.w = quat[3]
 pub.publish(pose_stamped)

 # GUI trajectory
 path_msg.header = pose_stamped.header
 path_msg.poses.append(pose_stamped)
 path_pub.publish(path_msg)

 tf_broadcaster = tf.TransformBroadcaster()
 tf_broadcaster.sendTransform((trans[0],trans[1],trans[2]),
                              (quat[0],quat[1],quat[2],quat[3]),rospy.Time.now(),"est_ee","panda_link0")

 point_stamped = PointStamped()
 point_stamped.header = pose_stamped.header
 point_stamped.point.x = trans[0]
 point_stamped.point.y = trans[1]
 point_stamped.point.z = trans[2]
 point_pub.publish(point_stamped)


def save_trajectory():
    with open("/home/vandalsnike/Results/trajectory_done.csv", "w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(['Time', 'x', 'y', 'z','q_x','q_y','q_z','q_w']) 
        writer.writerows(trajectory)  
        rospy.loginfo("Traiettoria salvata su trajectory_done.csv")



if __name__ == '__main__':
    rospy.init_node("estimation_ee")
    rospy.loginfo("Estimation end-effector started")

    # Taking data from joint states
    rospy.Subscriber('/joint_states',JointState,jointCallBack)
    
    # Publishing PoseStamped message on /estimation_ee
    pub = rospy.Publisher('/estimation_ee',PoseStamped,queue_size=100)
    
    # GUI publishing on /trajectory_ee
    path_pub = rospy.Publisher('/trajectory_ee',Path,queue_size=100)
    path_msg=Path()
  
    point_pub = rospy.Publisher('/point_ee', PointStamped, queue_size=100)

    rospy.on_shutdown(save_trajectory)

    rospy.spin()
