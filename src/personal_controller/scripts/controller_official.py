#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped,PointStamped
import numpy as np
from nav_msgs.msg import Path
import csv
import math
from scipy.spatial.transform import Rotation as R

trajectory=[]

if __name__ == '__main__':
    pub = rospy.Publisher('/joint_velocity_example_controller/trajectory_loader', PoseStamped, queue_size=10)
    path_pub = rospy.Publisher('/trajectory_controller',Path,queue_size=100)
    path_msg=Path()
    point_pub = rospy.Publisher('/point_controller',PointStamped,queue_size=100)
    rospy.init_node('trajectory_publisher')
    rate = rospy.Rate(100) 

    start_time = rospy.Time.now()

    try:
        rospy.sleep(1)
        while not rospy.is_shutdown():
            current_time = (rospy.Time.now() - start_time).to_sec()-7

            scale = 0.05
            omega = 0.2
            omega_rot=0.5
            center = np.array([0.45, 0.0, 0.50])

            
            x = center[0]+scale*np.sin(omega*current_time)+2*scale*np.sin(2*omega*current_time)
            y = center[1]
            z = center[2]+scale*np.cos(omega*current_time)-2*scale*np.cos(2*omega*current_time)
            # Rotation function
            yaw = (np.pi / 2  )* np.sin(omega_rot * current_time)#**2 * np.sign(np.sin(omega * current_time))
             
            rotation = R.from_euler('xyz', [np.pi, 0, yaw], degrees=False) 
            quat = rotation.as_quat()  # [x, y, z, w]

            theta = omega * current_time
            # ZYX (RPY) (0 0 180) respect to x
            q_x = quat[0]
            q_y = quat[1]
            q_z = quat[2]
            q_w = quat[3]
            

            trajectory.append([current_time+7,x,y,z,q_x, q_y, q_z, q_w])

            pose_msg = PoseStamped()
            pose_msg.header.stamp = rospy.Time.now()
            pose_msg.header.frame_id = "world"
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = z
            pose_msg.pose.orientation.x = q_x
            pose_msg.pose.orientation.y = q_y
            pose_msg.pose.orientation.z = q_z
            pose_msg.pose.orientation.w = q_w

            pub.publish(pose_msg)

            path_msg.header = pose_msg.header
            path_msg.poses.append(pose_msg)
            path_pub.publish(path_msg)

            point = PointStamped()
            point.header = pose_msg.header
            point.point.x = x
            point.point.y = y
            point.point.z = z
            point_pub.publish(point)

            rate.sleep()
    except rospy.ROSInterruptException:
     pass
    finally:
        
        with open("/home/vandalsnike/Results/trajectory.csv", mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['Time', 'x', 'y', 'z','q_x','q_y','q_z','q_w']) 
            writer.writerows(trajectory)
        rospy.loginfo("Traiettoria salvata su trajectory.csv")

 