#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped,Pose
import numpy as np
from nav_msgs.msg import Path

# #int((2*np.pi)/0.01)
# t_dom=np.linspace(0,2*np.pi,1000)
# x = np.sin(t_dom) + 2 * np.sin(2 * t_dom)
# y = 0*t_dom
# z = -np.sin(3 * t_dom)

init = True
init_pose = Pose()

# trajectory = np.vstack((x,y,z))

# # Specifica il percorso e il nome del file CSV
# file_path = 'trajectory_Official.csv'

# # Esporta l'array in un file CSV
# np.savetxt(file_path, trajectory, delimiter=',', header='x,y,z', comments='')


def trajectory(t):
    x = 0.05* (np.sin(t) + 2 * np.sin(2 * t))
    y = 0  
    z = 0.05 * (-np.sin(3 * t))
    return x, y, z

def get_init(msg: PoseStamped):
   global init_pose, init

   if init==True:
      init_pose = msg.pose
      init= False


if __name__=='__main__':

    rospy.init_node('personal_controller')
    rospy.loginfo("Controller started!")
    rospy.loginfo("Sending trajectory:")
    pub = rospy.Publisher('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped, queue_size=10)

    sub = rospy.Subscriber('/cartesian_impedance_example_controller/equilibrium_pose', PoseStamped,get_init)
    
    path_requested = rospy.Publisher('/trajectory_requested',Path,queue_size=100)
    
    path_msg=Path()
    
    rate = rospy.Rate(10)  

    t=0
    dt=0.033
  
    while not rospy.is_shutdown():
      if not init:
        x,y,z = trajectory(t)
        
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "panda_link0"

        msg.pose.position.x = x + init_pose.position.x
        msg.pose.position.y = 0
        msg.pose.position.z = z + init_pose.position.z

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        msg_s = PoseStamped()
        msg_s.header.stamp = rospy.Time.now()
        msg_s.header.frame_id = "panda_link0"

        msg_s.pose.position.x = x/0.05 + init_pose.position.x
        msg_s.pose.position.y = 0
        msg_s.pose.position.z = z/0.05 + init_pose.position.z

        msg_s.pose.orientation.x = 0.0
        msg_s.pose.orientation.y = 0.0
        msg_s.pose.orientation.z = 0.0
        msg_s.pose.orientation.w = 1.0
        rospy.loginfo("X: "+ str(np.round(msg.pose.position.x,2))+" Z: "+ str(np.round(msg.pose.position.z,2)))


        path_msg.header = msg.header
        path_msg.poses.append(msg)
        path_requested.publish(path_msg)


        pub.publish(msg_s)
        

        t += dt
        if t> 2*np.pi:
         t=0
        rate.sleep()