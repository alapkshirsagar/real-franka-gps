#!/usr/bin/env python
import rospy
import roslib
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
roslib.load_manifest('gps_agent_pkg')
from gps_agent_pkg.msg import TrialCommand
import tf
import numpy as np
import csv

class optitrack_publisher:
    def __init__(self):
        self.pub = rospy.Publisher('/mocap_optitrack_data_topic', Float64MultiArray,queue_size = 200)
        self.trial_subscriber = rospy.Subscriber('/gps_controller_trial_command', TrialCommand, self.update_conditions_callback)
        rospy.set_param("feat_topic", "/mocap_optitrack_data_topic" )    
        self.tf_listener = tf.TransformListener()

        # self.trans_human = [0.632822275162,-0.0804761648178,0.374321460724]
        #For training
        self.trans_human_total = [np.array([r*np.cos(theta*np.pi/180),r*np.sin(theta*np.pi/180),z]) for r in np.linspace(0.70, 0.80, num = 2)
                        for theta in np.linspace(0, 45, num = 2)
                            for z in np.linspace(0.2, 0.25, num = 2)]
        print(self.trans_human_total)
        

        #For testing
        # self.trans_human_total = [np.array([r*np.cos(theta*np.pi/180),r*np.sin(theta*np.pi/180),z]) for r in np.linspace(0.70, 0.80, num = 3)
        #         for theta in np.linspace(0, 45, num = 10)
        #             for z in np.linspace(0.2, 0.25, num = 3)]

        self.trans_human = self.trans_human_total[0]
        looprate = rospy.Rate(100) # 100hz
        while not rospy.is_shutdown():
            self.publish_optitrack_data()
            looprate.sleep()


    def update_conditions_callback(self, msg):
        print("Iteration:", msg.iteration)
        print("Condition:", msg.condition)
        print("Sample:", msg.sample)
        self.trans_human = self.trans_human_total[msg.condition]





    def publish_optitrack_data(self):
        try:
            (trans_robot_orig, rot_robot) = self.tf_listener.lookupTransform("optitrack_origin", "kinova_gripper", rospy.Time(0))
            # print (trans_robot_orig)
            trans_robot =[0,0,0]
            trans_robot[0] = trans_robot_orig[1]- 0.118 + 0.25
            trans_robot[1] = -trans_robot_orig[0]
            trans_robot[2] = trans_robot_orig[2]+ 0.085 -0.13
            # (trans_object, rot_object) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_object, rospy.Time(0))
            # (trans_human_orig, rot_human) = self.tf_listener.lookupTransform("optitrack_origin", "human_hand", rospy.Time(0))
            trans_human=[0,0,0]
            # print(trans_robot)
            trans_human[0] = self.trans_human[0] - trans_robot[0]    #for RELATIVE-fix targets
            trans_human[1] = self.trans_human[1] - trans_robot[1]    #for RELATIVE-fix targets
            trans_human[2] = self.trans_human[2] - trans_robot[2]    #for RELATIVE-fix targets
            # print("Human position =", trans_human)
            # print("Robot position =", trans_robot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print 'Could not find transform'
            return

        ## Create Float64MultiArray Message from optitrack_data
        optitrack_data = Float64MultiArray()
        optitrack_data.layout.dim.append(MultiArrayDimension())
        optitrack_data.layout.dim[0].size = 2*len(trans_human)
        optitrack_data.layout.dim[0].stride = 1
        optitrack_data.layout.dim[0].label = "Optitrack data"

        optitrack_data.data = trans_robot + trans_human  #for RELATIVE
        # optitrack_data.data = trans_human + trans_human + trans_robot


        # print(optitrack_data)
        #publish to rostopicsensor
        self.pub.publish(optitrack_data)



if __name__ == '__main__':
    # Initialize node
    rospy.init_node('optitrack_publisher')
    try:
        optitrackPublisher = optitrack_publisher()
    except rospy.ROSInterruptException:
        pass