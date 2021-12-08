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
        # self.human_data_file = open('/home/franka2/Kuka_Franka_GPS/src/real-franka-gps/gps/src/gps_agent_pkg/scripts/hand_data_train.csv')
        self.human_data_file = open('/home/franka2/Kuka_Franka_GPS/src/real-franka-gps/gps/src/gps_agent_pkg/scripts/hand_data_test.csv')
        csvreader = csv.reader(self.human_data_file)    
        nTrajectoriesRecorded = 18 #Number of trajectories recorded, 4 or 18 depending on train or test
        nTrajectoriesDesired = 90 #Number of trajectories desired, 8/12 or 90 depending on train with 8/12 or test

        trajectory_human = np.zeros((nTrajectoriesRecorded*100,3))
        trajectory_rotated = np.zeros((nTrajectoriesDesired*100,3)) #this includes all the trajectories of 100 timesteps each
        theta = np.linspace(0,np.pi/4,num = nTrajectoriesDesired/nTrajectoriesRecorded, endpoint = True, dtype = float)
        # print(range(len(theta)))
        row_number = 0
        for row in csvreader:
            # for column_number in range(3):
            trajectory_human[row_number, 0] = float(row[2])
            trajectory_human[row_number, 1] = float(row[0])
            trajectory_human[row_number, 2] = float(row[1])

            radius = (trajectory_human[row_number, 0]**2+trajectory_human[row_number, 1]**2)**0.5
            for angle in range(len(theta)):
                trajectory_rotated[angle*nTrajectoriesRecorded*100+row_number, :] = [radius*np.cos(theta[angle]), radius*np.sin(theta[angle]), float(row[1])]
            row_number+=1
        self.trans_human_total = []

        for i in range(nTrajectoriesDesired):
            self.trans_human_total.append(trajectory_rotated[i*100:(i+1)*100, :])
            print(trajectory_rotated[(i+1)*100-1,:])

        self.trans_human = self.trans_human_total[0]
        looprate = rospy.Rate(100) # 100hz
        self.timestep = -1 #timestep for each sample, -1 means trial not running
        while not rospy.is_shutdown():
            self.publish_optitrack_data()
            looprate.sleep()


    def update_conditions_callback(self, msg):
        print("Iteration:", msg.iteration)
        print("Condition:", msg.condition)
        print("Sample:", msg.sample)

        self.trans_human = self.trans_human_total[msg.condition]
        self.timestep = 0



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
            # print(self.trans_human[self.timestep,0])
            if self.timestep is not -1:
                trans_human[0] = self.trans_human[self.timestep, 0] - trans_robot[0]    #for RELATIVE-moving targets
                trans_human[1] = self.trans_human[self.timestep, 1] - trans_robot[1]    #for RELATIVE-moving targets
                trans_human[2] = self.trans_human[self.timestep, 2] - trans_robot[2]    #for RELATIVE-moving targets
            # trans_human[0] = self.trans_human[0] - trans_robot[0]    #for RELATIVE-fix targets
            # trans_human[1] = self.trans_human[1] - trans_robot[1]    #for RELATIVE-fix targets
            # trans_human[2] = self.trans_human[2] - trans_robot[2]    #for RELATIVE-fix targets
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
        if self.timestep == 99:
            self.timestep = 99
        elif self.timestep == -1:
            self.timestep = -1
        else:
            self.timestep+=1


if __name__ == '__main__':
    # Initialize node
    rospy.init_node('optitrack_publisher')
    try:
        optitrackPublisher = optitrack_publisher()
    except rospy.ROSInterruptException:
        pass