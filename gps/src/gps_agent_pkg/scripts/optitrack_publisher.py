#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64MultiArray, MultiArrayDimension
import tf

class optitrack_publisher:
    def __init__(self):
        self.pub = rospy.Publisher('/mocap_optitrack_data_topic', Float64MultiArray,queue_size = 10)
        rospy.set_param("feat_topic", "/mocap_optitrack_data_topic" )    
        self.tf_listener = tf.TransformListener()
        r = rospy.Rate(20) # 20hz
        while not rospy.is_shutdown():
            self.publish_optitrack_data()
            r.sleep()


    def publish_optitrack_data(self):
        try:
            (trans_robot, rot_robot) = self.tf_listener.lookupTransform("optitrack_origin", "kinova_gripper", rospy.Time(0))
            # (trans_object, rot_object) = self.tf_listener.lookupTransform(self.config.optitrack_tf_origin, self.config.optitrack_tf_object, rospy.Time(0))
            (trans_human, rot_human) = self.tf_listener.lookupTransform("optitrack_origin", "human_hand", rospy.Time(0))
            # print("Human position =", trans_human)
            # print("Robot position =", trans_robot)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print 'Could not find transform'
            return

        ##TODO## Create Float64MultiArray Message from optitrack_data
        optitrack_data = Float64MultiArray()
        optitrack_data.layout.dim.append(MultiArrayDimension())
        optitrack_data.layout.dim[0].size = len(trans_robot) + len(trans_human)
        optitrack_data.layout.dim[0].stride = 1
        optitrack_data.layout.dim[0].label = "Optitrack data"

        optitrack_data.data = trans_human + trans_robot



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