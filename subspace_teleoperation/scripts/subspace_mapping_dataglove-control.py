#!/usr/bin/env python

'''
Copyright Columbia University in the City of New York.

This program is free software: you can redistribute it and/or modify it
under the terms of the GNU General Public License as published by the Free
Software Foundation, either version 2 of the License, or (at your option)
any later version.

This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
more details.

You should have received a copy of the GNU General Public License along
with this program.  If not, see <http://www.gnu.org/licenses/>.

Author(s): Cassie Meeker

@brief
Program takes in a JointState message from a dataglove on a ROS topic '/master_hand/joint_angles'.
It assumes that the master hand is human. Asks the human user for calibration poses to determine
delta values in T and calculates delta values of robot hand based on joint maxima and minima.
Maps from human joint values to teleoperation subspace (T) and then from T to the slave hand's joint
values. Slave joint values are published on a ROS topic called '/slave_hand/command' (JointState
message).

Notation:
o - origin
A = projection matrix
delta = scaling factor
T = teleoperation subspace
'''

from subspace_teleoperation.load_from_file import load_hand_model_from_xml_file
from subspace_teleoperation.subspace_mapping import *
import rospy
import os
from sensor_msgs.msg import JointState

class SubspaceMappingTeleoperation(object):
    def __init__(self, master_hand, slave_hand, model_dir):
        #Read arguments in as member variables
        self.master_hand = master_hand
        self.slave_hand = slave_hand
        self.model_dir = model_dir

        #Load from XML files the pertinent information for the master and slave hands
        self.load_hand_information_from_file()

        #Subscribe to dataglove
        rospy.Subscriber('/cyberglove/raw/joint_states', JointState, self.master_joint_angles_callback)

        #Get subspace mapping - this assumes that the master hand is human
        self.mapping = SubspaceMapping(self.model_dir)
        human_axes_ranges, self.o_master = self.get_human_hand_ranges()
        self.mapping.get_subspace_scaling_factor(self.slave_hand, self.master_hand, human_axes_ranges)

        #Initialize publisher for the slave hand
        self.slave_pub = rospy.Publisher('slave_hand/command', JointState, queue_size=0)
        self.slave_msg = JointState()
        self.slave_msg.name = self.q_names_slave

        #initialize timer which will execute teleoperation
        teleop_rate = 0.05
        self.timer = rospy.Timer(rospy.Duration(teleop_rate), self.teleoperate)

    def master_joint_angles_callback(self, joint_angles_msg):
        '''Get master joint angles from ROS topic and convert to point in T'''
        self.master_joint_angles = joint_angles_msg.position
        self.T_master = project_joint_angles_to_teleop_subspace(self.master_joint_angles, self.A_master, self.o_master)

    def teleoperate(self, event):
        T_slave = self.mapping.map_from_master_to_slave(self.T_master)
        self.slave_msg.position = project_from_teleop_subspace_to_joint_angles(T_slave, self.A_slave, self.o_slave)
        # print self.slave_msg.position
        self.slave_msg.position = self.enforce_slave_joint_limits(self.num_dof_slave, self.q_min_slave, self.q_max_slave, self.slave_msg.position)

        #Publish slave joint angles
        self.publish_actions()

    def publish_actions(self, publish_value=None):
        self.slave_pub.publish(self.slave_msg)

    def load_hand_information_from_file(self):
        '''
        Loads in hand information - reads in number of DOFs, origin (o), projection matrix (A) and min/max joint values
        '''
        self.num_dof_slave, self.o_slave, self.A_slave, self.q_max_slave, self.q_min_slave, self.q_names_slave =load_hand_model_from_xml_file(self.slave_hand, self.model_dir)

        print "%s model loaded."%self.slave_hand
        self.num_dof_master, self.o_master, self.A_master, self.q_max_master, self.q_min_master, self.q_names_master = load_hand_model_from_xml_file(self.master_hand, self.model_dir)
        print "%s model loaded."%self.master_hand

    def get_human_hand_ranges(self):
        '''Asks the user for a series of calibration poses to find scaling and origin of the human hand'''

        print "\n For more information on calibration poses, see calibration_poses.pdf. \n"

        raw_input("With the dataglove on, perform Calibration Pose 1. Once you have done this, press Enter.")
        spread_min = self.T_master[0]
        size_max = self.T_master[1]
        raw_input("With the dataglove on, perform Calibration Pose 2. Once you have done this, press Enter.")
        spread_max = self.T_master[0]
        raw_input("With the dataglove on, perform Calibration Pose 3. Once you have done this, press Enter.")
        size_min = self.T_master[1]
        raw_input("With the dataglove on, perform Calibration Pose 4. Once you have done this, press Enter.")
        curl_max = self.T_master[2]
        raw_input("With the dataglove on, perform Calibration Pose 5. Once you have done this, press Enter.")
        curl_min = self.T_master[2]
        origin = self.master_joint_angles

        spread_range = abs(spread_max)+abs(spread_min)
        size_range = abs(size_max)+abs(size_min)
        curl_range = abs(curl_max)+abs(curl_min)
        axes_ranges = [spread_range, size_range, curl_range]

        return axes_ranges, origin

    def enforce_slave_joint_limits(self, num_dof, joint_angle_minima, joint_angle_maxima, current_joint_angles):
        '''Makes sure that the slave joint angles do not exceed the given maxima and minima from xml'''

        for i in range(0, num_dof):
            current_joint_angles[i] = max(joint_angle_minima[i], min(joint_angle_maxima[i], current_joint_angles[i]))
        return current_joint_angles


if __name__ == '__main__':
    rospy.init_node('subspace_mapping_teleop', anonymous=True)

    master_hand = rospy.get_param('~master', "human")
    slave_hand = rospy.get_param('~slave', "schunk")
    model_dir = rospy.get_param('~model_dir', os.getcwd() + '/hand_models')

    print "The input parameters show we will be teleoperating a slave %s with a master %s. "%(slave_hand, master_hand)

    SubspaceMappingTeleoperation(master_hand, slave_hand, model_dir)
    rospy.spin()
