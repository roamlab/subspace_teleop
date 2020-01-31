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

import rospy
import os
from subspace_teleoperation.subspace_mapping import SubspaceMappingTeleoperation
from subspace_teleoperation.hardware_safety import HardwareSafety 
from subspace_teleoperation.data_management import DataManager 



if __name__ == '__main__':
    rospy.init_node('subspace_mapping_teleop', anonymous=True)

    master_hand = rospy.get_param('~master', "human_15DOF")
    slave_hand = rospy.get_param('~slave', "schunk")
    model_dir = rospy.get_param('~model_dir', os.getcwd() + '/hand_models')

    print "The input parameters show we will be teleoperating a slave %s with a master %s. "%(slave_hand, master_hand)

    safety_class = HardwareSafety(slave_hand, model_dir)
    data_management_class = DataManager()
    SubspaceMappingTeleoperation(master_hand, slave_hand, model_dir, safety_class, data_management_class)
    rospy.spin()
