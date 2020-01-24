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
Contains linear projection functions, finds variables needed to map with 
teleoperation subspace, performs mapping from master to slave hands, and has
the subspace mapping teleoperation class which is agnostic to how the master
joint angles are recieved.
'''

import rospy
import numpy as np
import itertools
from sensor_msgs.msg import JointState
from subspace_teleoperation.load_from_file import load_hand_model_from_xml_file

def project_joint_angles_to_teleop_subspace(joint_angles, projection_matrix, origin):
    '''Given the joint angle of a hand, the projection matrix A and the origin O, 
    this will project from the joint angles to the pose in the teleoperation subspace'''
    pt_in_subspace = np.dot(np.subtract(joint_angles, origin), projection_matrix)
    pt_in_subspace = pt_in_subspace.astype(np.float64)
    return pt_in_subspace.tolist()

def project_from_teleop_subspace_to_joint_angles(pt_in_subspace, projection_matrix, origin):
    '''Given a pose in teleoperation subspace of a hand, the projection matrix A and the origin O, 
    this will project from the teleoperation subspace to the hand's joint angles'''
    projection_matrix_transposed = np.transpose(projection_matrix)
    joint_angles = np.add(np.dot(pt_in_subspace, projection_matrix_transposed), origin)
    return joint_angles.astype(np.float64)

class SubspaceMapping(object):
    '''Calculates delta values for robot hands and maps between master and slave teleoperation
    subspace points'''
    def __init__(self, model_dir):
        self.model_dir = model_dir

    def get_subspace_scaling_factor(self, slave_hand, master_hand, human_T_axes_ranges):
        '''Calculated delta values for the slave and master hand. Assumes that the master hand is human'''
        #Get unscaled subspace ranges
        T_axes_range_slave = self.calculate_T_axis_ranges(slave_hand)
        T_axes_range_master = human_T_axes_ranges

        #Get delta
        delta_master = np.zeros(3)
        delta_slave = np.zeros(3)
        for i in range(0,3):
            if T_axes_range_master[i]!=0: delta_master[i] = 1./T_axes_range_master[i]
            if T_axes_range_slave[i]!=0: delta_slave[i] = 1./T_axes_range_slave[i]
        delta_star_slave = T_axes_range_slave

        self.mapping_slope = np.multiply(delta_star_slave, delta_master)
        return delta_slave, self.mapping_slope

    def calculate_T_axis_ranges(self, hand):
        '''For each of the basis vectors in the teleoperation subspace, calculate
        the range of the possible values along the basis vectors for a given hand'''
        num_dof, origin, A, q_max, q_min, q_names = load_hand_model_from_xml_file(hand, self.model_dir)
        return self.get_robot_hand_ranges(origin, A, q_max, q_min)

    def get_robot_hand_ranges(self, origin, A, max_joint_angles, min_joint_angles):
        '''Based on the assumption that the maximum and minimum values of T will be found with some
        combination of joints at their maximum and minimum values. So for each component of T, we
        iterate through all the combinations of the relevant joints (defined by A) at their max and
        min values to find the ranges for T'''

        unscaled_T_min_and_max = np.zeros((3,2))
        component_index = 0

        #Look at the part of the projection matrix which corresponds to an axis in the subspace
        for component in np.transpose(A):
            #Find indices of joints which affect the current teleoperation subspace axis
            nonzeroind = np.nonzero(component)[0]

            #Iterate through to find all combinations of min and max for the joints we care about
            iteration_list = list(itertools.product([0, 1], repeat=len(nonzeroind)))

            #Initialize the unscaled max and min for the axis of T we are currently testing as zero
            axis_max = 0
            axis_min = 0

            #Create testing position where joints of interest are set to some combination of max and min
            for j in iteration_list:
                test_position = np.zeros(len(origin))
                for l in range(0, len(nonzeroind)):
                    if j[l] == 1: test_position[nonzeroind[l]] = max_joint_angles[nonzeroind[l]]
                    else: test_position[nonzeroind[l]] = min_joint_angles[nonzeroind[l]]
                #Project from the testing joint angles into teleoperation subspace
                test_T = project_joint_angles_to_teleop_subspace(test_position, A, origin)

                #Update unscaled T ranges based on newest testing position
                if test_T[component_index] > axis_max: axis_max = test_T[component_index]
                if test_T[component_index] < axis_min: axis_min = test_T[component_index]

            unscaled_T_min_and_max[component_index,:] = [axis_min, axis_max]
            #Move to next axis in teleoperation subspace
            component_index += 1

        #Based on the unscaled min and max values in T we have calculated, find ranges of the hand
        spread_range = abs(unscaled_T_min_and_max[0][0])+abs(unscaled_T_min_and_max[0][1])
        size_range = abs(unscaled_T_min_and_max[1][0])+abs(unscaled_T_min_and_max[1][1])
        envelope_range = abs(unscaled_T_min_and_max[2][0])+abs(unscaled_T_min_and_max[2][1])
        axes_range = [spread_range, size_range, envelope_range]

        #Return unscaled ranges of the hand
        return axes_range

    def map_from_master_to_slave(self, T_master):
        '''Combines the delta values of the master and slave hands into a mapping slope to convert
        an unscaled value in T for the master hand to an unscaled value in T for the slave hand'''
        T_slave = []
        for i in range(0,3):
            T_slave = np.append(T_slave, self.mapping_slope[i]*(T_master[i]))
        return T_slave


class SubspaceMappingTeleoperation(object):
    '''Implements the subspace mapping teleoperation using the subspace mapping class.
    This class will subscribe to the master joint angles, perform some hardware safety
    checks and publish the resulting slave joint angles. This class is completely 
    agnostic to how the master joint angles come in and how the slave joint angles
    are published (though we provide a default for publishing the slave joint angles).'''
    def __init__(self, master_hand, slave_hand, model_dir, hardware_safety_class, data_management_class):
        #Read arguments in as member variables
        self.master_hand = master_hand
        self.slave_hand = slave_hand
        self.model_dir = model_dir

        #Load from XML files the pertinent information for the master and slave hands
        self.load_hand_information_from_file()

        #Subscribe to the topic which contains the joint angles of the master hand
        self.create_subscriber()

        #Get subspace mapping - this assumes that the master hand is human
        self.mapping = SubspaceMapping(self.model_dir)
        human_axes_ranges, self.o_master = self.get_human_hand_ranges()
        self.mapping.get_subspace_scaling_factor(self.slave_hand, self.master_hand, human_axes_ranges)

        #Initialize publisher for the slave hand
        self.slave_pub = self.create_publisher()
        self.slave_msg = JointState()
        self.slave_msg.name = self.q_names_slave

        #Set member variable for the hardware safety class which was passed as an
        #argument. This class will provide limited checks to ensure safety of the slave robot
        self.safety_functions = hardware_safety_class

        #Set member variable for the data managment class. 
        self.data_manager = data_management_class

        #Initialize time variables
        self.start_time = rospy.Time.now()
        self.time_elapsed = (rospy.Time.now() - self.start_time).to_sec()

        #initialize timer which will execute teleoperation
        teleop_rate = 0.05
        self.timer = rospy.Timer(rospy.Duration(teleop_rate), self.teleoperate)

    def teleoperate(self, event):
        '''This is where the actual teleoperation happens. the master hand subscriber
        has calculated the master's position in the teleoperation subspace, so we use 
        the subspace mapping class to map from the master to slave. We then project
        this point in T to the joint space of the slave hand and perform some limited
        hardware safety checks.'''
        
        self.time_elapsed = (rospy.Time.now() - self.start_time).to_sec()

        T_slave = self.mapping.map_from_master_to_slave(self.T_master)
        self.slave_msg.position = project_from_teleop_subspace_to_joint_angles(T_slave, self.A_slave, self.o_slave)
        # print self.slave_msg.position

        #Perform safety checks, giving joint angle maximum and minimum from xml 
        #to the safety functions, which will check if the slave joint angles are
        #within the allowable range
        self.slave_msg.position = self.safety_functions.ensure_robot_safety(self.slave_msg.position, self.num_dof_slave, self.q_min_slave, self.q_max_slave)

        #Publish slave joint angles
        self.publish_actions()

        #Call data manager class to record data
        self.data_manager.record_data(self)

    def create_subscriber(self):
        '''Creates a ros subscriber which will fetch the joint angles of the master
        hand. Since this class is agnostic to how the joint angles from the master
        are recieved, we do not provide a default subscriber'''
        raise NotImplementedError

    def master_joint_angles_callback(self, joint_angles_msg):
        '''The callback which is called by the ROS subscriber which fetches the
        joint angles of the master. This function should set the master joint
        angles which come in as self.master_joint_angles. This function must also
        convert the master joint angles to a point in teleoperation subspace
        and assign this point to the variable self.T_master. 
        This can be accomplished using the line: 
        self.T_master = project_joint_angles_to_teleop_subspace(self.master_joint_angles, self.A_master, self.o_master)
        Since this lass is agnostic to how the joint angles from the master
        are recieved, we do not provide a default subscriber'''
        raise NotImplementedError        

    def create_publisher(self):
        '''Creates the ROS publisher to publish the slave joint angles at the 
        end of teleoperation. If another function does not overwrite this
        one, we publish the joint angles as a JointState msg on the topic 
        'slave_hand/command'. '''
        return rospy.Publisher('slave_hand/command', JointState, queue_size=0)

    def publish_actions(self, publish_value=None):
        '''Publishes the calculated joint angles of the slave hand. These joint
        angles and the joint names are stored in self.slave_msg (the position 
        and names field respectively). If another function does not overwrite this
        one, we publish the joint angles as a JointState msg. If you want to 
        overwrite this function, we suggest porting the information contained
        in self.slave_msg into a different message type within this function, 
        rather than changing the message type of self.slave_msg in the init function'''
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





class DatagloveTeleoperation(SubspaceMappingTeleoperation):
    '''This is a class which implements subspace mapping teleoperation controlled
    by a dataglove. Specifically, the dataglove should be publishing a JointState
    msg on the ROS topic /cyberglove/raw/joint_states. We allow the slave joint
    angles to be published using the default from the SubspaceMappingTeleoperation
    class, as a JointState msg on the topic slave_hand/command.'''
    def __init__(self, master_hand, slave_hand, model_dir, hardware_safety_class, data_management_class):
        super(DatagloveTeleoperation, self).__init__(master_hand, slave_hand, model_dir, 
                                                        hardware_safety_class, data_management_class)

    def master_joint_angles_callback(self, joint_angles_msg):
        '''Get master joint angles from ROS topic and convert to point in T'''
        self.master_joint_angles = joint_angles_msg.position
        self.T_master = project_joint_angles_to_teleop_subspace(self.master_joint_angles, self.A_master, self.o_master)

    def create_subscriber(self):
        rospy.Subscriber('/cyberglove/raw/joint_states', JointState, self.master_joint_angles_callback)
