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
Contains linear projection functions, finds variables needed to map with teleoperation subspace and performs mapping from master to slave hands.
'''


import numpy as np
from subspace_teleoperation.load_from_file import load_hand_model_from_xml_file
import itertools

def project_joint_angles_to_teleop_subspace(joint_angles, projection_matrix, origin):
    pt_in_subspace = np.dot(np.subtract(joint_angles, origin), projection_matrix)
    pt_in_subspace = pt_in_subspace.astype(np.float64)
    return pt_in_subspace.tolist()

def project_from_teleop_subspace_to_joint_angles(pt_in_subspace, projection_matrix, origin):
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
        and unscaled value in T for the master hand to an unscaled value in T for the slave hand'''
        T_slave = []
        for i in range(0,3):
            T_slave = np.append(T_slave, self.mapping_slope[i]*(T_master[i]))
        return T_slave
