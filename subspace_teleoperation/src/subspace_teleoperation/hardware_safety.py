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
Contains a hardwares safety class which performs very limited safety checks to
help protect hardware. We HIGHLY recommend adding more safety functions based
on your specific hardware and teleoepration needs. 
'''
from copy import deepcopy

class HardwareSafety(object):
    '''Hardware safety class, which performs safety checks for the slave hand. We 
    only check that the slave joint angles computed are within the range of  
    maximum and minimum joint angles, which are provided by the teleoperation
    function. This class should have a member function called ensure_robot_safety. 
    Within that function, you can call whatever safety checks you have created 
    for the slave robot. We have only provided a safety check which ensures that
    the slave joint angles are within a given range.'''
    def __init__(self):
        pass

    def enforce_slave_joint_limits(self, current_joint_angles, num_dof, q_minima, q_maxima):
        '''A function which, given the minimum and maximum each joint on the robot
        hand, checks that the joint angles for the slave are within that given range'''
        for i in range(0, num_dof):
            current_joint_angles[i] = max(q_minima[i], min(q_maxima[i], current_joint_angles[i]))
        return current_joint_angles

    def ensure_robot_safety(self, joint_positions, num_dof, q_min, q_max):
        '''This is the function which is called by the subspace teleoperation 
        function. Here you can call functions which will make sure that your
        hardware is safe.'''
        joint_positions = self.enforce_slave_joint_limits(deepcopy(joint_positions), num_dof, q_min, q_max)
        return joint_positions
