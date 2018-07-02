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
Loads in hand models from formatted xml files

'''

import xml.etree.ElementTree as ET
import numpy as np

def load_hand_model_from_xml_file(hand_name, file_dir):
    '''Loads and parses hand models from xml files'''

    #Get hand model file by combining model directory and hand name
    model_file = file_dir + "/" + hand_name + '.xml'

    #Load the xml tree
    hand_model = load_hand_model_from_file(model_file)

    #Initialize variables, so if we miss them in xml file, we still return something
    hand_name = None
    hand_dof = None
    hand_origin = None
    hand_projection_matrix = None
    hand_max_joint_angles = None
    hand_min_joint_angles = None
    hand_joint_names = None

    #Parse the tree into meaningful elements
    root = hand_model.getroot()

    hand_name = root.attrib.get('Name')
    hand_dof = int(root.attrib.get('Dimensions'))

    #Get joints/values involved in each grasping dimension and put into matrix
    projection_matrix_components = [root.find("Spread_Dimension").find("DimVals"), root.find("Size_Dimension").find("DimVals"), root.find("Curl_Dimension").find("DimVals")]
    hand_projection_matrix = np.zeros((hand_dof, 3))
    subspace_dimension_index = 0

    for component in projection_matrix_components:
        for dof in component.attrib:
            joint_index = int(''.join(c for c in dof if c.isdigit()))
            hand_projection_matrix[joint_index, subspace_dimension_index] = component.attrib[dof]
        subspace_dimension_index += 1

    #Load origin
    hand_origin = np.zeros(hand_dof)
    origin_values = root.find("Origin").find("DimVals")
    hand_origin = fill_in_array_using_xml_element_attributes(hand_origin, origin_values)

    #Load joint maxima
    hand_max_joint_angles = np.zeros(hand_dof)
    max_values = root.find("Joint_Maxima").find("DimVals")
    hand_max_joint_angles = fill_in_array_using_xml_element_attributes(hand_max_joint_angles, max_values)

    #Load joint minima
    hand_min_joint_angles = np.zeros(hand_dof)
    min_values = root.find("Joint_Minima").find("DimVals")
    hand_min_joint_angles = fill_in_array_using_xml_element_attributes(hand_min_joint_angles, min_values)

    #Load joint minima
    joint_names = ["" for x in range(hand_dof)]
    name_values = root.find("Joint_Names").find("DimVals")
    joint_names = fill_in_array_using_xml_element_attributes(joint_names, name_values)

    print "Loaded hand model from ", model_file

    return hand_dof, hand_origin, hand_projection_matrix, hand_max_joint_angles, hand_min_joint_angles, joint_names

def fill_in_array_using_xml_element_attributes(array, element):
    for i in element.attrib:
        joint_index = int(''.join(c for c in i if c.isdigit()))
        array[joint_index] = element.attrib[i]
    return array

def load_hand_model_from_file(model_file):
    hand_model = ET.parse(model_file)
    return hand_model
