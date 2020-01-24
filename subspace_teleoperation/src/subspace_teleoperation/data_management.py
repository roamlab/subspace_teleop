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
Contains an empty data management class which can be used to store data from 
teleoperation. 
'''

class DataManager(object):
    '''This class is used for data management. When variables or
    information from a class need to be stored, this DataManagment class can 
    extract the data from the class of interest and save that information to file. 
    Here we provide an empty data management class to be filled in with your 
    favorite data management scheme.
    '''
    def __init__(self):
        pass

    def record_data(self, class_containing_data):
        '''This function will be passed the class which contains the data in 
        which we are interested (the subspace mapping teleoperation class, in
        this case). You can extract information from this class and record the 
        data.'''
        pass
