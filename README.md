Subspace Teleoperation Package:

ROS package which enables teleoperation using a teleoperation subspace as
described in "Intuitive Hand Teleoperation by Novice Operators Using a 
Continuous Teleoperation Subspace" (Meeker, et al. 2018).

Hand model directory contains xml files of human hand and various robot hands.
For instructions on how to create an xml file for a new robot hand, see
hand_model_info.txt. 

Program takes in arguments 'master_hand', 'slave_hand' and 'model_dir'. 
Master_hand and slave_hand arguments are names of xml hand model files and
model_dir is the location of the folder containing the hand models.
Default model_dir used with launch file is the hand_models folder in the package. 
Hand models are read in from the xml file using functions found in
load_from_file.py and converted to projection matrices. See 
hand_model_info.txt for more details on how this conversion happens.

Program subscribes a JointState message from a dataglove on a ROS topic 
'/cyberglove/raw/joint_states'. It assumes that the master hand is human. 
Asks the human user for calibration poses to determine delta values in the
subspace and uses functions from subspace_mapping.py to calculate delta values 
of the robot hand based on joint maxima and minima. The delta values of 
the master and slave are combined into a 'mapping_slope', this slope is
not referenced in the paper but is simply a combination of delta and delta_star
for the master and slave hands. Functions in subspace_mapping.py are also 
used to map from human joint values to teleoperation subspace (T) and then 
from T to the slave hand's joint values. Slave joint values are published 
on a ROS topic called '/slave_hand/command' (JointState msg).

Although joint minima and maxima are enforced on the slave_hand/command
topic, when using a real robot for grasping, additional functions to 
provide overload protection are recommended.

Calibration poses which the user is asked to perform when running the program
can be found in the calibration_poses.pdf file.

Notation - same as in the referenced paper:
o = origin
A = projection matrix
delta = scaling factor
T = teleoperation subspace


To run:
roslaunch teleoperation_subspace teleoperation_subspace.launch


You can specify some parameters in the launch file:
* master: The name of the master hand (also the title of the master hand's xml file) 
* slave: The name of the slave hand (also the title of the slave hand's xml file) 
* model_dir: The path to the folder where the models are kept

