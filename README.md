# ECE470
The meat of the project is within the robotics.py file.

The UR3 arm is modelled as an object class, containing handle values for CoppeliaSim of the arm, joints, and gripper. 
Includes methods for calculating and updating joint angles as well as joint locations to relative to base.

The other key part of the project is within greedy.py, which is the entire sorting algorithm the project is based upon.
