# ObjectAugmentationAlgorithm
The algorithm computes object trajectories (6DoF) prescribed by manipulation of the same in human grasping trials.
In the absence of optical markers on objects, object marker trajectories are computed from optical markers located on the 
subjects hand, wrist and forearm.

For algorithm details and validation, please refer to the publication:
C. Herneth et al., "Object Augmentation Algorithm: Computing virtual object motion and
object induced interaction wrench from optical markers," 2024 IEEE International Conference on Intelligent Robots and Systems (IROS)

When using the algorithm in your work please cite the work mentioned above.

# Workflow
The algorithm uses the OpenSim4.4 Api to compute inverse kinematics and inverse dynamics.
## 1. Prepare MoCap data
This is the data of the human trial. 
- Required markers: Elbow, Wrist (Radial and Ulnar styloids), Hand (minimum 3 markers)
- Dataformat: .trc format
- Orientation: must match with the OpenSim coordinate frame (y-axis up). This is important for the correct direction of gravity

## 2. Compute hand coordinate frames
For each input marker frame a hand coordnate frame is computed, where objects are places

## 3. Make object models
Create an OpenSim model of the object being manipulated in human trials

## 4 Generate Object Kinematics
First trajectories of virtual object markers are computed from hand coordinate frames, by placing object markers (3 for each rigid object) within each hand 
frame that describe the location and orientation of the object to be simulated. For example, a can could be placed such that it is centrally
grasped in the hand. For this markers at the cans CoM, top and somewhere at the circumference could be chosen.

Second object motion is computed via inverse kinematics with the OpenSim inverse kinematics solver.

## 5 Generate Object Dynamics
The object model and object motion computed in the previos step are used to compute the wrench necessary to move objects along the prescribed trajectory via
inverse kinematics with the OpenSim inverse dynamics tool.

## 6 Simulate Object Manipulation
The inverse dynamics result can be used in an external loads file, to apply the object wrench to the hand of the subject, which simulated object manipulation.

# Dependencies
numpy, scipy, pickle, plotly, OpenSim4.4 api for python, 

## Install OpenSim 4.4 for Python in windows
1. delete previous OpenSim versions from path or at least move below opensim 4.4
2. Download the Anaconda environment file (conda_env.yml) https://simtk-confluence.stanford.edu:8443/download/attachments/29165295/conda_env.yml?version=1&modificationDate=1643835525076&api=v2 and place it in a directory of your choice (conveniently in C:\OpenSim4.4\sdk\Python). There is a copy of this file inside this repo too.
3. open an Anaconda powershell
4. conda init cmd.exe
5. cd C:\OpenSim4.4\sdk\Python
6. conda env create -f conda_env.yml
7. conda activate opensim_scripting
8. python setup_win_python38.py (it is here C:\OpenSim4.4\sdk\Python)
9. python -m pip install . ( the dot is important)
10. step necessary on some machines: conda install - opensim-org opensim
11. to use OpenSim 4.4 in a python script there must be the following lines before importing opensim:
    import os
    os.add_dll_directory("C:/OpenSim4.4/bin") # or however the location is called on your PC

## Common errors with the OpenSim Api
- module _simbody not found:
    follow the installation instructions above. Has os.add_dll_directory("C:/OpenSim4.4/bin") been added to the script?
  
- scale tool doesnt work
    use absolute path to required files. Otherwise the tool returns an error

- InverKinematicsSolver throws an error that the number or type of input is wrong:
    this is probably because you are using OpenSim 4.2 and the c++ is expecting a pointer
    I couldn't solve this problem. Use openSim 4.4 as that function implements call by value as well

- RuntimeError: std::exception in 'bool OpenSim::InverseDynamicsTool::run()': InverseDynamicsTool Failed, please see messages window for details...
    check the ID settings file. Is there a model name or does the field say unassigned?
    If the xml file was created by the GUI then the file will be unassigned and the solver does not know which model to load

- AttributeError: module 'os' has no attribute 'add_dll_directory'
    are you using the correct environment? OpenSim44? This error usually occurs if opensim is not installed in that environment
