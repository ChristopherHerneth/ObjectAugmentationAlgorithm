# ObjectAugmentationAlgorithm
The algorithm computes object trajectories (6DoF) prescribed by manipulation of the same in human grasping trials.
In the absence of optical markers on objects, object marker trajectories are computed from optical markers located on the 
subjects hand, wrist and forearm.

For algorithm details and validation, please refer to the publication:
C. Herneth et al., "Object Augmentation Algorithm: Computing virtual object motion and
object induced interaction wrench from optical markers," 2024 IEEE International Conference on Intelligent Robots and Systems (IROS)

When using the algorithm in your work please cite the work mentioned above.

# Dependencies
numpy, scipy, pickle, plotly, OpenSim4.4 api for python, 

## Install OpenSim 4.4 for Python in windows
Common errors: module _simbody not found:
1. delete previous OpenSim versions from path or at least move below opensim 4.4
2. Download the Anaconda environment file (conda_env.yml) https://simtk-confluence.stanford.edu:8443/download/attachments/29165295/conda_env.yml?version=1&modificationDate=1643835525076&api=v2 and place it in a directory of your choice (conveniently in C:\OpenSim4.4\sdk\Python)
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

# Prepare MoCap data
- needs to be in .trc format
- make sure its orientation matches with the OpenSim coordinate frame (y-axis up). This is important for the correct direction of gravity
