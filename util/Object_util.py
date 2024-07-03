import numpy as np
import pickle
import os
import scipy
from scipy.spatial.transform import Rotation as R

os.add_dll_directory("C:/OpenSim4.4/bin") # otherwise module _sombody not found error!

import opensim as osim

from OsimPython.osim_file_util import readFile_mot, createFile_mot, readFile_trc, createFile_trc, create_ExternalForce_mot, create_ExternalForce_xml, create_IDSettings_xml, createFile_ID_sto, readFile_ID_sto
from OsimPython.IK_util import SolverIK, plotIKError

from util.linalg import axisAngle_to_R
from util.plotUtil import addArrow

import plotly.graph_objs as go
import plotly as plotly

def createExperiment_dict(trc_file_path, trc_file_name, dev_ang, hmn, wmn, famn):
    '''
        Herneth et al. (2024) Object Augmentation Algorithm: Computing virtual object motion and object induced interaction wrench from optical markers
        param: trc_file_path: string path to the trial trc file
        param: trc_file_name: string trc file name of the specific experiment
        param: dev_ang_R: nd.array series of wrist deviation angles for each marker frame in radians
        param: hmnR: list of the Palm marker set
        param: wmnR: list of the Ulnar and then the Radial Styloid marker names
        parm: famnR: list of any 2 markers that show the forward direction of the forearm i.e. first the elbow marker and then any wrist marker
    '''
    # load prepared MoCap data
    markerNames,  marker_xyz, timeseries, frameseries, DataRate, data_dict_MoCap = readFile_trc(trc_file_path, trc_file_name + '.trc')

    experiment_dict = dict(
        experiment_name = trc_file_name,
        d = marker_xyz, 
        timeseries = timeseries, 
        data_dict_MoCap = data_dict_MoCap,
        # MoCap derived data
        HandMarkersR = marker_xyz[:, [markerNames.index(mn) for mn in hmn], :],
        WristMarkersR = marker_xyz[:, [markerNames.index(mn) for mn in wmn], :],
        ForearmMarkersR = marker_xyz[:, [markerNames.index(mn) for mn in famn], :],
        dev_ang_R = dev_ang, # numpy wants radians
    )

    return experiment_dict

def getHandVectors(experiment_dict, hand):
    '''
        computes coordinate frames of the hand based on markers on the forearm, wrist and hand.
        Herneth et al. (2024) Object Augmentation Algorithm: Computing virtual object motion and object induced interaction wrench from optical markers
        param: experiment_dict: dictionary that collects all necessary algorithm inputs
        param: hand = string can be 'R' for right and 'L' for left
        return: tupple of nd.arrays(3xN), where N is the number of Marker frames (hand frame center, palm normal vector, thumb up direction vector, finger forward vector)

    '''
    centroids = [] # centroid of palm
    n = [] # palm normal vector
    urs = [] # vector pointing from ulnar to radial styloid (thumbs up direction)
    p = [] # vector of radial to ulnar styloid prjected on plane defined by n and rotated by the hand deviation angle around n
    f = [] # vector pointing deribed from n cross p (pointing towards the fingers)

    counter = 0
    for frame in range(experiment_dict['HandMarkers' + hand].shape[0]):
        # WRIST
        wm = experiment_dict['WristMarkers' + hand][frame, :, :].squeeze()
        wm = wm[1] - wm[0]
        wm /= np.linalg.norm(wm)
        urs.append(wm)

        # FOREARM
        fam = experiment_dict['ForearmMarkers' + hand][frame, :, :].squeeze()
        fam = fam[1] - fam[0]
        if np.linalg.norm(fam) != 0.0:
            fam /= np.linalg.norm(fam)    

        c = np.cross (wm, fam) # the palm normal vector has to point in this direction

        # PALM
        dd = experiment_dict['HandMarkers' + hand][frame, :, :].squeeze()
        centroid = np.mean(dd, axis=0, keepdims=True)
        centroids.append(centroid) # centroids of hand markers in that frame

        if np.isinf(centroids).any() or np.isinf(fam).any(): # there are missing markers in the data, which cannot be handled
            raise Exception('there are missing markers in the data, which cannot be handled')

        # find normal vector to plane
        #https://math.stackexchange.com/questions/2810048/plane-fitting-using-svd-normal-vectorthe singular value decomposition 
        try: # the svd may not converge
            #svd = np.linalg.svd(dd.T - np.mean(centroid.T, axis=1, keepdims=True)) # the mean is redundant to the previous one
            svd = np.linalg.svd(dd.T - centroid.T) # careful this is the transpose!
            # extract the right singular vectors
            right = svd[0]
            n_vec = right[:, -1].squeeze()
            #print (np.linalg.norm(n_vec))
        except np.linalg.LinAlgError as e:
            # fallback in case svd did not converge
            def fun_normal(x, points):
                # minimizes the distance between points and a common plane

                r = points @ x[:3]
                r += np.zeros(points.shape[0]) * x[3] 
                r /= np.linalg.norm(x[:3])
                r = r**2

                return np.sum(r)
            res = scipy.optimize.minimize(fun_normal, x0=np.random.rand(4), args=(dd - centroid))
            n_vec = res.x[:3] / np.linalg.norm(res.x[:3])

        if n_vec @ c <= 0: # if its negative their angle is obtuse https://math.stackexchange.com/questions/2505184/two-vectors-are-in-the-same-direction-if
            n_vec *= -1
        
        n.append(n_vec)

        # Deviaton
        # https://math.stackexchange.com/questions/633181/formula-to-project-a-vector-onto-a-plane
        lm = wm @ n_vec / np.linalg.norm(n_vec) #/ (n_vec @ n_vec)
        # project dd onto plane defined by n
        p_vec = wm - lm * n_vec
        # rotate around n by deviation angle
        R = axisAngle_to_R(n_vec, experiment_dict['dev_ang_'+hand][frame])
        p_vec = R @ p_vec
        p_vec = p_vec / np.linalg.norm(p_vec)
        p.append(p_vec)

        # Finger direction
        f_vec = np.cross(n_vec, p_vec)
        f_vec = f_vec / np.linalg.norm(f_vec)
        f.append(f_vec)

        counter += 1

    centroids = np.array(centroids).squeeze() # palm centroids
    n = np.array(n) # unit normal vectors from palm centroids
    p = np.array(p)
    f = np.array(f)

    return (centroids, n, p, f)

def visualizeHandVectors(experiment_dict, hand_dict, hand, d_slice=None):
    '''
        visualizes the hand vectors created for a specific experiment at in the dataslive intervall
        param: experiment_dict: dict containing the information of the trial (see createExperiment_dict())
        param: hand_dict: dict contains the hand corrdinate frames of each marker frame (see getHandVectors())
        param: hand: string 'R' or 'L' for the example provided (for which hand the hand frames were computed)
        param: d_slice: tuple start and end index
    '''
    centroids = hand_dict['centroids'+hand]
    ns = hand_dict['n'+hand]
    ps = hand_dict['p'+hand]
    fs = hand_dict['f'+hand]

    d = experiment_dict['d']

    fig = go.Figure()
    i = 10
 
    if d_slice is not None:
        i_s = range(d_slice[0], d_slice[1])
    else:
        i_s = range(centroids.shape[0])

    for i in i_s:
    #for i in range(650, 750):
        fig = addArrow(fig, centroids[i], ns[i]*100, color='red', text=str(i), name='n')
        fig = addArrow(fig, centroids[i], ps[i]*100, color='green', name='p')#, text=str(i))
        fig = addArrow(fig, centroids[i], fs[i]*100, color='blue', name='f')
        #fig = addArrow(fig, centroidsR[i], ursR[i]*100, color='green')
        #fig = addArrow(fig, centroidsR[i], cc[i]*100, color='orange')

        fig.add_trace(go.Scatter3d(
        x=d[i:i+1, :, 0].squeeze(),
        y=d[i:i+1, :, 1].squeeze(),
        z=d[i:i+1, :, 2].squeeze(),
        mode='markers',
        marker=dict(
            size=5,
            color='cornflowerblue',
        )
        ))


    # fig.add_trace(go.Scatter3d(
    #         x=centroidsR[:, 0],
    #         y=centroidsR[:, 1],
    #         z=centroidsR[:, 2],
    #         text=[str(i) for i in range(centroidsR.shape[0])],
    #         mode='markers+text',
    #         marker=dict(
    #             size=1,
    #         )
    #     ))

    #fig.show()
    return fig

def createObjectTraj(experiment_dict, obj_dict, hand_dict, hand, Units, hand_vec=None, rad=0, c_offset=0, verbose=0):
    '''
        computes the object marker trajectory from previosly computed hand corrdinate frames (see getHandvectors()) and the location of object markers within 
        hand corrdinate frames (see createObjectModels.ipynb). Then the trajecotry of the objects' FreeJoint is computed via inverse kinematics in OpenSim.
        saves the generated object marker trajectories (.trc) and the inverse kinematics results (.mot)
        param: hand: string can be 'R' for right and 'L' for left
        param: hand_vec: nd.array a unit vector around which the markers should be rotated
        param: rad: double the rotation angle around hand_vec in rad
        param: c_offset: double the offset in the nF (Handvector pointing towards the fingers) direction of all markers
    '''
    # Motion Data
    if verbose:
        print('~~~~~~~~~~~~~~~  createObjectTraj ~~~~~~~~~~~~~~~~~~~~~~~~~~~~')

    centroids = hand_dict['centroids' + hand]
    n = hand_dict['n' + hand]
    p = hand_dict['p' + hand]
    f = hand_dict['f' + hand]

    experiment_name = experiment_dict['experiment_name']

    obj_name = obj_dict['obj_name'] # name of the osim model of the object loaded from object_path (start of this py file)
    obj_path = obj_dict['obj_path'] # savepath for all object related generated data
    filename = obj_dict['filename'] # filename prepend that is connected with this experiment and object

    with open(obj_path + obj_name + '.pkl', "rb") as input_file:
        obj_dict = pickle.load(input_file)

    # Bottle Markers
    # the object markers in the generated files are in m -> these are converted to respective mocap units
    mult = 1.0
    if Units == 'mm':
        mult = 1000
    elif Units == 'cm':
        mult = 100
    elif Units == 'dm':
        mult = 10

    object_markers = np.zeros([centroids.shape[0], len(obj_dict['markers']), 3])
    for i in range(len(obj_dict['markers'])):
        # n points along the palm normal
        # p points from the ulnar to the radial styloid and is then rotated around n by the wrist deviation angle
        # f is the cross product of nxp and points in the direction of the fingers
        object_markers[:, i, :] = centroids + n * obj_dict['markers'][i][0]*mult + p * obj_dict['markers'][i][1]*mult + f * obj_dict['markers'][i][2]*mult
    if hand_vec is not None:
        for d in range(object_markers.shape[0]): # for all markers
            r = R.from_rotvec(rad * hand_dict[hand_vec][d]).as_matrix().squeeze()
            c = hand_dict['centroidsR'][d] + f[d] * c_offset
            object_markers[d, :, :] = (r @ (object_markers[d, :, :] - c).T).T + c # move into hand frame, rotate arounf handvec and move back to global frame. (transpose becuase the marker coordinates are rows and not columns)

    createFile_trc(obj_path, filename+'.trc', object_markers, obj_dict['markerNames'])

    # Inverse Kinematics
    model = osim.Model(obj_path+obj_name+'.osim')

    # input to IK solver
    markerList = [marker.getName() for marker in model.get_MarkerSet()]
    weights = np.ones(len(markerList))

    accuracy = 1e-5
    Unit = 'deg'

    locations, errors, coord_val, coord_val_type, timeseries, momentarm = SolverIK(model, obj_path+filename+'.trc', markerList, weights, Unit=Unit, accuracy=accuracy)

    if verbose > 0:
        plotIKError(errors, timeseries) # should be almost 0 (look for exponent in plot)

    createFile_mot(obj_path, filename+'.mot', model=model, coord_val=coord_val, time=timeseries, Unit=Unit)

def createObjectID(obj_dict, timeseries, lowpass_cutoff_frequency_for_coordinates=6, verbose=0):
    '''
        Uses the object model and the trajectory of the objects' FreeJoint to compute the wrench necessary to move the object along the 
        prescribed trajectory. Saves the created inverse dynamics trajectory at a derived path

        param: obj_dict: dict with object path and names
        param: timeseries: list with all timeistants of the motion capture data
        param: lowpass_cutoff_frequency_for_coordinates: double low pass filter cutoff (Hz) of the IK data before ID. -1 means the the filter is not used
        return:  bool: True if ther was no error from the OpenSim ID tool
        

    '''
    if verbose:
        print('~~~~~~~~~~~~~~~  createObjectID  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~')

    obj_name = obj_dict['obj_name'] # name of the osim model of the object loaded from object_path (start of this py file)
    obj_path = obj_dict['obj_path'] # savepath for all object related generated data
    filename = obj_dict['filename'] # filename prepend that is connected with this experiment and object
    file_ending = obj_dict['filename_ending']

    # 1 create ID data for object following the manipulation trajectory
    settings = dict(
        results_directory = obj_path, 
        model_file = obj_path + obj_name + '.osim',
        time_range = [timeseries[0], timeseries[-1]],
        external_loads_file = 'Unassigned', # Unassigned ... for no external forces
        coordinates_file = obj_path + filename + file_ending +'.mot',
        # careful here: a filter to a stepwise applied acelleration will result in wrong results in the transient phase as 
        # the high frequency components in the spectrum of the step function are filtered out and we dont have a true stepfunction anymore
        lowpass_cutoff_frequency_for_coordinates = lowpass_cutoff_frequency_for_coordinates, # -1 ... for switching it off
        output_gen_force_file = obj_path + filename + file_ending +'.sto'
    )

    outfile = filename + '_p_IDSET' + '.xml'
    create_IDSettings_xml(settings, obj_path+outfile)
    idtool = osim.InverseDynamicsTool(obj_path+outfile)
    success = idtool.run()

    sto_data = readFile_ID_sto(obj_path, filename + file_ending +'.sto')
    sto_coord_val = sto_data[1]

    createFile_ID_sto(path=obj_path, filename=filename + file_ending +'.sto', coordList=sto_data[0], coord_val=sto_coord_val, time=sto_data[2])

    if verbose > 0:
        fig = go.Figure()
        for i, n in enumerate(sto_data[0]):
            fig.add_trace(go.Scatter(x=timeseries, y=sto_coord_val[:, i],
                        mode='lines',
                        name=n, 
                        line=dict(dash='dot')))
        fig.show()


    return success

def createExternalLoadsData(obj_dict, applied_to_body):
    '''
        Prepares object forces as external loads data to be processed by the OpenSim ID solver
        this creates a .mot file that contains the force and moment trajectories as well as the force attachement points for a FreeJoint
                and a .xml file that can be referenced in the settings xml for the IDtool and uses the external loads in the subject motion inverse dynamics

        prerequesite: Object.mot and .sto -> inverse kinematics and dynamics of the object along a given trajectory.
                        The forces necessary to move the object along that traectory can then be applied to the human, which emulates the manipulation forces with the object
        
        param: obj_dict: a dict with some object file names and paths
        param: applied_to_body: string of the name of the body of the human opensim model, where the object interaction wrench should be applied to

    '''
    obj_name = obj_dict['obj_name'] # name of the osim model of the object loaded from object_path (start of this py file)
    obj_path = obj_dict['obj_path'] # savepath for all object related generated data
    filename = obj_dict['filename'] # filename prepend that is connected with this experiment and object
    file_ending = obj_dict['filename_ending']

    model = osim.Model(obj_path+obj_name+'.osim')

    sto_data = readFile_ID_sto(obj_path, filename + file_ending + '.sto')
    mot_data = readFile_mot(obj_path, filename + file_ending + '.mot')
    trc_data = readFile_trc(obj_path, filename + file_ending + '.trc')

    forces = []
    sd = sto_data[-1] # last element = datadict
    #md = mot_data[-1] # last element = datadict
    td = trc_data[-1] # last element = datadict

    Units = td['Units']
    pos_mult = 1.0
    if Units == 'mm':
        pos_mult = 1/1000
    if Units == 'cm':
        pos_mult = 1/100
    if Units == 'dm':
        pos_mult = 1/10

    inDegrees = mot_data[-2] 

    # we assume all forces go to the CoM of the single Body in the Model
    # we assume all joints are free joints!
    p_keys = [mn.getName() for mn in model.get_MarkerSet()] # marker names
    modelBodies = [bn.getName() for bn in model.get_BodySet()] # we assume here that we just have a single body
    assert len(modelBodies) == 1, "The model can only have a single body, but has {}".format(len(modelBodies))
    #print(td[p_keys[0]])
    for joint in model.getJointSet(): # object (free) joints- this code is only meant for very simple objects connected to ground
        jn = joint.getName()
        assert joint.getConcreteClassName() in ['FreeJoint'] # only FreeJoints are supported but this is a {}'.format(joint.getConcreteClassName()) # for other joints the dict has the f_dict needs different force entries
        if joint.getConcreteClassName() == 'FreeJoint':
            f_keys = [key for key in sd.keys() if jn in key and '_force' in key]
            m_keys = [key for key in sd.keys() if jn in key and '_moment' in key]
            assert len(f_keys) == 3 and len(m_keys) == 3, f'less than 3 DoF in translation and/or 3 DoF in rotation found. Is this a freejoint?'
            
            # get relevant forces and moments:

            f_dict = dict(name=model.getName(), 
                        applied_to_body=applied_to_body, 
                        force_expressed_in_body='ground',
                        point_expressed_in_body='ground',
                        force_identifier=jn+'_force_v', 
                        point_identifier=jn+'_force_p',
                        torque_identifier=jn+'_torque_',
                        xfv = sd[f_keys[0]], # x forces
                        yfv = sd[f_keys[1]], # y forces
                        zfv = sd[f_keys[2]], # z forces
                        xfp = td[p_keys[0]][:, 0] * pos_mult, # x force points
                        yfp = td[p_keys[0]][:, 1] * pos_mult, # y force points
                        zfp = td[p_keys[0]][:, 2] * pos_mult, # z force points
                        xt = sd[m_keys[0]], # x moment
                        yt = sd[m_keys[1]], # y moment
                        zt = sd[m_keys[2]], # z moment
                        time = sd['time']
                    )
                        
            forces.append(f_dict)          

    force_mot_outfile = obj_path + filename + file_ending + '_' + 'ExtF' + '.mot'
    create_ExternalForce_mot(forces, inDegrees, outfile=force_mot_outfile, flip_sign_forces=True, flip_sign_torques=False)
    create_ExternalForce_xml(ExternalLoadsName='test', forces=forces, datafile=force_mot_outfile, outfile= obj_path + filename + file_ending + '_' + 'ExtF' + '.xml')