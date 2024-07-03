import os
os.add_dll_directory("C:/OpenSim4.4/bin") # otherwise module _sombody not found error!
import opensim as osim
import numpy as np
import plotly.graph_objs as go

from OsimPython.osim_file_util import readFile_trc

def SolverIK(model, trcpath, markerList, weights, Unit='deg', accuracy=1e-5, timeRange=None, verbose=0, model_initial_pose=None):
    '''
        model_initial_pose: Due to gimbal lock it can happen that the IK contains large angle jumps, which are not in the data. These can be 
                            circumvented by a different initial pose. None means no changes.
    '''
        
    # Initialize model.
    state = model.initSystem()
    nM = len(markerList)

    # change initial pose
    if model_initial_pose is not None:
        for joint in model.get_JointSet():
            if joint.getName() == 'groundthorax':
                for i in range(len(model_initial_pose)):
                    #print(joint.get_coordinates(i).getValue(state))
                    joint.get_coordinates(i).setValue(state, model_initial_pose[i])
    # --------------------------------------------------------------------------
    # MARKERS
    # --------------------------------------------------------------------------
    # Populate markers reference object with desired marker weights
    #l = set([osim.MarkerWeight(markerList[i], weights[i]) for i in range(nM)])
    markerWeights = osim.SetMarkerWeights()
    for i in range(nM):
        markerWeights.cloneAndAppend(osim.MarkerWeight(markerList[i], weights[i]))

    #print(markerWeights.getSize())

    # Set marker weights
    markerref = osim.MarkersReference(trcpath, markerWeights)
    markerref.setMarkerWeightSet(markerWeights)

    # --------------------------------------------------------------------------
    # COORDINATES
    # --------------------------------------------------------------------------
    # get model coordinate names
    coordName = [model.getCoordinateSet().get(i).getName() for i in range(model.getCoordinateSet().getSize())]
    # Create an arry of blank coordinate reference objects
    coordref = osim.SimTKArrayCoordinateReference()

    # Populate coordinate references object with desired coordinate values and weight
    for i in range(len(coordName)):
        coordRef = osim.CoordinateReference(coordName[i] , osim.Constant() )
        coordRef.setWeight(0)
        #coordRef.markAdopted()
        coordref.push_back(coordRef)

    # --------------------------------------------------------------------------
    # IKSolver
    # --------------------------------------------------------------------------

    # Get start and end times
    if timeRange is None:
        timeRange = markerref.getValidTimeRange()
        startTime = timeRange.get(0)
        endTime = timeRange.get(1)
    else:
        startTime = timeRange[0]
        endTime = timeRange[1]

    # Define constraint weight
    constweight = np.inf

    # Instantiate the ikSolver and set some values. 
    ikSolver = osim.InverseKinematicsSolver(model, markerref, coordref, constweight)

    # Set ikSolver accuracy
    ikSolver.setAccuracy(accuracy)

    #Assemble the model
    state.setTime(startTime)
    ikSolver.assemble(state)

    # Loop through IK coordinate tasks and define values and weights
    for i in range(model.getCoordinateSet().getSize()):
        ikSolver.updateCoordinateReference(coordName[i], 0)

    # Compute dt and nFrames
    dt = 1.0/markerref.getSamplingFrequency()
    nFrames = round(( (endTime-startTime)/dt )+1)

    # Perform IK analysis
    if verbose != 0:
        print('Starting IK analysis')
    errors = np.zeros([nFrames, nM])
    errors_squared = np.zeros([nFrames, nM])
    coord_val_t = []
    locations = np.zeros([nFrames, nM, 3])
    momentarm = []

    if verbose >= 2:
        path_trc = ''
        for ss in trcpath.split('/')[:-1]:
            path_trc += ss + '/'
        filename_trc = trcpath.split('/')[-1]
        print(path_trc)
        data_dict_mocap = readFile_trc(path_trc, filename_trc)[-1]

    for i in range(nFrames): # -1
        if verbose >= 2:
            print('time: ', startTime + i*dt)#, data_dict_mocap[-1]['marker_xyz'][i])

        # set the time
        state.setTime(startTime + i*dt)
        # run the iksolver for the time
        ikSolver.track(state)
        # Get the marker errors
        for u in range(nM): #-1
            errors[i, u] = ikSolver.computeCurrentMarkerError(u)
            errors_squared[i, u] = ikSolver.computeCurrentSquaredMarkerError(u)

        # Get the Model Marker Locations
        for u in range(nM): #-1
                location = ikSolver.computeCurrentMarkerLocation(u)
                locations[i, u, :] = [location.get(0), location.get(1), location.get(2)]

        #  Get the Coordinates Values
        model.realizeVelocity(state)
        #  Get the coordinate Set
        cs = model.getCoordinateSet()
        
        #FDS4 = model.getMuscles().get('FDS4')
        #momentarmi = FDS4.computeMomentArm(s,cs.get('MCP4_flex'))
        #momentarm[i] = momentarmi
        
        nvalues = cs.getSize()
        coord_val_row = []
        for u in range(nvalues):
            coord_val_row.append(cs.get(u).getValue(state))
        coord_val_t.append(coord_val_row)

    coord_val = np.array(coord_val_t) # angles are now in rad. But there are also translational and coupled joints:
    coord_val_type = []
    for i, coordinate in enumerate(model.getCoordinateSet()):
        coord_val_type.append(coordinate.getMotionType())
        #if coordinate.getMotionType() == 1 # 0 undefined, 1 rotational, 2 translational, 3 coupled
    coord_val_type = np.array(coord_val_type)
    
    if Unit == 'deg':
        coord_val[:, np.where(coord_val_type == 1)[0]] *= 180/np.pi

    # Pack the angles 
    # coord_valPacked = {}
    # for u in range(nvalues):
    #     name = cs.get(u).getName()
    #     if 'tx' not in name or 'ty' not in name or 'tz' not in name:
    #         coord_valPacked[name] = coord_val[:, u]
    #     else:
    #         # angles are in rad. Convert to deg
    #         if cs.get(u).getName() == 'auxprfem-auxprrud_coord':
    #             coord_valPacked['uxprfem_auxprrud_coord'] = coord_val[:, u] * 180 / np.pi
    #         else:
    #             coord_valPacked[name] = coord_val[:, u] * 180 / np.pi

    # coord_valPacked['time'] = np.arange(startTime, endTime, dt)
    time = np.arange(startTime, endTime+dt, dt)
    
    if verbose != 0:
        print('Done IK analysis')
    err = (errors, errors_squared)
    return locations, err, coord_val, coord_val_type, time, momentarm

def plotIKError(errors, timeseries, markerList=None):
    '''
        errors: list[tuple], for each frame in the motion data: error[0] = markererror, error[1] = squared marker error
        timeseries: list[float], timeinstant for each frame in the motion data
    '''
    fig = go.Figure()

    fig.add_trace(go.Scatter(
                x=timeseries,
                y=np.max(errors[1], axis = 1),
                name='total_squared_error',
                mode="lines",
                line = dict(color='blue', width=3))
                )
    fig.add_trace(go.Scatter(
                x=timeseries,
                y=np.max(errors[0], axis = 1),
                name='marker_error_max',
                mode="lines",
                line = dict(color='orange', width=3))
                )

    if markerList is not None:
        for i in range(errors[0].shape[1]):
            fig.add_trace(go.Scatter(
                    x=timeseries,
                    y=errors[0][:, i],
                    #name='marker_error {}'.format(i),
                    name = '{}'.format(markerList[i]),
                    mode="lines",
                    line = dict(width=1))
                    )

    fig.update_layout(
        width=800,
        height=600,
        title='Marker Error per Frame (should be very small)',
        xaxis_title="frame num",
        yaxis_title="Error [cm]",
        #legend_title="Legend Title",
    )

    fig.show()