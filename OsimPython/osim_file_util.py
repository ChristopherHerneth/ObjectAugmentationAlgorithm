import os
os.add_dll_directory("C:/OpenSim4.4/bin") # otherwise module _sombody not found error!
import numpy as np

def createFile_trc(path, filename, MarkerData, MarkerNames, DataRate=100, Unit='mm'): 
    # This function was ported to python from https://simtk.org/projects/batchprocesstrc
    # Datarate [Hz]
    # OpenSim: The x-axis of the model coordinate system points forward from the model, the y-axis points upward, and the z-axis points to the right of the model.
    def createRow4(MarkerNames):
        Temp  = ['Frame#' + '\t' + 'Time' + '\t']
        for i in range(len(MarkerNames)):
            Temp.append(MarkerNames[i] + '\t' + '\t' + '\t')
        return Temp

    def createRow5(MarkerNames):
        Temp  = ['\t' + '\t']
        XYZ   = ['X','Y','Z']
        for i in range(1, len(MarkerNames)+1):
            for j in range(0, 3):
                Temp.append(XYZ[j] + str(i) + '\t')
        #print(Temp)
        return Temp
    PathFileType       = 4
    CameraRate         = DataRate
    NumFrames          = MarkerData.shape[0]
    NumMarkers         = len(MarkerNames)
    Units              = Unit
    OrigDataRate       = DataRate
    OrigDataStartFrame = 1
    OrigNumFrames      = NumFrames
    Interval           = 1/DataRate

    # Header for trc file
    Header     = []
    Header.append('PathFileType' + '\t' + str(PathFileType) + '\t' + '(X/Y/Z)' + '\t' + filename)
    Header.append('DataRate' + '\t' + 'CameraRate' + '\t' + 'NumFrames' + '\t' + 'NumMarkers' + '\t' + 'Units' + '\t' + 'OrigDataRate' + '\t' + 'OrigDataStartFrame' + '\t' + 'OrigNumFrames')
    Header.append(str(DataRate) + '\t' + str(CameraRate) + '\t' + str(NumFrames) + '\t' + str(NumMarkers) + '\t' + Units + '\t' + str(OrigDataRate) + '\t' + str(OrigDataStartFrame) + '\t' + str(OrigNumFrames))

    with open(path+filename, mode='w', newline='') as out_file:
        #csv_writer = csv.writer(out_file, delimiter='', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        for row in Header:
            out_file.write(row)
            out_file.write('\n')
        for row in createRow4(MarkerNames):
            out_file.write(row)
        out_file.write('\n')
        for row in createRow5(MarkerNames):
            out_file.write(row)
        out_file.write('\n')   
        out_file.write('\n')     
        for i in range(NumFrames):
            row_dat = [MarkerData[i, j, k] if MarkerData[i, j, k] is not np.inf else None for j in range(MarkerData.shape[1]) for k in range(MarkerData.shape[2])]
            row_dat.insert(0, round(i*Interval,2)) # time
            row_dat.insert(0, i+1) # frame num
            outline = ''
            for rd in row_dat:
                outline += str(rd)
                outline += '\t'
            out_file.write(outline)    
            out_file.write('\n') 

def readFile_trc(path, filename):
    '''
        marker_xyz = [len timeseries, num markers, x, y, z]
    '''
    dr_flag = False # datarate flag in next line = False
    xyz_cntr = 0

    marker_xyz = []
    timeseries = []
    frameseries = []
    markerList = []
    DataRate = None
    Units = None
    info_header = None
    info_data = None

    with open(path+filename) as f:
        lines = f.readlines()

        for l, line in enumerate(lines):
            if 'DataRate' in line:
                info_header = line.split()
                dr_flag = True
                continue
            elif 'Frame#' in line and 'Time' in line:
                temp = line.split()
                markerList = temp[2:] # 0 frame#, 1 Time
                xyz_cntr += 1
                continue

            if dr_flag == True: # flag was set in previous line
                info_data = line.split()
                DataRate = float(info_data[0])
                Units = info_data[info_header.index('Units')]
                dr_flag = False

            if xyz_cntr >= 5:
                temp = line.split()
                if len(temp) > 0:
                    frameseries.append(temp[0])
                    timeseries.append(temp[1])
                    marker_xyz.append([float(t) if 'Nan' not in t else np.nan for t in temp[2:]])

            xyz_cntr += 1 # jump X1, X2, Z1 ... line, or empty line afterwards
        
        marker_xyz = np.array(marker_xyz)
        marker_xyz[np.where(np.isnan(marker_xyz))] = np.inf
        marker_xyz = marker_xyz.reshape(marker_xyz.shape[0], int(marker_xyz.shape[1]/3), 3)

        data_dict = {}
        data_dict['time'] = timeseries
        data_dict['frame'] = frameseries
        data_dict['info_header'] = info_header
        data_dict['info_data'] = info_data
        data_dict['DataRate'] = DataRate
        data_dict['Units'] = Units
        data_dict['markerList'] = markerList
        data_dict['marker_xyz'] = marker_xyz

        for mi, mn in enumerate(markerList):
            data_dict[mn] = marker_xyz[:, mi, :]

    return markerList, marker_xyz, timeseries, frameseries, DataRate, data_dict

def readFile_mot(path, filename):
    '''
        return:
            coordList: List(str) -> contains the names of the coordinates
            coord_val: nd.array(np.float32) -> numframes x num coordinates containing the coordinate velocities for each frame in the order of the names in coordList
            timseries: List(float) -> vector containing the timestamt of each frame
            inDegrees: bool -> True: rotational velocities are in deg/s otherwise in rad/s
    '''
    m_flag = False
    a_flag = False
    coord_val = []
    timeseries = []
    coordList = []
    inDegrees = False

    with open(path+filename) as f:
        lines = f.readlines()

    for line in lines:
        if 'inDegrees' in line and 'yes' in line:
            inDegrees = True
            continue
        elif 'endheader' in line:
            m_flag = True
            continue
        elif m_flag:
            temp = line.split()
            coordList = temp[1:] # 0 is 'time'
            m_flag = False
            a_flag = True
            continue
        elif a_flag:
            temp = line.split()
            timeseries.append(float(temp[0])) # 0 is time
            coord_val.append([float(t) for t in temp[1:]])
    a_flag = False 
    coord_val = np.array(coord_val)
    coord_val = coord_val.astype(np.float64)

    timeseries = np.array(timeseries)
    timeseries = timeseries.astype(np.float64)

    data_dict = {} # datadict, where the key is the coordname + _moment or _force
    data_dict['time'] = timeseries.flatten()
    data_dict['coordList'] = coordList
    for i, coord in enumerate(coordList):
        data_dict[coord] = coord_val[:, i].reshape(coord_val[:, i].shape[0])

    return coordList, coord_val, timeseries, inDegrees, data_dict

def createFile_mot(path, filename, model, coord_val, time, Unit='rad'):
    '''
        Unit = str: rad or deg
    '''
    def creatColumnTags(model):
        Temp  = ['time' + '\t']
        for joint in model.getJointSet():
            skip = False
            for component in joint.getComponentsList():
                n = component.getName()
                if 'offset' in n:
                    skip = True
                if not skip: # generator cant be escaped with break
                    Temp.append(n + '\t')
        return Temp

    Version = 1

    Header = []
    Header.append('IKResults')
    Header.append('version=' + str(Version))
    Header.append('nRows={}'.format(coord_val.shape[0]))
    Header.append('nColumns={}'.format(coord_val.shape[1]+1)) #+1 for time column
    if Unit == 'deg':
        Header.append('inDegrees=yes')
    elif Unit == 'rad':
        Header.append('inDegrees=no')
    else:
        print('error: unknown unit: should be "deg" or "rad"')

    Header.append('')
    Header.append('Units are S.I. units (second, meters, Newtons, ...)')
    Header.append("If the header above contains a line with 'inDegrees', this indicates whether rotational values are in degrees (yes) or radians (no).")
    Header.append('')
    Header.append('endheader')

    with open(path+filename, mode='w', newline='\n') as out_file:
        #csv_writer = csv.writer(out_file, delimiter='', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        for row in Header:
            out_file.write(row)
            out_file.write('\n')
        for row in creatColumnTags(model):
            out_file.write(row)
        out_file.write('\n')
        for i in range(coord_val.shape[0]): # num frames
            outline = ''
            outline += '\t  '
            outline += '{:.8f}'.format(time[i])
            for a in coord_val[i]:
                outline += '\t\t{:.8f}'.format(a)
            out_file.write(outline)    
            out_file.write('\n') 

def createFile_ID_sto(path, filename, coordList, coord_val, time, Unit='rad'):
    '''
        Unit = str: rad or deg
    '''     
    Version = 1

    Header = []
    Header.append('Inverse Dynamics Generalized Forces')
    Header.append('version=' + str(Version))
    Header.append('nRows={}'.format(coord_val.shape[0]))
    Header.append('nColumns={}'.format(coord_val.shape[1]+1)) #+1 for time column
    if Unit == 'deg':
        Header.append('inDegrees=yes')
    elif Unit == 'rad':
        Header.append('inDegrees=no')
    else:
        print('error: unknown unit: should be "deg" or "rad"')
    Header.append('endheader')

    columnHeader  = ['time' + '\t']
    for coord in coordList:
        columnHeader.append(coord + '\t')

    with open(path+filename, mode='w', newline='\n') as out_file:
        #csv_writer = csv.writer(out_file, delimiter='', quotechar='"', quoting=csv.QUOTE_MINIMAL)
        for row in Header:
            out_file.write(row)
            out_file.write('\n')
        for row in columnHeader:
            out_file.write(row)
        out_file.write('\n')
        for i in range(coord_val.shape[0]): # num frames
            outline = ''
            outline += '\t  '
            outline += '{:.8f}'.format(time[i])
            for a in coord_val[i]:
                outline += '\t\t{:.8f}'.format(a)
            out_file.write(outline)    
            out_file.write('\n') 

def readFile_ID_sto(path, filename):
    '''
        return:
            coordList: List(str) -> contains the names of the coordinates
            coord_val: nd.array(np.float32) -> numframes x num coordinates containing the coordinate velocities for each frame in the order of the names in coordList
            timseries: List(float) -> vector containing the timestamt of each frame
            inDegrees: bool -> True: rotational velocities are in deg/s otherwise in rad/s
    '''
    m_flag = False
    a_flag = False
    coord_val = []
    timeseries = []
    coordList = []
    inDegrees = False

    with open(path+filename) as f:
        lines = f.readlines()

    for line in lines:
        if 'inDegrees' in line and 'yes' in line:
            inDegrees = True
            continue
        elif 'endheader' in line:
            m_flag = True
            continue
        elif m_flag:
            temp = line.split()
            coordList = temp[1:] # 0 is 'time'
            m_flag = False
            a_flag = True
            continue
        elif a_flag:
            temp = line.split()
            timeseries.append(float(temp[0])) # 0 is time
            coord_val.append([float(t) for t in temp[1:]])
    a_flag = False 
    coord_val = np.array(coord_val)
    coord_val = coord_val.astype(np.float64)

    timeseries = np.array(timeseries)
    timeseries = timeseries.astype(np.float64)

    data_dict = {} # datadict, where the key is the coordname + _moment or _force
    data_dict['time'] = timeseries.flatten()
    data_dict['inDegrees'] = inDegrees
    data_dict['coordList'] = coordList
    data_dict['coord_val'] = coord_val
    
    for i, coord in enumerate(coordList):
        data_dict[coord] = coord_val[:, i].reshape(coord_val[:, i].shape[0])

    return coordList, coord_val, timeseries, inDegrees, data_dict

def create_ExternalForce_mot(forces, inDegrees, outfile, flip_sign_forces=False, flip_sign_torques=False):
    nRows = forces[0]['time'].shape[0]
    nCols = len(forces)*9  # 6 dof per force + 3 points per force
    time = forces[0]['time']
    Header = [outfile, 
              'vesion=1', 
              'nRows={}'.format(nRows), 
              'nColumns={}'.format(nCols+1), # + time column
              'inDegrees={}'.format('yes' if inDegrees else 'no'), 
              'endheader'
              ]
    
    columnTitles = 'time'

    # following the structure in the example models of openSim 4.4 (Gait2354_Simbody)
    data = np.zeros([nRows, nCols])
    ctr = 0

    f_mult = 1.0
    if flip_sign_forces:
        f_mult = -1.0 # this means the forces are now additional loads to the joints
    t_mult = 1.0
    if flip_sign_torques:
        t_mult = -1.0 # this means the forces are now additional loads to the joints
    for force in forces:
        columnTitles += '\t{}{}'.format(force['force_identifier'], 'x')
        if 'xfv' in force.keys():  
            data[:, ctr] = force['xfv'] * f_mult
        ctr += 1
        columnTitles += '\t{}{}'.format(force['force_identifier'], 'y')
        if 'yfv' in force.keys():
            data[:, ctr] = force['yfv'] * f_mult
        ctr += 1
        columnTitles += '\t{}{}'.format(force['force_identifier'], 'z')
        if 'zfv' in force.keys():   
            data[:, ctr] = force['zfv'] * f_mult
        ctr += 1

        columnTitles += '\t{}{}'.format(force['point_identifier'], 'x')
        if 'xfp' in force.keys():
            data[:, ctr] = force['xfp'] 
        ctr += 1
        columnTitles += '\t{}{}'.format(force['point_identifier'], 'y')
        if 'yfp' in force.keys():   
            data[:, ctr] = force['yfp'] 
        ctr += 1
        columnTitles += '\t{}{}'.format(force['point_identifier'], 'z')
        if 'zfp' in force.keys():     
            data[:, ctr] = force['zfp'] 
        ctr += 1

    for force in forces:
        columnTitles += '\t{}{}'.format(force['torque_identifier'], 'x')
        if 'xt' in force.keys(): 
            data[:, ctr] = force['xt'] * t_mult
        ctr += 1
        columnTitles += '\t{}{}'.format(force['torque_identifier'], 'y')
        if 'yt' in force.keys():
            data[:, ctr] = force['yt'] * t_mult
        ctr += 1
        columnTitles += '\t{}{}'.format(force['torque_identifier'], 'z')
        if 'zt' in force.keys():
            data[:, ctr] = force['zt'] * t_mult
        ctr += 1
    
    with open(outfile, 'w', newline='\n') as f:
        for line in Header:    
            f.write(line)
            f.write('\n')
        f.write(columnTitles)
        f.write('\n')
        for i in range(nRows): # num frames
            outline = ''
            outline += '\t  '
            outline += '{:.8f}'.format(time[i])
            for a in data[i]:
                outline += '\t{:.8f}'.format(a)
            f.write(outline)    
            f.write('\n')

def create_ExternalForce_xml(ExternalLoadsName, forces, datafile, outfile):
    HeaderS = ['<?xml version="1.0" encoding="UTF-8"?>', 
              '<OpenSimDocument Version="40000">', 
              '\t<ExternalLoads name="{}">'.format(ExternalLoadsName), 
              '\t\t<objects>'
              ]
    
    force_dat = []
    for force in forces:
        f_d = [	'\t\t\t<ExternalForce name="{}">'.format(force['name']),
				'\t\t\t\t<isDisabled> false </isDisabled>',
				'\t\t\t\t<!--Name of the body the force is applied to.-->',
				'\t\t\t\t<applied_to_body> {} </applied_to_body>'.format(force['applied_to_body']),
				'\t\t\t\t<!--Name of the body the force is expressed in (default is ground).-->',
				'\t\t\t\t<force_expressed_in_body> ground </force_expressed_in_body>',
				'\t\t\t\t<!--Name of the body the point is expressed in (default is ground).-->',
				'\t\t\t\t<point_expressed_in_body> ground </point_expressed_in_body>',
				'\t\t\t\t<!--Identifier (string) to locate the force to be applied in the data source.-->',
				'\t\t\t\t<force_identifier> {} </force_identifier>'.format(force['force_identifier']),
				'\t\t\t\t<!--Identifier (string) to locate the point to be applied in the data source.-->',
				'\t\t\t\t<point_identifier> {} </point_identifier>'.format(force['point_identifier']),
				'\t\t\t\t<!--Identifier (string) to locate the torque to be applied in the data source.-->',
				'\t\t\t\t<torque_identifier> {} </torque_identifier>'.format(force['torque_identifier']),
				'\t\t\t\t<!--Name of the data source (Storage) that will supply the force data.-->',
				'\t\t\t\t<data_source_name> Unassigned </data_source_name>',
			    '\t\t\t</ExternalForce>',
                ]
        force_dat.append(f_d)

    HeaderE = ['\t\t</objects>',
            '\t\t<groups/>,',
            '\t\t<!--Storage file (.sto) containing (3) components of force and/or torque',
            '\t\t\tand point of application.Note: this file overrides the data source',
            '\t\t\tspecified by the individual external forces if specified.-->',
            '\t\t<datafile> {} </datafile>'.format(datafile),
            '</ExternalLoads>',
            '</OpenSimDocument>'
            ]
    
    with open(outfile, 'w') as f:
        for line in HeaderS:    
            f.write(line)
            f.write('\n')
        for f_d in force_dat:
            for line in f_d:    
                f.write(line)
                f.write('\n')
        for line in HeaderE:    
            f.write(line)
            f.write('\n')

def create_IDSettings_xml(settings, outfile):
    lines = [
        '<?xml version="1.0" encoding="UTF-8" ?>',
        '<OpenSimDocument Version="40000">',
            '\t<InverseDynamicsTool name="point_mass">',
                '\t\t<!--Name of the directory where results are written. Be default this is the directory in which the setup file is be  executed.-->',
                '\t\t<results_directory>{}</results_directory>'.format(settings['results_directory']), # C:\Users\ge37del\Documents\HULD
                '\t\t<!--Name of the .osim file used to construct a model.-->',
                '\t\t<model_file>{}</model_file>'.format(settings['model_file']), # point_mass.osim
                '\t\t<!--Time range over which the inverse dynamics problem is solved.-->',
                '\t\t<time_range> {} {}</time_range>'.format(settings['time_range'][0], settings['time_range'][1]), # 0 1
                '\t\t<!--List of forces by individual or grouping name (e.g. All, actuators, muscles, ...) to be excluded when computing model dynamics. "All" also excludes external loads added via "external_loads_file".-->',
                '\t\t<forces_to_exclude> Muscles</forces_to_exclude>',
                '\t\t<!--XML file (.xml) containing the external loads applied to the model as a set of ExternalForce(s).-->',
                '\t\t<external_loads_file>{}</external_loads_file>'.format(settings['external_loads_file']), # point_mass_ExternalForces.xml
                '\t\t<!--The name of the file containing coordinate data. Can be a motion (.mot) or a states (.sto) file.-->',
                '\t\t<coordinates_file>{}</coordinates_file>'.format(settings['coordinates_file']), # point_mass.mot
                '\t\t<!--Low-pass cut-off frequency for filtering the coordinates_file data (currently does not apply to states_file or speeds_file). A negative value results in no filtering. The default value is -1.0, so no filtering.-->',
                '\t\t<lowpass_cutoff_frequency_for_coordinates>{}</lowpass_cutoff_frequency_for_coordinates>'.format(settings['lowpass_cutoff_frequency_for_coordinates']), # 6
                '\t\t<!--Name of the storage file (.sto) to which the generalized forces are written. Only a filename should be specified here (not a full path); the file will appear in the location provided in the results_directory property.-->',
                '\t\t<output_gen_force_file>{}</output_gen_force_file>'.format(settings['output_gen_force_file']), # point_mass_ExternalForces.sto
                '\t\t<!--List of joints (keyword All, for all joints) to report body forces acting at the joint frame expressed in ground.-->',
                '\t\t<joints_to_report_body_forces />',
                '\t\t<!--Name of the storage file (.sto) to which the body forces at specified joints are written.-->',
                '\t\t<output_body_forces_file>body_forces_at_joints.sto</output_body_forces_file>',
            '\t</InverseDynamicsTool>',
        '</OpenSimDocument>'
    ]

    with open(outfile, 'w') as f:
        for line in lines:    
            f.write(line)
            f.write('\n')

def create_ScaleSettings_xml(settings, outfile):
    lines = [
        '<?xml version="1.0" encoding="UTF-8" ?>',
        '<OpenSimDocument Version="40000">',
        '<ScaleTool name="Bimanual-scaled-scaled">',
        '\t<!--Mass of the subject in kg.  Subject-specific model generated by scaling step will have this total mass.-->',
        '\t<mass>{}</mass>'.format(settings['mass']),
        '\t<!--Height of the subject in mm.  For informational purposes only (not used by scaling).-->',
        '\t<height>-1</height>',
        '\t<!--Age of the subject in years.  For informational purposes only (not used by scaling).-->',
        '\t<age>-1</age>',
        '\t<!--Notes for the subject.-->',
        '\t<notes>Unassigned</notes>',
        '\t<!--Specifies the name of the unscaled model (.osim) and the marker set.-->',
        '\t<GenericModelMaker>',
        '\t\t<!--Model file (.osim) for the unscaled model.-->',
        '\t\t<model_file>{}</model_file>'.format(settings['model_in_file']),
        '\t\t<!--Set of model markers used to scale the model. Scaling is done based on distances between model markers compared to the same distances between the corresponding experimental markers.-->',
        '\t\t<marker_set_file>Unassigned</marker_set_file>',
        '\t</GenericModelMaker>',
        '\t<!--Specifies parameters for scaling the model.-->',
        '\t<ModelScaler>',
        '\t\t<!--Whether or not to use the model scaler during scale-->',
        '\t\t<apply>true</apply>',
        "\t\t<!--Specifies the scaling method and order. Valid options are 'measurements', 'manualScale', singly or both in any sequence.-->",
        '\t\t<scaling_order> manualScale</scaling_order>',
        '\t\t<!--Specifies the measurements by which body segments are to be scaled.-->',
        '\t\t<MeasurementSet>',
        '\t\t\t<objects />',
        '\t\t\t<groups />',
        '\t\t</MeasurementSet>',
        '\t\t<!--Scale factors to be used for manual scaling.-->',
        '\t\t<ScaleSet>',
        '\t\t\t<objects>',
    ]

    for bodyName, scalef in settings['body_scales'].items():
        lines.append('\t\t\t\t<Scale>')
        lines.append('\t\t\t\t\t<scales> {} {} {}</scales>'.format(scalef[0], scalef[1], scalef[2]))
        lines.append('\t\t\t\t\t<segment>{}</segment>'.format(bodyName))
        lines.append("\t\t\t\t\t<apply>true</apply>")
        lines.append('\t\t\t\t</Scale>')
	
    lines += [
        '\t\t\t</objects>',
        '\t\t\t<groups />',
        '\t\t</ScaleSet>',
        "\t\t<!--TRC file (.trc) containing the marker positions used for measurement-based scaling. This is usually a static trial, but doesn't need to be.  The marker-pair distances are computed for each time step in the TRC file and averaged across the time range.-->",
        '\t\t<marker_file>{}</marker_file>'.format(settings['trc_file']),
        '\t\t<!--Time range over which to average marker-pair distances in the marker file (.trc) for measurement-based scaling.-->',
        '\t\t<time_range> {} {}</time_range>'.format(settings['timerange'][0], settings['timerange'][1]),
        '\t\t<!--Flag (true or false) indicating whether or not to preserve relative mass between segments.-->',
        '\t\t<preserve_mass_distribution>true</preserve_mass_distribution>',
        '\t\t<!--Name of OpenSim model file (.osim) to write when done scaling.-->',
        '\t\t<output_model_file>{}</output_model_file>'.format(settings['model_out_file']),
        '\t\t<!--Name of file to write containing the scale factors that were applied to the unscaled model (optional).-->',
        '\t\t<output_scale_file>Unassigned</output_scale_file>',
        '\t</ModelScaler>',
        '\t<!--Specifies parameters for placing markers on the model once a model is scaled. -->',
        '\t<MarkerPlacer>',
        '\t\t<!--Whether or not to use the marker placer during scale-->',
        '\t\t<apply>{}</apply>'.format(settings['moveMarkers']), 
        '\t\t<!--Task set used to specify weights used in the IK computation of the static pose.-->',
        '\t\t<IKTaskSet>',
        '\t\t\t<objects>',
    ]

    for markerName, weight in settings['marker_weights'].items():
        lines.append('\t\t\t\t<IKMarkerTask name="{}">'.format(markerName))
        lines.append('\t\t\t\t\t<!--Whether or not this task will be used during inverse kinematics solve, default is true.-->')
        if weight != 0:
            lines.append('\t\t\t\t<apply>true</apply>')
        else:
            lines.append('\t\t\t\t<apply>false</apply>')
        lines.append('\t\t\t\t<!--Weight given to the task when solving inverse kinematics problems, default is 0.-->')
        lines.append('\t\t\t\t<weight>{}</weight>'.format(weight))
        lines.append('\t\t\t</IKMarkerTask>')
	
    lines += [
        '\t\t\t</objects>',
        '\t\t\t<groups />',
        '\t\t</IKTaskSet>',
        '\t\t<!--TRC file (.trc) containing the time history of experimental marker positions (usually a static trial).-->',
        '\t\t<marker_file>{}</marker_file>'.format(settings['trc_file']),
        '\t\t<!--Name of file containing the joint angles used to set the initial configuration of the model for the purpose of placing the markers. These coordinate values can also be included in the optimization problem used to place the markers. Before the model markers are placed, a single frame of an inverse kinematics (IK) problem is solved. The IK problem can be solved simply by matching marker positions, but if the model markers are not in the correct locations, the IK solution will not be very good and neither will marker placement. Alternatively, coordinate values (specified in this file) can be specified and used to influence the IK solution. This is valuable particularly if you have high confidence in the coordinate values. For example, you know for the static trial the subject was standing will all joint angles close to zero. If the coordinate set (see the CoordinateSet property) contains non-zero weights for coordinates, the IK solution will try to match not only the marker positions, but also the coordinates in this file. Least-squared error is used to solve the IK problem. -->',
        '\t\t<coordinate_file>Unassigned</coordinate_file>',
        '\t\t<!--Time range over which the marker positions are averaged.-->',
        '\t\t<time_range> {} {}</time_range>'.format(settings['timerange'][0], settings['timerange'][1]),
        '\t\t<!--Name of the motion file (.mot) written after marker relocation (optional).-->',
        '\t\t<output_motion_file>Unassigned</output_motion_file>',
        '\t\t<!--Output OpenSim model file (.osim) after scaling and maker placement.-->',
        '\t\t<output_model_file>{}</output_model_file>'.format(settings['model_out_file']),
        '\t\t<!--Output marker set containing the new marker locations after markers have been placed.-->',
        '\t\t<output_marker_file>Unassigned</output_marker_file>',
        '\t\t<!--Maximum amount of movement allowed in marker data when averaging frames of the static trial. A negative value means there is not limit.-->',
        '\t\t<max_marker_movement>-1</max_marker_movement>',
        '\t</MarkerPlacer>',
        '</ScaleTool>',
        '</OpenSimDocument>',
    ]

    with open(outfile, 'w') as f:
        for line in lines:    
            f.write(line)
            f.write('\n')

