import numpy as np

def axisAngle_to_R(axis, angle):
    '''
        converts an axis angle notation to a rotation matrix
        param: axis: nd.array rotaton axis as a unit vector
        param: ange: nd.array rotation angle around axis in rad

        return: nd.array the rotation matrix
    '''
    x = axis[0]
    y = axis[1]
    z = axis[2]
    
    c = np.cos(angle) # needs to be in rad
    s = np.sin(angle) # needs to be in rad
    t = 1 - c

    #print(x, y, z, c, s, t)

    R = np.array([[c+x**2*t, x*y*t-z*s, x*z*t+y*s], 
                [y*x*t+z*s, c+y**2*t, y*z*t-x*s], 
                [z*x*t-y*s, z*y*t+x*s, c+z**2*t]])
    
    return R

def rotAtoB(a, b):
    ''' 
        returns a rotation matrix R_AB such that R_AB @ b = a 
        param: b: nd.array 
        param: a: nd.array
        return nd.array
        
    '''
    #a = object_markers_cut[start, markers_for_target_vector[1], :] - object_markers_cut[start, markers_for_target_vector[0], :]
    a = a / np.linalg.norm(a)# normalize k
    #b = np.array([0, 1, 0]) # target direction
    #b = target_start_vector
    b = b / np.linalg.norm(b)

    cos_t = a @ b # cos angle between a and b
    sin_t = -np.linalg.norm(np.cross(a, b)) # sin angle between a and b
    G = np.array([[cos_t, -sin_t, 0], 
                [sin_t, cos_t, 0], 
                [0, 0, 1]])

    u = a
    v = (b - (a @ b) * a) / (np.linalg.norm(b - (a @ b) * a))
    w = np.cross(b, a)

    F_inv = np.array([u, v, w]).T
    F = np.linalg.inv(F_inv)

    R_ab = F_inv @ G @ F # from b to a
    #R_ba = R_ab.T # form a to b -> this is the one we need

    return R_ab

