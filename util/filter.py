import numpy as np
from scipy.signal import butter,filtfilt # Filter requirements.

def butter_lowpass_filter(data, cutoff, fs, order, axis=-1):
    '''
        computes and returns the low pass filtered input signal
        param: data: np.array
        param: cutoff: double cut off frequency
        param: fs: double sample frequency
        param: order: int filter order
        param: axis: int axis in the nd.array at which the filter should be applied
    '''
    #fs = sample frequency
    nyq = fs * 0.5
    normal_cutoff = cutoff / nyq
    # Get the filter coefficients 
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data, axis)
    return y