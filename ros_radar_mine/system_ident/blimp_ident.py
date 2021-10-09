#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb  2 17:22:03 2021

@author: marina

This file contains all the necessary functions to compute the blimp range after
a certain initial condition is given as well as some inputs for the control of
the blimp. It works with dataframes of recorded data from OptiTrack into .bag
files, that have been postprocessed with radar_filters/bag_processing.py or
equivalent

In order to work with synthetic data generated within a simulated environment,
please refer to neuro_learning/my_nets

"""

import pandas as pd
import numpy  as np
from dataclasses import dataclass
import matplotlib.pyplot as plt
from scipy import integrate

plt.close("all")

# Include Discrete Transfer Functions

@dataclass
class TF_vel:
    # TF for model_vel
    num: np.ndarray = np.array([0, 0.001356512594043, -0.001355960964971])
    den: np.ndarray = np.array([1.000000000000000, -1.992777787021512, 0.992794512538195])
    
@dataclass
class TF_ran:
    # TF for model_ran
    num: np.ndarray = np.array([0, -0.000969428630164, 0.001018772629138])
    den: np.ndarray = np.array([1.000000000000000, -1.990293321618669, 0.990311599031747])
    
# Moving Average Filter function definition
def MA(window_size, dataframe, str_in, str_out):
    """
    Parameters
    ----------
    window_size : number of points to which apply the moving average
    dataframe : DataFrame containing the relevant noisy data
    str_in : column name to which the filter is applied
    str_out : output column name that will be added to DataFrame

    Returns
    -------
    dataframe : new DataFrame containing the averaged data
    """
    
    array_store = [] # To store the 'window-values'
    dataframe_rows = dataframe.shape[0] # Total no. rows in DataFrame
    dataframe_store = np.zeros(shape=(dataframe_rows,1)) # New averaged column that is added to DataFrame
        
    for i in range(dataframe_rows):
        
        # Fill in array_store up until window_size (not average anything yet)
        if i < window_size:
            array_store.append(dataframe[str_in][i])
            dataframe_store[i] = sum(array_store)/len(array_store)
        # Make sure that array_store always has window_size + MA calculation
        elif i > (dataframe_rows - window_size):
            array_store.pop(0)
            dataframe_store[i-int(window_size/2)] = sum(array_store)/len(array_store)
            
        elif i >= window_size and i <= (dataframe_rows - window_size):
            array_store.append(dataframe[str_in][i])
            array_store.pop(0)
            
            # Linear average calculation
            dataframe_store[i-int(window_size/2)] = sum(array_store)/len(array_store)
        
        dataframe[str_out] = dataframe_store
    
    dataframe[str_out].replace(0,np.nan,inplace=True) # Throw 0 values out of DataFrame
    dataframe.dropna(axis=0,inplace=True)
    dataframe.reset_index(drop=True, inplace=True)    # Reset indexes
    dataframe["time"] -= dataframe["time"][0]         # Reset time
    return dataframe

# Loading Data Function Definition
def load_data(path, date, time, sample_freq, opti_offset, drop_points, smooth_points):
    """
    Parameters
    ----------
    path :          path where the input .csv data is located
    date :          input data date
    time :          input data time
    sample_freq :   sampling frequency
    opti_offset :   OptiTrack offset
    drop_points :   points to drop at beginning and end of DataFrame
    smooth_points : window_size for the MA smoothing

    Returns
    -------
    df :            DataFrame with all data: t (time), u (input), r (range), 
                                             v (velocity), a (acceleration)
    """
    
    csvfile = path + date + '-' + time + '.csv'
    
    #--> Import data as dataframe -> df + fix OptiTrack offset + reset time
    df = pd.read_csv(csvfile)
    df["range_op"] -= opti_offset
    
    #--> Apply range moving average
    df = MA(smooth_points,df,"range_op","r")
    
    #--> Compute velocity (range derivative)
    aux = np.diff(df["r"])/SAMPLE_FREQ
    aux = np.append(aux,np.nan)
    df["v"] = aux                               # Smooth speed
    aux = np.diff(df["range_op"])/SAMPLE_FREQ
    aux = np.append(aux,np.nan)
    df["v2"] = aux                              # Non-smooth speed
    
    #--> Compute acceleration (range double derivative)
    aux = np.diff(df["v"])/SAMPLE_FREQ
    aux = np.append(aux,np.nan)
    df["a"] = aux                               # Smooth acceleration
    aux = np.diff(df["v2"])/SAMPLE_FREQ
    aux = np.append(aux,np.nan)
    df["a2"] = aux                              # Non-smooth acceleration
    df = MA(10,df,"a","a_s")                    # Smooth x2 acceleration
    
    #--> Drop NaN values
    df.dropna(inplace=True)
    
    #--> Drop some points at beginning and end (drop_points)
    df.drop(np.arange(drop_points), inplace = True)
    df.drop(df.shape[0] - np.arange(drop_points), inplace = True)
    df.reset_index(drop=True, inplace=True)
    df["time"] -= df["time"][0] # Reset time
    
    return df
    
# Compute the modeled velocity + range from VELOCITY MODEL
def model_vel(df, tf):
    """
    Parameters
    ----------
    df : DataFrame containing the relevant processed data
    tf : Transfer Function (TF) corresponding to the model

    Returns
    -------
    df : new DataFrame containing the V-modeled data (v_modelV, r_modelV, r_errorV)
    """
    
    #--> Initialize + Apply INITIAL CONDITIONS
    new = np.zeros(df["v"].shape[0])
    new[0:max(len(tf.den)-1,len(tf.num))] = df["v"][0:max(len(tf.den)-1,len(tf.num))]
    
    #--> Apply model to obtain modeled velocity
    for i in range(max(len(tf.den)-1,len(tf.num)),df["v"].shape[0]):
        new[i] = np.dot(-np.flip(tf.den),new[i-len(tf.den)+1:i+1]) + np.dot(np.flip(tf.num),df["dcmotor"][i-len(tf.num)+1:i+1].to_numpy())
    
    #--> Integrate to get modeled range + Calculate error
    # Add initial condition here (df["r"][0])
    r_int = df["r"][0] + integrate.cumtrapz(new, df["time"].to_numpy(), initial=0)
    r_error = df["r"].to_numpy() - r_int
    
    #--> Add everything to DataFrame
    df["v_modelV"] = new
    df["r_modelV"] = r_int
    df["r_errorV"] = r_error
    return df

# Compute the modeled range from RANGE MODEL
def model_ran(df, tf):
    """
    Parameters
    ----------
    df : DataFrame containing the relevant processed data
    tf : Transfer Function (TF) corresponding to the model

    Returns
    -------
    df : new DataFrame containing the R-modeled data (r_modelR, r_errorR)
    """
    
    IC_CONDITION = 1.5               # Constant
    new = np.zeros(df["r"].shape[0]) # Initialize
    
    #--> Apply model to obtain modeled range
    for i in range(max(len(tf.den)-1,len(tf.num)),df["r"].shape[0]):
        new[i] = np.dot(-np.flip(tf.den),new[i-len(tf.den)+1:i+1]) + np.dot(np.flip(tf.num),df["dcmotor"][i-len(tf.num)+1:i+1].to_numpy())
    
    #--> Apply initial condition + Calculate error
    new = new + IC_CONDITION
    
    #--> Add everything to DataFrame
    r_error = df["r"].to_numpy() - new
    df["r_modelR"] = new
    df["r_errorR"] = r_error
    return df
    
#########################################################
# Main
#########################################################
if __name__ == '__main__':
    
    path = '/home/marina/Pi_Git/ros_radar_mine/record_radar_data/all_'
    date = '2021-02-01'
    
    #--> Array of all times to be considered
    times = ['13-44-01', '11-24-25', '11-45-40', '11-40-50', '13-52-42', '14-02-37']
    
    #--> Constants
    SAMPLE_FREQ   = 1/30
    POINTS        = 99
    OPTI_offset   = 0.51
    SMOOTH_POINTS = 120
    
    #--> List that will hold all DataFrames
    df_list = []
    
    #--> Errors DataFrame
    df_error = pd.DataFrame()
    r_errorR_list = []
    r_errorV_list = []
    
    #--> Loop through all considered files
    for time in times:
        
        #--> Load data
        df = load_data(path, date, time, SAMPLE_FREQ, OPTI_offset, POINTS, SMOOTH_POINTS)
        
        #--> Compute models
        model_vel(df,TF_vel)
        model_ran(df,TF_ran)
        
        #--> Save errors
        r_errorR_list.append(np.sqrt(np.mean(df["r_errorR"].to_numpy()**2)))
        r_errorV_list.append(np.sqrt(np.mean(df["r_errorV"].to_numpy()**2)))
        df_list.append(df) # TO DO [marina]: MOVE THIS TO BEFORE PLOTS

df_error["r_errorR"] = r_errorR_list
df_error["r_errorV"] = r_errorV_list

#--> Plots
number = 5
"""
ax = df_list[number].plot(x="time", y=["range_op","r","r_modelV","r_modelR"], 
                          style=['-', '-', '-.', '--'], linewidth=1.5, grid=True,
                          xlabel="Time [sec]", ylabel="Range [m]",
                          title="Range Model. TF with 2p and 1z")
ax.legend(["Raw range", "Smooth range", "Modeled range (V)","Modeled range (R)"])
"""

ax = df_list[number].plot(x="time", y=["range_op","r_modelV","r_modelR"], 
                          style=['-', '-.', '--'], linewidth=1.5, grid=True,
                          xlabel="Time [sec]", ylabel="Range [m]",
                          title="Range Model. TF with 2p and 1z")
ax.legend(["Raw range", "Modeled range (V)","Modeled range (R)"])

ax = df_list[number].plot(x="time", y=["v","v_modelV"], style=['-', '--'], grid=True,
                     xlabel="Time [sec]", ylabel="Velocity [m/s]",
                     title="Velocity Model. TF with 2p and 1z")
ax.legend(["True speed", "Modeled speed"])

# Error plots
ax = df_error.plot(style=['o-','o-'], xlabel="Dataset Number",
                   ylabel="Range RMSE", title="Range RMSE for velocity + range model", 
                   grid =True)
ax.legend(["Range Model", "Velocity Model"])

"""
TRIANGULAR MOVING AVERAGE

def smoothTriangle(data, degree, dropVals=False):
    triangle=np.array(list(range(degree)) + [degree] + list(range(degree)[::-1])) + 1
    smoothed=[]

    for i in range(degree, len(data) - degree * 2):
        point=data[i:i + len(triangle)] * triangle
        smoothed.append(sum(point)/sum(triangle))
    if dropVals:
        return smoothed
    smoothed=[smoothed[0]]*int(degree + degree/2) + smoothed
    while len(smoothed) < len(data):
        smoothed.append(smoothed[-1])
    return smoothed
"""    
    
    