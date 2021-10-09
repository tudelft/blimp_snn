#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Feb 23 08:59:15 2021

@author: marina
"""

import numpy  as np
import pandas as pd
import random
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
    
    
# Compute the modeled range from the RANGE MODEL
def mymodel_ran(u, tf, r_stored):
    
    try:
        
        aux = np.append(r_stored, 0)
        
        #--> Apply model to obtain modeled range
        r_new = np.dot(-np.flip(tf.den), aux) + np.dot(np.flip(tf.num), u)
        
        r_stored = np.delete(r_stored, 0)
        r_stored = np.append(r_stored, r_new)
        
        """
        [NOTE]: Do not forget to apply initial condition outside of this function!!!
        """
    
    except RuntimeError:
        print("Oops! There is a problem with the sizes here")
    
    return r_new, r_stored

# Loading Data Function Definition
def load_data(path, date, time, opti_offset, drop_points):
        
    csvfile = path + date + '-' + time + '.csv'
    
    #--> Import data as dataframe -> df + fix OptiTrack offset + reset time
    df = pd.read_csv(csvfile)
    df["range_op"] -= opti_offset
    
    #--> Drop NaN values
    df.dropna(inplace=True)
    
    #--> Drop some points at beginning and end (drop_points)
    df.drop(np.arange(drop_points), inplace = True)
    df.drop(df.shape[0] - np.arange(drop_points), inplace = True)
    df.reset_index(drop=True, inplace=True)
    df["time"] -= df["time"][0] # Reset time
    
    return df

def gen_inputs(n, u_min, u_max, ndup):
    """
    Generate synthetic inputs: each generated the SAME no. times
    """
    u_array = []
    
    for i in range(0,n):
        aux = random.randint(u_min,u_max)       # Generate random input    
        u_array.append(aux)                     # Append to final inputs array
    
    u_array = np.repeat(u_array,ndup)           # Duplicate all inputs ndup times
    
    return u_array

def gen_inputs_dup(n, u_min, u_max, nd_min, nd_max):
    """
    Generate synthetic inputs: each generated a random DIFFERENT no. of times
    """
    u_array = []
    
    for i in range(0,n):
        aux = random.randint(u_min,u_max)       # Generate random input
        ndup = random.randint(nd_min,nd_max)    # Select random duplication number
        aux_array = np.repeat(aux,ndup)         # Duplicate aux, ndup times
        u_array.extend(aux_array)               # Append to final inputs array
    
    return u_array


#########################################################
# Main
#########################################################
if __name__ == '__main__':
    
    DEFAULT = 12

    plt.rc('font', size=DEFAULT)          # controls default text sizes
    plt.rc('axes', titlesize=DEFAULT)     # fontsize of the axes title
    plt.rc('axes', labelsize=14)    # fontsize of the x and y labels
    plt.rc('xtick', labelsize=DEFAULT)    # fontsize of the tick labels
    plt.rc('ytick', labelsize=DEFAULT)    # fontsize of the tick labels
    plt.rc('legend', fontsize=DEFAULT)    # legend fontsize
    plt.rc('figure', titlesize=DEFAULT)  # fontsize of the figure title
    
    # Constants
    NUMBER = 4
    INIT_ARRAY = [1.6, 1.8, 1.5, 1.5, 1.7, 2]
    N = 50
    U_MIN = -3
    U_MAX = 3
    N_DUP = 40
    ND_MIN = 50
    ND_MAX = 100
    IC_CONDITION = INIT_ARRAY[NUMBER]
    POINTS = 0
    OPTI_OFFSET = 0.51
    DROP_POINTS = 99
    XMIN = 6
    XMAX = 300
    
    path = '/home/marina/Pi_Git/ros_radar_mine/record_radar_data/all_'
    date = '2021-02-01'
    times = ['13-44-01', '11-24-25', '11-45-40', '11-40-50', '13-52-42', '14-02-37']
    time = times[NUMBER] #4
    
    # Set a random seed
    np.random.seed(seed=0)
    
    # Load or generate synthetic input data
    #u_array = gen_inputs(N, U_MIN, U_MAX, N_DUP)
    #u_array = gen_inputs_dup(N, U_MIN, U_MAX, ND_MIN, ND_MAX)
    #u_array = np.ones(5000)*1
    df = load_data(path, date, time, OPTI_OFFSET, DROP_POINTS)
    u_array = df["dcmotor"].to_numpy()
    
    # Initialize some arrays
    r_array = []
    r_stored = np.zeros(len(TF_ran.den)-1)
    u_in = np.zeros(len(TF_ran.num))
    """
    TO DO [marina]: Check if the way u_in is built is correct ¡¡¡!!!
    """
    
    # Loop over the inputs
    for u in u_array:
        
        # Get the proper u_in array
        u_in = np.delete(u_in, 0)
        u_in = np.append(u_in, u)
        
        # Apply model and store range
        r_new, r_stored = mymodel_ran(u_in, TF_ran, r_stored)
        r_array.append(r_new)
    
    # Add initial condition
    r_array = np.array(r_array) + IC_CONDITION
    
    # Plots
    f, (a0, a1) = plt.subplots(2, 1, gridspec_kw={'height_ratios': [2, 1]})
    f.set_size_inches(6,3.5)
    
    #plt.plot(r_array, '-')
    #plt.figure()
    #plt.plot(u_array, '.-')
    
    #fig.suptitle('Range [m] & Motor command')
    a0.plot(df["time"],df["range_op"], '-')
    a0.plot(df["time"],r_array, '--', color='black')
    a0.fill_between(df["time"],r_array, df["range_op"],color='C0',alpha=0.3)
    a0.set_xlim([XMIN,XMAX])#df["time"].iloc[-1]])
    a0.set_ylabel('$h$ $[m]$')
    a0.tick_params(labelbottom=False)
    a0.legend(["$h_{real}$","$h_{model}$"],loc='upper center', bbox_to_anchor=(0.5, 1.4),ncol=2, fancybox=True, shadow=True)
    
    #a0.legend(["$h_{real}$","$h_{model}$"],loc='center left', bbox_to_anchor=(1, 0.5))
    
    #ax.legend(loc='upper center', bbox_to_anchor=(0.5, 1.05),
    #      ncol=3, fancybox=True, shadow=True)
    
    a0.set_xticks(np.linspace(XMIN,XMAX,7).astype(int), minor=False)
    a1.set_xticks(np.linspace(XMIN,XMAX,7).astype(int), minor=False)
    a1.set_xticklabels(np.linspace(0,XMAX,7).astype(int))
    
    a1.plot(df["time"],u_array*0.33, '-')#, color = 'black')
    a1.set_xlim([XMIN,XMAX])#df["time"].iloc[-1]])
    a1.set_ylabel('$u$ $[V]$')
    a1.set_ylim([-1.5,1.5])
    a1.set_xlabel("Time [s]")
    #a1.set_yticks([-3,0,3])
    a0.grid()
    a1.grid()
    plt.tight_layout()
    plt.subplots_adjust(hspace=0)
    plt.show()

    df["r_model"] = r_array

    plt.savefig("model_legend.pdf", dpi=200, bbox_inches = 'tight', pad_inches = 0)

#ax1.set_xlabel("Time [s]")
#ax1.set_ylabel("$h \,\, [m]$")
#ax1.legend(["$h_{ref}$","$h_{simulation}$", "$h_{real}$"],loc='center left', bbox_to_anchor=(1, 0.5))
#ax2.set_xlim(0,300)
#ax2.set_ylim(-3, 6)
#ax2.tick_params(labelbottom=False)















