#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 18 14:36:26 2021

@author: marina
"""

import pandas as pd
import matplotlib.pyplot as plt
import radar_filters as flt
from os import listdir

#from matplotlib import rc
#rc('font',**{'family':'sans-serif','sans-serif':['Helvetica']})
## for Palatino and other serif fonts use:
#rc('font',**{'family':'serif','serif':['Palatino']})
#rc('text', usetex=True)

"""
TO DO [marina]:
    3) Fix xmin / xmax so that the values are dropped from the dataset
        instead of just specified at set_xlabel. This way also the error
        will be calculated within the desired interval
    4) Apply to more appropriate rosbag file with less error
    5) Fix ROS
"""


if __name__ == '__main__':
    
    DEFAULT = 12
    #to_V = 0.33
    
    plt.rc('font', size=DEFAULT)          # controls default text sizes
    plt.rc('axes', titlesize=14)     # fontsize of the axes title
    plt.rc('axes', labelsize=14)    # fontsize of the x and y labels
    plt.rc('xtick', labelsize=DEFAULT)    # fontsize of the tick labels
    plt.rc('ytick', labelsize=DEFAULT)    # fontsize of the tick labels
    plt.rc('legend', fontsize=DEFAULT)    # legend fontsize
    plt.rc('figure', titlesize=DEFAULT)  # fontsize of the figure title
    
    #mypath = "/home/marina/Pi_Git/ros_radar_mine/record_radar_data/"
    #onlyfiles = [f for f in listdir(mypath) if "all" in f and ".bag" in f]
    
    #input("Press Enter to continue...")
    
    #--> Specify file to be analyzed
    path = '/home/marina/Pi_Git/ros_radar_mine/record_radar_data/all_'
    date = '2021-01-15'
    time = '16-24-00'
    csvfile = path + date + '-' + time + '.csv'
    OPTI_offset  = 0.51
    
    #--> Import data as dataframe -> df and fix OptiTrack offset
    df = pd.read_csv(csvfile)
    df["range_op"] -= OPTI_offset
    
    #--> Constants
    MF_size      = 15
    MA_size      = 15
    MA_sub_size  = 9
    EMA_size     = 15           # *** Definitive
    EMA_sub_size = 9            # *** Definitive
    EMA_alpha    = 0.1          # *** Definitive
    GMA_size     = 15
    GMA_sub_size = 9
    GMA_sigma    = 14
    time_offset  = 0
    x_min        = 0 
    x_max        = df["time"].iloc[-1]
    y_min        = 0 
    y_max        = 4   
    
    #--> Apply smoothing filters
    df = flt.MF(MF_size, df, "range_ra", "range_MF")
    df = flt.MA(MA_size, df, "range_MF", "range_MA")
    df = flt.MF_and_MA(MF_size, MA_sub_size, df, "range_ra", "range_MF_and_MA")
    df = flt.MF_and_EMA(MF_size, EMA_sub_size, EMA_alpha, df, "range_ra", "range_MF_and_EMA")
    df = flt.MF_and_GMA(MF_size, GMA_sub_size, GMA_sigma, df, "range_ra", "range_MF_and_GMA")
    df = flt.EMA(EMA_size, EMA_alpha, df, "range_MF", "range_EMA")

    # PLOT filtered range
    ax1 = df.plot(x="time", y=["range_ra"], grid=True, 
                title="Range vs. Time", alpha=0.8, style=['-'])
    df.plot(x="time", y=["range_MF_and_MA", "range_MF_and_EMA", "range_op"], grid=True, 
                title="Range vs. Time", style=['-','-','--'], linewidth=2, ax=ax1)
    ax1.set_xlim(x_min,x_max)
    ax1.set_ylim(y_min,y_max)
    ax1.set_xlabel("Time [s]")
    ax1.set_ylabel("Height [m]")
    ax1.legend(["$h_{radar}$","$h_{MF+MA}$","$h_{MF+EMA}$", "$h_{OptiTrack}$"],loc='center left', bbox_to_anchor=(1, 0.5))
    plt.tight_layout()
    plt.show()

    #--> Only keep df values that satisfy this time condition
    df = df[df["time"] > time_offset]

    #--> Relative error calculation
    df = flt.rel_error(df, "range_MA", "range_op", "err_rel_MA")
    df = flt.rel_error(df, "range_EMA", "range_op", "err_rel_EMA")
    df = flt.rel_error(df, "range_MF_and_MA", "range_op", "err_rel_MF_and_MA")
    df = flt.rel_error(df, "range_MF_and_EMA", "range_op", "err_rel_MF_and_EMA")
    
    # PLOT relative error
    
    # Normal:
    ax2 = df.plot(x="time", y=["err_rel_MA", "err_rel_EMA", "err_rel_MF_and_MA", "err_rel_MF_and_EMA"], grid=True, 
                title="Relative Error (%) vs. Time")
    ax2.set_xlim(x_min,x_max)
    ax2.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    plt.tight_layout()
    plt.show()
    
    # Boxplot:
    fig, ax_rel = plt.subplots()
    cols = [c for c in df.columns if "rel" in c]
    df_rel = df[cols]
    labels = ["MF -> MA", "MF -> EMA", "MF + MA", "MF + EMA"]
    
    boxprops = dict(linestyle='-', linewidth=1.5)
    medianprops = dict(linestyle='-', linewidth=1.5)
    
    ax22 = ax_rel.boxplot(df_rel.T, boxprops=boxprops, medianprops=medianprops, 
                          labels=labels, patch_artist=True, showfliers=False, 
                          notch=True)

    plt.rcParams['boxplot.whiskerprops.linestyle'] = '--'
    #plt.tight_layout()
    ax_rel.yaxis.grid(True, linestyle='-', which='major', color='lightgrey', alpha=0.5)
    ax_rel.xaxis.grid(True, linestyle='-', which='major', color='lightgrey', alpha=0.5)
    ax_rel.set_title("Relative Error")#, fontsize = 13)
    ax_rel.set_xlabel("Smoothing filters")
    ax_rel.set_ylabel("Error [%]")
    colors = ['lightskyblue', 'lightgreen', 'pink', 'navajowhite']
    for patch, color in zip(ax22['boxes'], colors):
        patch.set_facecolor(color)
    
    #--> Absolute error calculation
    df = flt.abs_error(df, "range_MA", "range_op", "err_abs_MA")
    df = flt.abs_error(df, "range_EMA", "range_op", "err_abs_EMA")
    df = flt.abs_error(df, "range_MF_and_MA", "range_op", "err_abs_MF_and_MA")
    df = flt.abs_error(df, "range_MF_and_EMA", "range_op", "err_abs_MF_and_EMA")
    
    # PLOT absolute error
    
    # Normal:
    ax3 = df.plot(x="time", y=["err_abs_MA", "err_abs_EMA", "err_abs_MF_and_MA", "err_abs_MF_and_EMA"], grid=True, 
                title="Absolute Error (%) vs. Time") #style=['r*-','bo-','y^-','g-'], linewidth=2.0)
    ax3.set_xlim(x_min,100)
    ax3.set_ylim(0,0.5)
    ax3.legend(loc='center left', bbox_to_anchor=(1, 0.5))
    plt.tight_layout()
    plt.show()
    
    # Boxplot:
    fig, ax_abs = plt.subplots()
    cols = [c for c in df.columns if "abs" in c]
    df_abs = df[cols]
    labels = ["MF, then MA", "MF, then EMA", "MF and MA", "MF and EMA"]
    
    boxprops = dict(linestyle='-', linewidth=1.5)
    medianprops = dict(linestyle='-', linewidth=1.5)
    
    ax33 = ax_abs.boxplot(df_abs.T, boxprops=boxprops, medianprops=medianprops, 
                          labels=labels, patch_artist=True, showfliers=False, 
                          showmeans=True, notch=True)
    
    #plt.tight_layout()
    ax_abs.yaxis.grid(True, linestyle='-', which='major', color='lightgrey', alpha=0.5)
    ax_abs.xaxis.grid(True, linestyle='-', which='major', color='lightgrey', alpha=0.5)
    ax_abs.set_title("Absolute Error")#, fontsize = 13)
    ax_abs.set_xlabel("Smoothing filters")
    ax_abs.set_ylabel("Error [m]")
    
    colors = ['lightskyblue', 'lightgreen', 'pink', 'navajowhite']
    for patch, color in zip(ax33['boxes'], colors):
        patch.set_facecolor(color)
        
    #--> MSE Calculation for all cases
    MSE_MA = flt.rmse_error(df, "range_MA", "range_op")
    MSE_EMA = flt.rmse_error(df, "range_EMA", "range_op")
    MSE_MF_and_MA = flt.rmse_error(df, "range_MF_and_MA", "range_op")
    MSE_MF_and_EMA = flt.rmse_error(df, "range_MF_and_EMA", "range_op")
    
    
    