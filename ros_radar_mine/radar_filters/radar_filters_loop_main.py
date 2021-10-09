#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jan 19 10:28:08 2021

@author: marina
"""

import pandas as pd
import matplotlib.pyplot as plt
import radar_filters as flt

if __name__ == '__main__':
    
    # Specify file to be analyzed
    path = '/home/marina/Pi_Git/ros_radar_mine/record_radar_data/all_'
    date = '2021-01-15'
    time = '16-28-38'
    csvfile = path + date + '-' + time + '.csv'
    
    # Import data as dataframe -> df
    df = pd.read_csv(csvfile)
    
    # Constants
    MF_size      = 15
    MA_size      = 15
    MA_sub_size  = 15
    EMA_size     = 15
    EMA_sub_size = 9
    EMA_alpha    = 0.1
    OPTI_offset  = 0.51
    time_offset  = 2
    x_min        = 0 
    x_max        = df["time"].iloc[-1]
    y_min        = 0 
    y_max        = 4   
    
    df["range_op"] -= OPTI_offset
    
    #df_MF_and_MA = df.copy(deep=True)
    
    # What to plot
    plot_MF_and_MA_bigwin  = False
    plot_MF_and_MA_subwin  = True
    
    plot_MF_and_EMA_bigwin = False
    plot_MF_and_EMA_subwin = False
    plot_MF_and_EMA_alpha  = False
    
    # --------------------- Apply filters MF_and_MA ----------------------- #
    # Check effect of MA_sub window size
    
    if plot_MF_and_MA_subwin:
    
        MF_size = 15
        MA_sub_size_list = [3, 5, 7, 9, 11, 13]
        
        for i in MA_sub_size_list:
        
            df_MF_and_MA = flt.MF_and_MA(MF_size, i, df, "range_ra", "range_MA_" + str(i))
            df_MF_and_MA  = flt.rel_error(df_MF_and_MA, "range_MA_" + str(i) , "range_op", "err_rel_MA_" + str(i))
            df_MF_and_MA  = flt.abs_error(df_MF_and_MA, "range_MA_" + str(i) , "range_op", "err_abs_MA_" + str(i))
    
        #Only keep df values that satisfy this time condition
        df_MF_and_MA = df_MF_and_MA[df_MF_and_MA["time"] > time_offset]
        df_MF_and_MA.set_index('time', inplace=True)
        df_MF_and_MA.drop(columns=['num_targets', 'speed', 'angle', 'strength', 'range_ra'], inplace=True)
        
        cols = [c for c in df_MF_and_MA.columns if "range_MA" in c or "range_op" in c]
        df_MF_and_MA_range = df_MF_and_MA[cols]
        
        cols = [c for c in df_MF_and_MA.columns if "rel_MA" in c]
        df_MF_and_MA_rel = df_MF_and_MA[cols]
        
        cols = [c for c in df_MF_and_MA.columns if "abs_MA" in c]
        df_MF_and_MA_abs = df_MF_and_MA[cols]

    
        ax1 = df_MF_and_MA_range.plot(grid=True, title="Range vs. Time [MA: MA\_sub\_size]")
        ax1.set_xlim(x_min,x_max)
        ax1.set_ylim(y_min,y_max)
        ax1.set_xlabel("Time [s]")
        ax1.set_ylabel("Height [m]")
        ax1.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.show()
        
        ax1 = df_MF_and_MA_rel.plot(grid=True, title="Relative Error vs. Time [MA: MA\_sub\_size]")
        ax1.set_xlim(x_min,x_max)
        ax1.set_xlabel("Time [s]")
        ax1.set_ylabel("Error [m]")
        ax1.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.show()
        
        ax1 = df_MF_and_MA_abs.plot(grid=True, title="Absolute Error vs. Time [MA: MA\_sub\_size]")
        ax1.set_xlim(x_min,x_max)
        ax1.set_xlabel("Time [s]")
        ax1.set_ylabel("Error [m]")
        ax1.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.show()
    
    # --------------------- Apply filters MF_and_MA ----------------------- #
    # Check effect of MF window size
    
    if plot_MF_and_MA_bigwin:
    
        MF_size_list = [11, 13, 15, 17, 19, 21, 23, 25, 27]
        MA_sub_size = 9
        
        for i in MF_size_list:
        
            df_MF_and_MA = flt.MF_and_MA(i, MA_sub_size, df, "range_ra", "range_MF_" + str(i))
            df_MF_and_MA  = flt.rel_error(df_MF_and_MA, "range_MF_" + str(i) , "range_op", "err_rel_MF_" + str(i))
            df_MF_and_MA  = flt.abs_error(df_MF_and_MA, "range_MF_" + str(i) , "range_op", "err_abs_MF_" + str(i))
    
        #Only keep df values that satisfy this time condition
        df_MF_and_MA = df_MF_and_MA[df_MF_and_MA["time"] > time_offset]
        df_MF_and_MA.set_index('time', inplace=True)
        df_MF_and_MA.drop(columns=['num_targets', 'speed', 'angle', 'strength', 'range_ra'], inplace=True)
        
        cols = [c for c in df_MF_and_MA.columns if "range_MF" in c or "range_op" in c]
        df_MF_and_MA_range = df_MF_and_MA[cols]
        
        cols = [c for c in df_MF_and_MA.columns if "rel_MF" in c]
        df_MF_and_MA_rel = df_MF_and_MA[cols]
        
        cols = [c for c in df_MF_and_MA.columns if "abs_MF" in c]
        df_MF_and_MA_abs = df_MF_and_MA[cols]

    
        ax2 = df_MF_and_MA_range.plot(grid=True, title="Range vs. Time [MA: MF\_size]")
        ax2.set_xlim(x_min,x_max)
        ax2.set_ylim(y_min,y_max)
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Height [m]")
        ax2.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.show()
    
        ax2 = df_MF_and_MA_rel.plot(grid=True, title="Relative Error vs. Time [MA: MF\_size]")
        ax2.set_xlim(x_min,x_max)
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Error [m]")
        ax2.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.show()
        
        ax2 = df_MF_and_MA_abs.plot(grid=True, title="Absolute Error vs. Time [MA: MF\_size]")
        ax2.set_xlim(x_min,x_max)
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Error [m]")
        ax2.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.show()
    
    # --------------------- Apply filters MF_and_EMA ----------------------- #
    # Check effect of MF window size
    
    if plot_MF_and_EMA_bigwin:
    
        MF_size_list = [11, 13, 15, 17, 19, 21, 23, 25, 27]
        EMA_sub_size = 9
        EMA_alpha    = 0.1
        
        for i in MF_size_list:
        
            df_MF_and_EMA = flt.MF_and_EMA(i, EMA_sub_size, EMA_alpha, df, "range_ra", "range_MF_EMA_" + str(i))
            df_MF_and_EMA  = flt.rel_error(df_MF_and_EMA, "range_MF_EMA_" + str(i) , "range_op", "err_rel_MF_EMA_" + str(i))
            df_MF_and_EMA  = flt.abs_error(df_MF_and_EMA, "range_MF_EMA_" + str(i) , "range_op", "err_abs_MF_EMA_" + str(i))
    
        #Only keep df values that satisfy this time condition
        df_MF_and_EMA = df_MF_and_EMA[df_MF_and_EMA["time"] > time_offset]
        df_MF_and_EMA.set_index('time', inplace=True)
        df_MF_and_EMA.drop(columns=['num_targets', 'speed', 'angle', 'strength', 'range_ra'], inplace=True)
        
        cols = [c for c in df_MF_and_EMA.columns if "range_MF_EMA" in c or "range_op" in c]
        df_MF_and_EMA_range = df_MF_and_EMA[cols]
        
        cols = [c for c in df_MF_and_EMA.columns if "rel_MF_EMA" in c]
        df_MF_and_EMA_rel = df_MF_and_EMA[cols]
        
        cols = [c for c in df_MF_and_EMA.columns if "abs_MF_EMA" in c]
        df_MF_and_EMA_abs = df_MF_and_EMA[cols]
            
        ax2 = df_MF_and_EMA_range.plot(grid=True, title="Range vs. Time [EMA: MF\_size]")
        ax2.set_xlim(x_min,x_max)
        ax2.set_ylim(y_min,y_max)
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Height [m]")
        ax2.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.show()
        
        ax2 = df_MF_and_EMA_rel.plot(grid=True, title="Relative Error vs. Time [EMA: MF\_size]")
        ax2.set_xlim(x_min,x_max)
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Error [m]")
        ax2.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.show()
        
        ax2 = df_MF_and_EMA_abs.plot(grid=True, title="Absolute Error vs. Time [EMA: MF\_size]")
        ax2.set_xlim(x_min,x_max)
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Error [m]")
        ax2.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.show()
    
    # --------------------- Apply filters MF_and_EMA ----------------------- #
    # Check effect of EMA_sub window size
    
    if plot_MF_and_EMA_subwin:
    
        MF_size = 15
        EMA_sub_size_list = [3, 5, 7, 9, 11, 13]
        EMA_alpha = 0.1
        
        for i in EMA_sub_size_list:
        
            df_MF_and_EMA = flt.MF_and_EMA(MF_size, i, EMA_alpha, df, "range_ra", "range_EMA_" + str(i))
            df_MF_and_EMA  = flt.rel_error(df_MF_and_EMA, "range_EMA_" + str(i) , "range_op", "err_rel_EMA_" + str(i))
            df_MF_and_EMA  = flt.abs_error(df_MF_and_EMA, "range_EMA_" + str(i) , "range_op", "err_abs_EMA_" + str(i))
    
        #Only keep df values that satisfy this time condition
        df_MF_and_EMA = df_MF_and_EMA[df_MF_and_EMA["time"] > time_offset]
        df_MF_and_EMA.set_index('time', inplace=True)
        df_MF_and_EMA.drop(columns=['num_targets', 'speed', 'angle', 'strength', 'range_ra'], inplace=True)
        
        cols = [c for c in df_MF_and_EMA.columns if "range_EMA" in c or "range_op" in c]
        df_MF_and_EMA_range = df_MF_and_EMA[cols]
        
        cols = [c for c in df_MF_and_EMA.columns if "rel_EMA" in c]
        df_MF_and_EMA_rel = df_MF_and_EMA[cols]
        
        cols = [c for c in df_MF_and_EMA.columns if "abs_EMA" in c]
        df_MF_and_EMA_abs = df_MF_and_EMA[cols]
    
        ax2 = df_MF_and_EMA_range.plot(grid=True, title="Range vs. Time [EMA: EMA\_sub\_size]")
        ax2.set_xlim(x_min,x_max)
        ax2.set_ylim(y_min,y_max)
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Height [m]")
        ax2.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.show()
        
        ax2 = df_MF_and_EMA_rel.plot(grid=True, title="Relative Error vs. Time [EMA: EMA\_sub\_size]")
        ax2.set_xlim(x_min,x_max)
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Error [m]")
        ax2.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.show()
        
        ax2 = df_MF_and_EMA_abs.plot(grid=True, title="Absolute Error vs. Time [EMA: EMA\_sub\_size]")
        ax2.set_xlim(x_min,x_max)
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Error [m]")
        ax2.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.show()
        
        # --------------------- Apply filters MF_and_EMA ----------------------- #
    # Check effect of EMA_sub window size
    
    if plot_MF_and_EMA_alpha:
    
        MF_size = 15
        EMA_sub_size = 9
        EMA_alpha_list = [0.1, 0.3, 0.5, 0.7, 0.9]
        
        for i in EMA_alpha_list:
        
            df_MF_and_EMA = flt.MF_and_EMA(MF_size, EMA_sub_size, i, df, "range_ra", "range_EMA_alpha_" + str(i))
            df_MF_and_EMA  = flt.rel_error(df_MF_and_EMA, "range_EMA_alpha_" + str(i) , "range_op", "err_rel_EMA_alpha_" + str(i))
            df_MF_and_EMA  = flt.abs_error(df_MF_and_EMA, "range_EMA_alpha_" + str(i) , "range_op", "err_abs_EMA_alpha_" + str(i))
    
        #Only keep df values that satisfy this time condition
        df_MF_and_EMA = df_MF_and_EMA[df_MF_and_EMA["time"] > time_offset]
        df_MF_and_EMA.set_index('time', inplace=True)
        df_MF_and_EMA.drop(columns=['num_targets', 'speed', 'angle', 'strength', 'range_ra'], inplace=True)
        
        cols = [c for c in df_MF_and_EMA.columns if "range_EMA_alpha" in c or "range_op" in c]
        df_MF_and_EMA_range = df_MF_and_EMA[cols]
        
        cols = [c for c in df_MF_and_EMA.columns if "rel_EMA_alpha" in c]
        df_MF_and_EMA_rel = df_MF_and_EMA[cols]
        
        cols = [c for c in df_MF_and_EMA.columns if "abs_EMA_alpha" in c]
        df_MF_and_EMA_abs = df_MF_and_EMA[cols]
    
        ax2 = df_MF_and_EMA_range.plot(grid=True, title="Range vs. Time [EMA: EMA\_alpha]")
        ax2.set_xlim(x_min,x_max)
        ax2.set_ylim(y_min,y_max)
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Height [m]")
        ax2.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.show()
        
        ax2 = df_MF_and_EMA_rel.plot(grid=True, title="Relative Error vs. Time [EMA: EMA\_alpha]")
        ax2.set_xlim(x_min,x_max)
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Error [m]")
        ax2.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.show()
        
        ax2 = df_MF_and_EMA_abs.plot(grid=True, title="Absolute Error vs. Time [EMA: EMA\_alpha]")
        ax2.set_xlim(x_min,x_max)
        ax2.set_xlabel("Time [s]")
        ax2.set_ylabel("Error [m]")
        ax2.legend(loc='center left', bbox_to_anchor=(1, 0.5))
        plt.show()
    