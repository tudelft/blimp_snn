#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jan 18 10:55:58 2021

# TO DO [marina]:
    
    [IMPORTANT]: FIX RADAR PACKAGE -> NOT NECESSARY TO COMPUTE FILTERS IN
        DIFFERENT FILE
    
@author: marina
"""

import numpy as np
import copy

# Median Filter function definition
def MF(window_size, dataframe, str_in, str_out):
    
    # list of ranges with window_size size
    range_store = [] 
    # total number of range measurements
    dataframe_rows = dataframe.shape[0] 
    # array that will hold the filtered range values and be added to dataframe
    range_dataframe = np.zeros(shape=(dataframe_rows,1)) 
    
    for i in range(dataframe_rows):
        
        if len(range_store) < window_size:
            range_store.append(dataframe[str_in][i])
            range_dataframe[i] = 0
        else:
            range_store.append(dataframe[str_in][i])
            range_store.pop(0)
            
            aux_range = range_store * 1
            aux_range.sort()
            range_dataframe[i] = aux_range[int(window_size/2)]
        
        dataframe[str_out] = range_dataframe
        
    return dataframe

# Moving Average Filter function definition
def MA(window_size, dataframe, str_in, str_out):
        
    range_store = []
    dataframe_rows = dataframe.shape[0]
    range_dataframe = np.zeros(shape=(dataframe_rows,1))
        
    for i in range(dataframe_rows):
        
        if len(range_store) < window_size:
            range_store.append(dataframe[str_in][i])
            range_dataframe[i] = 0
        else:
            range_store.append(dataframe[str_in][i])
            range_store.pop(0)
            
            # Linear average calculation
            range_dataframe[i] = sum(range_store)/len(range_store)
        
        dataframe[str_out] = range_dataframe
        
    return dataframe

# Exponential Moving Average Filter function definition
def EMA(window_size, alpha, dataframe, str_in, str_out):
        
    range_store = []
    dataframe_rows = dataframe.shape[0]
    range_dataframe = np.zeros(shape=(dataframe_rows,1))
        
    for i in range(dataframe_rows):
        
        if len(range_store) < window_size:
            range_store.append(dataframe[str_in][i])
            range_dataframe[i] = 0
        else:
            range_store.append(dataframe[str_in][i])
            range_store.pop(0)
            
            # Exponential average calculation
            s = range_store[0]
            
            for j in range(window_size):
                s = alpha*range_store[j]+(1-alpha)*s
            
            range_dataframe[i] = s
        
        dataframe[str_out] = range_dataframe
        
    return dataframe

# Get central smaller window indexes within bigger window indexes
def get_indexlist(window_big, window_small):
    
    if window_big >= window_small:
        start_index = int((window_big - window_small)/2)
        index_list = [start_index + i for i in range(window_small+1)]
        
    else:
        print("[ERROR]: in get_indexlist() -> argument window_big MUST BE >= than window_small")
        return -1
            
    return index_list
            

# Median Filter and Linear Moving Average function definition
def MF_and_MA(MF_window, MA_window, dataframe, str_in, str_out):
    
    # list of ranges with window_size size
    range_store = [] 
    # total number of range measurements
    dataframe_rows = dataframe.shape[0] 
    # array that will hold the filtered range values and be added to dataframe
    range_dataframe = np.zeros(shape=(dataframe_rows,1)) 
    
    for i in range(dataframe_rows):
        
        if len(range_store) < MF_window:
            range_store.append(dataframe[str_in][i])
            range_dataframe[i] = 0
        else:
            range_store.append(dataframe[str_in][i])
            range_store.pop(0)
            
            aux_range = copy.deepcopy(range_store)
            aux_range.sort()
            
            # Linear Moving Average calculation
            index_list = get_indexlist(MF_window, MA_window)
            
            aux_range = aux_range[index_list[0]:index_list[-1]]
            
            range_dataframe[i] = sum(aux_range)/len(aux_range)
        
        dataframe[str_out] = range_dataframe
        
    return dataframe

# Median Filter and Exponential Moving Average function definition
def MF_and_EMA(MF_window, EMA_window, alpha, dataframe, str_in, str_out):
    
    # list of ranges with window_size size
    range_store = [] 
    # total number of range measurements
    dataframe_rows = dataframe.shape[0] 
    # array that will hold the filtered range values and be added to dataframe
    range_dataframe = np.zeros(shape=(dataframe_rows,1)) 
    
    for i in range(dataframe_rows):
        
        if len(range_store) < MF_window:
            range_store.append(dataframe[str_in][i])
            range_dataframe[i] = 0
        else:
            range_store.append(dataframe[str_in][i])
            range_store.pop(0)
            
            aux_range = copy.deepcopy(range_store)
            aux_range.sort()
            
            index_list = get_indexlist(MF_window, EMA_window)
            aux_range = aux_range[index_list[0]:index_list[-1]]
            
            # Exponential average calculation
            s = aux_range[0]
            
            for j in aux_range:
                s = alpha*j+(1-alpha)*s
            
            range_dataframe[i] = s
            
        dataframe[str_out] = range_dataframe
        
    return dataframe

# Median Filter and Gaussian Moving Average function definition
def MF_and_GMA(MF_window, GMA_window, sigma, dataframe, str_in, str_out):
    
    # list of ranges with window_size size
    range_store = [] 
    # total number of range measurements
    dataframe_rows = dataframe.shape[0] 
    # array that will hold the filtered range values and be added to dataframe
    range_dataframe = np.zeros(shape=(dataframe_rows,1)) 
    # gaussian window
    window_list = np.linspace(0,GMA_window-1,GMA_window)
    f_gaussian = np.exp(-0.5*((GMA_window-1-window_list)/sigma)**2)
    
    for i in range(dataframe_rows):
        
        if len(range_store) < MF_window:
            range_store.append(dataframe[str_in][i])
            range_dataframe[i] = 0
        else:
            range_store.append(dataframe[str_in][i])
            range_store.pop(0)
            
            aux_range = range_store * 1
            aux_range.sort()
            
            index_list = get_indexlist(MF_window, GMA_window)
            aux_range = aux_range[index_list[0]:(index_list[-1])]
            
            aux = 0
            # Gaussian average calculation
            for j in range(GMA_window):
                aux += aux_range[j] * f_gaussian[j]
            
            range_dataframe[i] = aux/GMA_window
            
        dataframe[str_out] = range_dataframe
        
    return dataframe

# Absolute Error calculation
def abs_error(df, str_1, str_2, str_out):
    
    error = (df[str_1] - df[str_2]).abs()
    df[str_out] = error
    return df

# Relative Error calculation
def rel_error(df, str_1, str_2, str_out):
    
    error = (df[str_1] - df[str_2]).abs() / df[str_2]
    df[str_out] = error
    return df

# Root Mean Squared Error calculation
def rmse_error(df, str_1, str_2):
    mse = sum((df[str_1] - df[str_2])**2)/df.shape[0]
    mse = np.sqrt(mse)
    print("[RMSE " + str_1 + "]: " + str(mse))
    return mse
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    