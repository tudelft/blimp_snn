#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 30 17:42:23 2021

@author: marina
"""

# Set absolute package path
import sys, os
sys.path.append(os.path.abspath(".."))

import torch
import numpy as np
import random
import network.ann_automate as ann  
import blimp_model.blimp_id_synthetic as blimp
#import plot_funcs as pf # :D
import extra.plots_automate as pf # :D
import copy

import pid.myPID as PID

############################################################
# FUNCTIONS EVOLUTION: initialization, mutation, evaluation
############################################################

# Initialization individual
def initializeIndividual(cf):
    """
    Initialize a SNN individual with all its desired parameters
    """
    
    # Network creation
    network = ann.MyANN(cf)
    network.update_params()
    
    if cf["device"] == "cuda:0":  
        network = network.to(torch.float16).cuda()
    
    return network

# Mutation individual
def mutation(individual):
    """
    Shuffle certain parameters of the network to keep evolving it. Concretely:
        - thresh, tau_v, tau_t, alpha_v, alpha_t, q
    """    

    individual[0].update_params()
    
    return individual, # <------ Comma

# Evaluation function
def evaluate(individual, cf, h_refList, h_init): 
    """
    1) Altitude initialization
    2) Start simulation (from t=0 to t=T):
        - Compute the error                                       [error]
        - Feed error to SNN and do forward pass                   [snn]
        - Save output speed, u (to proper size array)             [u]
        - Feed u and range array (u_in + r_stored) to blimp model [plant]
        - Get new range                                           [range]
        ----> Loop all over again
    3) Calculate MSE and return
    """    
    
    dic = {}
    dic["error"] = []
    if cf["evol"]["plot"]:
        dic["r"] = []
        dic["h_ref"] = []
        dic["u"] = []
        dic["values"] = {}
        for i in range(len(cf["n_layers"])):
            dic["values"][i] = []

    h_curr = h_init
    error_prev = 0
    
    # Array initialization for the blimp model
    u_in = np.zeros(len(blimp.TF_ran.num))
    r_stored = np.ones(len(blimp.TF_ran.den)-1) * h_curr
    # Reset state before evaluating
    #individual[0].reset_state() 
    
    count = 0
    for h_ref in h_refList:
        
        individual[0].heights.append((h_ref,h_init))
        """
        if count != 0:
            h_curr = h_refList[count-1]
            u_in = np.zeros(len(blimp.TF_ran.num))
            r_stored = np.ones(len(blimp.TF_ran.den)-1) * h_curr
        count += 1
        """
        # Start simulation with duration T
        for j in range(cf["evol"]["T"]):
        
            # Desired (reference) altitude for each timestep
            error = h_ref - h_curr
            dic["error"].append(error)
            
            error_diff = float((error-error_prev))/cf["init"]["dt"]
            error_prev = error
            
            #inp = torch.tensor([float(error), float(error_diff)])
            inp = torch.tensor([float(error)])
            
            # Do forward pass
            x_array, u = individual[0].forward(inp)
            
            # Round / discretize u
            u = torch.round(u)
            
            if u < -cf["evol"]["u_lim"]:
                u = -cf["evol"]["u_lim"]
            elif u > cf["evol"]["u_lim"]:
                u = cf["evol"]["u_lim"]
            
            # Penalize big u (motor command) and traces
            #if abs(u) > cf["evol"]["u_lim"]:
            #    mse = 5000
            #    return (mse,)
            
            # Get the proper u_in array
            u_in = np.delete(u_in, 0)
            u_in = np.append(u_in, u)
            
            # Apply model and store range
            h_curr, r_stored = blimp.mymodel_ran(u_in, blimp.TF_ran, r_stored)
            
            # Add noise to height
            h_curr += random.uniform(-cf["evol"]["noise"], cf["evol"]["noise"])
            
            # For plotting
            if cf["evol"]["plot"]:
                dic["u"].append(u)
                dic["r"].append(h_curr)
                dic["h_ref"].append(h_ref)
                for i in range(len(cf["n_layers"])):
                    dic["values"][i].append(copy.deepcopy(x_array[i].cpu().view(-1).numpy().transpose()))
 
    if cf["evol"]["plot"]:
        pf.plotANN(dic)
        pf.plotBlimp(dic)
    
    error_array = np.array(dic["error"])
    
    # Mean squared error calculation
    mse = np.sqrt(np.mean(error_array**2))
    return (mse,)  # <------ Comma # Actually here is the value of the error

def evaluate_ANNyPID(individual, cf, h_refList, h_init): 
    """
    1) Altitude initialization
    2) Start simulation (from t=0 to t=T):
        - Compute the error                                       [error]
        - Feed error to SNN and do forward pass                   [snn]
        - Save output speed, u (to proper size array)             [u]
        - Feed u and range array (u_in + r_stored) to blimp model [plant]
        - Get new range                                           [range]
        ----> Loop all over again
    3) Calculate MSE and return
    """    
    
    P = individual[0].pid[0]
    I = individual[0].pid[1]
    D = individual[0].pid[2]
    
    pid = PID.PID(P, I, D, 1/30, True)
    
    dic = {}
    dic["error"] = []
    if cf["evol"]["plot"]:
        dic["r"] = []
        dic["h_ref"] = []
        dic["u"] = []
        dic["u_ann"] = []
        dic["u_pid"] = []
        dic["values"] = {}
        for i in range(len(cf["n_layers"])):
            dic["values"][i] = []

    h_curr = h_init
    error_prev = 0
    
    # Array initialization for the blimp model
    u_in = np.zeros(len(blimp.TF_ran.num))
    r_stored = np.ones(len(blimp.TF_ran.den)-1) * h_curr
    # Reset state before evaluating
    #individual[0].reset_state() 
    
    count = 0
    for h_ref in h_refList:
        
        individual[0].heights.append((h_ref,h_init))
        """
        if count != 0:
            h_curr = h_refList[count-1]
            u_in = np.zeros(len(blimp.TF_ran.num))
            r_stored = np.ones(len(blimp.TF_ran.den)-1) * h_curr
        count += 1
        """
        # Start simulation with duration T
        for j in range(cf["evol"]["T"]):
        
            # Desired (reference) altitude for each timestep
            error = h_ref - h_curr
            dic["error"].append(error)
            
            error_diff = float((error-error_prev))/cf["init"]["dt"]
            error_prev = error
            
            #inp = torch.tensor([float(error), float(error_diff)])
            inp = torch.tensor([float(error)])
            
            # Do forward pass
            x_array, u_ann = individual[0].forward(inp)
            
            u_pid = pid.update_simple(error)
            
            u = u_ann + u_pid
            
            # Round / discretize u
            #u = torch.round(u)
            
            if u < -cf["evol"]["u_lim"]:
                u = -cf["evol"]["u_lim"]
            elif u > cf["evol"]["u_lim"]:
                u = cf["evol"]["u_lim"]
            
            # Penalize big u (motor command) and traces
            #if abs(u) > cf["evol"]["u_lim"]:
            #    mse = 5000
            #    return (mse,)
            
            # Get the proper u_in array
            u_in = np.delete(u_in, 0)
            u_in = np.append(u_in, u)
            
            # Apply model and store range
            h_curr, r_stored = blimp.mymodel_ran(u_in, blimp.TF_ran, r_stored)
            
            # Add noise to height
            h_curr += random.uniform(-cf["evol"]["noise"], cf["evol"]["noise"])
            
            # For plotting
            if cf["evol"]["plot"]:
                dic["u"].append(u)
                dic["u_ann"].append(u_ann)
                dic["u_pid"].append(u_pid)
                dic["r"].append(h_curr)
                dic["h_ref"].append(h_ref)
                for i in range(len(cf["n_layers"])):
                    dic["values"][i].append(copy.deepcopy(x_array[i].cpu().view(-1).numpy().transpose()))
 
    if cf["evol"]["plot"]:
        pf.plotANN(dic)
        pf.plotBlimp(dic)
    
    error_array = np.array(dic["error"])
    
    # Mean squared error calculation
    mse = np.sqrt(np.mean(error_array**2))
    return (mse,)  # <------ Comma # Actually here is the value of the error

'''
def evaluate_PID(individual, cf, pid, inputList):
        
    dic = {}
    dic["error"] = []
    if cf["evol"]["plot"]:
        dic["inp"] = []
        dic["u_pid"] = []
        dic["u_snn"] = []
        dic["values"] = {}
        for i in range(len(cf["n_layers"])):
            dic["values"][i] = []


    for inp in inputList:
    
        # Add noise to error
        inp += random.uniform(-cf["evol"]["noise"], cf["evol"]["noise"])    
        
        # Do forward pass
        x_array, u_nn = individual[0].forward(inp)
        
        if cf["pid"]["simple"]:
            u_pid = pid.update_simple(inp)
        else:
            u_pid = pid.update(inp)
        
        # Penalize big u (motor command) and traces
        if abs(u_nn) > cf["evol"]["u_lim"]:
            mse = 5000
            return (mse,)
        
        dic["error"].append(abs(u_pid-u_nn))
        
        # For plotting
        if cf["evol"]["plot"]:
            dic["inp"].append(inp)
            dic["u_pid"].append(u_pid)
            dic["u_snn"].append(u_nn)
            for i in range(len(cf["n_layers"])):
                dic["values"][i].append(copy.deepcopy(x_array[i].cpu().view(-1).numpy().transpose()))

 
    if cf["evol"]["plot"]:
        #pf.plotANN(dic)
        pf.plot_PID_SNN(dic)
    
    error_array = np.array(dic["error"])
    
    # Mean squared error calculation
    mse = np.sqrt(np.mean(error_array**2))
    return (mse,)  # <------ Comma # Actually here is the value of the error
'''