#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 10 18:02:52 2021

@author: marina
"""

# Set absolute package path
import sys, os
sys.path.append(os.path.abspath(".."))

import torch
import numpy as np
import random
#import network.snn_automate as snn
import network.snn_fed as snn
import blimp_model.blimp_id_synthetic as blimp
#import plot_funcs as pf # :D
import extra.plots_automate as pf # :D
import copy

import pid.myPID as PID

############################################################
# FUNCTIONS EVOLUTION: initialization, mutation, evaluation
############################################################

def customInit(cf):
    # Decoding
    q               = cf["init"]["q"]
    
    # Neuronal Dynamics
    thresh          = cf["init"]["thresh"]   # !**
    tau_v           = cf["init"]["tau_v"]    # !**
    tau_t           = cf["init"]["tau_t"]    # !**
    alpha_v         = cf["init"]["alpha_v"]  # !**
    alpha_t         = cf["init"]["alpha_t"]  # !** 
    v_rest          = cf["init"]["v_rest"]      
    duration_refrac = cf["init"]["dur_refrac"]   
    dt              = cf["init"]["dt"]                       
    delay           = cf["init"]["delay"]
    i_dynamics      = [dt, alpha_t, tau_t]       
    n_dynamics      = [thresh, v_rest, alpha_v, alpha_t, dt, duration_refrac, tau_v, tau_t] 
    c_dynamics      = [cf["batch_size"], dt, delay] 
    en_dynamics     = [cf["enc"]["xmin"], cf["enc"]["xmax"], cf["n_layers"][0]-1, cf["enc"]["sigma"], cf["enc"]["plot"]]
    
    return i_dynamics, n_dynamics, c_dynamics, en_dynamics, q

def randomInit(cf):
    # Decoding
    q               = cf["init"]["q"]
    #q               = random.uniform(conf.q_lim[0], conf.q_lim[1])
    
    # Neuronal Dynamics
    thresh          = random.uniform(cf["clamp"]["thresh"][0], cf["clamp"]["thresh"][1])     # !**
    tau_v           = random.uniform(cf["clamp"]["tau_v"][0], cf["clamp"]["tau_v"][1])   # !**
    tau_t           = random.uniform(cf["clamp"]["tau_t"][0], cf["clamp"]["tau_t"][1])   # !**
    alpha_v         = random.uniform(cf["clamp"]["alpha_v"][0], cf["clamp"]["alpha_v"][1]) # !**
    alpha_t         = random.uniform(cf["clamp"]["alpha_t"][0], cf["clamp"]["alpha_t"][1]) # !**
    v_rest          = cf["init"]["v_rest"]      
    duration_refrac = cf["init"]["dur_refrac"]   
    dt              = cf["init"]["dt"]                       
    delay           = cf["init"]["delay"]
    i_dynamics      = [dt, alpha_t, tau_t]       
    n_dynamics      = [thresh, v_rest, alpha_v, alpha_t, dt, duration_refrac, tau_v, tau_t] 
    c_dynamics      = [cf["batch_size"], dt, delay] 
    en_dynamics     = [cf["enc"]["xmin"], cf["enc"]["xmax"], cf["n_layers"][0]-1, cf["enc"]["sigma"], cf["enc"]["plot"]]
    
    return i_dynamics, n_dynamics, c_dynamics, en_dynamics, q

# Initialization individual
def initializeIndividual(cf):
    """
    Initialize a SNN individual with all its desired parameters
    """
    
    # Network creation
    if cf["evol"]["rnd_init"]:
        i_dynamics, n_dynamics, c_dynamics, en_dynamics, q = randomInit(cf)
        network = snn.MySNN(i_dynamics, n_dynamics, c_dynamics, en_dynamics, q, cf)
        network.update_params() # random param update
    else:
        i_dynamics, n_dynamics, c_dynamics, en_dynamics, q = customInit(cf)
        network = snn.MySNN(i_dynamics, n_dynamics, c_dynamics, en_dynamics, q, cf)
        #network.update_params()
    
    if cf["device"] == "cuda:0":  
        network = network.to(torch.float16).cuda()
    
    return network

# Mutation individual
def mutShuffleParams(individual):
    """
    Shuffle certain parameters of the network to keep evolving it. Concretely:
        - thresh, tau_v, tau_t, alpha_v, alpha_t, q
    """    
    #individual[0].reset_state()   
    # The actual update
    individual[0].update_params()
    #individual[0].clamp_params()
    
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
        dic["spikes"] = {}
        dic["traces"] = {}
        for i in range(len(cf["n_layers"])):
            dic["spikes"][i] = []
            dic["traces"][i] = []

    h_curr = h_init
    
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
        #individual[0].reset_state() 
        # Start simulation with duration T
        for j in range(cf["evol"]["T"]):
        
            # Desired (reference) altitude for each timestep
            error = h_ref - h_curr
            dic["error"].append(error)
            
            # Do forward pass
            spikes, traces, u = individual[0].forward(error)
            
            #u = round(u)
            
            # Penalize big u (motor command) and traces
            """
            if abs(u) > cf["evol"]["u_lim"]:
                mse = 5000
                return (mse,)
            """
            
            if u > cf["evol"]["u_lim"]:
                u = cf["evol"]["u_lim"]
            elif u < -cf["evol"]["u_lim"]:
                u = -cf["evol"]["u_lim"]
            """
            for trace in traces:
                max_trace = torch.max(trace).item()
                if max_trace > cf["evol"]["trace_lim"]:
                    mse = 5000
                    return (mse,)
            """
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
                for i in range(len(spikes)):
                    dic["spikes"][i].append(copy.deepcopy(spikes[i].cpu().view(-1).numpy().transpose()))
                    dic["traces"][i].append(copy.deepcopy(traces[i].cpu().view(-1).numpy().transpose()))
 
    if cf["evol"]["plot"]:
        pf.plotNetwork(dic)
        pf.plotBlimp(dic)
    
    error_array = np.array(dic["error"])
    
    # Mean squared error calculation
    mse = np.sqrt(np.mean(error_array**2))
    return (mse,)  # <------ Comma # Actually here is the value of the error


def evaluate_SNNyPID(individual, cf, h_refList, h_init): 
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
        dic["u_snn"] = []
        dic["u_pid"] = []
        dic["spikes"] = {}
        dic["traces"] = {}
        for i in range(len(cf["n_layers"])):
            dic["spikes"][i] = []
            dic["traces"][i] = []

    h_curr = h_init
    
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
        #individual[0].reset_state() 
        # Start simulation with duration T
        for j in range(cf["evol"]["T"]):
        
            # Desired (reference) altitude for each timestep
            error = h_ref - h_curr
            dic["error"].append(error)
            
            # Do forward pass
            spikes, traces, u_snn = individual[0].forward(error)
            
            u_pid = pid.update_simple(error)
            
            u = u_snn + u_pid # Sum of u from and from PID
            
            #u = round(u)
            
            # Penalize big u (motor command) and traces
            """
            if abs(u) > cf["evol"]["u_lim"]:
                mse = 5000
                return (mse,)
            """
            
            if u > cf["evol"]["u_lim"]:
                u = cf["evol"]["u_lim"]
            elif u < -cf["evol"]["u_lim"]:
                u = -cf["evol"]["u_lim"]

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
                dic["u_snn"].append(u_snn)
                dic["u_pid"].append(u_pid)
                dic["r"].append(h_curr)
                dic["h_ref"].append(h_ref)
                for i in range(len(spikes)):
                    dic["spikes"][i].append(copy.deepcopy(spikes[i].cpu().view(-1).numpy().transpose()))
                    dic["traces"][i].append(copy.deepcopy(traces[i].cpu().view(-1).numpy().transpose()))
 
    if cf["evol"]["plot"]:
        pf.plotNetwork(dic)
        pf.plotBlimp(dic)
    
    error_array = np.array(dic["error"])
    
    # Mean squared error calculation
    mse = np.sqrt(np.mean(error_array**2))
    return (mse,)  # <------ Comma # Actually here is the value of the error

'''
def evaluateNSGA2(individual, cf, h_refList, h_init): 
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
    dic["udiff"] = []
    if cf["evol"]["plot"]:
        dic["r"] = []
        dic["h_ref"] = []
        dic["u"] = []
        dic["spikes"] = {}
        dic["traces"] = {}
        for i in range(len(cf["n_layers"])):
            dic["spikes"][i] = []
            dic["traces"][i] = []

    h_curr = h_init
    
    # Array initialization for the blimp model
    u_in = np.zeros(len(blimp.TF_ran.num))
    r_stored = np.ones(len(blimp.TF_ran.den)-1) * h_curr
    # Reset state before evaluating
    #individual[0].reset_state() 
    
    u_prev = 0
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
        # individual[0].reset_state() 
        # Start simulation with duration T
        for j in range(cf["evol"]["T"]):
        
            # Desired (reference) altitude for each timestep
            error = h_ref - h_curr
            dic["error"].append(error)
            
            # Do forward pass
            spikes, traces, u = individual[0].forward(error)
            
            udiff = abs(u - u_prev)
            dic["udiff"].append(udiff)
            
            u_prev = u
            
            # Penalize big u (motor command) and traces
            if u > cf["evol"]["u_lim"]:
                u = cf["evol"]["u_lim"]
            elif u < -cf["evol"]["u_lim"]:
                u = -cf["evol"]["u_lim"]
                
            """
            if abs(u) > cf["evol"]["u_lim"]:
                mse = 5000
                return (mse,)
            """
            
            """
            for trace in traces:
                max_trace = torch.max(trace).item()
                if max_trace > cf["evol"]["trace_lim"]:
                    mse = 5000
                    return (mse,)
            """
            
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
                for i in range(len(spikes)):
                    dic["spikes"][i].append(copy.deepcopy(spikes[i].cpu().view(-1).numpy().transpose()))
                    dic["traces"][i].append(copy.deepcopy(traces[i].cpu().view(-1).numpy().transpose()))
 
    if cf["evol"]["plot"]:
        pf.plotNetwork(dic)
        pf.plotBlimp(dic)
    
    error_array = np.array(dic["error"])
    udiff_array = np.array(dic["udiff"])
    # Mean squared error calculation
    mse = np.sqrt(np.mean(error_array**2))
    #udiff_mse = np.sqrt(np.mean(udiff_array**2))
    udiff_mse = np.sum(udiff_array)
    return (mse,udiff_mse,)  # <------ Comma # Actually here is the value of the error
'''
