#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 15 20:38:29 2021

@author: marina
"""

# Set absolute package path
import sys, os
sys.path.append(os.path.abspath(".."))

import numpy as np
import random
import copy
import matplotlib.pyplot as plt
from datetime import datetime

from deap import algorithms
from deap import base
from deap import creator
from deap import tools
import multiprocessing
 
import pid.myPID as PID
import blimp_model.blimp_id_synthetic as blimp
import extra.aux_funcs as af # :)
import extra.plots_automate as pf # :D

from evol_algo.evol_algos import myeaSimple

import time

# hof[0] =  [0.8735822376592428, 0.061441163793195155, 0.0028269842479982966]
# hof[1] =  [2.168137903151525, 0.05988119305371925, 0.10983607256306606]

############################################################
# CONFIG GLOBAL VARIABLE
############################################################
cf = af.set_config('../config/config.yaml')
PLOT = False
P_LIM = 6
I_LIM = 1
D_LIM = 1
DT = 1/30
SIMPLE = True

############################################################
# FUNCTIONS EVOLUTION: initialization, mutation, evaluation
############################################################

creator.create("FitnessMax", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMax)

toolbox = base.Toolbox()
# Attribute generator 
toolbox.register("attr_bool", random.uniform, -I_LIM, I_LIM)
# Structure initializers
toolbox.register("individual", tools.initRepeat, creator.Individual, 
    toolbox.attr_bool, 3)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

def mutation(individual):
    
    p = cf["lim"]["var"]
    
    individual[0] += random.uniform(0, P_LIM) * p
    individual[1] += random.uniform(0, I_LIM) * p
    individual[2] += random.uniform(0, D_LIM) * p

    for i in range(3):
        individual[i] = max(0, individual[i])

    return individual,

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
    
    P = individual[0]
    I = individual[1]
    D = individual[2]
    dt = DT
    simple = SIMPLE
    
    pid = PID.PID(P, I, D, dt, simple)
    
    dic = {}
    dic["error"] = []
    if cf["evol"]["plot"]:
        dic["r"] = []
        dic["h_ref"] = []
        dic["u"] = []

    h_curr = h_init
    
    # Array initialization for the blimp model
    u_in = np.zeros(len(blimp.TF_ran.num))
    r_stored = np.ones(len(blimp.TF_ran.den)-1) * h_curr
    
    count = 0
    TList = cf["evol"]["T"]
    
    for h_ref in h_refList:
        
        #individual[0].heights.append((h_ref,h_init))
        
        # Start simulation with duration T
        #for j in range(cf["evol"]["T"]):
            
        for j in range(TList[count]):
        
            # Desired (reference) altitude for each timestep
            error = h_ref - h_curr
            dic["error"].append(error)
        
            # Apply PID to error signal
            if simple:
                u = pid.update_simple(error)
            else:
                u = pid.update(error)
            
            #u = round(u)
            
            # Penalize big u (motor command) and traces
            #if abs(u) > cf["evol"]["u_lim"]:
            #    mse = 5000
            
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
                dic["r"].append(h_curr)
                dic["h_ref"].append(h_ref)
 
        count += 1 
 
    if cf["evol"]["plot"]:
        pf.plotBlimp_PID(dic, dt)
    
    error_array = np.array(dic["error"])
    
    # Mean squared error calculation
    mse = np.sqrt(np.mean(error_array**2))
    
    return (mse,)  # <------ Comma # Actually here is the value of the error

toolbox.register("evaluate", evaluate)
toolbox.register("mutate", mutation)
toolbox.register("select", tools.selTournament, tournsize=cf["evol"]["tourn_size"])

if __name__ == '__main__':
    
    t1 = time.time()
    
    if cf["evol"]["parallel"]:
        CPU_count = multiprocessing.cpu_count()#-1   
        pool = multiprocessing.Pool(CPU_count)
        toolbox.register("map", pool.starmap)    
    
    pop = toolbox.population(n=cf["evol"]["n_pop"])
    
    # --> SAME AS *deap_one_max.py* UP UNTIL HERE
    
    hof = tools.HallOfFame(cf["evol"]["n_hof"])
    stats = tools.Statistics(lambda ind: ind.fitness.values)
    stats.register("avg", np.mean)
    stats.register("std", np.std)
    stats.register("min", np.min)
    stats.register("max", np.max)
    
    # Create directory to save networks
    mainDir = cf["evol"]["save_name"]
    
    pop, hof, log = myeaSimple(pop, toolbox, mutpb=cf["evol"]["p_mut_ind"], ngen=cf["evol"]["n_gen"], 
                               mainDir = mainDir, cf = cf, stats=stats, halloffame=hof, verbose=True)
    
    if cf["evol"]["parallel"]:
        pool.close()
        pool.join()
        t2 = time.time()
        print("Multiprocessing with",CPU_count,"core(s) took",round((t2-t1),2),"s")
        
    cf["evol"]["plot"] =True
    evaluate(hof[0],cf,cf["evol"]["h_ref"],cf["evol"]["h_init"])




































