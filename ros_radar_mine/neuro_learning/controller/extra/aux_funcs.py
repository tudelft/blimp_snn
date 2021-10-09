#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 10 17:45:30 2021

@author: marina
"""

# Set absolute package path
import sys, os
sys.path.insert(0, '../network/')
sys.path.append(os.path.abspath(".."))

import yaml
import torch
import copy
import matplotlib.pyplot as plt
import random
import numpy as np
from datetime import datetime
from itertools import repeat
from os import listdir
from os.path import isfile, join
#import evol_funcs.evol_mut_eval as isnn
#import evol_funcs.evol_funcs_ANN as iann

from deap import algorithms
from deap import base
from deap import creator
from deap import tools

def set_config(path):
    with open(path, mode='r') as file:
        
        # Load configuration .yaml file into dict
        conf = yaml.load(file, Loader=yaml.FullLoader)
    
        # Create some additional variables:
        # 1) Save path
        conf["path"] = os.path.abspath("..") + "/" + conf["path"]
            
        # 2) Save name for population / hof
        now = datetime.now()
        now = now.strftime("%d-%m-%Y_%H-%M-%S")
        
        arch = str(conf["n_layers"])
        arch = arch.replace("[","")
        arch = arch.replace("]","")
        arch = arch.replace(",","-")
        arch = arch.replace(" ","")
        
        name = "T" + str(conf["evol"]["T"]) + "_NPOP" + str(conf["evol"]["n_pop"]) + "_NGEN" + str(conf["evol"]["n_gen"]) + "_NEU" + arch + "_" + now
    
        # Append them to dict
        save_name = conf["path"] + name
        conf["evol"]["save_name"] = save_name
        
        # Percentage variation on each mutation
        var = conf["lim"]["var"]
        conf["lim"]["thresh"]  = [-conf["clamp"]["thresh"][1]*var,conf["clamp"]["thresh"][1]*var]
        conf["lim"]["tau_t"] = [-conf["clamp"]["tau_t"][1]*var,conf["clamp"]["tau_t"][1]*var]
        conf["lim"]["tau_v"] = [-conf["clamp"]["tau_v"][1]*var,conf["clamp"]["tau_v"][1]*var]
        conf["lim"]["alpha_t"] = [-conf["clamp"]["alpha_t"][1]*var,conf["clamp"]["alpha_t"][1]*var]
        conf["lim"]["alpha_v"] = [-conf["clamp"]["alpha_v"][1]*var,conf["clamp"]["alpha_v"][1]*var]
        conf["lim"]["weights"]  = [-conf["clamp"]["weights"][1]*var,conf["clamp"]["weights"][1]*var]
        conf["lim"]["q"]  = [-conf["clamp"]["q"][1]*var,conf["clamp"]["q"][1]*var]    
        return conf

#########################################################
# Population reader
#########################################################    
def readPopulationSD(cf, net_name, gen, hof):
    path = cf["path"] + net_name + "/gen_#" + str(gen) + "/"
    files = [f for f in listdir(path) if isfile(join(path, f))]
    
    pop = []
    
    if hof == True:
        for file in files:
            if "HOF" in file:
                network = iann.initializeIndividual(cf)
                network.load_state_dict(torch.load(join(path, file)))
                network.eval()
                pop.append(network)
    else:
        for file in files:
            if "IND" in file:
                network = iann.initializeIndividual(cf)
                network.load_state_dict(torch.load(join(path, file)))
                network.eval()
                pop.append(network)
    
    if len(pop) == 0:
        print("Careful! No networks were loaded! Check that folder is not empty")
    
    return pop

def readPopulation(cf, net_name, gen, hof):
    path = cf["path"] + net_name + "/gen_#" + str(gen) + "/"
    files = [f for f in listdir(path) if isfile(join(path, f))]
    
    pop = []
    
    if hof == True:
        for file in files:
            if "HOF" in file:
                network = torch.load(join(path, file))
                network.eval()
                #print(network.perc_lim)
                pop.append(network)
    else:
        for file in files:
            if "IND" in file:
                network = torch.load(join(path, file))
                network.eval()
                #print(network.perc_lim)
                pop.append(network)
    
    if len(pop) == 0:
        print("Careful! No networks were loaded! Check that folder is not empty")
    
    return pop

def loadPopulation(cf, net_name, gen):
    path = cf["path"] + net_name + "/gen_#" + str(gen) + "/"
    files = [f for f in listdir(path) if isfile(join(path, f))]
    
    pop = []
    
    for file in files:
        if "IND" in file:
            network = torch.load(join(path, file))
            network.eval()
            #print(network.perc_lim)
            pop.append([network])

    if len(pop) == 0:
        print("Careful! No networks were loaded! Check that folder is not empty")
        
    return pop  

#########################################################
# Random generator functions
#########################################################
def randomArray(a, b, n_elem):
    result = np.zeros(n_elem)
    for i in range(n_elem):
        result[i] = a + (b - a) * random.random()
    return result

def randomTensor(a, b, n_elem):
    result = torch.zeros(1,1,n_elem)
    for i in range(n_elem):
        result[0,0,i] = a + (b - a) * random.random()
    return result

def randomTensorW(a, b, n_elem1, n_elem2):
    result = torch.zeros(n_elem1,n_elem2)
    for i in range(n_elem1):
        for j in range(n_elem2):
            result[i,j] = a + (b - a) * random.random()
    return result

def randomTensorParam(a, b, n_elem):
    aux_tensor = torch.zeros(1,1,n_elem)
    for i in range(n_elem):
        aux_tensor[0,0,i] = a + (b - a) * random.random()
    result = torch.nn.Parameter(aux_tensor, requires_grad = False)
    return result

def gen_inputs_dup(n, n_total, x_min, x_max, nd_min, nd_max):
    """
    Generate synthetic inputs: each generated a random DIFFERENT no. of times
    """
    result = []
    
    for i in range(0,n):
        aux = random.randint(x_min,x_max)       # Generate random input
        ndup = random.randint(nd_min,nd_max)    # Select random duplication number
        aux_array = np.repeat(aux,ndup)         # Duplicate aux, ndup times
        result.extend(aux_array)                # Append to final inputs array
    
    result = result[:n_total]
    
    return result