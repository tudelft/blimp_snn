#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar  2 20:41:01 2021

@author: marina

Interesting:
    - T2000_NPOP100_NGEN500_NEU2-10-5-2-1_05-04-2021_19-08-51: Really smooth and contained between (-3,3)
    - T2000_NPOP100_NGEN500_NEU2-10-5-2-1_05-04-2021_19-25-20

"""
# Set absolute package path
import sys, os
sys.path.append(os.path.abspath(".."))

import os
import extra.aux_funcs as af # :)
import numpy as np
from evol_funcs.evol_mut_eval import evaluate, evaluate_SNNyPID
#from evol_funcs.evol_funcs_ANN import evaluate, evaluate_ANNyPID

import pid.myPID as PID
# Some constants
custom_config = True
#config_file = "config_night/config1.yaml"
config_file = "config.yaml"

# T2500_NPOP50_NGEN200_24-03-2021_23-47-25/
# T1500_NPOP60_NGEN400_24-03-2021_22-21-03/
# T3000_NPOP50_NGEN200_24-03-2021_22-06-17/

# T1800_NPOP40_NGEN500_NEU10-5-1-1_17-06-2021_11-40-05

# "T1800_NPOP40_NGEN500_NEU1-3-2-1_17-06-2021_15-10-09"
net_name = "T1200_NPOP40_NGEN500_NEU10-5-5-1_26-05-2021_13-34-44"
gen = 30
hof = True
num = 0

# Load configuration "cf" dir
cf = af.set_config("../config/" + config_file)

# Load population (IND or HOF)
pop = af.readPopulation(cf, net_name, gen, hof)
#pop = af.readPopulationSD(cf, net_name, gen, hof)
network = pop[num]

# Load network configuration
if not custom_config:
    cf = network.cf

# Activate plotting
cf["evol"]["plot"] = True

# Evaluate network + plot
'''
network.pid[0] = -1
network.pid[1] = 0
network.pid[2] = 0
'''
#network.pid = [2,0,0]

individual = [network]
mse = evaluate(individual, cf, h_refList = cf["evol"]["h_ref"], h_init = cf["evol"]["h_init"])


#pid = PID.PID(*cf["pid"]["PID"], cf["pid"]["dt"], cf["pid"]["simple"])
#inputList = np.linspace(*cf["pid"]["input_lim"], cf["pid"]["n_points"])
#inputList = np.random.uniform(low=cf["pid"]["input_lim"][0], high=cf["pid"]["input_lim"][1], size = (cf["pid"]["n_points"],))
#mse = evaluate_PID(individual, cf, pid=pid, inputList=inputList)

print("MSE = ", mse)

#torch.save(model.state_dict(), PATH)