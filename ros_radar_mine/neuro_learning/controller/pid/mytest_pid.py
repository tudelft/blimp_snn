#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 15 20:08:18 2021

@author: marina
"""

import numpy as np
import random
import copy
import matplotlib.pyplot as plt
from datetime import datetime
import os

# Set absolute package path
import sys, os
sys.path.append(os.path.abspath(".."))
 
import pid.myPID as PID
import blimp_model.blimp_id_synthetic as blimp
import extra.aux_funcs as af # :)
from evol_main.evol_pid import evaluate

import time


############################################################
# CONFIG GLOBAL VARIABLE
############################################################
curr_path = os.path.abspath(os.getcwd())
cf = af.set_config('../config/config.yaml')

############################################################
# PID initialization
############################################################

# hof[0] =  [0.8735822376592428, 0.061441163793195155, 0.0028269842479982966]
# hof[1] =  [2.168137903151525, 0.05988119305371925, 0.10983607256306606]

#P = 0.8735822376592428
#I = 0.061441163793195155
#D = 0.0028269842479982966

P = 4.6
I = 0.08
D = 5

dt = 1/30
simple = True

pid = PID.PID(P, I, D, dt, simple)
ind = [P, I, D]

############################################################
# Simulation
############################################################

h_refList = cf["evol"]["h_ref"]
h_init    = cf["evol"]["h_init"]
cf["evol"]["plot"] = True
evaluate(ind,cf,h_refList,h_init)