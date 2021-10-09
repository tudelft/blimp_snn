#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 30 17:41:45 2021

@author: marina
"""

# Set absolute package path
import sys, os
sys.path.append(os.path.abspath(".."))

import torch
import array
import numpy as np
import random
import copy
import matplotlib.pyplot as plt
import pandas as pd
from datetime import datetime

from deap import algorithms
from deap import base
from deap import creator
from deap import tools
from evol_algo.evol_algo_ANN import myeaSimple, myeaPID
from evol_funcs.evol_funcs_ANN import initializeIndividual, mutation, evaluate, evaluate_ANNyPID
import multiprocessing
#from torch import multiprocessing as mp

import extra.aux_funcs as af  # :)

import time
import sys

############################################################
# CONFIG GLOBAL VARIABLE
############################################################
cf = af.set_config('../config/config.yaml')
#cf = af.set_config(curr_path + '/' + str(sys.argv[1]))

#########################################################
# DEAP FRAMEWORK DEFINITIONS
#########################################################

# Basic initializations
creator.create("FitnessMin", base.Fitness, weights=(-1.0,))
creator.create("Individual", list, fitness=creator.FitnessMin)
toolbox = base.Toolbox()
toolbox.register("attr_params", initializeIndividual, cf = cf)
toolbox.register("individual", tools.initRepeat, creator.Individual, 
    toolbox.attr_params, 1)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

# Evaluation and mutation functions
toolbox.register("evaluate", evaluate_ANNyPID)
toolbox.register("mutate", mutation)

# Selection function: modify??
toolbox.register("select", tools.selTournament, tournsize = cf["evol"]["tourn_size"])
"""
toolbox.register("select", tools.selBest)
toolbox.register("select", tools.selNSGA2)
"""

t1 = time.time()

if cf["evol"]["parallel"]:
    #os.environ['MKL_THREADING_LAYER'] = 'GNU'
    #multiprocessing.set_start_method('spawn', force =True)
    CPU_count = multiprocessing.cpu_count()#-1   
    pool = multiprocessing.Pool(CPU_count)
    toolbox.register("map", pool.starmap)    

pop = toolbox.population(n=cf["evol"]["n_pop"])

hof = tools.HallOfFame(cf["evol"]["n_hof"])

fit_stats = tools.Statistics(lambda ind: ind.fitness.values)
fit_stats.register("avg", np.mean)
fit_stats.register("std", np.std)
fit_stats.register("min", np.min)
fit_stats.register("max", np.max)

# Create directory to save networks
mainDir = cf["evol"]["save_name"]

# myeaSimple, myeaPID, myeaJPaper, myeaMuPlusLambda
pop, hof, log = myeaSimple(pop, toolbox, mutpb=cf["evol"]["p_mut_ind"], ngen=cf["evol"]["n_gen"], 
                               mainDir = mainDir, cf = cf, stats=fit_stats, halloffame=hof, verbose=True)
#pop, hof, log = myeaMuPlusLambda(pop, toolbox, mu=cf["evol"]["n_pop"], lambda_=cf["evol"]["n_pop"]*2, mutpb=cf["evol"]["p_mut_ind"], ngen=cf["evol"]["n_gen"], mainDir = mainDir, stats=fit_stats, halloffame=hof, verbose=True)

if cf["evol"]["save"]:
    df = pd.DataFrame(log)
    df.to_csv(mainDir + '_log.csv')

t2 = time.time()
if cf["evol"]["parallel"]:
    pool.close()
    pool.join()
    print("Multiprocessing with",CPU_count,"core(s) took",round((t2-t1),2),"s")
else:
    print("Non-parallel processing took",round((t2-t1),2),"s")