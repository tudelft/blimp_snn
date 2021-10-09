#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 24 10:14:14 2021

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
from evol_algo.evol_algos import myeaSimple, myeaPID, myeaMuPlusLambda, myeaJPaper
from evol_funcs.evol_mut_eval import initializeIndividual, mutShuffleParams, evaluate, evaluate_SNNyPID
import multiprocessing
#from torch import multiprocessing as mp

import extra.aux_funcs as af  # :)

import time

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
toolbox.register("evaluate", evaluate_SNNyPID)
toolbox.register("mutate", mutShuffleParams)

# Selection function: modify??
if cf["evol"]["easimple"]:
    toolbox.register("select", tools.selTournament, tournsize = cf["evol"]["tourn_size"])
else:
    toolbox.register("select", tools.selBest)

t1 = time.time()

if cf["evol"]["parallel"]:
    CPU_count = multiprocessing.cpu_count()#-1   
    pool = multiprocessing.Pool(CPU_count)
    #toolbox.register("map", pool.starmap)    
    toolbox.register("map", pool.starmap)    


pop = toolbox.population(n=cf["evol"]["n_pop"])

# To load population from file uncomment the next two lines
# pop_list = af.loadPopulation(cf, "T2000_NPOP10_NGEN100_NEU40-20-10-5_02-04-2021_14-15-09", 5)
# pop = [creator.Individual(i) for i in pop_list]

hof = tools.HallOfFame(cf["evol"]["n_hof"])

fit_stats = tools.Statistics(lambda ind: ind.fitness.values)
fit_stats.register("avg", np.mean)
fit_stats.register("std", np.std)
fit_stats.register("min", np.min)
fit_stats.register("max", np.max)

# Create directory to save networks
mainDir = cf["evol"]["save_name"]

# myeaSimple, myeaPID, myeaJPaper, myeaMuPlusLambda
if cf["evol"]["easimple"]:
    pop, hof, log = myeaSimple(pop, toolbox, mutpb=cf["evol"]["p_mut_ind"], ngen=cf["evol"]["n_gen"], 
                               mainDir = mainDir, cf = cf, stats=fit_stats, halloffame=hof, verbose=True)
else:
    pop, hof, log = myeaJPaper(pop, toolbox, mutpb=cf["evol"]["p_mut_ind"], ngen=cf["evol"]["n_gen"], 
                                   mainDir = mainDir, cf = cf, stats=fit_stats, halloffame=hof, verbose=True)
#pop, hof, log = myeaMuPlusLambda(pop, toolbox, mu=cf["evol"]["n_pop"], lambda_=cf["evol"]["n_pop"]*2, mutpb=cf["evol"]["p_mut_ind"], ngen=cf["evol"]["n_gen"], mainDir = mainDir, stats=fit_stats, halloffame=hof, verbose=True)

t2 = time.time()
if cf["evol"]["parallel"]:
    pool.close()
    pool.join()
    print("Multiprocessing with",CPU_count,"core(s) took",round((t2-t1),2),"s")
else:
    print("Non-parallel processing took",round((t2-t1),2),"s")