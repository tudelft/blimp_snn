#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 12 17:03:35 2021

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
from evol_algo.evol_algos import myeaSimple, myeaNSGA2, myeaPID, myeaMuPlusLambda, myeaJPaper
from evol_funcs.evol_mut_eval import initializeIndividual, mutShuffleParams, evaluateNSGA2, evaluate, evaluate_PID
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
creator.create("FitnessMin", base.Fitness, weights=(-100.0,-0.1))
creator.create("Individual", list, fitness=creator.FitnessMin)
toolbox = base.Toolbox()
toolbox.register("attr_params", initializeIndividual, cf = cf)
toolbox.register("individual", tools.initRepeat, creator.Individual, 
    toolbox.attr_params, 1)
toolbox.register("population", tools.initRepeat, list, toolbox.individual)

# Evaluation and mutation functions
toolbox.register("evaluate", evaluateNSGA2)
toolbox.register("mutate", mutShuffleParams)

# Selection function:
toolbox.register("select", tools.selNSGA2)


t1 = time.time()

if cf["evol"]["parallel"]:
    CPU_count = multiprocessing.cpu_count()#-1   
    pool = multiprocessing.Pool(CPU_count)
    toolbox.register("map", pool.starmap)    


pop = toolbox.population(n=cf["evol"]["n_pop"])

# To load population from file uncomment the next two lines
# pop_list = af.loadPopulation(cf, "T2000_NPOP10_NGEN100_NEU40-20-10-5_02-04-2021_14-15-09", 5)
# pop = [creator.Individual(i) for i in pop_list]

hof = tools.HallOfFame(cf["evol"]["n_hof"])

fit_stats = tools.Statistics(lambda ind: ind.fitness.values)
fit_stats.register("avg", np.mean)
fit_stats.register("std", np.std)
fit_stats.register("min", np.min, axis=0)
fit_stats.register("max", np.max, axis=0)

# Create directory to save networks
mainDir = cf["evol"]["save_name"]

# myeaSimple, myeaPID, myeaJPaper, myeaMuPlusLambda
# pop, hof, log = myeaSimple(pop, toolbox, mutpb=cf["evol"]["p_mut_ind"], ngen=cf["evol"]["n_gen"], 
#                           mainDir = mainDir, cf = cf, stats=fit_stats, halloffame=hof, verbose=True)

pop, hof, log = myeaNSGA2(pop, toolbox, mu=cf["evol"]["n_pop"], mutpb=cf["evol"]["p_mut_ind"], ngen=cf["evol"]["n_gen"], 
                           mainDir = mainDir, cf = cf, stats=fit_stats, halloffame=hof, verbose=True)

#pop, hof, log = myeaMuPlusLambda(pop, toolbox, mu=cf["evol"]["n_pop"], lambda_=cf["evol"]["n_pop"]*2, mutpb=cf["evol"]["p_mut_ind"], ngen=cf["evol"]["n_gen"], mainDir = mainDir, stats=fit_stats, halloffame=hof, verbose=True)

t2 = time.time()
if cf["evol"]["parallel"]:
    pool.close()
    pool.join()
    print("Multiprocessing with",CPU_count,"core(s) took",round((t2-t1),2),"s")
else:
    print("Non-parallel processing took",round((t2-t1),2),"s")