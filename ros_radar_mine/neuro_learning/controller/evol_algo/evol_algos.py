#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar 22 17:52:43 2021

@author: marina

This file comprises the EVOLUTIONARY ALGORITHM functions employed to train the 
SNN as an altitude controller (with the DEAP framework) are defined

"""

# Set absolute package path
import sys, os
sys.path.append(os.path.abspath(".."))

import torch
import numpy as np
import random
import matplotlib.pyplot as plt
import pandas as pd

from deap import algorithms
from deap import base
from deap import creator
from deap import tools
import multiprocessing
#from torch import multiprocessing as mp

import pid.myPID as PID

import time

############################################################
# Auxiliary functions
############################################################


def multi_run_wrapper(toolbox, args):
    return toolbox.evaluate(args[1])

def evaluateFitness(cf, toolbox, evaluate_inds):
    """
    Function that calculates the fitness of each individual in evaluate_inds
    depending on whether the altitude is generated randomly per generation
    or not
    
    · Important parameters: h_ref, h_init, h_lim, n_sim and rnd_height in config.yaml
    """
    # Custom static height case (H_REF, H_INIT in config.yaml)
    if not cf["evol"]["rnd_height"]:
        h_refList = cf["evol"]["h_ref"]
        h_init    = cf["evol"]["h_init"]
        #fitnesses = [toolbox.evaluate(individual = ind, cf=cf, h_refList=h_refList, h_init=h_init) for ind in evaluate_inds]
    # Random height case (N_SIM in config.yaml)
    else:
        h_refList = np.round(np.random.uniform(*cf["evol"]["h_lim"],cf["evol"]["n_sim"]),2)
        h_init    = random.uniform(*cf["evol"]["h_lim"])
        
    args = [(ind, cf, h_refList, h_init) for ind in evaluate_inds]
    fitnesses = toolbox.map(toolbox.evaluate, args)
    
    #fitnesses = [toolbox.evaluate(individual = ind, cf=cf, h_refList=h_refList, h_init=h_init) for ind in evaluate_inds]
    
    return fitnesses, h_refList, h_init

def evaluate_PID_SNN(cf, toolbox, evaluate_inds):
 
    pid = PID.PID(*cf["pid"]["PID"], cf["pid"]["dt"], cf["pid"]["simple"])
    inputList = np.linspace(*cf["pid"]["input_lim"], cf["pid"]["n_points"])
    
    args = [(ind, cf, pid, inputList) for ind in evaluate_inds]
    fitnesses = toolbox.map(toolbox.evaluate, args)
    
    #fitnesses = [toolbox.evaluate(individual = ind, cf=cf, pid=pid, inputList=inputList) for ind in evaluate_inds]
    
    return fitnesses

def saveSD(cf, gen, mainDir, population, halloffame):
    """
    Function that saves all the networks in POPULATION and HALLOFFAME every
    cf["evol"]["save_gen"] generations
    """
    if cf["evol"]["save"] and not gen%cf["evol"]["save_gen"]:
            try:
                genSave = mainDir + "/gen_#" + str(gen)
                os.mkdir(genSave)
                count1 = 0
                for net in population:
                    torch.save(net[0].state_dict(), genSave + "/IND_#" + str(count1) + ".pth")
                    count1 += 1
                if halloffame is not None:
                    count2 = 0
                    for net in halloffame:
                        torch.save(net[0].state_dict(), genSave + "/HOF_#" + str(count2) + ".pth")
                        count2 += 1
            except OSError:
                print ("Creation of the directory %s failed" % mainDir)

def saveNet(cf, gen, mainDir, population, halloffame):
    """
    Function that saves all the networks in POPULATION and HALLOFFAME every
    cf["evol"]["save_gen"] generations
    """
    if cf["evol"]["save"] and not gen%cf["evol"]["save_gen"]:
            try:
                genSave = mainDir + "/gen_#" + str(gen)
                os.mkdir(genSave)
                count1 = 0
                for net in population:
                    torch.save(net[0], genSave + "/IND_#" + str(count1) + ".pth")
                    count1 += 1
                if halloffame is not None:
                    count2 = 0
                    for net in halloffame:
                        torch.save(net[0], genSave + "/HOF_#" + str(count2) + ".pth")
                        count2 += 1
            except OSError:
                print ("Creation of the directory %s failed" % mainDir)

############################################################
# 1st evol algo: SIMPLE
############################################################
def varAnd(population, toolbox, mutpb):
    """Part of an evolutionary algorithm applying only the variation part
    (mutation). The modified individuals have their fitness invalidated. 
    The individuals are cloned so returned population is independent of the 
    input population.
    
    · Individuals in the input population are mutated only with a propability of mutpb
    """
    offspring = [toolbox.clone(ind) for ind in population]

    # Apply mutation
    for i in range(len(offspring)):
        if random.random() < mutpb:
            offspring[i], = toolbox.mutate(offspring[i])
            del offspring[i].fitness.values

    return offspring

def myeaSimple(population, toolbox, mutpb, ngen, mainDir, cf, stats=None,
             halloffame=None, verbose=__debug__):
    """
    evaluate(population)
    for g in range(ngen):
        population = select(population, len(population))
        offspring = varAnd(population, toolbox, mutpb)
        evaluate(offspring)
        population = offspring
    
    Evolution algorithm:
        1) Select fittest individuals in random tournament with repetition + return new offspring
        2) Clone all individuals (variation)
        3) Mutate them with a "mutpb" probability
        4) Assign an invalid fitness to the newly mutated individuals
        5) Evaluate inds with invalid fitness
        6) Update population
            ... repeat process
            
    - NOTE1: Necessarily needs random selection algo (such as selTournament or selRoulette)
    - NOTE2: The evaluation can be done with CUSTOM or RANDOM altitudes
    """
    # Show stats
    t0 = time.time()
    logbook = tools.Logbook()
    logbook.header = ['gen', 'nevals', 'h_ref', 'h_init', 'time', 'minT', 'NE'] + (stats.fields if stats else [])
    fitmin_array  = []
    gen_array     = []
    fitTest_array = []
    genTest_array = []

    # Evaluate initial population
    invalid_ind = [ind for ind in population if not ind.fitness.valid]
    fitnesses, h_refList, h_init = evaluateFitness(cf, toolbox, invalid_ind)
        
    for ind, fit in zip(invalid_ind, fitnesses):
        ind.fitness.values = fit

    # Update HOF
    if halloffame is not None:
        halloffame.update(population)

    # Show stats
    record = stats.compile(population) if stats else {}
    fitmin_array.append(round(record["min"],2))
    gen_array.append(0)
    tmp = round((time.time()-t0))
    logbook.record(gen=0, nevals=len(invalid_ind), h_ref=h_refList, h_init=h_init, minT="        ", NE=0, time = tmp, **record)
    if verbose:
        print(logbook.stream)

    # Creation of main dir to save individuals
    if cf["evol"]["save"]:
        try:
            os.mkdir(mainDir)
        except OSError:
            print ("Creation of the directory %s failed" % mainDir)

    # Begin the generational process
    for gen in range(1, ngen + 1):
        
        t0_gen = time.time()
        
        # 1) SELECTION: (selTournament)
        offspring = toolbox.select(population, len(population))

        # 2, 3, 4) MUTATION
        offspring = varAnd(offspring, toolbox, mutpb)

        # 5) EVALUATION: individuals with invalid fitness
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses, h_refList, h_init = evaluateFitness(cf, toolbox, invalid_ind)
        
        # Count how many indivs where directly discarded
        count = 0
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit
            if fit[0] > 4000:
                count += 1

        # Update HOF
        if halloffame is not None:
            halloffame.update(offspring)

        # 6) UPDATE POP
        population[:] = offspring
        
        # 7) EVALUATE ON TEST SET
        best_test = "        "
        if cf["test"]["active"] and not gen % cf["test"]["test_gen"]:
            args = [(ind, cf, cf["test"]["h_ref"], cf["test"]["h_init"]) for ind in population]
            test_fitnesses = toolbox.map(toolbox.evaluate, args)
            best_test = round(min(test_fitnesses)[0],2)
            genTest_array.append(gen)
            fitTest_array.append(best_test)
        
        # Save individuals
        saveNet(cf, gen, mainDir, population, halloffame)

        # Show stats
        record = stats.compile(population) if stats else {}
        fitmin_array.append(round(record["min"],2))
        gen_array.append(gen)
        plt.plot(gen_array, fitmin_array, 'C0*-')
        if cf["test"]["active"] and not gen % cf["test"]["test_gen"]:
            plt.plot(genTest_array,fitTest_array, 'C1*-')
        elif not cf["test"]["active"]:
            #plt.gca().legend(('MSE training',))
            pass
        #plt.gca().legend(('MSE training','MSE test',))
        plt.title("Min MSE per generation")
        plt.xlabel("# generation")
        plt.pause(0.01)
        t_gen   = round(time.time()-t0_gen)
        t_total = round(time.time()-t0)
        t = str(t_gen) + "/" + str(t_total)
        logbook.record(gen=gen, nevals=len(invalid_ind), h_ref=h_refList, h_init=h_init, minT=best_test, NE=count, time = t, **record)
        if verbose:
            print(logbook.stream)
    
        # Save plot + training statistics
        plt.legend(('MSE training','MSE test',))
        plt.savefig(mainDir + '_plot.png')
        df = pd.DataFrame(logbook)
        df.to_csv(mainDir + '_log.csv')
        
    plt.show()
    plt.close('all')
    return population, halloffame, logbook

def myeaNSGA2(population, toolbox, mu, mutpb, ngen, mainDir, cf, stats=None,
             halloffame=None, verbose=__debug__):
    """
    evaluate(population)
    for g in range(ngen):
        population = select(population, len(population))
        offspring = varAnd(population, toolbox, mutpb)
        evaluate(offspring)
        population = offspring
    
    Evolution algorithm:
        1) Select fittest individuals in random tournament with repetition + return new offspring
        2) Clone all individuals (variation)
        3) Mutate them with a "mutpb" probability
        4) Assign an invalid fitness to the newly mutated individuals
        5) Evaluate inds with invalid fitness
        6) Update population
            ... repeat process
            
    - NOTE1: Necessarily needs random selection algo (such as selTournament or selRoulette)
    - NOTE2: The evaluation can be done with CUSTOM or RANDOM altitudes
    """
    # Show stats
    t0 = time.time()
    logbook = tools.Logbook()
    logbook.header = ['gen', 'nevals', 'h_ref', 'h_init', 'time', 'minT', 'NE'] + (stats.fields if stats else [])
    fitmin_array  = []
    gen_array     = []
    fitTest_array = []
    genTest_array = []

    # Evaluate initial population
    invalid_ind = [ind for ind in population if not ind.fitness.valid]
    fitnesses, h_refList, h_init = evaluateFitness(cf, toolbox, invalid_ind)
        
    for ind, fit in zip(invalid_ind, fitnesses):
        ind.fitness.values = fit

    # To assign crowding distance to individuals (no actual selection is done)
    population = toolbox.select(population, len(population))

    # Update HOF
    if halloffame is not None:
        halloffame.update(population)

    # Show stats
    record = stats.compile(population) if stats else {}
    fitmin_array.append(record["min"])
    gen_array.append(0)
    tmp = round((time.time()-t0))
    logbook.record(gen=0, nevals=len(invalid_ind), h_ref=h_refList, h_init=h_init, minT="        ", NE=0, time = tmp, **record)
    if verbose:
        print(logbook.stream)

    # Creation of main dir to save individuals
    if cf["evol"]["save"]:
        try:
            os.mkdir(mainDir)
        except OSError:
            print ("Creation of the directory %s failed" % mainDir)

    # Begin the generational process
    for gen in range(1, ngen + 1):
        
        t0_gen = time.time()
        
        # 1) SELECTION: (selTournament)
        offspring = tools.selTournamentDCD(population, len(population))

        # 2, 3, 4) MUTATION
        offspring = varAnd(offspring, toolbox, mutpb)

        # 5) EVALUATION: individuals with invalid fitness
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses, h_refList, h_init = evaluateFitness(cf, toolbox, invalid_ind)
        
        # Count how many indivs where directly discarded
        count = 0
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit
            if fit[0] > 4000:
                count += 1

        # Update HOF
        if halloffame is not None:
            halloffame.update(offspring)

        # 6) UPDATE POP
        # population[:] = offspring
        population = toolbox.select(population + offspring, mu)
        
        # 7) EVALUATE ON TEST SET
        best_test = "        "
        if cf["test"]["active"] and not gen % cf["test"]["test_gen"]:
            args = [(ind, cf, cf["test"]["h_ref"], cf["test"]["h_init"]) for ind in population]
            test_fitnesses = toolbox.map(toolbox.evaluate, args)
            best_test = round(min(test_fitnesses)[0],2)
            genTest_array.append(gen)
            fitTest_array.append(best_test)
        
        # Save individuals
        saveNet(cf, gen, mainDir, population, halloffame)

        # Show stats
        record = stats.compile(population) if stats else {}
        fitmin_array.append(record["min"])
        gen_array.append(gen)
        plt.plot(gen_array, fitmin_array, 'C0*-')
        if cf["test"]["active"] and not gen % cf["test"]["test_gen"]:
            plt.plot(genTest_array,fitTest_array, 'C1*-')
        elif not cf["test"]["active"]:
            #plt.gca().legend(('MSE training',))
            pass
        #plt.gca().legend(('MSE training','MSE test',))
        plt.title("Min MSE per generation")
        plt.xlabel("# generation")
        plt.pause(0.01)
        t_gen   = round(time.time()-t0_gen)
        t_total = round(time.time()-t0)
        t = str(t_gen) + "/" + str(t_total)
        logbook.record(gen=gen, nevals=len(invalid_ind), h_ref=h_refList, h_init=h_init, minT=best_test, NE=count, time = t, **record)
        if verbose:
            print(logbook.stream)
    
        # Save plot + training statistics
        #plt.legend(('MSE training','MSE test',))
        plt.savefig(mainDir + '_plot.png')
        df = pd.DataFrame(logbook)
        df.to_csv(mainDir + '_log.csv')
        
    plt.show()
    plt.close('all')
    return population, halloffame, logbook

def myeaPID(population, toolbox, mutpb, ngen, mainDir, cf, stats=None,
             halloffame=None, verbose=__debug__):
    """
    evaluate(population)
    for g in range(ngen):
        population = select(population, len(population))
        offspring = varAnd(population, toolbox, mutpb)
        evaluate(offspring)
        population = offspring
    
    Evolution algorithm:
        1) Select fittest individuals in random tournament with repetition + return new offspring
        2) Clone all individuals (variation)
        3) Mutate them with a "mutpb" probability
        4) Assign an invalid fitness to the newly mutated individuals
        5) Evaluate inds with invalid fitness
        6) Update population
            ... repeat process
            
    - NOTE1: Necessarily needs random selection algo (such as selTournament or selRoulette)
    - NOTE2: The evaluation can be done with CUSTOM or RANDOM altitudes
    """
    # Show stats
    t0 = time.time()
    logbook = tools.Logbook()
    logbook.header = ['gen', 'nevals', 'time', 'NE'] + (stats.fields if stats else [])
    fitmin_array  = []
    gen_array     = []
    fitTest_array = []
    genTest_array = []

    # Evaluate initial population
    invalid_ind = [ind for ind in population if not ind.fitness.valid]
    fitnesses = evaluate_PID_SNN(cf, toolbox, invalid_ind)
        
    for ind, fit in zip(invalid_ind, fitnesses):
        ind.fitness.values = fit

    # Update HOF
    if halloffame is not None:
        halloffame.update(population)

    # Show stats
    record = stats.compile(population) if stats else {}
    fitmin_array.append(round(record["min"],2))
    gen_array.append(0)
    tmp = round((time.time()-t0))
    logbook.record(gen=0, nevals=len(invalid_ind), time = tmp, **record)
    if verbose:
        print(logbook.stream)

    # Creation of main dir to save individuals
    if cf["evol"]["save"]:
        try:
            os.mkdir(mainDir)
        except OSError:
            print ("Creation of the directory %s failed" % mainDir)

    # Begin the generational process
    for gen in range(1, ngen + 1):
        
        t0_gen = time.time()
        
        # 1) SELECTION: (selTournament)
        offspring = toolbox.select(population, len(population))

        # 2, 3, 4) MUTATION
        offspring = varAnd(offspring, toolbox, mutpb)

        # 5) EVALUATION: individuals with invalid fitness
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = evaluate_PID_SNN(cf, toolbox, invalid_ind)
        
        count = 0
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit
            if fit[0] > 4000:
                count += 1

        # Update HOF
        if halloffame is not None:
            halloffame.update(offspring)

        # 6) UPDATE POP
        population[:] = offspring
        
        # Save individuals
        saveNet(cf, gen, mainDir, population, halloffame)

        # Show stats
        record = stats.compile(population) if stats else {}
        fitmin_array.append(round(record["min"],2))
        gen_array.append(gen)
        plt.plot(gen_array, fitmin_array, 'C0*-')
        #plt.plot(0,0,'C1*-')
        #plt.gca().legend(('MSE training','MSE test',))
        plt.title("Min MSE per generation")
        plt.xlabel("# generation")
        plt.pause(0.01)
        t_gen   = round(time.time()-t0_gen)
        t_total = round(time.time()-t0)
        t = str(t_gen) + "/" + str(t_total)
        logbook.record(gen=gen, nevals=len(invalid_ind), time = t, **record)
        if verbose:
            print(logbook.stream)
    
        # Save plot + training statistics
        plt.legend(('MSE training','MSE test',))
        plt.savefig(mainDir + '_plot.png')
        df = pd.DataFrame(logbook)
        df.to_csv(mainDir + '_log.csv')
        
    plt.show()
    plt.close('all')
    return population, halloffame, logbook

############################################################
# 2nd evol algo: JULIEN PAPER
############################################################
def varJPaper(population, toolbox):
    """Part of an evolutionary algorithm applying only the variation part
    (mutation). The modified individuals have their fitness invalidated. 
    The individuals are cloned so returned population is independent of the 
    input population.
    
    · Individuals in the input population are ALWAYS mutated
    """
    offspring = [toolbox.clone(ind) for ind in population]

    # Apply mutation
    for i in range(len(offspring)):
        offspring[i], = toolbox.mutate(offspring[i])
        del offspring[i].fitness.values

    return offspring

def myeaJPaper(population, toolbox, mutpb, ngen, mainDir, cf, stats=None,
             halloffame=None, verbose=__debug__):
    """
    evaluate(population)
    for g in range(ngen):
        offspring = selBest(population, len(population)/2)
        offspring = varJPaper(offspring, toolbox)
        evaluate(offspring + population)
        offspring = selBest(population, len(population))
        population = offspring
    
    Evolution algorithm:
        1) Select best half of individuals from population with len(pop)/2 -> offBest
        2) Clone all individuals (variation) from offBest
        3) Mutate all individuals from offBest
        4) Assign an invalid fitness to the newly mutated individuals offBest + the whole population
        5) Evaluate offBest individuals + Reevaluate population (all)
        6) Select best individuals from offBest + population with len(pop)
        6) Update population
            ... repeat process
    
    - NOTE: The evaluation can be done with CUSTOM or RANDOM altitudes
    """
    # Show stats
    t0 = time.time()
    logbook = tools.Logbook()
    logbook.header = ['gen', 'nevals', 'h_ref', 'h_init', 'time', 'minT', 'NE'] + (stats.fields if stats else [])
    fitmin_array  = []
    gen_array     = []
    fitTest_array = []
    genTest_array = []
    
    # Evaluate initial population
    # invalid_ind = [ind for ind in population if not ind.fitness.valid]
    fitnesses, h_refList, h_init = evaluateFitness(cf, toolbox, population)
        
    for ind, fit in zip(population, fitnesses):
        ind.fitness.values = fit

    # Update HOF
    if halloffame is not None:
        halloffame.update(population)

    # Show stats
    record = stats.compile(population) if stats else {}
    fitmin_array.append(round(record["min"],2))
    gen_array.append(0)
    tmp = round((time.time()-t0))
    logbook.record(gen=0, nevals=len(population), h_ref=h_refList, h_init=h_init, minT="        ", NE=0, time = tmp, **record)
    if verbose:
        print(logbook.stream)

    # Creation of main dir to save individuals
    if cf["evol"]["save"]:
        try:
            os.mkdir(mainDir)
        except OSError:
            print ("Creation of the directory %s failed" % mainDir)

    # Begin the generational process
    for gen in range(1, ngen + 1):
        
        t0_gen = time.time()
        
        # 1) SELECTION: (selBest)
        offspring = toolbox.select(population, int(len(population)/2))

        # 2, 3, 4) MUTATION
        offspring = varJPaper(offspring, toolbox)
        for ind in population:
            del ind.fitness.values
            offspring.append(ind)    # len(offspring) = 1.5 * len(population)

        # 5) EVALUATION: Evaluate offspring + population
        fitnesses, h_refList, h_init = evaluateFitness(cf, toolbox, offspring)      # population here??
    
        nevals = len(offspring)    
    
        count = 0
        for ind, fit in zip(offspring, fitnesses):
            ind.fitness.values = fit
            if fit[0] > 4000:
                count += 1

        # Update HOF
        if halloffame is not None:
            halloffame.update(offspring)

        # 6) UPDATE POP: Replace the current population by the offspring
        offspring = toolbox.select(offspring, len(population))
        population[:] = offspring
        
        # 7) EVALUATE ON TEST SET
        best_test = "        "
        if cf["test"]["active"] and not gen % cf["test"]["test_gen"]:
            args = [(ind, cf, cf["test"]["h_ref"], cf["test"]["h_init"]) for ind in population]
            test_fitnesses = toolbox.map(toolbox.evaluate, args)
            best_test = round(min(test_fitnesses)[0],2)
            genTest_array.append(gen)
            fitTest_array.append(best_test)
        
        # Save individuals
        saveNet(cf, gen, mainDir, population, halloffame)
                
        # Show stats
        record = stats.compile(population) if stats else {}
        fitmin_array.append(round(record["min"],2))
        gen_array.append(gen)
        plt.plot(gen_array, fitmin_array, 'C0*-')
        #plt.plot(0,0,'C1*-')
        if cf["test"]["active"] and not gen % cf["test"]["test_gen"]:
            plt.plot(genTest_array,fitTest_array, 'C1*-')
        elif not cf["test"]["active"]:
            #plt.gca().legend(('MSE training',))
            pass
        #plt.gca().legend(('MSE training','MSE test',))
        plt.title("Min MSE per generation")
        plt.xlabel("# generation")
        plt.pause(0.01)
        t_gen   = round(time.time()-t0_gen)
        t_total = round(time.time()-t0)
        t = str(t_gen) + "/" + str(t_total)
        logbook.record(gen=gen, nevals=nevals, h_ref=h_refList, h_init=h_init, minT=best_test, NE=count, time = t, **record)
        if verbose:
            print(logbook.stream)
            
        # Save plot + training statistics
        plt.legend(('MSE training','MSE test',))
        plt.savefig(mainDir + '_plot.png')
        df = pd.DataFrame(logbook)
        df.to_csv(mainDir + '_log.csv')
        
    plt.show()
    plt.close('all')

    return population, halloffame, logbook

############################################################
# 3rd evol algo: DEAP 
############################################################
def varOr(population, toolbox, lambda_, mutpb):
    """
    ...
    """

    offspring = []
    for _ in range(lambda_):
        op_choice = random.random()
        if op_choice < mutpb:  # Apply mutation
            ind = toolbox.clone(random.choice(population))
            ind, = toolbox.mutate(ind)
            del ind.fitness.values
            offspring.append(ind)
        else:                  # Apply reproduction
            offspring.append(random.choice(population))

    return offspring

def myeaMuPlusLambda(population, toolbox, mu, lambda_, mutpb, ngen, mainDir, cf,  
                   stats=None, halloffame=None, verbose=__debug__):
    """
    evaluate(population)
    for g in range(ngen):
        offspring = varOr(population, toolbox, lambda_, cxpb, mutpb)
        evaluate(offspring)
        population = select(population + offspring, mu)
        
    Evolution algorithm:
        1) Select fittest individuals in random tournament with repetition + return new offspring
        2) Clone all individuals (variation)
        3) Mutate them with a "mutpb" probability
        4) Assign an invalid fitness to the newly mutated individuals
        5) Evaluate inds with invalid fitness
        6) Update population
            ... repeat process
    """
    
    logbook = tools.Logbook()
    logbook.header = ['gen', 'nevals'] + (stats.fields if stats else [])

    # Evaluate the individuals with an invalid fitness
    invalid_ind = [ind for ind in population if not ind.fitness.valid]
    fitnesses = toolbox.map(toolbox.evaluate, invalid_ind)
    for ind, fit in zip(invalid_ind, fitnesses):
        ind.fitness.values = fit

    if halloffame is not None:
        halloffame.update(population)

    record = stats.compile(population) if stats is not None else {}
    logbook.record(gen=0, nevals=len(invalid_ind), **record)
    if verbose:
        print(logbook.stream)

    # Creation of main directory to save certain individuals
    if cf["evol"]["save"]:
        try:
            os.mkdir(mainDir)
        except OSError:
            print ("Creation of the directory %s failed" % mainDir)

    # Begin the generational process
    for gen in range(1, ngen + 1):
        # Vary the population
        
        # Add selBest here?
        
        offspring = varOr(population, toolbox, lambda_, mutpb)

        # Evaluate the individuals with an invalid fitness
        invalid_ind = [ind for ind in offspring if not ind.fitness.valid]
        fitnesses = toolbox.map(toolbox.evaluate, invalid_ind)
        for ind, fit in zip(invalid_ind, fitnesses):
            ind.fitness.values = fit

        # Update the hall of fame with the generated individuals
        if halloffame is not None:
            halloffame.update(offspring)

        # Select the next generation population
        population[:] = toolbox.select(population + offspring, mu)

        # Save individuals every conf.SAVE_GEN generations
        if cf["evol"]["save"] and not gen%cf["evol"]["save_gen"]:
            try:
                genSave = mainDir + "/gen_#" + str(gen)
                os.mkdir(genSave)
                count1 = 0
                for net in population:
                    torch.save(net[0], genSave + "/IND_#" + str(count1) + ".pth")
                    count1 += 1
                if halloffame is not None:
                    count2 = 0
                    for net in halloffame:
                        torch.save(net[0], genSave + "/HOF_#" + str(count2) + ".pth")
                        count2 += 1
            except OSError:
                print ("Creation of the directory %s failed" % mainDir)

        # Update the statistics with the new population
        record = stats.compile(population) if stats is not None else {}
        logbook.record(gen=gen, nevals=len(invalid_ind), **record)
        if verbose:
            print(logbook.stream)

    return population, halloffame, logbook