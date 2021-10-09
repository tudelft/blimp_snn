#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 26 11:19:52 2021

@author: marina
"""

# Set absolute package path
import sys, os
sys.path.append(os.path.abspath(".."))

import copy
import matplotlib.pyplot as plt
import random
import numpy as np
from itertools import repeat

#########################################################
# Plot functions
#########################################################
def saveSpikes(spikes, traces, dicPlot):
    
    for i in len(spikes):
        dicPlot["spikes"][i].append(copy.deepcopy(spikes[i].cpu().view(-1).numpy().transpose()))
        dicPlot["traces"][i].append(copy.deepcopy(traces[i].cpu().view(-1).numpy().transpose()))
    
    return dicPlot

def getSpikingNeuron(spikes):
    
    x = []
    y = []
    
    count = 0
    for spike in spikes:
        
        index_sNeurons = np.argwhere(spike==True)
        no_sNeurons = len(index_sNeurons)
        
        if no_sNeurons != 0:
            x.extend(repeat(count,no_sNeurons))
            y.extend(index_sNeurons.tolist())
        count += 1
    
        assert len(x) == len(y)
    
    y = np.array(y)
    y = y.flatten()
    
    x_plot= []
    y_plot = []
    
    for neuron in range(len(spike)):
        
        tmp = np.argwhere(y==neuron)
        if len(tmp) != 0:
            x_tmp = np.array([x[i.item()] for i in tmp])
        else:
            x_tmp = []
        
        x_plot.append(x_tmp)
        y_plot.append(np.ones(len(x_tmp)) * (neuron+1))  
        
    x = np.array([x,x])
    y = np.array([y, y + 1])    
    
    return x,y,x_plot,y_plot

def plotSpikes(spikes, axs, xticks, num, xlim, string):
    neuron_lims = len(spikes[0])
    x_points, spikes, x_plot, y_plot = getSpikingNeuron(spikes)
    
    for i in range(neuron_lims):
        if len(x_plot[i]) != 0:
            if i < 10:
                color = "C" + str(i)
            else:
                color = "C" + str(random.randint(0,9))
            axs[num][0].stem(x_plot[i], y_plot[i], linefmt = color, markerfmt = " ", basefmt = " ", bottom=i, use_line_collection=True)
    
    if len(x_points[0]) != 0:
        for i in range(neuron_lims+1):
            axs[num][0].plot([x_points[0][0],x_points[0][-1]], [i, i], 'k--', linewidth=0.75)
    
    axs[num][0].set_xlim(xlim)
    axs[num][0].set_ylim([0, neuron_lims])
    if not xticks:
        axs[num][0].set_xticklabels([])
    #axs[num][0].set_title(string)
    axs[num][0].set_ylabel(str(neuron_lims) + " neurons\n" + string)
    axs[num][0].grid()

def plotNetwork(dc):
    
    n_plots = len(dc["spikes"])
    fig, axs = plt.subplots(int(n_plots)+1, 2)
    fig.suptitle('Firing network snapshot', fontsize=16)
    
    xlim = [0,len(dc["error"])]
    #xlim = [2499.999,2500.0001]
    
    # Error plot
    axs[0][0].plot(dc["error"])
    axs[0][0].set_title("Error: $e = h_{ref} - h_{current}$")
    axs[0][0].set_xlim(xlim)
    axs[0][0].set_xticklabels([])
    axs[0][0].grid()  
    
    # u_array plot
    axs[0][1].plot(dc["u"])
    axs[0][1].set_title("Output: $u$")
    axs[0][1].set_xlim(xlim)
    axs[0][1].set_xticklabels([])
    axs[0][1].grid()
    
    # spikes plot
    axs[1][0].set_title("Spikes")
    for i in range(n_plots):
        xticks_show = False
        if i == n_plots-1:
            xticks_show = True
        plotSpikes(dc["spikes"][i], axs, xticks_show, i+1, xlim, "L" + str(i+1) + " Spikes")
    
    # traces
    axs[1][1].set_title("Traces")
    for j in range(n_plots):
        axs[j+1][1].plot(dc["traces"][j])
        axs[j+1][1].set_xlim(xlim)
        if j != n_plots-1:
            axs[j+1][1].set_xticklabels([])
        axs[j+1][1].set_ylabel("L" + str(j+1) + " Trace")
        axs[j+1][1].grid()

def plotANN(dc):
    n_plots = len(dc["values"])
    fig, axs = plt.subplots(int(n_plots)+1)
    fig.suptitle('Network snapshot', fontsize=16)
    
    xlim = [0,len(dc["error"])]
    #xlim = [2499.999,2500.0001]
    
    # Error plot
    axs[0].plot(dc["r"], '-')
    axs[0].plot(dc["h_ref"], '--')
    axs[0].plot(dc["error"], '-.', linewidth=2.5)
    axs[0].legend(['$h_{current}$[m]','$h_{ref}$ [m]', 'Error [m]'])
    axs[0].set_title("Error (ANN input): $e = h_{ref} - h_{current}$")
    axs[0].set_xlim(xlim)
    axs[0].set_xticklabels([])
    axs[0].grid()  
    
    # Neuron outputs plot
    for i in range(n_plots):
        axs[i+1].plot(dc["values"][i])
        axs[i+1].set_xlim(xlim)
        if i != n_plots-1:
            axs[i+1].set_xticklabels([])
        axs[i+1].grid()
        axs[i+1].set_ylabel(str(len(dc["values"][i][0])) + " neuron(s)")
        axs[i+1].set_title("Layer #" + str(i))

def plotBlimp(dic):
    fig, ax = plt.subplots(2)
    ax[0].plot(dic["r"], '-', label = "Range")
    ax[0].plot(dic["h_ref"], '--')
    ax[0].plot(dic["error"], '-.')
    ax[0].legend(['Current Height [m]','Reference Height [m]', 'Error [m]'])
    ax[0].grid()
    
    ax[1].plot(dic["u"], '-')
    #ax[1].plot(dic["u_snn"], '-')
    #ax[1].plot(dic["u_pid"], '-')
    #ax[1].set_xlabel('Time [sec]')
    ax[1].set_ylabel('Motor command: u')
    ax[1].grid()
    fig.tight_layout()
    plt.show()
    
def plotBlimp_PID(dic, dt):
    fig, ax = plt.subplots(2)
    
    t_array = np.arange(len(dic["r"])) * dt
    
    ax[0].plot(t_array, dic["r"], '-', label = "Range")
    ax[0].plot(t_array, dic["h_ref"], '--')
    ax[0].plot(t_array, dic["error"], '-.')
    ax[0].legend(['Current Height [m]','Reference Height [m]', 'Error [m]'])
    ax[0].grid()
    
    ax[1].plot(t_array, dic["u"], '-')
    #ax[1].set_xlabel('Time [sec]')
    ax[1].set_ylabel('Motor command: u')
    ax[1].grid()
    plt.show()
    
def plot_Blimp_NNandPID(dic):
    fig, ax = plt.subplots(2)
    ax[0].plot(dic["r"], '-', label = "Range")
    ax[0].plot(dic["h_ref"], '--')
    ax[0].plot(dic["error"], '-.')
    ax[0].legend(['Current Height [m]','Reference Height [m]', 'Error [m]'])
    ax[0].grid()
    
    ax[1].plot(dic["u"], '-')
    ax[1].plot(dic["u_snn"], '-')
    ax[1].plot(dic["u_pid"], '-')
    #ax[1].set_xlabel('Time [sec]')
    ax[1].set_ylabel('Motor command: u')
    ax[1].grid()
    fig.tight_layout()
    plt.show()

def plotNetwork_shitty(error_array, i_spikes, h_spikes, o_spikes, i_traces, h_traces, o_traces, u_array):

    fig, axs = plt.subplots(8, 1)
    count = 0
    titles_list = [
        "Error (input): e = h_ref - h_current",
        "1st layer: placeholders (encoding)",
        "2nd layer: post-synaptic spikes",
        "3rd layer: post-synaptic spikes",
        "1st layer: trace",
        "2nd layer: trace",
        "3rd layer: trace",
        "Motor commands (output): u"
        ]
    
    for ax, data in zip(
        axs,
        [
            error_array,
            i_spikes,
            h_spikes,
            o_spikes,
            i_traces,
            h_traces,
            o_traces,
            u_array
        ],
    ):
        ax.plot(data)
        ax.set_title(titles_list[count])
        count += 1
    #fig.tight_layout()
    plt.show()
