#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Apr  9 11:00:52 2021

@author: marina

"snn_ons.py" comes from SNN Output Non-Spiking of type Input

-> Requirements for the config.yaml file:
    The last two layers should have the same nunmber of neurons
"""

# Set absolute package path
import sys, os
sys.path.append(os.path.abspath(".."))

import torch

from pysnn.network import SNNNetwork
from pysnn.neuron import Input, LIFNeuron
from pysnn.connection import Linear
import numpy as np
import copy
import scipy.interpolate as sc
import matplotlib.pyplot as plt
import random
import torch.nn as nn

import extra.aux_funcs as af # (:

#########################################################
# Network
#########################################################
class MySNN(SNNNetwork):
    def __init__(self, i_dynamics, n_dynamics, c_dynamics, en_dynamics, q, cf):
        """
        Default Constructor
        """
        super(MySNN, self).__init__()
        
        # Extra params
        self.cf = cf
        self.heights = []
        self.perc_lim = cf["lim"]["var"]
        
        # Layers creation
        self.neuron    = nn.ModuleList()
        self.linear    = nn.ModuleList()
        
        self.neuron.append(Input((cf["batch_size"], 1, cf["n_layers"][0]), *i_dynamics))
        
        for i in range(0,len(cf["n_layers"])-2):
            self.neuron.append(LIFNeuron((cf["batch_size"], 1, cf["n_layers"][i+1]), *n_dynamics))
            self.linear.append(Linear(cf["n_layers"][i], cf["n_layers"][i+1], *c_dynamics))
            self.add_layer("fc{0}".format(i), self.linear[i], self.neuron[i+1])
            
        self.neuron.append(Input((cf["batch_size"], 1, cf["n_layers"][-1]), *i_dynamics))
        self.linear.append(Linear(cf["n_layers"][-2], cf["n_layers"][-1], *c_dynamics))
        #self.add_layer("fcN", self.linear[-1], self.neuron[-1])
        
        # Initialize weights based on CLAMP interval   ¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿¿???????????????????s
        for j in range(0,len(cf["n_layers"])-1):
            self.linear[j].reset_weights(a = self.cf["clamp"]["weights"][0], b = self.cf["clamp"]["weights"][1])
        
        # Encoding
        if cf["enc"]["gaussian"]:
            self.inter = self._gaussian_sampling(*en_dynamics)
        else:
            self.inter = np.linspace(cf["enc"]["xmin"], cf["enc"]["xmax"], cf["n_layers"][0]-1)
        
        # Decoding
        self.q = q
        self.w_average = torch.zeros(cf["batch_size"],1,cf["n_layers"][-1])
        self.w_average[0,0,:] = torch.linspace(-q,q,cf["n_layers"][-1])
        self.w_average = self.w_average.to(cf["device"])
        
        self.dec_pop = torch.linspace(-cf["dec"]["u_pop"], cf["dec"]["u_pop"], cf["n_layers"][-1])
        self.u_pop = 0.0
        self.non_spike = 0.0
    
    def update_params(self):
        
        # Probability of mutating a parameter
        p = self.cf["evol"]["p_mut_param"]
        
        # Apply mutation decay
        self.perc_lim = self.perc_lim * self.cf["lim"]["decay"]
        
        self.cf["lim"]["thresh"]  = [-self.cf["lim"]["thresh"][1] *self.perc_lim,self.cf["lim"]["thresh"][1] *self.perc_lim]
        self.cf["lim"]["tau_t"]   = [-self.cf["lim"]["tau_t"][1]  *self.perc_lim,self.cf["lim"]["tau_t"][1]  *self.perc_lim]
        self.cf["lim"]["tau_v"]   = [-self.cf["lim"]["tau_v"][1]  *self.perc_lim,self.cf["lim"]["tau_v"][1]  *self.perc_lim]
        self.cf["lim"]["alpha_t"] = [-self.cf["lim"]["alpha_t"][1]*self.perc_lim,self.cf["lim"]["alpha_t"][1]*self.perc_lim]
        self.cf["lim"]["alpha_v"] = [-self.cf["lim"]["alpha_v"][1]*self.perc_lim,self.cf["lim"]["alpha_v"][1]*self.perc_lim]
        self.cf["lim"]["weights"] = [-self.cf["lim"]["weights"][1]*self.perc_lim,self.cf["lim"]["weights"][1]*self.perc_lim]
        self.cf["lim"]["q"]       = [-self.cf["lim"]["q"][1]      *self.perc_lim,self.cf["lim"]["q"][1]      *self.perc_lim]    
        
        # Input layer mutation
        inputGenes = self.cf["evol"]["paramInput"] #["tau_t", "alpha_t"]
        neuron_list = [0, -1]
        for neuron in neuron_list:
            for gene in inputGenes:
                param = getattr(self.neuron[neuron],gene)
                param.data += af.randomTensor(*self.cf["lim"][gene], self.cf["n_layers"][neuron]) * (torch.rand_like(param.data) < p)
                param.data = torch.clamp(param.data, *self.cf["clamp"][gene])
        
        # Rest of layers mutation
        netGenes = self.cf["evol"]["paramLIF"] #["thresh", "tau_v", "tau_t", "alpha_v", "alpha_t"]
        for i in range(1,len(self.cf["n_layers"])-1):
            # Weights mutation
            weight = getattr(self.linear[i-1],"weight")
            weight.data += af.randomTensorW(*self.cf["lim"]["weights"], self.cf["n_layers"][i], self.cf["n_layers"][i-1]) * (torch.rand_like(weight.data) < p)
            weight.data = torch.clamp(weight.data, *self.cf["clamp"]["weights"])
            # Parameters mutation
            for gene in netGenes:
                param = getattr(self.neuron[i],gene)
                param.data += af.randomTensor(*self.cf["lim"][gene], self.cf["n_layers"][i]) * (torch.rand_like(param.data) < p)
                param.data = torch.clamp(param.data, *self.cf["clamp"][gene])
        
        # Update "thresh_center" based on "thresh" mutations
        for i in range(1,len(self.cf["n_layers"])-1):
            param = getattr(self.neuron[i],"thresh_center")
            param.data = torch.mean(self.neuron[i].thresh)
        
        # Decoding mutation
        if self.cf["evol"]["q_mut"] and random.random() < self.cf["evol"]["p_mut_param"]:
            self.q = random.uniform(*self.cf["clamp"]["q"])
            self.w_average[0,0,:] = torch.linspace(-self.q,self.q,self.cf["n_layers"][-1])
            self.w_average = self.w_average.to(self.cf["device"])
        
        #self.linear1.reset_weights(a = self.cf["clamp"]["weights"][0], b = self.cf["clamp"]["weights"][1])
        #self.linear2.reset_weights(a = self.cf["clamp"]["weights"][0], b = self.cf["clamp"]["weights"][1])
        
    def reset_state(self):
        self.neuron[0].reset_state()
        for i in range(1,len(self.cf["n_layers"])):
              self.neuron[i].reset_state()
              self.linear[i-1].reset_state()
        
    def forward(self, x):
        """
        Make predictions
        """
        
        # Encoding
        x = self._encode(x)
        x = x.to(self.cf["device"])
        
        spike, trace = self.neuron[0](x)
        spikes = [spike]
        traces = [trace]
        
        for i in range(1,len(self.cf["n_layers"])-1):
            x, _ = self.linear[i-1](spike, trace)
            spike, trace = self.neuron[i](x)
            spikes.append(spike)
            traces.append(trace)
        
        spike, trace = self.neuron[i](spike)
        spikes.append(spike)
        traces.append(trace)
        
        # Decoding
        out_real = self._decode(trace)
        #out_real = self._decode_pop(spike)
        
        return spikes, traces, out_real
        
    def _gaussian_sampling(self, xmin, xmax, nsamples, sigma, plot):
        """
        Generate the gaussian distribution intervals for the encoding
        """
        # Compute mean
        mu = (xmax+xmin)/2.0
        
        # Get samples
        np.random.seed(0) # Take always same random seed
        x  = np.random.normal(mu,sigma,int(nsamples)*1000)
        
        # Truncate normal
        x  = x[x>=xmin]
        x  = x[x<=xmax]
        
        # Get nsamples
        x  = np.sort(x)
        indexes = np.rint(np.linspace(0,len(x)-1,nsamples)).astype(int)
        x = x[indexes] * 1
        
        if plot:
            plt.plot(xmin*np.ones(len(x)),np.linspace(-1,1,len(x)),'k--')
            plt.plot(xmax*np.ones(len(x)),np.linspace(-1,1,len(x)),'k--')
            plt.plot(x,np.zeros(len(x)),'bo',zorder = 20,alpha=0.9)
            plt.ylim(-0.1,0.1)
            plt.yticks([])
    
        return x
    
    def _encode(self, x):
        """
        Encode real numbers into spikes based on the gaussian intervals
        """
        count = 0
        if x <= self.inter[0]:
            pass
        elif x >= self.inter[-1]:
            count = len(self.inter)
            pass
        else:
            for first, second in zip(self.inter, self.inter[1:]):
                count += 1
                if x > first and x < second:
                    break  
                 
        #x = torch.zeros(self.cf["n_in"],1)
        #x[count] = 1.0
        x = torch.zeros(self.cf["batch_size"],1,self.cf["n_layers"][0])
        x[0,0,count] = 1.0
        return x
    
    def _decode(self, trace):
        
        numerator = torch.dot(self.w_average[0,0,:], trace[0,0,:])
        denominator = torch.sum(trace)
    
        if round(denominator.item(),4) == 0:
            result = 0.0
            return result
        else:
            result = numerator/denominator
        #result = denominator
        #print(result)
        return result.item()
    
    def _decode_pop(self, spike):
        
        n_out = len(spike.view(-1))
        assert n_out == self.cf["n_layers"][-1]
        active_neuron = np.where(spike.view(-1) != False)[0]
        # If only one neuron spikes
        if len(active_neuron) == 1:
            self.u_pop = self.dec_pop[active_neuron.item()].item()
            return self.u_pop
        # If non spikes
        elif len(active_neuron) == 0:
            self.non_spike += 1.0
            if self.non_spike > 25.0:
                self.non_spike = 0.0
                self.u_pop = 0.0
            return self.u_pop
        # If several spike
        else:
            #u_mean = 0.0
            #for i in active_neuron:
            #    u_mean += self.dec_pop[i].item()
            #self.u_pop = u_mean/len(active_neuron)
            return self.u_pop
    
    
    
    
    
    
    
    
    
    
    
    