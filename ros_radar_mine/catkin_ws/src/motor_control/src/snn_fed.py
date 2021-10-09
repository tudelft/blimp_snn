#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Apr 12 20:22:31 2021

@author: marina

"snn_fed.py" 

-> Requirements for the config.yaml file:
    At at least 4 layers:
        1) Input
        2) LIF
        3) ANN
        4) Only 1 ANN for the decoding
"""

# Set absolute package path
import sys, os
sys.path.append(os.path.abspath(".."))

import torch

from network import SNNNetwork
from neuron import Input, LIFNeuron
from connection import Linear
import numpy as np
import copy
import matplotlib.pyplot as plt
import random
import torch.nn as nn

#import extra.aux_funcs as af # (:


LAYERS = [10,5,5,1]
ENCODING = [-0.4,0.4] #xmin, xmax
DEC_POP = 4

#########################################################
# Network
#########################################################
class MySNN(SNNNetwork):
    def __init__(self, i_dynamics, n_dynamics, c_dynamics, en_dynamics, q):
        """
        Default Constructor
        """
        super(MySNN, self).__init__()
        
        # Extra params
        self.LAYERS = LAYERS
        self.ENCODING = ENCODING
        self.DECODING = DEC_POP

        # Layers creation
        self.neuron    = nn.ModuleList()
        self.linear    = nn.ModuleList()
        
        # Initial layer of Input neruons
        self.neuron.append(Input((1, 1, LAYERS[0]), *i_dynamics))
        
        # Fill in with layers of LIF neurons
        for i in range(0,len(LAYERS)-3):
            self.neuron.append(LIFNeuron((1, 1, LAYERS[i+1]), *n_dynamics))
            self.linear.append(Linear(LAYERS[i], LAYERS[i+1], *c_dynamics))
            self.add_layer("fc{0}".format(i), self.linear[i], self.neuron[i+1])
            
        # Add two final layers of ANN neurons for the decoding
        self.linear.append(nn.Linear(LAYERS[-3], LAYERS[-2]))
        for param in self.linear[-1].parameters():
                param.requires_grad = False
        self.linear.append(nn.Linear(LAYERS[-2], LAYERS[-1]))
        for param in self.linear[-1].parameters():
                param.requires_grad = False
                
        # Initialize weights based on CLAMP interval
        #for j in range(0,len(LAYERS)-3):
        #    self.linear[j].reset_weights(a = self.cf["clamp"]["weights"][0], b = self.cf["clamp"]["weights"][1])
        
        # Encoding
        self.inter = np.linspace(ENCODING[0], ENCODING[1], LAYERS[0]-1)
        
        # Decoding
        self.q = q
        self.w_average = torch.zeros(1,1,LAYERS[-1])
        self.w_average[0,0,:] = torch.linspace(-q,q,LAYERS[-1])
        #self.w_average = self.w_average.to(cf["device"])
        
        self.dec_pop = torch.linspace(-DEC_POP, DEC_POP, LAYERS[-1])
        self.u_pop = 0.0
        self.non_spike = 0.0
        
    def reset_state(self):
        
        """
        Do not reset ANN: therefore the -2. Change if the structure changes!!
        """
        
        self.neuron[0].reset_state()
        for i in range(1,len(self.LAYERS)-2):
              self.neuron[i].reset_state()
              self.linear[i-1].reset_state()
        
    def forward(self, x):
        """
        Make predictions
        """
        
        # Encoding
        x = self._encode(x)
        #x = x.to(self.cf["device"])
        
        spike, trace = self.neuron[0](x)
        spikes = [spike]
        traces = [trace]
        
        L = len(self.LAYERS)
        
        for i in range(1,L):
            if i < L-2:
                x, _ = self.linear[i-1](spike, trace)
                spike, trace = self.neuron[i](x)
                spikes.append(spike)
                traces.append(trace)
            elif i == L-2:
                x = torch.tanh(self.linear[i-1](spike.float()))  # Set to either trace or spike.float()
                spikes.append(torch.zeros([1,1,self.LAYERS[i]]))#.bool()
                traces.append(x)
            elif i == L-1:
                out_real = self.linear[i-1](x)
                spikes.append(torch.zeros([1,1,self.LAYERS[i]]))#.bool()
                traces.append(out_real)
        
        # Decoding
        #out_real = self._decode(trace)
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
        x = torch.zeros(1,1,self.LAYERS[0])
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
        assert n_out == self.LAYERS[-1]
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
    
    
    
    
    
    
    
    
    
    
    
    