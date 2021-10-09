#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Mar 26 07:56:52 2021

@author: marina
"""

# Set absolute package path
#import sys, os
#sys.path.append(os.path.abspath(".."))

import torch

from network import SNNNetwork
from neuron import Input, LIFNeuron
from connection import Linear
import numpy as np
import matplotlib.pyplot as plt
import torch.nn as nn


LAYERS = [10,3]
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
        self.heights = []
        
        # Layers creation
        self.neuron    = nn.ModuleList()
        self.linear    = nn.ModuleList()
        
        self.neuron.append(Input((1, 1, LAYERS[0]), *i_dynamics))
        for i in range(0,len(LAYERS)-1):
            self.neuron.append(LIFNeuron((1, 1, LAYERS[i+1]), *n_dynamics))
            self.linear.append(Linear(LAYERS[i], LAYERS[i+1], *c_dynamics))
            self.add_layer("fc{0}".format(i), self.linear[i], self.neuron[i+1])
        
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
        self.neuron[0].reset_state()
        for i in range(1,len(self.LAYERS)):
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
        
        for i in range(1,len(self.LAYERS)):
            x, _ = self.linear[i-1](spike, trace)
            spike, trace = self.neuron[i](x)
            spikes.append(spike)
            traces.append(trace)
        
        # Decoding
        out_real = self._decode(trace)
        #out_real = self._decode_pop(spike)
        
        return spikes, traces, out_real
    
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
    
    
    
    
    
    
    
    
    
    
    
    