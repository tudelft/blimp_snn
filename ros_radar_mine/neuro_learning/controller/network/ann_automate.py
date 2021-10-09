#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar 30 10:34:56 2021

@author: marina
"""

# Set absolute package path
import sys, os
sys.path.append(os.path.abspath(".."))

import torch
from torch import nn, optim
import torch.nn.functional as F
import os
import extra.aux_funcs as af
import numpy as np
import random

############################################################
# CONFIG GLOBAL VARIABLE
############################################################
cf = af.set_config('../config/config.yaml')

class MyANN(nn.Module):
    
    def __init__(self, cf):
        super(MyANN,self).__init__()
        self.layer = nn.ModuleList()
        self.pid = [random.uniform(0,cf["pid"]["P_lim"]), random.uniform(0,cf["pid"]["I_lim"]), random.uniform(0,cf["pid"]["D_lim"])]
        
        # Layer generation
        for input_size, output_size in zip(cf["n_layers"], cf["n_layers"][1:]):
            self.layer.append(nn.Linear(input_size, output_size))

        # Set all parameters to required_grad False
        # for evolution (not normal training: e.g. backprop)
        for layer in self.layer:
            for param in layer.parameters():
                param.requires_grad = False

        # Extra params
        self.cf = cf
        self.heights = []
        self.perc_lim = cf["lim"]["var"]
        
    def forward(self,x):
        
        L = len(self.layer)
        
        # Encode scalar into tensor
        #x = torch.tensor([float(x)])
        
        x_array = [x]
    
        # Pass through all the layers
        for (l,linear_transform) in zip(range(L), self.layer):
            # Hidden
            if l < L-1:
                x = torch.tanh(linear_transform(x)) #torch.relu, tanh, sigmoid, F.leaky_relu, softmax
            # Output
            else:
                x = linear_transform(x)
                
            x_array.append(x)
        
        return x_array, x
    
    def update_params(self):
        
        # Probability of mutating a parameter
        p = self.cf["evol"]["p_mut_param"]
        
        for layer in self.layer:
            
            weight = getattr(layer,"weight")
            weight.data += af.randomTensorW(*self.cf["lim"]["weights"], *list(weight.shape)) * (torch.rand_like(weight.data) < p)
            weight.data = torch.clamp(weight.data, *self.cf["clamp"]["weights"])
            
            bias = getattr(layer,"bias")
            bias.data += af.randomTensorW(*self.cf["lim"]["weights"], 1, bias.shape[0]).view(-1) * (torch.rand_like(bias.data) < p)
            bias.data = torch.clamp(bias.data, *self.cf["clamp"]["weights"])

        # Apply mutation decay
        self.perc_lim = self.perc_lim * self.cf["lim"]["decay"]
        self.cf["lim"]["weights"] = list(np.array(self.cf["lim"]["weights"]) * self.perc_lim)

        # PID Mutation
        self.pid[0]   += random.uniform(-self.cf["pid"]["P_lim"], self.cf["pid"]["P_lim"]) * p
        self.pid[1]   += random.uniform(-self.cf["pid"]["I_lim"], self.cf["pid"]["I_lim"]) * p
        self.pid[2]   += random.uniform(-self.cf["pid"]["D_lim"], self.cf["pid"]["D_lim"]) * p
        
        for i in range(0,3):
            self.pid[i] = max(0,self.pid[i])
        
        self.pid[0] =  min(self.pid[0], self.cf["pid"]["P_lim"])
        self.pid[1] =  min(self.pid[1], self.cf["pid"]["I_lim"])
        self.pid[2] =  min(self.pid[2], self.cf["pid"]["D_lim"])

#Layers = [1,3,4,3]
#model = MyANN(cf)
#model.eval()

#print(list(model.parameters()))

#inp = torch.tensor([5.0])
#model(inp)

#model.update_params()