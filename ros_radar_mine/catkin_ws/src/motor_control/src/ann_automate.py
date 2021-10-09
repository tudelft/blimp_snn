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
#import torch.nn.functional as F
import os
import numpy as np

############################################################
# CONFIG GLOBAL VARIABLE
############################################################

LAYERS = [1,3,2,1]

class MyANN(nn.Module):
    
    def __init__(self):
        super(MyANN,self).__init__()
        self.layer = nn.ModuleList()
        
        # Layer generation
        self.n_layers = LAYERS
        for input_size, output_size in zip(self.n_layers, self.n_layers[1:]):
            self.layer.append(nn.Linear(input_size, output_size))

        # Set all parameters to required_grad False
        # for evolution (not normal training: e.g. backprop)
        for layer in self.layer:
            for param in layer.parameters():
                param.requires_grad = False

        # Extra params
        self.heights = []
        
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

#Layers = [1,3,4,3]
#model = MyANN(cf)
#model.eval()

#print(list(model.parameters()))

#inp = torch.tensor([5.0])
#model(inp)

#model.update_params()