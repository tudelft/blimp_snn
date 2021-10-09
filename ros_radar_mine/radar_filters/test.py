#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jan 20 10:36:58 2021

@author: marina
"""

import numpy as np
import matplotlib.pyplot as plt

sigma = 5
size = 15
window = np.linspace(0,size-1,size)

for i in range(10):

    f = np.exp(-0.5*((size-1-window)/i)**2)

    plt.plot(f)