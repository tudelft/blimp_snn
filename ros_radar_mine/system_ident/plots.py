#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Mar  9 09:17:54 2021

@author: marina
"""

import matplotlib.pyplot as plt
import numpy as np

sig        = np.array([-1,0,0,1,0,3,0,-2,0,0,0,0,-2])
x          = np.arange(len(sig))
sig_sorted = np.sort(sig)
mf_sig     = np.array([0,0,2,4,4,4,3,2,1,-1,-2,-1,-1])
ma_sig     = np.array([0,0,1,2,4,4,3.5,2.5,1.5,0,-1.5,-1.5,-1])
ylim       = (-3.5,3.5)

fig, axs = plt.subplots(3)
fig.suptitle('Median Filter (MF) -> Moving Average (MA)')
axs[0].stem(x, sig, linefmt='grey')
axs[0].set_ylabel('Signal')
axs[0].set_ylim([ylim[0],ylim[1]])
axs[0].grid()
axs[1].stem(x, sig_sorted, linefmt='grey')
axs[1].plot(6,0,'x',markersize = 10, color = 'red')
axs[1].set_ylabel('Signal sorted')
axs[1].set_ylim([ylim[0],ylim[1]])
axs[1].grid()
axs[2].stem(x, mf_sig, markerfmt = 'rx', linefmt='grey')
axs[2].plot(x,ma_sig)
axs[2].set_ylabel('Apply MA')
axs[2].set_ylim([ylim[0]-1,ylim[1]+1])
axs[2].grid()
plt.show()

fig, axs = plt.subplots(2)
fig.suptitle('Median Filter (MF) + Moving Average (MA)')
axs[0].stem(x, sig, linefmt='grey')
axs[0].set_ylabel('Signal')
axs[0].set_ylim([ylim[0],ylim[1]])
axs[0].grid()
axs[1].stem(x, sig_sorted, linefmt='grey')
axs[1].plot([3,3],[ylim[0],ylim[1]],'k--')
axs[1].plot([9,9],[ylim[0],ylim[1]],'k--')
axs[1].set_ylabel('Signal sorted')
axs[1].set_ylim([ylim[0],ylim[1]])
axs[1].grid()
plt.show()