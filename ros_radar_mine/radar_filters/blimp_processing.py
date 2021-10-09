#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Apr 10 19:54:46 2021

@author: marina
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from os import listdir

# Root Mean Squared Error calculation
def rmse_error(df, str_1, str_2):
    mse = sum((df[str_1] - df[str_2])**2)/df.shape[0]
    mse = np.sqrt(mse)
    print("[RMSE " + str_1 + "]: " + str(mse))
    return mse

#--> Specify file to be analyzed
path = '/home/marina/Pi_Git/ros_radar_mine/record_radar_data/all_'
date = '2021-05-07'
time = '09-58-09'
csvfile = path + date + '-' + time + '.csv'
OPTI_offset  = 0.42 #0.42 #0.51
   
#--> Import data as dataframe -> df and fix OptiTrack offset
df = pd.read_csv(csvfile)
df["range_op"] -= OPTI_offset

# PLOT filtered range
fig, axs = plt.subplots(2)

### SUBPLOT 1 ###

#ax1 = df.plot(x="time", y=["range_flt", "range_op", "h_ref"], grid=True, 
#            title="Range vs. Time", style=['-','-','--'])
ax1 = df.plot(x="time", y=["range_op", "h_ref"], grid=True, 
            title="Range vs. Time", style=['-','--'], ax=axs[0])
#ax1.set_xlim(x_min,x_max)
#ax1.set_ylim(y_min,y_max)
ax1.set_xlabel("Time [s]")
ax1.set_ylabel("Height [m]")
ax1.legend(["$r_{measured}$", "$r_{OptiTrack}$", "$h_{ref}$"], loc='center left', bbox_to_anchor=(1, 0.5))
#ax1.legend(["$r_{measured}$", "$r_{OptiTrack}$", "$h_{ref}$"], loc='center left', bbox_to_anchor=(1, 0.5))

### SUBPLOT 2 ###

ax2 = df.plot(x="time", y=["dcmotor"], grid=True, 
            title="Motor command vs. Time", style=['-'], ax=axs[1])
#ax1.set_xlim(x_min,x_max)
#ax1.set_ylim(y_min,y_max)
ax2.set_xlabel("Time [s]")
ax2.set_ylabel("Motor command")
#ax1.legend(["$r_{measured}$", "$r_{OptiTrack}$", "$h_{ref}$"], loc='center left', bbox_to_anchor=(1, 0.5))

plt.tight_layout()
plt.show()

mse = rmse_error(df, "h_ref", "range_op")