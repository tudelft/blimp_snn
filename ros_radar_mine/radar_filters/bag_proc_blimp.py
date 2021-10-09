#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Apr  9 16:47:08 2021

@author: marina
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jan 17 18:35:12 2021

TO DO [marina]: Automate to check all files in folder in a FOR LOOP !!!

@author: marina
"""

import rosbag
import pandas as pd
from os import listdir
import numpy as np

mypath = "/home/marina/Pi_Git/ros_radar_mine/record_radar_data/"
files = [f for f in listdir(mypath) if "all" in f and ".bag" in f]

# Specify file to be analyzed
#path = '/home/marina/Pi_Git/ros_radar_mine/record_radar_data/all_'
#date = '2021-04-07'
#time = '15-31-11'
#files = path + date + "-" + time + ".bag"
#files = [files]
time_offset = 0
number_samples = 30
radar_targets = False
radar_filter  = False
optitrack     = True
motor_control = True
h_ref         = True

for file in files:

    if "05-07-09-58-09" in file: 
    
        bagfile = mypath + file
        #bagfile = file
        print(bagfile)
        
        bag = rosbag.Bag(bagfile)
        
        # ---> Build RADAR dataframe: df_targets
        
        if radar_targets:
            
            column_names = ['time','num_targets', 'speed', 'angle', 'strength', 'range_ra']
            df_targets = pd.DataFrame(columns=column_names)
        
            for topic, msg, t in bag.read_messages(topics='/radar_targets'):
                ts = t.to_sec()
                
                if ts > time_offset:
                    total_num_targets = msg.total_num_targets
                    angle = msg.target_events[0].angle
                    speed = msg.target_events[0].speed
                    strength = msg.target_events[0].strength
                    ran = msg.target_events[0].range
                    ts = t.to_sec()
                
                    df_targets = df_targets.append(
                        {'time': ts,
                         'num_targets': total_num_targets,
                         'speed': speed,
                         'angle': angle,
                         'strength': strength,
                         'range_ra': ran},
                        ignore_index=True
                    )
        
            # Convert df_targets time to seconds
            #df_targets["time"] = df_targets["time"] - df_targets["time"][0]
        
        
        # ---> Build RADAR FILTERING dataframe: df_filter
        
        if radar_filter:
            column_names = ['time', 'range_flt']
            df_filter = pd.DataFrame(columns=column_names)
            
            for topic, msg, t in bag.read_messages(topics='/radar_filter'):
                ts = t.to_sec()
                
                if ts > time_offset:
                    range_flt = msg.range_filter[0]
                    ts = t.to_sec()
                
                    df_filter = df_filter.append(
                        {'time': ts,
                         'range_flt': range_flt},
                        ignore_index=True
                    )
                    
        # ---> Build H_REF dataframe: df_href
        
        if h_ref:
            column_names = ['time', 'h_ref']
            df_href = pd.DataFrame(columns=column_names)
            
            for topic, msg, t in bag.read_messages(topics='/h_ref'):
                ts = t.to_sec()
                
                if ts > time_offset:
                    h_ref = msg.data
                    ts = t.to_sec()
                
                    df_href = df_href.append(
                        {'time': ts,
                         'h_ref': h_ref},
                        ignore_index=True
                    )
        
        # ---> Build OPTITRACK dataframe: df_optitrack
        if optitrack:
        
            column_names = ['time','range_op']
            df_optitrack = pd.DataFrame(columns=column_names)
            
            for topic, msg, t in bag.read_messages(topics='/blimp/pose'):
                height = msg.position.z
                ts = t.to_sec()
                
                if ts > time_offset:
            
                    df_optitrack = df_optitrack.append(
                        {'time': ts,
                         'range_op': height},
                        ignore_index=True
                    )
                
            # Convert df_optitrack time to seconds
            #df_optitrack["time"] = df_optitrack["time"] - df_optitrack["time"][0]
            
            
        if motor_control:
            # ---> Build MOTOR_CONTROL dataframe: df_motor
            column_names = ['time','ccw','cw','servo']
            df_motor = pd.DataFrame(columns=column_names)
            
            for topic, msg, t in bag.read_messages(topics='/motor_control'):
                ccw = msg.ccw_speed
                cw  = msg.cw_speed
                servo = msg.angle
                
                #if servo == 10:
                #    ccw = -ccw
                #    cw = -cw
                    
                ts = t.to_sec()
            
                if ts > time_offset:
            
                    df_motor = df_motor.append(
                        {'time': ts,
                         'ccw': ccw,
                         'cw': cw,
                         'servo': servo},
                        ignore_index=True
                    )
            
            # Convert df_optitrack time to seconds
            #df_motor["time"] = df_motor["time"] - df_motor["time"][0]
        
        if radar_targets and radar_filter and h_ref and optitrack and motor_control:
            # ---> Merge previous df's by closest TIME -> df_final
            df_final = pd.merge_asof(df_targets, df_optitrack, on="time")
            df_final = pd.merge_asof(df_final, df_filter, on="time")
            df_final = pd.merge_asof(df_final, df_motor, on="time")
            df_final = pd.merge_asof(df_final, df_href, on="time")
    
        elif not radar_targets and optitrack and motor_control:
            first_sec = int(df_optitrack["time"].iloc[0])
            final_sec = int(df_optitrack["time"].iloc[-1])
            time_samples = np.linspace(time_offset+first_sec, final_sec, number_samples*(final_sec-first_sec))
            df_samples = pd.DataFrame({"time": time_samples})
            df_final = pd.merge_asof(df_optitrack, df_motor, on="time")
            df_final = pd.merge_asof(df_samples, df_final, on="time")
            df_final = pd.merge_asof(df_final, df_href, on="time")
        
        if df_final["cw"].equals(df_final["ccw"]):
            df_final.drop(["ccw"], axis=1, inplace=True)
        
        df_final.rename(columns={"cw": "dcmotor"}, inplace=True)
        
        condition = df_final["servo"] == 10
        df_final["dcmotor"][condition] = df_final["dcmotor"][condition] * (-1)
        
        df_final.dropna(inplace=True)
        df_final.reset_index(drop=True, inplace=True)
        df_final["time"] = df_final["time"] - df_final["time"][0]
        
        csvfile = bagfile.split(".")[0] + ".csv"
        df_final.to_csv(path_or_buf=csvfile, index=False)

    
    
    
    
    
    
