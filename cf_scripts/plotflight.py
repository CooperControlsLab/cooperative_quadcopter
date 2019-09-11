#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 14 19:29:47 2018

@author: bitcraze
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 12 20:25:01 2018

@author: bitcraze
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 11 13:07:52 2018

@author: bitcraze
"""

# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2016 Bitcraze AB
#
#  Crazyflie Nano Quadcopter Client
#
#  This program is free software; you can redistribute it and/or
#  modify it under the terms of the GNU General Public License
#  as published by the Free Software Foundation; either version 2
#  of the License, or (at your option) any later version.
#
#  This program is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, write to the Free Software
#  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
#  MA  02110-1301, USA.
"""
Simple example that connects to the crazyflie at `URI` and runs a figure 8
sequence. This script requires some kind of location system, it has been
tested with (and designed for) the flow deck.

Change the URI variable to your Crazyflie configuration.
"""
import logging
import time
import keyboard
import numpy as np
import matplotlib.pyplot as plt

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

URI = 'radio://0/80/250K'




def add_Data(log_entry,x_arr,t,start):
    timestamp = log_entry[0]
    data = log_entry[1]
    logconf_name = log_entry[2]
    x_arr = np.append(x_arr,data['stateEstimate.x'])
    t = np.append(t,timestamp-start)

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
   
    cflib.crtp.init_drivers(enable_debug_driver=False)
    x_arr = np.array([])
    t = np.array([])
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        lg_stab = LogConfig(name='stateEstimate', period_in_ms=10)
        lg_stab.add_variable('stateEstimate.x','float')
        with SyncLogger(scf,lg_stab) as logger:
            cf.param.set_value('kalman.resetEstimation', '1')
            time.sleep(0.1)
            cf.param.set_value('kalman.resetEstimation', '0')
            time.sleep(2)
            y = 0
            k = 0
            rising = False
            falling = False
            for log_entry in logger:
                if k==0:
                    start = log_entry[0]
                    k=1
                if keyboard.is_pressed('w'):
                    rising = True
                if rising:
                    if y<19:
                        cf.commander.send_hover_setpoint(0, 0, 0, y / 25)
                        time.sleep(0.1)
                        y+=1
                        add_Data(log_entry,x_arr,t,start)
                    else:
                        y = 19
                        cf.commander.send_hover_setpoint(0,0,0,y/25)
                        add_Data(log_entry,x_arr,t,start)
                        break
                else:
                    add_Data(log_entry,x_arr,t,start)

#                time.sleep(1)
            i = y
            print('Rose')
            for log_entry in logger:
                if keyboard.is_pressed('s'):
                    falling = True
                if falling:
                    if i>0:
                        cf.commander.send_hover_setpoint(0,0,0,i/25)
                        i-=1
                        time.sleep(0.1)
                        #add_Data(log_entry,x_arr,t,start)
                        #cf.commander.send_hover_setpoint(0,0,0,0)
                    else:
                        break
                else:
                    cf.commander.send_hover_setpoint(0,0,0,i/25)
                    #add_Data(log_entry,x_arr,t,start)
                    
                    
            plt.plot(t,x_arr)
            plt.xlabel('Time [s]')
            plt.ylabel('x Displacement [m]')
            plt.title('State Estimate x Data')
            plt.show()

#        for _ in range(20):
#            cf.commander.send_hover_setpoint(0, 0, 0, 0.4)
#            time.sleep(0.1)
#
#        for _ in range(50):
#            cf.commander.send_hover_setpoint(0.5, 0, 36 * 2, 0.4)
#            time.sleep(0.1)
#
#        for _ in  (50):
#            cf.commander.send_hover_setpoint(0.5, 0, -36 * 2, 0.4)
#            time.sleep(0.1)
#   
#        for _ in range(20):
#            cf.commander.send_hover_setpoint(0, 0, 0, 0.4)
#            time.sleep(0.1)
#
#        for y in range(10):
#            cf.commander.send_hover_setpoint(0, 0, 0, (10 - y) / 25)
#            time.sleep(0.1)
        
        cf.commander.send_stop_setpoint()

        print('Done')

