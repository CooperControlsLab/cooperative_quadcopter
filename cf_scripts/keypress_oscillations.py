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

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = 'radio://0/80/250K'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
   
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)

        m = float(input("Input a mass value"))
        k = float(input("Input a spring constant"))
        x_0 = float(input("Input an intial displacement"))
        v_0 = float(input("Input an intial velocity"))


        while True:
            if keyboard.is_pressed('w'):
                for y in range(20):
                    cf.commander.send_hover_setpoint(0, 0, 0, y / 25)
                    time.sleep(0.1)
                break
        i = y/25
        while True:
            cf.commander.send_hover_setpoint(0,0,0,i)
            if keyboard.is_pressed('p'):

                t = 0
                w_n = np.sqrt(k/m)
                cf.commander.send_hover_setpoint(0,0,0, x_0 + i)
                time.sleep(1)
                while x_0>0:
                    
                    cf.commander.send_hover_setpoint(0,0,0, i+(x_0)*np.cos(w_n*t)+(v_0/w_n)*np.sin(w_n*t))
                    cf.commander.send_velocity_world_setpoint(0,0,-(x_0)*np.sin(w_n*(t+0.01))+(v_0)*np.cos(w_n*(t+0.01)),0)
                    if keyboard.is_pressed('q'):
                        cf.commander.send_hover_setpoint(0, 0, 0, y / 25)
                        break
                    else:
                        t = t+0.01
                        time.sleep(0.01)
                break
                        

        time.sleep(1)
        i = y
        while True:
            if keyboard.is_pressed('s'):
                while i>=0:
                    cf.commander.send_hover_setpoint(0,0,0,i/25)
                    i-=1
                    time.sleep(0.1)
                    #cf.commander.send_hover_setpoint(0,0,0,0)
                break
            else:
                cf.commander.send_hover_setpoint(0,0,0,y/25)
        
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
        cf.close_link()