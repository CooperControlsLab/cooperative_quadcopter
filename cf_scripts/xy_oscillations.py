# -*- coding: utf-8 -*-
"""
Created on Mon Jun 18 12:53:27 2018

@author: andre
"""

# -*- coding: utf-8 -*-
#
#     ||          ____  _ __
#  +------+      / __ )(_) /_______________ _____  ___
#  | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
#  +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
#   ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
#
#  Copyright (C) 2017 Bitcraze AB
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
This script shows the basic use of the MotionCommander class.

Simple example that connects to the crazyflie at `URI` and runs a
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
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/250K'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    m = float(input("Input a mass value"))
    k = float(input("Input a spring constant"))
    x_0 = float(input("Input an intial displacement"))
    v_0 = float(input("Input an intial velocity"))
    t = 0    
    w_n = np.sqrt(k/m)
    
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # We take off when the commander is created
        with MotionCommander(scf) as mc:
            time.sleep(1)

            # There is a set of functions that move a specific distance
            # We can move in all directions

            mc.move_distance(x_0, 0.0, 0.0, velocity=0.6)
            time.sleep(1)
            # We can move along a line in 3D space
            
            while x_0>0:
                
                mc.move_distance((x_0)*np.cos(w_n*t)+(v_0/w_n)*np.sin(w_n*t), 0.0, 0.0, velocity=abs(-(x_0)*np.sin(w_n*(t+0.01))+(v_0)*np.cos(w_n*(t+0.01))))
                time.sleep(0.1)
                if  keyboard.is_pressed('s'):
                    mc.stop()
                    break
                else:
                    t = t+0.01

            # And we can stop
            mc.stop()

            # We land when the MotionCommander goes out of scope