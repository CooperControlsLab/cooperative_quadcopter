#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 26 20:35:50 2018

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

URI = 'radio://0/30/2M'

oscillate = False
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

m = float(input("Input a mass value"))
k = float(input("Input a spring constant"))
c = float(input("Input a damping coeficient"))

z = c/(2*np.sqrt(m*k))  # Damping Ratio
wn = np.sqrt(k/m)       # natural Freq.
w = wn*np.sqrt(1-z**2)  # Damped Nat. Freq.
f = w/(2*np.pi)         # Frequncy  (hz)
tau = 1/f               # Period (s)
t = 0                   # initial time (s)

sleep = (0.2*0.758)/f
amplitude = 0.25
v = (f/0.758)*(amplitude/0.25)

v_inp = 1/((1/v)+((sleep - 0.2)/(2*amplitude)))#-(1.94*(m/k)-1.1*c)
print(sleep)
print(v)
print(w)
print(wn)
print(z)
print(tau)
print(v_inp)

peak = 2*amplitude

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        # We take off when the commander is created
        with MotionCommander(scf) as mc:
            time.sleep(1)

            # There is a set of functions that move a specific distance
            # We can move in all directions
#            mc.forward(0.8)
#            mc.back(0.8)
#            time.sleep(1)
#
#            mc.up(0.5)
#            mc.down(0.5)
#            time.sleep(1)
#
#            # We can also set the velocity
#            mc.right(0.5, velocity=0.8)
#            time.sleep(1)
#            mc.left(0.5, velocity=0.4)
#            time.sleep(1)
#
#            # We can do circles or parts of circles
#            mc.circle_right(0.5, velocity=0.5, angle_degrees=180)
#
#            # Or turn
#            mc.turn_left(90)
#            time.sleep(1)
#
#            # We can move along a line in 3D space

            while True:
                if keyboard.is_pressed('w'):
                    oscillate = True
                    while oscillate == True:
                        mc.move_distance(0,0,peak,v_inp)
                        
                        t = t+tau/2
                        disp = np.exp(-z*wn*t)*amplitude
                     
                        peak = amplitude + disp
                        amplitude = disp
                        print(peak)
                        time.sleep(0.2)
                        
                        mc.move_distance(0,0,-peak,v_inp)
                        
                        t = t+tau/2
                        disp = np.exp(-z*wn*t)*amplitude
                        peak = amplitude + disp
                        amplitude = disp
                        print(peak)
                        time.sleep(0.2)
                 #       while True:
                        if keyboard.is_pressed('p') or peak < 2.18*10**(-7):
                            oscillate = False
                          #  break
                    break
                
            while True:
                if keyboard.is_pressed('s'):
                    oscillate == False
                    mc.land(velocity = 0.6)
                    break
#
#            # There is also a set of functions that start a motion. The
#            # Crazyflie will keep on going until it gets a new command.
#
#            mc.start_left(velocity=0.5)
#            # The motion is started and we can do other stuff, printing for
#            # instance
#            for _ in range(5):
#                print('Doing other work')
#                time.sleep(0.2)

            # And we can stop
 ############           mc.stop()
            #cf.close_link()

            # We land when the MotionCommander goes out of scope
