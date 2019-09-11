#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 18 18:18:25 2018

@author: bitcraze
"""
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  6 15:27:39 2018

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
Simple example that connects to the first Crazyflie found, logs the Stabilizer
and prints it to the console. After 10s the application disconnects and exits.

This example utilizes the SyncCrazyflie and SyncLogger classes.
"""
import logging
import time
import numpy as np
import matplotlib.pyplot as plt
import keyboard


import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
URI = 'radio://0/80/250K'
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
#    print('Scanning interfaces for Crazyflies...')
#    available = cflib.crtp.scan_interfaces()
#    print('Crazyflies found:')
#    for i in available:
#        print(i[0])
#
#    if len(available) == 0:
#        print('No Crazyflies found, cannot run example')
#    else:
        #lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
        #lg_stab.add_variable('stabilizer.roll', 'float')
        #lg_stab.add_variable('stabilizer.pitch', 'float')
        #lg_stab.add_variable('stabilizer.yaw', 'float')

    lg_stab = LogConfig(name='stateEstimate', period_in_ms=10)
    lg_stab.add_variable('stateEstimate.x', 'float')
    lg_stab.add_variable('stateEstimate.y', 'float')
    lg_stab.add_variable('stateEstimate.z', 'float')
    x_arr = np.array([])
    t = np.array([])
    cf = Crazyflie(rw_cache='./cache')
        #Connecting to radio
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)
        cf.param.set_value("flightmode.althold","False")
        with SyncLogger(scf, lg_stab) as logger:

            endTime = time.time() + 10
            start = time.time()
            i = 0
            j = 0
            rise = False
            fall = False
            y = 0
            epsilon = 0.05
            defaultz = 0.033
            height = 1
            for log_entry in logger:
                    
                if i==0:
                    print('Start')
                    start = log_entry[0]
                    i = 1
                timestamp = log_entry[0] #time is logged in ms
                data = log_entry[1]
                logconf_name = log_entry[2]
                x_arr = np.append(x_arr,data['stateEstimate.z'])
                t = np.append(t,timestamp-start)
                #print('[%d][%s]: %s' % (timestamp, logconf_name, data))
                if rise==False and fall==False:
                    cf.commander.send_hover_setpoint(0,0,0,height)
                    if abs(height-data['stateEstimate.z'])<epsilon:
                        print('Risen')
                        rise = True
                        hoverstart = timestamp-start
                if rise and not fall:
                    cf.commander.send_hover_setpoint(0,0,0,height)
                    if j==0:
                        starting = time.time()
                        ending = starting+10
                        j=1
                    if j==1:
                        if time.time()>ending:
                            fall = True
                            fallstart = timestamp-start
                if rise and fall:
                    cf.commander.send_hover_setpoint(0,0,0,0)
                    if abs(data['stateEstimate.z'])<epsilon:
                        print('Fallen')
                        last = (timestamp-start)/1000
                        break
            
            highest = np.amax(x_arr)
            lowest = np.amin(x_arr)
            line = np.array([highest,lowest])
            t1 = np.array([hoverstart,hoverstart])
            t2 = np.array([fallstart,fallstart])
            cf.commander.send_stop_setpoint()
            print(start)
            print(t1)
            print(t2)
            plt.plot(t/1000,x_arr)
            plt.plot(t1/1000,line)
            plt.plot(t2/1000,line)
            plt.xlabel('Time [s]')
            plt.ylabel('x Displacement [m]')
            plt.title('Gyro.x Data')
            plt.show()
            cf.param.set_value("flightmode.althold","False")
            print('Done')
            cf.close_link()
#            break
        

    