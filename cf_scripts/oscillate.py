#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Jun 28 18:17:36 2018

@author: bitcraze
"""

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun 19 21:44:52 2018

@author: bitcraze
"""

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
import keyboard


import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

URI = 'radio://0/80/250K'
# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

def check_epsilon(value,setpoint,epsilon):
    if abs(value-setpoint)<epsilon:
        return True




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

    x_arr = np.array([])
    t = np.array([])
    cf = Crazyflie(rw_cache='./cache')
        #Connecting to radio
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        #DEFAULTS
        cf.param.set_value('posCtlPid.zKp','{:.2f}'.format(2))
        cf.param.set_value('posCtlPid.zKi','{:.2f}'.format(0.5))
        cf.param.set_value('posCtlPid.zKd','{:.2f}'.format(0))
        #New Values
        kp = 2
        ki = 0.5
        kd = 0
#        cf.param.set_value('posCtlPid.xKp','{:.2f}'.format(kp))
#        cf.param.set_value('posCtlPid.xKi','{:.2f}'.format(ki))
#        cf.param.set_value('posCtlPid.xKd','{:.2f}'.format(kd))
#        cf.param.set_value('posCtlPid.yKp','{:.2f}'.format(kp))
#        cf.param.set_value('posCtlPid.yKi','{:.2f}'.format(ki))
#        cf.param.set_value('posCtlPid.yKd','{:.2f}'.format(kd))
        cf.param.set_value('posCtlPid.zKp','{:.2f}'.format(kp))
        cf.param.set_value('posCtlPid.zKi','{:.2f}'.format(ki))
        cf.param.set_value('posCtlPid.zKd','{:.2f}'.format(kd))
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)
        #cf.param.set_value("flightmode.althold","False")
        height = 0.5
        start = time.time()
        #cf.commander.send_hover_setpoint(0,0,0,height)
        i = 0
        j=0
        for i in range(6):
            cf.commander.send_hover_setpoint(0,0,0,i/10)
            time.sleep(0.1)
            
        start = time.time()
        while True:
            #cf.commander.send_velocity_world_setpoint(0,0,0.2,0)
            #print(0.25*np.cos(time.time()-start)+height)
            if j==0:
                cf.commander.send_hover_setpoint(0,0,0,0.25)
                j=1
                time.sleep(0.2)
            if j==1:
                cf.commander.send_hover_setpoint(0,0,0,0.5)
                j=2
                time.sleep(0.2)
            if j==2:
                cf.commander.send_hover_setpoint(0,0,0,0.75)
                j=3
                time.sleep(0.2)
            if j==3:
                cf.commander.send_hover_setpoint(0,0,0,0.5)
                j=0
                time.sleep(0.2)
            if keyboard.is_pressed('s'):
                break
        print('Down')
        #time.sleep(5)
        cf.commander.send_hover_setpoint(0,0,0,0)
        

            
        cf.commander.send_stop_setpoint()
        cf.param.set_value('posCtlPid.zKp','{:.2f}'.format(2))
        cf.param.set_value('posCtlPid.zKi','{:.2f}'.format(0.5))
        cf.param.set_value('posCtlPid.zKd','{:.2f}'.format(0))
        cf.param.set_value('posCtlPid.xKp','{:.2f}'.format(2))
        cf.param.set_value('posCtlPid.xKi','{:.2f}'.format(0))
        cf.param.set_value('posCtlPid.xKd','{:.2f}'.format(0))
        cf.param.set_value('posCtlPid.yKp','{:.2f}'.format(2))
        cf.param.set_value('posCtlPid.yKi','{:.2f}'.format(0))
        cf.param.set_value('posCtlPid.yKd','{:.2f}'.format(0))

            #cf.param.set_value("flightmode.althold","False")
        print('Done')
        cf.close_link()
#            break
        

    