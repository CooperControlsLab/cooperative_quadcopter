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
        #DEFAULTS
        cf.param.set_value('posCtlPid.zKp','{:.2f}'.format(2))
        cf.param.set_value('posCtlPid.zKi','{:.2f}'.format(0.5))
        cf.param.set_value('posCtlPid.zKd','{:.2f}'.format(0))
        #New Values
        kp = 9
        ki = 1
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
        
        kp2 = 3
        ki2 = 3
        kd2 = 0.5
        
        cf.param.set_value('pid_attitude.roll_kp','{:.2f}'.format(kp2))
        cf.param.set_value('pid_attitude.roll_ki','{:.2f}'.format(ki2))
        cf.param.set_value('pid_attitude.roll_kd','{:.2f}'.format(kd2))
        cf.param.set_value('pid_attitude.pitch_kp','{:.2f}'.format(kp2))
        cf.param.set_value('pid_attitude.pitch_ki','{:.2f}'.format(ki2))
        cf.param.set_value('pid_attitude.pitch_kd','{:.2f}'.format(kd2))
        cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)
        #cf.param.set_value("flightmode.althold","False")
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
            height = 0.5
            for log_entry in logger:
                if keyboard.is_pressed('s'):
                    cf.commander.send_stop_setpoint()
                    break
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
                    if check_epsilon(data['stateEstimate.z'],height,epsilon):
                        print('Risen')
                        rise = True
                        hoverstart = timestamp-start
                if rise and not fall:
                    cf.commander.send_hover_setpoint(0,0,0,height)
                    #cf.commander.send_zdistance_setpoint(0,0,0,height)
                    if j==0:
                        starting = time.time()
                        ending = starting+20
                        j=1
                    if j==1:
                        if time.time()>ending:
                            fall = True
                            fallstart = timestamp-start
                if rise and fall:
                    cf.commander.send_hover_setpoint(0,0,0,0.1)
                    if check_epsilon(data['stateEstimate.z'],0,0.2):
                        print('Fallen')
                        last = timestamp-start
                        break
            
            cf.commander.send_stop_setpoint()
            highest = np.amax(x_arr)
            lowest = np.amin(x_arr)
            line = np.array([highest,lowest])
            line2 = np.array([height,height])
            span = np.array([0,last])
            t1 = np.array([hoverstart,hoverstart])
            t2 = np.array([fallstart,fallstart])
            cf.param.set_value('posCtlPid.zKp','{:.2f}'.format(2))
            cf.param.set_value('posCtlPid.zKi','{:.2f}'.format(0.5))
            cf.param.set_value('posCtlPid.zKd','{:.2f}'.format(0))
            cf.param.set_value('posCtlPid.xKp','{:.2f}'.format(2))
            cf.param.set_value('posCtlPid.xKi','{:.2f}'.format(0))
            cf.param.set_value('posCtlPid.xKd','{:.2f}'.format(0))
            cf.param.set_value('posCtlPid.yKp','{:.2f}'.format(2))
            cf.param.set_value('posCtlPid.yKi','{:.2f}'.format(0))
            cf.param.set_value('posCtlPid.yKd','{:.2f}'.format(0))
            cf.param.set_value('pid_attitude.roll_kp','{:.2f}'.format(6))
            cf.param.set_value('pid_attitude.roll_ki','{:.2f}'.format(3))
            cf.param.set_value('pid_attitude.roll_kd','{:.2f}'.format(0))
            cf.param.set_value('pid_attitude.pitch_kp','{:.2f}'.format(6))
            cf.param.set_value('pid_attitude.pitch_ki','{:.2f}'.format(3))
            cf.param.set_value('pid_attitude.pitch_kd','{:.2f}'.format(0))
            print(start)
            print(t1)
            print(t2)
            plt.plot(t/1000,x_arr-defaultz,'.')
            plt.plot(t1/1000,line)
            plt.plot(t2/1000,line)
            plt.plot(span/1000,line2)
            plt.xlabel('Time [s]')
            plt.ylabel('z Displacement [m]')
            plt.title('StateEstimate.z Data: kp=%f ki=%f kd=%f' % (kp,ki,kd))
            plt.show()
            #cf.param.set_value("flightmode.althold","False")
            print('Done')
            cf.close_link()
#            break
        

    