#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  6 11:30:49 2018

@author: bitcraze
"""

import logging
import time
from threading import Thread

import cflib
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

logging.basicConfig(level=logging.ERROR)

class AltHoldExample:

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie()

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)
        self.is_connected = True
        # Variable used to keep main loop occupied until disconnect
        print('Connecting to %s' % link_uri)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)
        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        Thread(target=self._hover_test).start()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False

    def _hover_test(self):
        print("sending initial thrust of 0")

        self._cf.commander.send_setpoint(0,0,0,0);
        time.sleep(0.5);

        print("putting in althold")
        self._cf.param.set_value("flightmode.althold","True")

        print("Stay in althold for 7s")

        lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
        lg_stab.add_variable('stabilizer.roll','float')
        lg_stab.add_variable('stabilizer.pitch','float')
        lg_stab.add_variable('stabilizer.yaw','float')
        
        cf = Crazyflie(rw_cache='./cache')
        with SyncCrazyflie(available[0][0],cf=cf) as scf:
            with SyncLogger(scf, lg_stab) as logger:
                endTime = time.time()+10


        




        it=0
        self._cf.commander.send_setpoint(0.66,1,0,35000)
        time.sleep(2)
        #Bug here
        #self._cf.param.set_value("flightmode.althold","True")
        for log_entry in logger:
            print('HI')
            timestamp = log_entry[0]
            data = log_entry[1]
            logconf_name = log_entry[2]
            print('[%d][%s]: %s' % (timestamp,logconf_name,data))
        #while it<200:
            #self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
            #self._cf.commander.send_setpoint(0.66,-1,0,32767)
            self._cf.commander.send_setpoint(0.66,1,0,32767)
            self._cf.param.set_value("flightmode.althold","True")
            #self._cf.param.set_value("flightmode.poshold","False")
            #time.sleep(0.01)
            #it+=1
            if time.time()>endTime:
                print("Close connection")
                self._cf.commander.send_setpoint(0,0,0,0)
                self._cf.close_link()
                break
 


























if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Scan for Crazyflies and use the first one found
    print('Scanning interfaces for Crazyflies...')
    available = cflib.crtp.scan_interfaces()
    print('Crazyflies found:')
    for i in available:
        print(i[0])

    if len(available) > 0:
        lg_stab = LogConfig(name='Stabilizer', period_in_ms=10)
        lg_stab.add_variable('stabilizer.roll', 'float')
        lg_stab.add_variable('stabilizer.pitch', 'float')
        lg_stab.add_variable('stabilizer.yaw', 'float')

        cf = Crazyflie(rw_cache='./cache')
        #Connecting to radio
        with SyncCrazyflie(available[0][0], cf=cf) as scf:
            with SyncLogger(scf, lg_stab) as logger:
                print(time.time())
                endTime = time.time() + 10
                print(endTime)
                cf.commander.send_setpoint(0,0,0,0)
                cf.param.set_value("flightmode.althold","True")
                cf.commander.send_setpoint(0,0,0,35000)
                print('Start')
                for log_entry in logger:
                    timestamp = log_entry[0]
                    data = log_entry[1]
                    logconf_name = log_entry[2]

                    print('[%d][%s]: %s' % (timestamp, logconf_name, data))
                    cf.commander.send_setpoint(0.66,1,0,32767)
                    cf.param.set_value("flightmode.althold","True")
                    if time.time() > endTime:
                        print("Close connection")
                        cf.commander.send_setpoint(0,0,0,0)
                        cf.close_link()
                        break

        
    else:
        print('No Crazyflies found, cannot run example')