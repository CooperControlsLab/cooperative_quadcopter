#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Jun  5 11:27:20 2018

@author: bitcraze
"""

#justin github test

import logging
import time
from threading import Thread

import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig

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

        it=0
        self._cf.commander.send_setpoint(0,0,0,40000)
        #self._cf.param.set_value("flightmode.althold","True")
        while it<300:
            #self._cf.commander.send_setpoint(roll, pitch, yawrate, thrust)
            #self._cf.commander.send_setpoint(0.66,-1,0,32767)
            self._cf.commander.send_setpoint(0,0,0,32767)
            self._cf.param.set_value("flightmode.althold","True")
            #self._cf.param.set_value("flightmode.poshold","False")
            time.sleep(0.01)
            it+=1
 
        print("Close connection")
        self._cf.commander.send_setpoint(0,0,0,0)
        self._cf.close_link()


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
        le = AltHoldExample(available[0][0])
        
    else:
        print('No Crazyflies found, cannot run example')
