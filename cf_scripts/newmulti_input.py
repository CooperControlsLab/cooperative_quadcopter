#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 21 18:39:35 2018

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
#  Copyright (C) 2014 Bitcraze AB
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
Simple example that connects 2 Crazyflies, ramp up-down the motors and
disconnects.
"""

import math
import logging
import time
from threading import Thread
import keyboard
import numpy as np


from queue import Queue, Empty

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.multi_commander import MotionCommander

import cflib

oscillate = False

logging.basicConfig(level=logging.ERROR)

m1 = float(input("Input a mass value"))
k1 = float(input("Input a spring constant"))
c1 = float(input("Input a damping coeficient"))

z1 = c1/(2*np.sqrt(m1*k1))  # Damping Ratio
wn1 = np.sqrt(k1/m1)       # natural Freq.
w1 = wn1*np.sqrt(1-z1**2)  # Damped Nat. Freq.
f1 = w1/(2*np.pi)         # Frequncy  (hz)
tau1 = 1/f1               # Period (s)
t = 0                   # initial time (s)

sleep1 = (0.2*0.758)/f1
amplitude = 0.25
v1 = (f1/0.758)*(amplitude/0.25)

v_inp1 = 1/((1/v1)+((sleep1 - 0.2)/(2*amplitude)))#-(1.94*(m/k)-1.1*c)

######## Drone 2 inputs ############

m2 = float(input("Input a mass value"))
k2 = float(input("Input a spring constant"))
c2 = float(input("Input a damping coeficient"))

z2 = c2/(2*np.sqrt(m2*k2))  # Damping Ratio
wn2 = np.sqrt(k2/m2)       # natural Freq.
w2 = wn2*np.sqrt(1-z2**2)  # Damped Nat. Freq.
f2 = w2/(2*np.pi)         # Frequncy  (hz)
tau2 = 1/f2               # Period (s)
t = 0                   # initial time (s)

sleep2 = (0.2*0.758)/f2
amplitude = 0.25
v2 = (f2/0.758)*(amplitude/0.25)

v_inp2 = 1/((1/v1)+((sleep1 - 0.2)/(2*amplitude)))

print(sleep1)
print(v1)
print(w1)
print(wn1)
print(z1)
print(tau1)
print(v_inp1)

peak = 2*amplitude



URI_1 = 'radio://0/30/2M'
URI_2 = 'radio://0/35/2M'

class Drone:
    VELOCITY = 0.2
    RATE = 360.0 / 5

    

    def __init__(self, link_uri):
        """ Initialize and run the example with the specified link_uri """

        self._cf = Crazyflie(rw_cache='./cache')

        self._cf.connected.add_callback(self._connected)
        self._cf.disconnected.add_callback(self._disconnected)
        self._cf.connection_failed.add_callback(self._connection_failed)
        self._cf.connection_lost.add_callback(self._connection_lost)

        self._cf.open_link(link_uri)
  #      self._thread = None
        self._thread = _SetPointThread(self._cf)
        self._is_flying = False
        self.connected = True

        print('Connecting to %s' % link_uri)

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""

        # Start a separate thread to do the motor test.
        # Do not hijack the calling thread!
        Thread(target=self._oscillate).start()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))
        self.connected = False

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.connected = False
        
#########################This is From the Motion Commander Library #########

    def take_off(self, height=None, velocity=VELOCITY):
        """
        Takes off, that is starts the motors, goes straigt up and hovers.
        Do not call this function if you use the with keyword. Take off is
        done automatically when the context is created.

        :param height: the height (meters) to hover at. None uses the default
                       height set when constructed.
        :param velocity: the velocity (meters/second) when taking off
        :return:
        """
        if self._is_flying:
            raise Exception('Already flying')

        if not self._cf.is_connected():
            raise Exception('Crazyflie is not connected')

        self._is_flying = True
        self._reset_position_estimator()

        self._thread = _SetPointThread(self._cf)
        self._thread.start()

        if height is None:
            height = self.default_height

        self.up(height, velocity)

    def land(self, velocity=VELOCITY):
        """
        Go straight down and turn off the motors.

        Do not call this function if you use the with keyword. Landing is
        done automatically when the context goes out of scope.

        :param velocity: The velocity (meters/second) when going down
        :return:
        """
        if self._is_flying:
            self.down(self._thread.get_height(), velocity)

            self._thread.stop()
            self._thread = None

            self._cf.commander.send_stop_setpoint()
            self._is_flying = False

    def __enter__(self):
        self.take_off()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.land()

    def left(self, distance_m, velocity=VELOCITY):
        """
        Go left

        :param distance_m: the distance to travel (meters)
        :param velocity: the velocity of the motion (meters/second)
        :return:
        """
        self.move_distance(0.0, distance_m, 0.0, velocity)

    def right(self, distance_m, velocity=VELOCITY):
        """
        Go right

        :param distance_m: the distance to travel (meters)
        :param velocity: the velocity of the motion (meters/second)
        :return:
        """
        self.move_distance(0.0, -distance_m, 0.0, velocity)

    def forward(self, distance_m, velocity=VELOCITY):
        """
        Go forward

        :param distance_m: the distance to travel (meters)
        :param velocity: the velocity of the motion (meters/second)
        :return:
        """
        self.move_distance(distance_m, 0.0, 0.0, velocity)

    def back(self, distance_m, velocity=VELOCITY):
        """
        Go backwards

        :param distance_m: the distance to travel (meters)
        :param velocity: the velocity of the motion (meters/second)
        :return:
        """
        self.move_distance(-distance_m, 0.0, 0.0, velocity)

    def up(self, distance_m, velocity=VELOCITY):
        """
        Go up

        :param distance_m: the distance to travel (meters)
        :param velocity: the velocity of the motion (meters/second)
        :return:
        """
        self.move_distance(0.0, 0.0, distance_m, velocity)

    def down(self, distance_m, velocity=VELOCITY):
        """
        Go down

        :param distance_m: the distance to travel (meters)
        :param velocity: the velocity of the motion (meters/second)
        :return:
        """
        self.move_distance(0.0, 0.0, -distance_m, velocity)

    

    def move_distance(self, distance_x_m, distance_y_m, distance_z_m,
                      velocity=VELOCITY):
        """
        Move in a straight line.
        positive X is forward
        positive Y is left
        positive Z is up

        :param distance_x_m: The distance to travel along the X-axis (meters)
        :param distance_y_m: The distance to travel along the Y-axis (meters)
        :param distance_z_m: The distance to travel along the Z-axis (meters)
        :param velocity: the velocity of the motion (meters/second)
        :return:
        """
        distance = math.sqrt(distance_x_m * distance_x_m +
                             distance_y_m * distance_y_m +
                             distance_z_m * distance_z_m)
        flight_time = distance / velocity

        velocity_x = velocity * distance_x_m / distance
        velocity_y = velocity * distance_y_m / distance
        velocity_z = velocity * distance_z_m / distance

        self.start_linear_motion(velocity_x, velocity_y, velocity_z)
        time.sleep(flight_time)
        self.stop()

    # Velocity based primitives

    def start_left(self, velocity=VELOCITY):
        """
        Start moving left. This function returns immediately.

        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        self.start_linear_motion(0.0, velocity, 0.0)

    def start_right(self, velocity=VELOCITY):
        """
        Start moving right. This function returns immediately.

        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        self.start_linear_motion(0.0, -velocity, 0.0)

    def start_forward(self, velocity=VELOCITY):
        """
        Start moving forward. This function returns immediately.

        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        self.start_linear_motion(velocity, 0.0, 0.0)

    def start_back(self, velocity=VELOCITY):
        """
        Start moving backwards. This function returns immediately.

        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        self.start_linear_motion(-velocity, 0.0, 0.0)

    def start_up(self, velocity=VELOCITY):
        """
        Start moving up. This function returns immediately.

        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        self.start_linear_motion(0.0, 0.0, velocity)

    def start_down(self, velocity=VELOCITY):
        """
        Start moving down. This function returns immediately.

        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        self.start_linear_motion(0.0, 0.0, -velocity)

    def stop(self):
        """
        Stop any motion and hover.

        :return:
        """
        self._set_vel_setpoint(0.0, 0.0, 0.0, 0.0)

    def start_turn_left(self, rate=RATE):
        """
        Start turning left. This function returns immediately.

        :param rate: The angular rate (degrees/second)
        :return:
        """
        self._set_vel_setpoint(0.0, 0.0, 0.0, -rate)

    def start_turn_right(self, rate=RATE):
        """
        Start turning right. This function returns immediately.

        :param rate: The angular rate (degrees/second)
        :return:
        """
        self._set_vel_setpoint(0.0, 0.0, 0.0, rate)

    def start_circle_left(self, radius_m, velocity=VELOCITY):
        """
        Start a circular motion to the left. This function returns immediately.

        :param radius_m: The radius of the circle (meters)
        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        circumference = 2 * radius_m * math.pi
        rate = 360.0 * velocity / circumference

        self._set_vel_setpoint(velocity, 0.0, 0.0, -rate)

    def start_circle_right(self, radius_m, velocity=VELOCITY):
        """
        Start a circular motion to the right. This function returns immediately

        :param radius_m: The radius of the circle (meters)
        :param velocity: The velocity of the motion (meters/second)
        :return:
        """
        circumference = 2 * radius_m * math.pi
        rate = 360.0 * velocity / circumference

        self._set_vel_setpoint(velocity, 0.0, 0.0, rate)

    def start_linear_motion(self, velocity_x_m, velocity_y_m, velocity_z_m):
        """
        Start a linear motion. This function returns immediately.

        positive X is forward
        positive Y is left
        positive Z is up

        :param velocity_x_m: The velocity along the X-axis (meters/second)
        :param velocity_y_m: The velocity along the Y-axis (meters/second)
        :param velocity_z_m: The velocity along the Z-axis (meters/second)
        :return:
        """
        self._set_vel_setpoint(
            velocity_x_m, velocity_y_m, velocity_z_m, 0.0)

    def _set_vel_setpoint(self, velocity_x, velocity_y, velocity_z, rate_yaw):
        if not self._is_flying:
            raise Exception('Can not move on the ground. Take off first!')
        self._thread.set_vel_setpoint(velocity_x, velocity_y, velocity_z, rate_yaw)

    def _reset_position_estimator(self):
        self._cf.param.set_value('kalman.resetEstimation', '1')
        time.sleep(0.1)
        self._cf.param.set_value('kalman.resetEstimation', '0')
        time.sleep(2)



####### Actually getting it to oscillate ##########################
        
class Drone1(Drone):
    
    
    def _oscillate(self):
        amplitude = 0.25
        self.take_off(0.35,0.6)
        #self.move_distance(0,0,0.25,0.6)
        peak = 2*amplitude
        
        global t

        while True:

            if keyboard.is_pressed('w'):
                oscillate = True
                while oscillate == True:
                    
                    
                    self.move_distance(0,0,peak,v_inp1)
                        
                    t = t+tau1/2
                    disp = np.exp(-z1*wn1*t)*amplitude
                     
                    peak = amplitude + disp
                    amplitude = disp
                    print(peak)
                    time.sleep(0.2)
                        
                    self.move_distance(0,0,-peak,v_inp1)
                        
                    t = t+tau1/2
                    disp = np.exp(-z1*wn1*t)*amplitude
                    peak = amplitude + disp
                    amplitude = disp
                    print(peak)
                    time.sleep(0.2)
             #       self.move_distance(0,0,peak,v_inp1)
                 #       while True:
                    if keyboard.is_pressed('p') or peak < 2.18*10**(-7):
                        oscillate = False
                          #  break
                break
                
        while True:
            if keyboard.is_pressed('s'):
                oscillate == False
                self.land(velocity = 0.6)
                break
        self._cf.close_link()

class Drone2(Drone):



####### Actually getting it to oscillate ##########################
    def _oscillate(self):
        
        self.take_off(0.35,0.6)
        #self.move_distance(0,0,0.25,0.6)
        amplitude = 0.25
        peak = 2*amplitude
        global t

        while True:
            if keyboard.is_pressed('w'):
                oscillate = True
                while oscillate == True:
                    
                    
                    self.move_distance(0,0,peak,v_inp2)
                        
                    t = t+tau2/2
                    disp = np.exp(-z2*wn2*t)*amplitude
                     
                    peak = amplitude + disp
                    amplitude = disp
                    print(peak)
                    time.sleep(0.2)
                        
                    self.move_distance(0,0,-peak,v_inp2)
                        
                    t = t+tau2/2
                    disp = np.exp(-z2*wn2*t)*amplitude
                    peak = amplitude + disp
                    amplitude = disp
                    print(peak)
                    time.sleep(0.2)
               #     self.move_distance(0,0,-peak,v_inp2)
                 #       while True:
                    if keyboard.is_pressed('p') or peak < 2.18*10**(-7):
                        oscillate = False
                          #  break
                break
                
        while True:
            if keyboard.is_pressed('s'):
                oscillate == False
                self.land(velocity = 0.6)
                break
        self._cf.close_link()
##### New Class from motion Commander ######################
                
class _SetPointThread(Thread):
    TERMINATE_EVENT = 'terminate'
    UPDATE_PERIOD = 0.2
    ABS_Z_INDEX = 3

    def __init__(self, cf, update_period=UPDATE_PERIOD):
        Thread.__init__(self)
        self.update_period = update_period

        self._queue = Queue()
        self._cf = cf

        self._hover_setpoint = [0.0, 0.0, 0.0, 0.0]

        self._z_base = 0.0
        self._z_velocity = 0.0
        self._z_base_time = 0.0

    def stop(self):
        """
        Stop the thread and wait for it to terminate

        :return:
        """
        self._queue.put(self.TERMINATE_EVENT)
        self.join()

    def set_vel_setpoint(self, velocity_x, velocity_y, velocity_z, rate_yaw):
        """Set the velocity setpoint to use for the future motion"""
        self._queue.put((velocity_x, velocity_y, velocity_z, rate_yaw))

    def get_height(self):
        """
        Get the current height of the Crazyflie.

        :return: The height (meters)
        """
        return self._hover_setpoint[self.ABS_Z_INDEX]

    def run(self):
        while True:
            try:
                event = self._queue.get(block=True, timeout=self.update_period)
                if event == self.TERMINATE_EVENT:
                    return

                self._new_setpoint(*event)
            except Empty:
                pass

            self._update_z_in_setpoint()
            self._cf.commander.send_hover_setpoint(*self._hover_setpoint)

    def _new_setpoint(self, velocity_x, velocity_y, velocity_z, rate_yaw):
        self._z_base = self._current_z()
        self._z_velocity = velocity_z
        self._z_base_time = time.time()

        self._hover_setpoint = [velocity_x, velocity_y, rate_yaw, self._z_base]

    def _update_z_in_setpoint(self):
        self._hover_setpoint[self.ABS_Z_INDEX] = self._current_z()

    def _current_z(self):
        now = time.time()
        return self._z_base + self._z_velocity * (now - self._z_base_time)

###### Running multiple Drones ####################

if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    # Connect the two Crazyflies and ramps them up-Fadown
    drone1 = Drone1('radio://0/30/2M')
    drone2 = Drone2('radio://1/35/2M')
   # while(le0.connected or le1.connected):
    time.sleep(0.1)