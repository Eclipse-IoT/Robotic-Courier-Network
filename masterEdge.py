# Copyright (c) 2020 Eclipse Foundation.
#
# See the NOTICE file(s) distributed with this work for additional
# information regarding copyright ownership.
#
# This program and the accompanying materials are made available under the
# terms of the Eclipse Public License 2.0 which is available at
# http://www.eclipse.org/legal/epl-2.0, or the Apache License, Version 2.0
# which is available at https://www.apache.org/licenses/LICENSE-2.0.
#
# SPDX-License-Identifier: EPL-2.0 OR Apache-2.0
#
# Original Author: Gabriele Baldoni
# Modified By : Gregory Ivo

#Used to translate Zenoh-to-ROS commands
#Usage: Python3 pickupcontroller.py <station1> <station2> <stationN>

import os
import select
import sys
import tty
import signal

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from math import atan2

import rclpy
from rclpy.qos import QoSProfile

from zenoh import Zenoh, Value
import time
import json


BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84

WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.22
ANG_VEL_STEP_SIZE = 0.71

TURTLEBOT3_MODEL = 'burger'

CONTROL_RESOURCE = '/turtlebot/move'
STATE_RESOURCE = '/turtlebot/status'
STATION_RESOURCE = '/robots/nav/'


global_x = 0
global_y = 0

global_theta = 0

class Controller():
    def __init__(self):
        self.zenoh = Zenoh({})
        self.ws = self.zenoh.workspace('/')
        rclpy.init()
        self.qos = QoSProfile(depth=10)
        self.node = rclpy.create_node('teleop_yaks')
        self.pub = self.node.create_publisher(Twist, 'cmd_vel', self.qos)
        self.sub = self.node.create_publisher(Odometry, '/odometry/filtered', self.qos)
        self.running = False
        self.target_linear_velocity   = 0.0
        self.target_angular_velocity  = 0.0
        self.control_linear_velocity  = 0.0
        self.control_angular_velocity = 0.0


    def listener(self, change):
        print('>> [Subscription listener] Received PUT : "{}"'.format(change))
        v = json.loads(change.value.get_content())
        self.move(v)

    """
        For simplicity, used simple string commands from zenoh, ideally this would be done with JSON

        listening for "echo*<station1>*<station2>*<station1Pos>*<station2pos>"
        No error checking has been done for simplicity so be careful!

        robot moves to station1 enters, tells servo to open (Drop),exits, moves to station2, enters, tells servo to close (Pickup), exits
    """
    def stationListener(self, change):
        print(change.value.get_content())
        if (change.value.get_content().split('*')[0] == 'goto'):
            print(change.value.get_content())
            print("Pick up from station: " + change.value.get_content().split('*')[1])
            #Goto station 1
            print('\n' + "ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose " + change.value.get_content().split('*')[3] + '\n')
            os.system("ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose " + change.value.get_content().split('*')[3]) ##Time was tight so i had to cheat a bit
            self.fwd()
            time.sleep(5)
            self.halt()
            time.sleep(1)
            self.ws.put('/robots/station/'+change.value.get_content().split('*')[1]+'/control/', 'dropoff')
            time.sleep(1)
            self.bwd()
            time.sleep(5)
            self.halt()
            print("Drop off at station: " + change.value.get_content().split('*')[2])
            #GotoStation 2
            print("ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose " + change.value.get_content().split('*')[4])
            os.system("ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose " + change.value.get_content().split('*')[4])
            self.fwd()
            time.sleep(5)
            self.halt()
            time.sleep(1)
            self.ws.put('/robots/station/'+change.value.get_content().split('*')[2]+'/control/', 'pickup')
            time.sleep(1)
            self.bwd()
            time.sleep(5)
            self.halt()


    def move(self, v):
        print('>> Move data is {}"'.format(v))
        # d = {
        #     'fwd':self.fwd,
        #     'bwd':self.bwd,
        #     'h':self.halt,
        #     'sx':self.sx,
        #     'dx':self.dx
        # }
        # f = d.get(v, None)
        # if f is not None:
        #     f()
        self.target_angular_velocity = v['control_angular_velocity']
        self.target_linear_velocity = v['control_linear_velocity']
        self.send_vel()


    def fwd(self):
        self.target_linear_velocity = check_linear_limit_velocity(self.target_linear_velocity + LIN_VEL_STEP_SIZE)
        self.send_vel()

    def bwd(self):
        self.target_linear_velocity = check_linear_limit_velocity(self.target_linear_velocity - LIN_VEL_STEP_SIZE)
        self.send_vel()

    def halt(self):
        self.target_linear_velocity   = 0.0
        self.target_angular_velocity  = 0.0
        self.control_linear_velocity  = 0.0
        self.control_angular_velocity = 0.0
        self.send_vel()

    def sx(self):
        self.target_angular_velocity = check_angular_limit_velocity(self.target_angular_velocity + ANG_VEL_STEP_SIZE)
        self.send_vel()

    def dx(self):
        self.target_angular_velocity = check_angular_limit_velocity(self.target_angular_velocity - ANG_VEL_STEP_SIZE)
        self.send_vel()

    def start(self):
        self.running = True
        self.sid = self.ws.subscribe(CONTROL_RESOURCE, self.listener)
        self.sid = self.ws.subscribe(STATION_RESOURCE, self.stationListener)
        self.send_vel()
        while self.running:
            time.sleep(1)


    def stop(self):
        self.ws.unsubscribe(self.sid)
        self.running = False
    
    def gotoPoint(self, x, y):
        speed = Twist()
        goal = Point()
        goal.x = x
        goal.y = y
        while True:
            inc_x = goal.x -x
            inc_y = goal.y -y

            angle_to_goal = atan2(inc_y, inc_x)

            if abs(angle_to_goal - theta) > 0.1:
                speed.linear.x = 0.0
                speed.angular.z = 0.3
            else:
                speed.linear.x = 0.5
                speed.angular.z = 0.0

            self.pub.publish(speed)    

    def send_vel(self):
        twist = Twist()

        self.control_linear_velocity = make_simple_profile(
                    self.control_linear_velocity,
                    self.target_linear_velocity,
                    (LIN_VEL_STEP_SIZE/2.0))

        twist.linear.x = self.control_linear_velocity
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        self.control_angular_velocity = make_simple_profile(
                self.control_angular_velocity,
                self.target_angular_velocity,
                (ANG_VEL_STEP_SIZE/2.0))

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = self.control_angular_velocity

        self.pub.publish(twist)
        d = {
        'target_linear_velocity': self.target_linear_velocity,
        'target_angular_velocity': self.target_angular_velocity,
        'control_linear_velocity': self.control_linear_velocity,
        'control_angular_velocity': self.control_angular_velocity
        }
        js_state = json.dumps(d)
        self.ws.put(STATE_RESOURCE, Value.Json(js_state))



def make_simple_profile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_linear_limit_velocity(velocity):
    if TURTLEBOT3_MODEL == 'burger':
        return constrain(velocity, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    else:
        return constrain(velocity, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)

def check_angular_limit_velocity(velocity):
    if TURTLEBOT3_MODEL == 'burger':
        return constrain(velocity, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    else:
        return constrain(velocity, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)


def main():
    c = Controller()

    def sh(sig, frame):
        c.stop()

    signal.signal(signal.SIGINT, sh)

    c.start()






if __name__ == '__main__':
    main()
