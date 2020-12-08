# Copyright (c) 2014,2020 ADLINK Technology Inc.
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
# Author : Gregory Ivo

#This script must run on each eadge raspberry pi
#Usage: python3 stationManager.py <stationName> <station Cords>
#i.e. : python3 stationManager.py 1 "pose: {header: {frame_id: map}, pose: {position: {x: -2.7, y: -3.3, z: 0.0}, orientation:{x: 0.0, y: 0.0, z: -0.70, w: 0.70}}}"

from zenoh import Zenoh, ChangeKind
import RPi.GPIO as GPIO
import time
from enum import Enum
import sys


class StationStates(Enum):
    WAITING = 1
    PAYLOAD = 2
    SENDING = 3

if len(sys.argv) > 1:
    clientName = sys.argv[1]
    position = '"'+sys.argv[2]+'"'
else:
    clientName = "1"
    position = '"pose: {header: {frame_id: map}, pose: {position: {x: -2.7, y: -3.3, z: 0.0}, orientation:{x: 0.0, y: 0.0, z: -0.70, w: 0.70}}}"'

print('\n' + position + '\n') #Validate if entered position is corrects

openStatus = False

def toggleServo():
    global openStatus
    if openStatus:
        pickUpPayLoad()
    else:
        dropPayload()
    openStatus = not(openStatus)

def button_callback(t):
    print("Button was pushed!")
    callForMoreMatrial()

currentState = StationStates.WAITING
GPIO.setmode(GPIO.BOARD)
GPIO.setup(11,GPIO.OUT)
servo1 = GPIO.PWM(11,50)
servo1.start(0)
GPIO.setup(10, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.add_event_detect(10,GPIO.FALLING,callback=button_callback, bouncetime=3000)

def dropPayload():
    servo1.ChangeDutyCycle(2)
    time.sleep(2)
    servo1.ChangeDutyCycle(0)

def pickUpPayLoad():
    servo1.ChangeDutyCycle(12)
    time.sleep(2)
    servo1.ChangeDutyCycle(0)

"""
    listens to topic: '/robots/station/' + clientName + '/control/'
"""
def listener(change):
    if change.kind == ChangeKind.PUT:
        print('Publication received: "{}" = "{}"'.format(change.path, change.value))
        print(change.value)
        if (change.value.get_content() == 'pickup'):
            dropPayload()
        if (change.value.get_content() == 'dropoff'):
            pickUpPayLoad()

"""
    listens to topic '/robots/station/cross/'
"""
def listenerCross(change):
    if change.kind == ChangeKind.PUT: 
        if (change.value.get_content().split('*')[0] == 'need' and change.value.get_content().split('*')[1] != clientName):
            #requestRobot
            print("calling robot to help station: " + change.value.get_content().split('*')[1])
            w.put('/robots/station/cross/', 'have ' + clientName)
            w.put('/robots/nav/', 'goto*' + clientName + '*' + change.value.get_content().split('*')[1] + '*' + position + '*' + change.value.get_content().split('*')[2] + '*') #Pass Instructions to nav script

"""
    (none)=>(none)
    used to ask robot to visit station
"""
def callForMoreMatrial():
    print('need ' + clientName)
    w.put('/robots/station/cross/', 'need*' + clientName + '*' + position) ##Brodcast that part X is needed

if __name__ == "__main__":
    z = Zenoh({})
    w = z.workspace('/')
    results = w.subscribe('/robots/station/' + clientName + '/control/', listener)
    crossCommunication = w.subscribe('/robots/station/cross/', listenerCross)
    while True:
        ##pushing the physical button and pressing 1 do the same thing
        time.sleep(1)
        r = input("1) if you require more material\n or push pysical button")
        if r == "1":
            callForMoreMatrial()

