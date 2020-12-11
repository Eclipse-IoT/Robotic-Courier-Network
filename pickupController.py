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
# Author : Gregory Ivo

#Used to manually control the servos, the station names must be passed as args
#Python3 pickupcontroller.py <station1> <station2> <stationN>

from zenoh import Zenoh
import random
import time
import sys



if __name__ == "__main__":
    z = Zenoh({})
    w = z.workspace('/')
    stationNames = []
    if len(sys.argv) < 2:
        stationNames = ['1']
    else:
        stationNames = sys.argv[1:]
    while True:
        for name in stationNames:
            print("station: " + name)
            r = input("1) pickup, 2) Dropoff")
            if (r == '1'):
                print("pickup")
                w.put('/robots/station/'+name+'/control/', 'pickup')
            elif (r == '2'):
                print("dropoff")
                w.put('/robots/station/'+name+'/control/', 'dropoff')
        time.sleep(1)
