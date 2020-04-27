# -*- coding: utf-8 -*-
"""
Created on Sun Jul 05 15:01:58 2015

@author: ACSECKIN
"""

import sim
import time
import cv2
import numpy as np

sim.simxFinish(-1)

clientID = sim.simxStart('127.0.0.1', 19999, True, True, 5000, 5)

if clientID != -1:
    print ('Connected to remote API server')
    print ('Vision Sensor object handling')
    res, v1 = sim.simxGetObjectHandle(clientID, 'blobDetectionCamera_camera', sim.simx_opmode_oneshot_wait)
    print ('Getting first image')
    err, resolution, image = sim.simxGetVisionSensorImage(clientID, v1, 0, sim.simx_opmode_streaming)
    while (sim.simxGetConnectionId(clientID) != -1):
        err, resolution, image = sim.simxGetVisionSensorImage(clientID, v1, 0, sim.simx_opmode_buffer)
        if err == sim.simx_return_ok:
            print("image OK!!!")
            img = np.array(image,dtype=np.uint8)
            img.resize([resolution[0],resolution[1],3])
            cv2.imshow('image',img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        elif err == sim.simx_return_novalue_flag:
            print ("no image yet")
            pass
        else:
          print(err)
else:
  print ("Failed to connect to remote API Server")
  sim.simxFinish(clientID)

cv2.destroyAllWindows()
