# Make sure to have the server side running in CoppeliaSim: 
# in a child script of a CoppeliaSim scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!
def radians(angle):
    return angle * (math.pi/180)

try:
    import sim
except:
    print ('--------------------------------------------------------------')
    print ('"sim.py" could not be imported. This means very probably that')
    print ('either "sim.py" or the remoteApi library could not be found.')
    print ('Make sure both are in the same folder as this file,')
    print ('or appropriately adjust the file "sim.py"')
    print ('--------------------------------------------------------------')
    print ('')

import time
import math

print ('Program started')
sim.simxFinish(-1) # just in case, close all opened connections
clientID=sim.simxStart('127.0.0.1',19999,True,True,5000,5) # Connect to CoppeliaSim
if clientID!=-1:
    print ('Connected to remote API server')

    # Now try to retrieve data in a blocking fashion (i.e. a service call):
    res,objs=sim.simxGetObjects(clientID,sim.sim_handle_all,sim.simx_opmode_blocking)
    if res==sim.simx_return_ok:
        print ('Number of objects in the scene: ',len(objs))
    else:
        print ('Remote API function call returned with error code: ',res)

    time.sleep(2)

    #print(objs)

    jointHandles=[-1,-1,-1,-1,-1,-1]
    for i in range(6):
       jointHandles[i]=sim.simxGetObjectHandle(clientID, 'UR3_joint' + str(i+1)+'#', sim.simx_opmode_blocking)
       print(jointHandles[i])

    #Set-up some of the RML vectors:
    vel=180
    accel=40
    jerk=80
    currentVel=[0,0,0,0,0,0,0]
    currentAccel=[0,0,0,0,0,0,0]
    maxVel=[vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180]
    maxAccel=[accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180]
    maxJerk=[jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180]
    targetVel=[0,0,0,0,0,0]
    
    targetPos0 = [radians(90),radians(90),radians(-92),radians(233),radians(33),radians(24)]
    sim.simxPauseCommunication(clientID, True)
    for i in range(6):
        print(jointHandles[i])
        print(targetPos0[i])
        if(i==0):
            sim.simxSetJointTargetPosition(clientID, jointHandles[i][1], targetPos0[i], sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientID, jointHandles[i][1], targetVel[i], sim.simx_opmode_streaming)
        else:
           sim.simxSetJointTargetPosition(clientID, jointHandles[i][1], targetPos0[i], sim.simx_opmode_buffer)
           sim.simxSetJointTargetVelocity(clientID, jointHandles[i][1], targetVel[i], sim.simx_opmode_buffer) 

    sim.simxPauseCommunication(clientID, False)
    # sim.rmlMoveToJointPositions(jointHandles, -1, currentVel, currentAccel, maxVel, maxAccel, maxJerk, targetPos0, targetVel)
    
    targetPos1 = [radians(90),radians(90),radians(-92),radians(233),radians(33),radians(24)]
    sim.simxPauseCommunication(clientID, True)
    for i in range(6):
        print(jointHandles[i])
        print(targetPos0[i])
        if(i==0):
            sim.simxSetJointTargetPosition(clientID, jointHandles[i][1], targetPos1[i], sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientID, jointHandles[i][1], targetVel[i], sim.simx_opmode_streaming)
        else:
           sim.simxSetJointTargetPosition(clientID, jointHandles[i][1], targetPos1[i], sim.simx_opmode_buffer)
           sim.simxSetJointTargetVelocity(clientID, jointHandles[i][1], targetVel[i], sim.simx_opmode_buffer) 

    sim.simxPauseCommunication(clientID, False)
    # sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos1,targetVel)
    time.sleep(1)
    targetPos2 = [radians(90),radians(90),radians(-92),radians(233),radians(33),radians(24)]
    sim.simxPauseCommunication(clientID, True)
    for i in range(6):
        print(jointHandles[i])
        print(targetPos0[i])
        if(i==0):
            sim.simxSetJointTargetPosition(clientID, jointHandles[i][1], targetPos2[i], sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientID, jointHandles[i][1], targetVel[i], sim.simx_opmode_streaming)
        else:
           sim.simxSetJointTargetPosition(clientID, jointHandles[i][1], targetPos2[i], sim.simx_opmode_buffer)
           sim.simxSetJointTargetVelocity(clientID, jointHandles[i][1], targetVel[i], sim.simx_opmode_buffer) 

    sim.simxPauseCommunication(clientID, False)
    # sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos2,targetVel)
    time.sleep(1)
    targetPos3=[0,0,0,0,0,0]
    #targetPos2 = [radians(90),radians(90),radians(-92),radians(233),radians(33),radians(24)]
    sim.simxPauseCommunication(clientID, True)
    for i in range(6):
        print(jointHandles[i])
        print(targetPos0[i])
        if(i==0):
            sim.simxSetJointTargetPosition(clientID, jointHandles[i][1], targetPos3[i], sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientID, jointHandles[i][1], targetVel[i], sim.simx_opmode_streaming)
        else:
           sim.simxSetJointTargetPosition(clientID, jointHandles[i][1], targetPos3[i], sim.simx_opmode_buffer)
           sim.simxSetJointTargetVelocity(clientID, jointHandles[i][1], targetVel[i], sim.simx_opmode_buffer) 

    sim.simxPauseCommunication(clientID, False)    
    time.sleep(1)
    # sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos3,targetVel)
    # # Now retrieve streaming data (i.e. in a non-blocking fashion):
    startTime=time.time()
    sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_streaming) # Initialize streaming
    while time.time()-startTime < 5:
        returnCode,data=sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_buffer) # Try to retrieve the streamed data
        if returnCode==sim.simx_return_ok: # After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
            print ('Mouse position x: ',data) # Mouse position x is actualized when the cursor is over CoppeliaSim's window
        time.sleep(0.005)

    # Now send some data to CoppeliaSim in a non-blocking fashion:
    sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
else:
    print ('Failed connecting to remote API server')
print ('Program ended')
