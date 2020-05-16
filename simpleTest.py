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
from scipy.spatial.transform import Rotation as R
from scipy.linalg import expm

import sys
sys.path.append('./')
import inverse_kinematics as ik
from bouncing_ball import BouncingBall
#import modern_robotics as mr

def radians(angle):
    return angle * (math.pi/180)
def degrees(angle):
    return angle * (180/math.pi)

def skew_symmetric(screw, theta):
    s_k = np.array([[0,       -screw[2], screw[1], screw[3]],
                    [screw[2],    0,     -screw[0], screw[4]],
                    [-screw[1], screw[0],   0,      screw[5]],
                    [0,          0,         0,         0]])
    s_k *= theta
    return s_k

def get_transform(M, S, theta_list):
    T = M
    for i in range(S.shape[0]):
        T = T @ expm(skew_symmetric(S[i,:].transpose(), theta_list[i]))
    return T


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
import numpy as np

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

    # using proximity sensor to detect when the ball hits the wall (enters proximity range)
    detect_handle = sim.simxGetObjectHandle(clientID, 'Sphere', sim.simx_opmode_blocking)
    prox_handle = sim.simxGetObjectHandle(clientID, 'Proximity_sensor', sim.simx_opmode_blocking)
    sim.simxSetObjectPosition(clientID, detect_handle[1], -1, [0.5, 0.475, 0.0463], sim.simx_opmode_oneshot)
    sim.simxPauseCommunication(clientID, True)
    sim.simxSetObjectFloatParameter(clientID, detect_handle[1], 3001, 2, sim.simx_opmode_oneshot)
    sim.simxSetObjectFloatParameter(clientID, detect_handle[1], 3002, 2, sim.simx_opmode_oneshot)
    sim.simxPauseCommunication(clientID, False)
    sim.simxReadProximitySensor(clientID, prox_handle[1], sim.simx_opmode_streaming)
    sim.simxGetObjectVelocity(clientID, detect_handle[1], sim.simx_opmode_streaming)

    y = 0
    points = []
    velocities = []
    leave = 0

    # enter while loop to read proximity sensor
    while (y == 0):
        # read prox sensor and get detected points
        ret, dS, dP, dOH, dSNV = sim.simxReadProximitySensor(clientID, prox_handle[1], sim.simx_opmode_buffer)
        ret, linear, angular = sim.simxGetObjectVelocity(clientID, detect_handle[1], sim.simx_opmode_buffer)
        # detecting
        if(dS == 1):
            # store all the detected points
            points.append(dP)
            velocities.append(linear)
            leave = 1
        # not detecting and is heading away from wall
        elif(dS == 0 and leave == 1):
            y = 1
    print("Last Ball Detection Coordinates: ", points[-1])
    print("Last Ball Velocities: ", velocities[-1])

    #Let's set the velocity of the ball
    ball_handle = sim.simxGetObjectHandle(clientID, 'Sphere', sim.simx_opmode_blocking)
    # sim.simxSetObjectPosition(clientID, ball_handle[1], -1, [-1, -1, 1], sim.simx_opmode_streaming)
    # sim.simxPauseCommunication(clientID, True)
    # sim.simxSetObjectFloatParameter(clientID, ball_handle[1], sim.sim_objfloatparam_abs_y_velocity, 3, sim.simx_opmode_streaming)
    # sim.simxSetObjectFloatParameter(clientID, ball_handle[1], sim.sim_objfloatparam_abs_z_velocity, 5, sim.simx_opmode_streaming)
    # sim.simxPauseCommunication(clientID, False)
    print("Joint 1 Handles ... ")
    jointHandles=[-1,-1,-1,-1,-1,-1]
    for i in range(6):
       jointHandles[i]=sim.simxGetObjectHandle(clientID, 'UR3_joint' + str(i+1)+'#', sim.simx_opmode_blocking)
       print(jointHandles[i])
    print("    \n")
    base_frame = sim.simxGetObjectHandle(clientID, 'Dummy0', sim.simx_opmode_blocking)
    ee_frame = sim.simxGetObjectHandle(clientID, 'Dummy', sim.simx_opmode_blocking)
    print("Base Frame handle: ", base_frame)
    print("EE Frame handle: ", ee_frame)
    print("    \n")
    print("Joint 2 Handles ... ")
    jointHandles_2 = [-1, -1, -1, -1, -1, -1]
    for i in range(6):
        jointHandles_2[i]=sim.simxGetObjectHandle(clientID, 'UR3_joint' + str(i+1)+'#0', sim.simx_opmode_blocking)
        print(jointHandles_2[i])

    print(" ")
    #Let's try getting the position and orientation of each of the joints:

    ####pos0 = sim.simxGetObjectPosition(clientID, jointHandles_2[0][1], base_frame[1], sim.simx_opmode_blocking)
    ####pos1 = sim.simxGetObjectPosition(clientID, jointHandles_2[0][1], base_frame[1], sim.simx_opmode_blocking)
    # # pos_b_ee = sim.simxGetObjectPosition(clientID, ee_frame[1], base_frame[1], sim.simx_opmode_blocking)
    # # #print(pos_b_ee[1])
    # # ori_b_ee = sim.simxGetObjectOrientation(clientID, ee_frame[1], base_frame[1], sim.simx_opmode_blocking)
    # # #print(ori_b_ee)
    # # rot_m = R.from_euler('xyz', np.array(ori_b_ee[1]))
    # # #print(rot_m.as_dcm().shape)
    # # M = np.zeros((4,4))
    # # M[3][3] = 1
    # # M[0:3, 0:3] = rot_m.as_dcm()
    # # M[0:3, 3] = pos_b_ee[1]
    # # print("Homogeneous transformation Matrix: ")
    # # print(M)
    # # print(" ")
    # # ###ret_code, rot_matrix = sim.simxGetJointMatrix(clientID, jointHandles_2[5][1], sim.simx_opmode_blocking)
    # # ###Get the joint positions and orientations with respect to base frame
    # # np.set_printoptions(precision=3)
    # # w = np.array([[0,0,1], [-1,0,0], [-1,0,0], [-1,0,0], [0,0,1], [1,0,0]])
    # # print("Angular Velocities: ")
    # # print(w, "\n")
    # # v = np.zeros((6,3))
    # # ret_code = 0
    # # for i in range(6):
    # #     ret_code, q = sim.simxGetObjectPosition(clientID, jointHandles_2[i][1], base_frame[1], sim.simx_opmode_blocking)
    # #     q = np.array(q)
    # #     v[i,:] = np.cross(-1 * w[i,:], q)

    # # print("Linear Velocities: ")
    # # print(v, "\n")
    # # S = np.zeros((6,6))
    # # S[:, 0:3] = w
    # # S[:, 3:6] = v
    # # S_t = S.transpose()
    # # print("Screw Matrix: ")
    # # print(S_t, "\n") # debugging

    # # #T = mr.FKinSpace(M, S_t, np.array([radians(90),radians(90),radians(-92),radians(233),radians(33),radians(24)])) #Fill the array with actual values later
    # # T = get_transform(M, S_t, np.zeros((6,)))#print(type(T))
    # # print("New Transformation Matrix: ")
    # # print(T, "\n")
    # # old_pos = np.ones((4,1))
    # # old_pos[0:3] = 0
    # # old_pos.resize((4,1))
    # # new_pose = T @ old_pos
    # # print("New Position: ")
    # # print(new_pose)

    # # new_pose.resize((4,))

    #Set-up some of the RML vectors:
    vel=90
    accel=20
    jerk=40
    currentVel=[0,0,0,0,0,0,0]
    currentAccel=[0,0,0,0,0,0,0]
    maxVel=[vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180]
    maxAccel=[accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180]
    maxJerk=[jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180]
    targetVel=[0,0,0,0,0,0]

    #I'm gonna try looking at the various ways in which the ping pong ball can hit
    #Right side:
    #Max(Theta2): 64.1, Max(Theta3): 84.1, Theta 5: -90, Theta 6: 90
    #Left side:
    #Max(Theta2): -64.1, Max(Theta3): -84.1, Theta 5: 90, Theta 6: 90
    # targetPos0 = [radians(0),radians(64.1),radians(0),radians(0),radians(-90),radians(90)]
    # sim.simxPauseCommunication(clientID, True)
    # for i in range(6):
    #     #print(jointHandles[i])
    #     #print(targetPos0[i])
    #     sim.simxSetJointTargetPosition(clientID, jointHandles_2[i][1], targetPos0[i], sim.simx_opmode_streaming)
    #     sim.simxSetJointTargetVelocity(clientID, jointHandles_2[i][1], targetVel[i], sim.simx_opmode_streaming)

    # sim.simxPauseCommunication(clientID, False)
    # time.sleep(1)
    # targetPos0 = [radians(10),radians(64.1),radians(0),radians(0),radians(-90),radians(90)]
    # sim.simxPauseCommunication(clientID, True)
    # for i in range(6):
    #     #print(jointHandles[i])
    #     #print(targetPos0[i])
    #     sim.simxSetJointTargetPosition(clientID, jointHandles_2[i][1], targetPos0[i], sim.simx_opmode_streaming)
    #     sim.simxSetJointTargetVelocity(clientID, jointHandles_2[i][1], targetVel[i], sim.simx_opmode_streaming)

    # sim.simxPauseCommunication(clientID, False)
    # time.sleep(1)
    sim.simxPauseCommunication(clientID, True)
    for i in range(6):
        #print(jointHandles[i])
        #print(targetPos0[i])
        sim.simxSetJointTargetPosition(clientID, jointHandles_2[i][1], 0, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, jointHandles_2[i][1], targetVel[i], sim.simx_opmode_streaming)

    sim.simxPauseCommunication(clientID, False)
    time.sleep(2)

    # ##Now, let us set up the Position of our dummy to match the position of the end_effector
    # sim.simxSetObjectPosition(clientID, ee_frame[1], base_frame[1], new_pose[0:3], sim.simx_opmode_streaming)

    # ##Then, let's also setup the orientation, after converting them to euler angles
    # new_rot_mat = T[0:3,0:3]
    # new_rot = R.from_dcm(new_rot_mat)
    # new_ori_b_ee = new_rot.as_euler('xyz')
    # sim.simxSetObjectOrientation(clientID, ee_frame[1], base_frame[1], new_ori_b_ee, sim.simx_opmode_streaming
    print("\n", "Now trying inverse kinematics: ")
    # time.sleep(2)
    # _, ball_pos = sim.simxGetObjectPosition(clientID, ball_handle[1], -1, sim.simx_opmode_blocking)

    # _, ball_ori = sim.simxGetObjectOrientation(clientID, ball_handle[1], -1, sim.simx_opmode_blocking)

    # ball_rot = R.from_euler('xyz', ball_ori)
    # ball_rot_mat = ball_rot.as_dcm()
    # ball_pos[0] = ball_pos[0]
    # ball_pos[1] = ball_pos
    
    print("initial velocities: ", velocities[-1])
    print("position from sensor: ", points[-1])
    b_ball = BouncingBall()
    
    pred_pos = b_ball.trajectory(points[-1][0], points[-1][1], points[-1][2], velocities[-1])
    _, prox_dist = sim.simxGetObjectPosition(clientID, prox_handle[1], -1, sim.simx_opmode_streaming)
    T_prox = np.array([[-1, 0, 0, 0.025], 
                        [0, -1, 0, 2.85],
                        [0,0,1,.043], 
                        [0,0,0,1]])
    prox_pos = np.array([[pred_pos[0]], [pred_pos[1]], [pred_pos[2]],[1]])
    ball_pos = T_prox @ prox_pos
    
    # prox_pos = np.array([[-pred_pos[0]+0.025], [pred_pos[1]+2.85], [pred_pos[2]], [1]])
    # ball_pos = prox_pos[:3]

    print("Predicted Position: ", pred_pos)
    print("Ball pred: ", ball_pos[:3])
    #Offsets are for making sure that we can hit the ball
    world_coord = np.array([ball_pos[0,:], ball_pos[1,:], ball_pos[2,:]])
    world_coord = np.abs(world_coord)

    guess_angles = ik.findJointAngles(world_coord[0]+.07, world_coord[1], world_coord[2]+0.1)
    targetPos0_ik = guess_angles

    if(ball_pos[0,:] < 0):
        targetPos0_ik[0] = -guess_angles[0]
        targetPos0_ik[1] = -guess_angles[1]
        targetPos0_ik[2] = -guess_angles[2]
        targetPos0_ik[3] = -guess_angles[3]
        targetPos0_ik[4] = radians(90)
    

    sim.simxPauseCommunication(clientID, True)
    for i in range(6):
        #print(jointHandles[i])
        #print(targetPos0[i])
        sim.simxSetJointTargetPosition(clientID, jointHandles_2[i][1], targetPos0_ik[i], sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, jointHandles_2[i][1], targetVel[i], sim.simx_opmode_streaming)

    sim.simxPauseCommunication(clientID, False)
    time.sleep(1)
    targetPos0_ik[0] = radians(degrees(targetPos0_ik[0] + radians(50)))

    sim.simxPauseCommunication(clientID, True)
    for i in range(6):
        #print(jointHandles[i])
        #print(targetPos0[i])
        sim.simxSetJointTargetPosition(clientID, jointHandles_2[i][1], targetPos0_ik[i], sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, jointHandles_2[i][1], targetVel[i], sim.simx_opmode_streaming)

    sim.simxPauseCommunication(clientID, False)
    time.sleep(2)

    # world_coord = np.array([-.5, .475, .043])
    # world_coord = np.abs(world_coord)

    # guess_angles = ik.findJointAngles(world_coord[0]+.07, world_coord[1], world_coord[2]+0.1)
    # targetPos0_ik = guess_angles

    # if(ball_pos[0,:] > 0):
    #     targetPos0_ik[0] = -guess_angles[0]
    #     targetPos0_ik[1] = -guess_angles[1]
    #     targetPos0_ik[2] = -guess_angles[2]
    #     targetPos0_ik[3] = -guess_angles[3]
    #     targetPos0_ik[4] = radians(90)
    
    
    # sim.simxPauseCommunication(clientID, True)
    # for i in range(6):
    #     #print(jointHandles[i])
    #     #print(targetPos0[i])
    #     sim.simxSetJointTargetPosition(clientID, jointHandles_2[i][1], targetPos0_ik[i], sim.simx_opmode_streaming)
    #     sim.simxSetJointTargetVelocity(clientID, jointHandles_2[i][1], targetVel[i], sim.simx_opmode_streaming)

    # sim.simxPauseCommunication(clientID, False)
    # time.sleep(2)
    # targetPos0_ik[0] = radians(degrees(targetPos0_ik[0] + 50))

    # sim.simxPauseCommunication(clientID, True)
    # for i in range(6):
    #     #print(jointHandles[i])
    #     #print(targetPos0[i])
    #     sim.simxSetJointTargetPosition(clientID, jointHandles_2[i][1], 0, sim.simx_opmode_streaming)
    #     sim.simxSetJointTargetVelocity(clientID, jointHandles_2[i][1], targetVel[i], sim.simx_opmode_streaming)

    # sim.simxPauseCommunication(clientID, False)
    # time.sleep(2)
    # T_b = np.zeros((4,4))
    # T_b[3,3] = 1
    # T_b[0:3, 0:3] = ball_rot_mat
    # T_b[0:3, 3] = ball_pos

    # print("Ball Position: ", ball_pos)
    # print("Ball Rotation: ", ball_rot.as_dcm())

    # #theta_list_guess = np.array([radians(-70), radians(-50), radians(-50), radians(-30), radians(50), radians(0)])
    # theta_list_guess = np.array([radians(0),radians(64.1),radians(0),radians(0),radians(90),radians(90)])
    # theta_list, success = mr.IKinSpace(S, M, T_b, theta_list_guess, 0.01, 0.01)

    # if success:
    #     print(theta_list)
    # else:
    #     print("Not successful, but here's the guess: \n", theta_list)

    #Now, let's check how close we are to the actual ball:
    # sim.simxPauseCommunication(clientID, True)
    # for i in range(6):
    #     #print(jointHandles[i])
    #     #print(targetPos0[i])
    #     sim.simxSetJointTargetPosition(clientID, jointHandles_2[i][1], targetPos0_ik[i], sim.simx_opmode_streaming)
    #     sim.simxSetJointTargetVelocity(clientID, jointHandles_2[i][1], targetVel[i], sim.simx_opmode_streaming)

    # sim.simxPauseCommunication(clientID, False)
    # time.sleep(2)

    #sim.simxSetObjectPosition(clientID, P_ball_handle[1], -1, [-1, -1, 1], sim.simx_opmode_streaming)

    # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
    sim.simxGetPingTime(clientID)

    # Now close the connection to CoppeliaSim:
    sim.simxFinish(clientID)
    # sim.rmlMoveToJointPositions(jointHandles, -1, currentVel, currentAccel, maxVel, maxAccel, maxJerk, targetPos0, targetVel)
    # time.sleep(1)
    # targetPos1 = [radians(90),radians(90),radians(-92),radians(233),radians(33),radians(24)]
    # sim.simxPauseCommunication(clientID, True)
    # for i in range(6):
    #     #print(jointHandles[i])
    #     #print(targetPos0[i])
    #     if(i==0):
    #         sim.simxSetJointTargetPosition(clientID, jointHandles_2[i][1], targetPos1[i], sim.simx_opmode_streaming)
    #         sim.simxSetJointTargetVelocity(clientID, jointHandles_2[i][1], targetVel[i], sim.simx_opmode_streaming)
    #     else:
    #        sim.simxSetJointTargetPosition(clientID, jointHandles_2[i][1], targetPos1[i], sim.simx_opmode_buffer)
    #        sim.simxSetJointTargetVelocity(clientID, jointHandles_2[i][1], targetVel[i], sim.simx_opmode_buffer)

    # sim.simxPauseCommunication(clientID, False)
    # time.sleep(1)

    # # sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos1,targetVel)
    # time.sleep(1)
    # targetPos2 = [radians(90),radians(90),radians(-92),radians(233),radians(33),radians(24)]
    # sim.simxPauseCommunication(clientID, True)
    # for i in range(6):
    #     #print(jointHandles[i])
    #     #print(targetPos0[i])
    #     if(i==0):
    #         sim.simxSetJointTargetPosition(clientID, jointHandles[i][1], targetPos2[i], sim.simx_opmode_streaming)
    #         sim.simxSetJointTargetVelocity(clientID, jointHandles[i][1], targetVel[i], sim.simx_opmode_streaming)
    #     else:
    #        sim.simxSetJointTargetPosition(clientID, jointHandles[i][1], targetPos2[i], sim.simx_opmode_buffer)
    #        sim.simxSetJointTargetVelocity(clientID, jointHandles[i][1], targetVel[i], sim.simx_opmode_buffer)

    # sim.simxPauseCommunication(clientID, False)
    # # sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos2,targetVel)
    # time.sleep(1)
    # targetPos3=[0,0,0,0,0,0]
    # #targetPos2 = [radians(90),radians(90),radians(-92),radians(233),radians(33),radians(24)]
    # sim.simxPauseCommunication(clientID, True)
    # for i in range(6):
    #     #print(jointHandles[i])
    #     #print(targetPos0[i])
    #     if(i==0):
    #         sim.simxSetJointTargetPosition(clientID, jointHandles[i][1], targetPos3[i], sim.simx_opmode_streaming)
    #         sim.simxSetJointTargetVelocity(clientID, jointHandles[i][1], targetVel[i], sim.simx_opmode_streaming)
    #     else:
    #        sim.simxSetJointTargetPosition(clientID, jointHandles[i][1], targetPos3[i], sim.simx_opmode_buffer)
    #        sim.simxSetJointTargetVelocity(clientID, jointHandles[i][1], targetVel[i], sim.simx_opmode_buffer)

    # sim.simxPauseCommunication(clientID, False)
    # time.sleep(1)
    # # sim.rmlMoveToJointPositions(jointHandles,-1,currentVel,currentAccel,maxVel,maxAccel,maxJerk,targetPos3,targetVel)
    # # # Now retrieve streaming data (i.e. in a non-blocking fashion):
    # # startTime=time.time()
    # # sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_streaming) # Initialize streaming
    # # while time.time()-startTime < 5:
    # #     returnCode,data=sim.simxGetIntegerParameter(clientID,sim.sim_intparam_mouse_x,sim.simx_opmode_buffer) # Try to retrieve the streamed data
    # #     if returnCode==sim.simx_return_ok: # After initialization of streaming, it will take a few ms before the first value arrives, so check the return code
    # #         print ('Mouse position x: ',data) # Mouse position x is actualized when the cursor is over CoppeliaSim's window
    # #     time.sleep(0.005)

    # # Now send some data to CoppeliaSim in a non-blocking fashion:
    # sim.simxAddStatusbarMessage(clientID,'Hello CoppeliaSim!',sim.simx_opmode_oneshot)


    # #Let's try reading some vision sensor data
    # sensor_handle = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_blocking)

    # #img = sim.simxGetVisionSensorImage(clientID, sensor_handle[1], 0 , sim.simx_opmode_buffer)
    # #print(type(img))
    # #print(img)
    # #cv2.imshow(img)
    # #return_code, detection_state, aux_packets = sim.simxReadVisionSensor(clientID, sensor_handle[1], sim.simx_opmode_buffer)
    # img = sim.simxGetVisionSensorImage(clientID, sensor_handle[1], 0, sim.simx_opmode_streaming)
    # print(img)
    # #print(return_code)
    # #print(detection_state)

else:
    print ('Failed connecting to remote API server')
print ('Program ended')

#https://youtu.be/l0C5E4iqqRI

# Video link
#https://youtu.be/Qj7Sv0V9yec
