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
import bouncing_ball
import threading
import time
import math
import numpy as np

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
    sys.exit()

ball_coord = []
ball_linear = []
ball_angular = []
hit_wall = False
ball_hit = True
detect_handle = (0, 2)
prox_handle = (0, 5)
stop_prog = False

#ball_hit_lock = threading.Lock()

#import modern_robotics as mr
def get_ball_info(clientID):
    """
    stores the linear velocity and the position variable of the last time that the ball hit the wall
    stores it in the global coordinates ball_coord and ball_linear

    @param clientID: ID used to connect to CoppeliaSim's environment
    @return None
    """
    global ball_coord
    global ball_linear
    global hit_wall
    global ball_hit
    global detect_handle
    global prox_handle
    global stop_prog
    print("Getting ball info: ")
    # while 1:
    #     if cnt == 0:
    #         ret, ball_handle = sim.simxGetObjectHandle(clientID, 'Sphere', sim.simx_opmode_blocking)
    #         ret, ball_coord = sim.simxGetObjectPosition(clientID, ball_handle, -1, sim.simx_opmode_streaming)
    #         ret, ball_linear, ball_angular = sim.simxGetObjectVelocity(clientID, ball_handle, sim.simx_opmode_streaming)
    #         cnt += 1
    #     else:
    #         ret, ball_coord = sim.simxGetObjectPosition(clientID, ball_handle, -1, sim.simx_opmode_buffer)
    #         ret, ball_linear_new, ball_angular = sim.simxGetObjectVelocity(clientID, ball_handle, sim.simx_opmode_buffer)
    #         if(ball_linear_new[1] < 0 and ball_linear[1] > 0):
    #             #if we come here, then the velocity direction has changed, meaning we hit the wall
    #             hit_wall = True
    # detect_handle = sim.simxGetObjectHandle(clientID, 'Sphere', sim.simx_opmode_blocking)
    # print("Sphere Handle: ", detect_handle[1])
    # prox_handle = sim.simxGetObjectHandle(clientID, 'Proximity_sensor', sim.simx_opmode_blocking)
    # print("Proximity Sensor Handle: ", prox_handle[1])
    y = 0
    points = []
    velocities = []
    leave = 0
    while True:
        if stop_prog:
            break
        if ball_hit:
            #print("We hit the ball, we are now processing")
            y = 0 #Represents if the ball is moving away from the wall or not
        else:
            #print("waiting for ball to be hit")
            continue
        while (y == 0): #While ball still coming towards wall
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
                hit_wall = True #To show we have just hit the wall
                ball_hit = False #To show that the ball has not been hit yet. Ideally, we don't start this loop
                                #until the ball has been hit
                leave = 0
                print("Storing left velocity and position")
                if len(velocities) > 0 :
                    ball_linear = velocities[-1]
                if len(points) > 0:
                    ball_coord = points[-1]
                velocities = []
                points = []


def hit_ball(clientID):
    """
    Method to actually hit the ball with the robot

    """
    global hit_wall
    global ball_coord
    global ball_linear
    global ball_hit

    print("Hitting Ball: ")
    # #Get handles for detecting object
    # detect_handle = sim.simxGetObjectHandle(clientID, 'Sphere', sim.simx_opmode_blocking)

    # #Get handles for detecting the proximity sensor
    # prox_handle = sim.simxGetObjectHandle(clientID, 'Proximity_sensor', sim.simx_opmode_blocking)

    #Create an instance to compute the ball trajectory
    b_ball = bouncing_ball.BouncingBall()

    #getting joint handles and initializing the joints in the simulation
    jointHandles=[-1,-1,-1,-1,-1,-1]
    for i in range(6):
       jointHandles[i]=sim.simxGetObjectHandle(clientID, 'UR3_joint' + str(i+1)+'#0', sim.simx_opmode_blocking)
    
    for i in range(6): print (jointHandles[i])
    
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
    
    sim.simxPauseCommunication(clientID, True)
    for i in range(6):
        sim.simxSetJointTargetPosition(clientID, jointHandles[i][1], 0, sim.simx_opmode_streaming)
        sim.simxSetJointTargetVelocity(clientID, jointHandles[i][1], targetVel[i], sim.simx_opmode_streaming)

    sim.simxPauseCommunication(clientID, False)
    time.sleep(2)

    #start the feedback loop    
    while True:
        if not hit_wall:
            print("Ball hasn't hit wall yet")
            continue

        if hit_wall:
            print("Ball has hit the wall")

        #Predict the position of the ball from the wall and transform it to world coordinates
        print("Predicting Trajectory ...")
        pred_pos = b_ball.trajectory(ball_coord[0], ball_coord[1], ball_coord[2], ball_linear)
        T_prox = np.array([[-1, 0, 0, 0.025], 
                        [0, -1,  0, 2.85],
                        [0, 0,  1,  .043], 
                        [0, 0,  0,     1]])
        prox_pos = np.array([[pred_pos[0]], [pred_pos[1]], [pred_pos[2]],[1]])
        ball_pos = np.linalg.inv(T_prox) @ prox_pos
        print(ball_pos)
        
        targetPos0_ik = ik.findJointAngles(ball_pos[0, :], ball_pos[1,:]+0.1, ball_pos[2,:]+0.1)

        print(targetPos0_ik)

        #Invert joint angles if on left side of robot
        if ball_pos[0] < 0:
            for i in range(len(targetPos0_ik)):
                targetPos0_ik[i] = -targetPos0_ik[i]
        
        print("Applying the hitting motion")
        print(ball_pos)
        #Apply the hitting motion using the new joint angles
        sim.simxPauseCommunication(clientID, True)
        for i in range(6):
            #print(jointHandles[i])
            #print(targetPos0[i])
            sim.simxSetJointTargetPosition(clientID, jointHandles[i][1], targetPos0_ik[i], sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientID, jointHandles[i][1], targetVel[i], sim.simx_opmode_streaming)

        sim.simxPauseCommunication(clientID, False)
        time.sleep(2)

        #Now, attach a proximity sensor to the racquet and see if it detects a ball. If it does, let's start the 
        #hitting motion
        #Actually, for now, let's just not worry about this. Let's make sure that the trajectory generation and the ball hitting
        #works
        targetPos0_ik[0] = targetPos0_ik[0] + (40 * np.pi/180) #To simulate a hit
        sim.simxPauseCommunication(clientID, True)
        for i in range(6):
            #print(jointHandles[i])
            #print(targetPos0[i])
            sim.simxSetJointTargetPosition(clientID, jointHandles[i][1], targetPos0_ik[i], sim.simx_opmode_streaming)
            sim.simxSetJointTargetVelocity(clientID, jointHandles[i][1], targetVel[i], sim.simx_opmode_streaming)
        sim.simxPauseCommunication(clientID, False)
        ball_hit=True
        hit_wall = False
    



def radians(angle):
    return angle * (math.pi/180)

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


def main():
    global hit_wall
    global ball_coord
    global ball_linear
    global ball_hit
    global detect_handle
    global prox_handle
    global stop_prog
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
        #Giving an initial velocity to the ball before starting the thread
        detect_handle = sim.simxGetObjectHandle(clientID, 'Sphere', sim.simx_opmode_blocking)
        prox_handle = sim.simxGetObjectHandle(clientID, 'Proximity_sensor', sim.simx_opmode_blocking)
        racket_handle = sim.simxGetObjectHandle(clientID, 'Proximity_sensor0', sim.simx_opmode_blocking)
        dummy_handle = sim.simxGetObjectHandle(clientID, 'Cylinder', sim.simx_opmode_blocking)
        sim.simxSetObjectPosition(clientID, detect_handle[1], -1, [0.65, 0.47, 0.0463], sim.simx_opmode_oneshot)
        sim.simxPauseCommunication(clientID, True)
        sim.simxSetObjectFloatParameter(clientID, detect_handle[1], 3001, 1, sim.simx_opmode_oneshot)
        #sim.simxSetObjectFloatParameter(clientID, detect_handle[1], 3000, -0.01, sim.simx_opmode_oneshot)
        sim.simxSetObjectFloatParameter(clientID, detect_handle[1], 3002, 1, sim.simx_opmode_oneshot)
        sim.simxPauseCommunication(clientID, False)
        sim.simxReadProximitySensor(clientID, prox_handle[1], sim.simx_opmode_streaming)
        sim.simxGetObjectVelocity(clientID, detect_handle[1], sim.simx_opmode_streaming)
        sim.simxReadProximitySensor(clientID, racket_handle[1], sim.simx_opmode_streaming)
        sim.simxGetObjectPosition(clientID, dummy_handle[1], -1, sim.simx_opmode_streaming)
        ball_thread = threading.Thread(target=get_ball_info, args=({clientID:clientID}))
        try:        
            #getting joint handles and initializing the joints in the simulation
            print("1. getting joint angles")
            jointHandles = []
            for i in range(6):
                handle = sim.simxGetObjectHandle(clientID, 'UR3_joint' + str(i+1)+'#0', sim.simx_opmode_blocking)
                jointHandles.append(handle)
                time.sleep(0.01)
            
            for i in range(6): print (jointHandles[i])
            
            #hit_thread = threading.Thread(target=hit_ball, args=({clientID:clientID}))
            ball_thread.daemon = True
            #hit_thread.daemon = True
            #hit_thread.start()
            ball_thread.start()
            # #Get handles for detecting object
            # detect_handle = sim.simxGetObjectHandle(clientID, 'Sphere', sim.simx_opmode_blocking)

            # #Get handles for detecting the proximity sensor
            # prox_handle = sim.simxGetObjectHandle(clientID, 'Proximity_sensor', sim.simx_opmode_blocking)

            #Create an instance to compute the ball trajectory
            print("1. Initializing bouncing trajectory function")
            b_ball = bouncing_ball.BouncingBall()

            
            #Set-up some of the RML vectors:
            vel=60
            accel=10
            jerk=20
            currentVel=[0,0,0,0,0,0,0]
            currentAccel=[0,0,0,0,0,0,0]
            maxVel=[vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180]
            maxAccel=[accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180]
            maxJerk=[jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180]
            targetVel=[0,0,0,0,0,0]
            
            sim.simxPauseCommunication(clientID, True)
            for i in range(6):
                sim.simxSetJointTargetPosition(clientID, jointHandles[i][1], 0, sim.simx_opmode_streaming)
                sim.simxSetJointTargetVelocity(clientID, jointHandles[i][1], targetVel[i], sim.simx_opmode_streaming)
            sim.simxPauseCommunication(clientID, False)
            time.sleep(2)

            #start the feedback loop    
            while True:
                if not hit_wall:
                    #print("We haven't hit the wall yet")
                    continue
                    
                if hit_wall:
                    print("We hit wall ....")

                #Predict the position of the ball from the wall and transform it to world coordinates
                print("Predicting Trajectory ...")
                pred_pos = b_ball.trajectory(ball_coord[0], ball_coord[1], ball_coord[2], ball_linear)
                T_prox = np.array([[-1, 0, 0, 0.025], 
                                [0, -1,  0, 2.85],
                                [0, 0,  1,  -.05], 
                                [0, 0,  0,     1]])
                prox_pos = np.array([[pred_pos[0]], [pred_pos[1]], [pred_pos[2]],[1]])
                ball_pos = T_prox @ prox_pos
                print(ball_pos)
                #set the left flag to true if on left side
                left = 0
                if ball_pos[0,:] < 0: left=1

                #convert to right-side coordinates for IK
                ball_pos[0,:] = np.abs(ball_pos[0,:])

                if ball_pos[0, :] > 0.9:
                    print("Ball too far away from robot. will not hit it ...")
                    raise ValueError
                elif ball_pos[0,:] < 0.2:
                    print("Ball too close. cannot hit it...")
                    raise ValueError
                
                twist_angle = (-92.85) * (ball_pos[0,:]) + 90
                print(ball_pos, "left?: ", left)
                targetPos0_ik = ik.findJointAngles(ball_pos[0,:], ball_pos[1,:], ball_pos[2,:] + 0.15)

                #Invert joint angles if on left side of robot
                
                for i in range(len(targetPos0_ik)):
                    targetPos0_ik[i] = ((-2 * left) + 1) * targetPos0_ik[i]
                
                print("Applying the hitting motion")
                #Apply the hitting motion using the new joint angles
                end_pose = []
                targetPos0_ik[0] = targetPos0_ik[0] + ((-2 * left) + 1) * (0 * np.pi/180) #To simulate a hit
                targetPos0_ik[4] = targetPos0_ik[4] + ((-2 * left) + 1) * (0 * np.pi/180)
                sim.simxPauseCommunication(clientID, True)
                for i in range(6):
                    #print(jointHandles[i])
                    #print(targetPos0[i])
                    sim.simxSetJointTargetPosition(clientID, jointHandles[i][1], targetPos0_ik[i], sim.simx_opmode_streaming)
                    sim.simxSetJointTargetVelocity(clientID, jointHandles[i][1], targetVel[i], sim.simx_opmode_streaming)

                sim.simxPauseCommunication(clientID, False)
                

                #Now read from the proximity sensor to see if it detects any ball
                ret, dS, dP, dOH, dSNV = sim.simxReadProximitySensor(clientID, racket_handle[1], sim.simx_opmode_buffer)
                while(dS == 0):
                    _, end_pose = sim.simxGetObjectPosition(clientID, dummy_handle[1], -1, sim.simx_opmode_buffer)
                    ret, dS, dP, dOH, dSNV = sim.simxReadProximitySensor(clientID, racket_handle[1], sim.simx_opmode_buffer)
                
                print("Status of Ball is: ", dS)
                #Now, attach a proximity sensor to the racquet and see if it detects a ball. If it does, let's start the 
                #hitting motion
                #Actually, for now, let's just not worry about this. Let's make sure that the trajectory generation and the ball hitting
                #works
                print("Twist angle is: ", twist_angle)
                targetPos0_ik[0] = targetPos0_ik[0] + ((-2 * left) + 1) * ((1 * twist_angle) * np.pi/180) #To simulate a hit
                
                targetPos0_ik[4] = targetPos0_ik[4] + ((-2 * left) + 1) * (-1 * twist_angle * np.pi/180)
                # sim.simxPauseCommunication(clientID, True)
                # for i in range(6):
                #     #print(jointHandles[i])
                #     #print(targetPos0[i])
                #     sim.simxSetJointTargetPosition(clientID, jointHandles[i][1], targetPos0_ik[i], sim.simx_opmode_buffer)
                #     sim.simxSetJointTargetVelocity(clientID, jointHandles[i][1], targetVel[i], sim.simx_opmode_buffer)
                # sim.simxPauseCommunication(clientID, False)
                # time.sleep(2)
                ball_hit = True
                hit_wall = False
                sim.simxPauseCommunication(clientID, True)
                for i in [0,4]:
                    sim.simxSetJointTargetPosition(clientID, jointHandles[i][1], targetPos0_ik[i], sim.simx_opmode_streaming)
                    sim.simxSetJointTargetVelocity(clientID, jointHandles[i][1], targetVel[i], sim.simx_opmode_streaming)
                sim.simxPauseCommunication(clientID, False)
                
                time.sleep(2)

                sim.simxPauseCommunication(clientID, True)
                #Reset after hitting ball
                for i in range(6):
                    #print(jointHandles[i])
                    #print(targetPos0[i])
                    sim.simxSetJointTargetPosition(clientID, jointHandles[i][1], 0, sim.simx_opmode_streaming)
                    sim.simxSetJointTargetVelocity(clientID, jointHandles[i][1], targetVel[i], sim.simx_opmode_streaming)
                sim.simxPauseCommunication(clientID, False)
                ##Inverse Kinematics experiment and analysis: CAN BE COMMENTED OUT
                ball_pos[0,:] = (-2 * left + 1) * ball_pos[0,:]
                ball_pos[2,:] = ball_pos[2, :] + 0.15
                end_pose  = np.array(end_pose).reshape((3,1))
                print("Desired position: \n", ball_pos[:3])
                print("End-effector position: \n", end_pose[:3])
                # print("Error (MMSE): ", np.linalg.norm(ball_pos[:3] - end_pose[:3]))
                # x_err = np.abs((end_pose[0,:] - ball_pos[0,:])/(ball_pos[0,:])) * 100
                # y_err = np.abs((end_pose[1,:] - ball_pos[1,:])/(ball_pos[1,:])) * 100
                # z_err = np.abs((end_pose[2,:] - ball_pos[2,:])/(ball_pos[2,:])) * 100
                # print("Error in X (%): ", x_err)
                # print("Error in Y (%): ", y_err)
                # print("Error in Z (%): ", z_err)
                # print("Total Error (%): ", (x_err + y_err + z_err)/3)
        except:
            print(sys.exc_info())
            #hit_thread.join()
            stop_prog = True
            print("Exception encountered, stopping program")
            #ball_thread.join()
            print("Ball thread stopped")
            sim.simxGetPingTime(clientID)
            # Now close the connection to CoppeliaSim:
            sim.simxFinish(clientID)
    else:
        print ('Failed connecting to remote API server')
        print ('Program ended')
        sim.simxGetPingTime(clientID)
        # Now close the connection to CoppeliaSim:
        sim.simxFinish(clientID)
    
    
        #print(objs)

        # # trying to get blob camera to detect the green ball in its line of sight
        # # need to replace simxReadVisionSensor with simxCheckVisionSensor so
        # # need to add this function to the sim.py library
        # blob_camera = sim.simxGetObjectHandle(clientID, 'blobDetectionCamera_camera', sim.simx_opmode_blocking)
        # vision_camera = sim.simxGetObjectHandle(clientID, 'Vision_sensor', sim.simx_opmode_blocking)
        # img_B = sim.simxGetVisionSensorImage(clientID, vision_camera[1], 0 , sim.simx_opmode_streaming)
        # detecting = sim.simxReadVisionSensor(clientID, vision_camera[1], sim.simx_opmode_blocking)
        # if(detecting[1] == 1):
        #     print('Dectecting Ball \n')
        # else:
        #     print('Not Dectecting Ball \n')

        # #Let's set the velocity of the ball
        # ball_handle = sim.simxGetObjectHandle(clientID, 'Sphere', sim.simx_opmode_blocking)
        # # sim.simxSetObjectPosition(clientID, ball_handle[1], -1, [-1, -1, 1], sim.simx_opmode_streaming)
        # # sim.simxPauseCommunication(clientID, True)
        # # sim.simxSetObjectFloatParameter(clientID, ball_handle[1], sim.sim_objfloatparam_abs_y_velocity, 3, sim.simx_opmode_streaming)
        # # sim.simxSetObjectFloatParameter(clientID, ball_handle[1], sim.sim_objfloatparam_abs_z_velocity, 5, sim.simx_opmode_streaming)
        # # sim.simxPauseCommunication(clientID, False)
        # print("Joint 1 Handles ... ")
        # jointHandles=[-1,-1,-1,-1,-1,-1]
        # for i in range(6):
        # jointHandles[i]=sim.simxGetObjectHandle(clientID, 'UR3_joint' + str(i+1)+'#', sim.simx_opmode_blocking)
        # print(jointHandles[i])
        # print("    \n")
        # base_frame = sim.simxGetObjectHandle(clientID, 'Dummy0', sim.simx_opmode_blocking)
        # ee_frame = sim.simxGetObjectHandle(clientID, 'Dummy', sim.simx_opmode_blocking)
        # print("Base Frame handle: ", base_frame)
        # print("EE Frame handle: ", ee_frame)
        # print("    \n")
        # print("Joint 2 Handles ... ")
        # jointHandles_2 = [-1, -1, -1, -1, -1, -1]
        # for i in range(6):
        #     jointHandles_2[i]=sim.simxGetObjectHandle(clientID, 'UR3_joint' + str(i+1)+'#0', sim.simx_opmode_blocking)
        #     print(jointHandles_2[i])

        # print(" ")
        # #Let's try getting the position and orientation of each of the joints:

        # ####pos0 = sim.simxGetObjectPosition(clientID, jointHandles_2[0][1], base_frame[1], sim.simx_opmode_blocking)
        # ####pos1 = sim.simxGetObjectPosition(clientID, jointHandles_2[0][1], base_frame[1], sim.simx_opmode_blocking)
        # pos_b_ee = sim.simxGetObjectPosition(clientID, ee_frame[1], base_frame[1], sim.simx_opmode_blocking)
        # #print(pos_b_ee[1])
        # ori_b_ee = sim.simxGetObjectOrientation(clientID, ee_frame[1], base_frame[1], sim.simx_opmode_blocking)
        # #print(ori_b_ee)
        # rot_m = R.from_euler('xyz', np.array(ori_b_ee[1]))
        # #print(rot_m.as_dcm().shape)
        # M = np.zeros((4,4))
        # M[3][3] = 1
        # M[0:3, 0:3] = rot_m.as_dcm()
        # M[0:3, 3] = pos_b_ee[1]
        # print("Homogeneous transformation Matrix: ")
        # print(M)
        # print(" ")
        # ###ret_code, rot_matrix = sim.simxGetJointMatrix(clientID, jointHandles_2[5][1], sim.simx_opmode_blocking)
        # ###Get the joint positions and orientations with respect to base frame
        # np.set_printoptions(precision=3)
        # w = np.array([[0,0,1], [-1,0,0], [-1,0,0], [-1,0,0], [0,0,1], [1,0,0]])
        # print("Angular Velocities: ")
        # print(w, "\n")
        # v = np.zeros((6,3))
        # ret_code = 0
        # for i in range(6):
        #     ret_code, q = sim.simxGetObjectPosition(clientID, jointHandles_2[i][1], base_frame[1], sim.simx_opmode_blocking)
        #     q = np.array(q)
        #     v[i,:] = np.cross(-1 * w[i,:], q)

        # print("Linear Velocities: ")
        # print(v, "\n")
        # S = np.zeros((6,6))
        # S[:, 0:3] = w
        # S[:, 3:6] = v
        # S_t = S.transpose()
        # print("Screw Matrix: ")
        # print(S_t, "\n") # debugging

        # #T = mr.FKinSpace(M, S_t, np.array([radians(90),radians(90),radians(-92),radians(233),radians(33),radians(24)])) #Fill the array with actual values later
        # T = get_transform(M, S_t, np.zeros((6,)))#print(type(T))
        # print("New Transformation Matrix: ")
        # print(T, "\n")
        # old_pos = np.ones((4,1))
        # old_pos[0:3] = 0
        # old_pos.resize((4,1))
        # new_pose = T @ old_pos
        # print("New Position: ")
        # print(new_pose)

        # new_pose.resize((4,))

        # #Set-up some of the RML vectors:
        # vel=90
        # accel=20
        # jerk=40
        # currentVel=[0,0,0,0,0,0,0]
        # currentAccel=[0,0,0,0,0,0,0]
        # maxVel=[vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180,vel*math.pi/180]
        # maxAccel=[accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180,accel*math.pi/180]
        # maxJerk=[jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180,jerk*math.pi/180]
        # targetVel=[0,0,0,0,0,0]
        
        # #I'm gonna try looking at the various ways in which the ping pong ball can hit
        # #Right side: 
        # #Max(Theta2): 64.1, Max(Theta3): 84.1, Theta 5: -90, Theta 6: 90
        # #Left side: 
        # #Max(Theta2): -64.1, Max(Theta3): -84.1, Theta 5: 90, Theta 6: 90
        # # targetPos0 = [radians(0),radians(64.1),radians(0),radians(0),radians(-90),radians(90)]
        # # sim.simxPauseCommunication(clientID, True)
        # # for i in range(6):
        # #     #print(jointHandles[i])
        # #     #print(targetPos0[i])
        # #     sim.simxSetJointTargetPosition(clientID, jointHandles_2[i][1], targetPos0[i], sim.simx_opmode_streaming)
        # #     sim.simxSetJointTargetVelocity(clientID, jointHandles_2[i][1], targetVel[i], sim.simx_opmode_streaming)

        # # sim.simxPauseCommunication(clientID, False)
        # # time.sleep(1)
        # # targetPos0 = [radians(10),radians(64.1),radians(0),radians(0),radians(-90),radians(90)]
        # # sim.simxPauseCommunication(clientID, True)
        # # for i in range(6):
        # #     #print(jointHandles[i])
        # #     #print(targetPos0[i])
        # #     sim.simxSetJointTargetPosition(clientID, jointHandles_2[i][1], targetPos0[i], sim.simx_opmode_streaming)
        # #     sim.simxSetJointTargetVelocity(clientID, jointHandles_2[i][1], targetVel[i], sim.simx_opmode_streaming)

        # # sim.simxPauseCommunication(clientID, False)
        # # time.sleep(1)
        # sim.simxPauseCommunication(clientID, True)
        # for i in range(6):
        #     #print(jointHandles[i])
        #     #print(targetPos0[i])
        #     sim.simxSetJointTargetPosition(clientID, jointHandles_2[i][1], 0, sim.simx_opmode_streaming)
        #     sim.simxSetJointTargetVelocity(clientID, jointHandles_2[i][1], targetVel[i], sim.simx_opmode_streaming)

        # sim.simxPauseCommunication(clientID, False)
        # time.sleep(2)

        # # ##Now, let us set up the Position of our dummy to match the position of the end_effector
        # # sim.simxSetObjectPosition(clientID, ee_frame[1], base_frame[1], new_pose[0:3], sim.simx_opmode_streaming)

        # # ##Then, let's also setup the orientation, after converting them to euler angles 
        # # new_rot_mat = T[0:3,0:3]
        # # new_rot = R.from_dcm(new_rot_mat)
        # # new_ori_b_ee = new_rot.as_euler('xyz')
        # # sim.simxSetObjectOrientation(clientID, ee_frame[1], base_frame[1], new_ori_b_ee, sim.simx_opmode_streaming
        # print("\n", "Now trying inverse kinematics: ")
        # time.sleep(2)
        # _, ball_pos = sim.simxGetObjectPosition(clientID, ball_handle[1], -1, sim.simx_opmode_blocking)

        # _, ball_ori = sim.simxGetObjectOrientation(clientID, ball_handle[1], -1, sim.simx_opmode_blocking)

        # ball_rot = R.from_euler('xyz', ball_ori)
        # ball_rot_mat = ball_rot.as_dcm()
        # # ball_pos[0] = ball_pos[0]
        # # ball_pos[1] = ball_pos[1]

        # world_coord = np.array([ball_pos[0], ball_pos[1]-.07, ball_pos[2]])
        # world_coord = np.abs(world_coord)
    
        # guess_angles = ik.findJointAngles(world_coord[0]+.07, world_coord[1], world_coord[2]+0.1)
        # targetPos0_ik = guess_angles

        # #Invert the angles if on left side
        # if(ball_pos[0] < 0):
        #     targetPos0_ik[0] = -guess_angles[0]
        #     targetPos0_ik[1] = -guess_angles[1]
        #     targetPos0_ik[2] = -guess_angles[2]
        #     targetPos0_ik[3] = -guess_angles[3]
        #     targetPos0_ik[4] = radians(90)
        #     targetPos0_ik[5] = radians(90)

        # print(guess_angles) #DEBUG
        # sim.simxPauseCommunication(clientID, True)
        # for i in range(6):
        #     #print(jointHandles[i])
        #     #print(targetPos0[i])
        #     sim.simxSetJointTargetPosition(clientID, jointHandles_2[i][1], 0, sim.simx_opmode_streaming)
        #     sim.simxSetJointTargetVelocity(clientID, jointHandles_2[i][1], targetVel[i], sim.simx_opmode_streaming)

        # sim.simxPauseCommunication(clientID, False)
        
        # sim.simxPauseCommunication(clientID, True)
        # for i in range(6):
        #     #print(jointHandles[i])
        #     #print(targetPos0[i])
        #     sim.simxSetJointTargetPosition(clientID, jointHandles_2[i][1], targetPos0_ik[i], sim.simx_opmode_streaming)
        #     sim.simxSetJointTargetVelocity(clientID, jointHandles_2[i][1], targetVel[i], sim.simx_opmode_streaming)

        # sim.simxPauseCommunication(clientID, False)
        # time.sleep(2)

        # # sim.simxPauseCommunication(clientID, True)
        # # for i in range(6):
        # #     #print(jointHandles[i])
        # #     #print(targetPos0[i])
        # #     sim.simxSetJointTargetPosition(clientID, jointHandles_2[i][1], 0, sim.simx_opmode_streaming)
        # #     sim.simxSetJointTargetVelocity(clientID, jointHandles_2[i][1], targetVel[i], sim.simx_opmode_streaming)

        # # sim.simxPauseCommunication(clientID, False)
        # # time.sleep(2)
        # # T_b = np.zeros((4,4))
        # # T_b[3,3] = 1
        # # T_b[0:3, 0:3] = ball_rot_mat
        # # T_b[0:3, 3] = ball_pos

        # # print("Ball Position: ", ball_pos)
        # # print("Ball Rotation: ", ball_rot.as_dcm())

        # # #theta_list_guess = np.array([radians(-70), radians(-50), radians(-50), radians(-30), radians(50), radians(0)]) 
        # # theta_list_guess = np.array([radians(0),radians(64.1),radians(0),radians(0),radians(90),radians(90)])
        # # theta_list, success = mr.IKinSpace(S, M, T_b, theta_list_guess, 0.01, 0.01)

        # # if success:
        # #     print(theta_list)
        # # else:
        # #     print("Not successful, but here's the guess: \n", theta_list)

        # #Now, let's check how close we are to the actual ball:
        # # sim.simxPauseCommunication(clientID, True)
        # # for i in range(6):
        # #     #print(jointHandles[i])
        # #     #print(targetPos0[i])
        # #     sim.simxSetJointTargetPosition(clientID, jointHandles_2[i][1], targetPos0_ik[i], sim.simx_opmode_streaming)
        # #     sim.simxSetJointTargetVelocity(clientID, jointHandles_2[i][1], targetVel[i], sim.simx_opmode_streaming)

        # # sim.simxPauseCommunication(clientID, False)
        # # time.sleep(2)

        # #sim.simxSetObjectPosition(clientID, P_ball_handle[1], -1, [-1, -1, 1], sim.simx_opmode_streaming)

        # Before closing the connection to CoppeliaSim, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        
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

if __name__ == '__main__':
    main()
    
#https://youtu.be/l0C5E4iqqRI

# Video link
#https://youtu.be/Qj7Sv0V9yec
