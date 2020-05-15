import numpy as np

def radians(degree):
    """
    helper function to convert angle from degrees to radians
    """
    return degree * (np.pi/180)
def degrees(radian):
    """
    helper function to convert angle from radians to degrees
    """
    
    return radian * (180/np.pi)

def findJointAngles(x_wgrip, y_wgrip, z_wgrip):
    """
    @param x_w: desired x-coordinate of end-effector in world frame
    @param y_w: desired y-coordinate of end-effector world frame
    @param z_w: desired z-coordinate of end-effector in world frame
    @param yaw: the desired yaw (deviation in angle) from the base angle of the robot

    @return value: [theta1, theta2, theta3, theta4, theta5, theta6]: joint angles needed for
    desired configuration
    """
    L01  =  0.152
    L02  =  0.120
    L03  =  0.244
    L04  =  0.093
    L05  =  0.213
    L06  =  0.083
    L07  =  0.083
    L08  =  0.082
    L09  =  0.238
    L10  =  0.059
    Lend = L06 + 0.027 #abstract extended link 6 to help in calculations
    L_link = L08 + L09
    
    theta1 = 0
    theta2 = 0
    theta3 = 0
    theta4 = 0
    theta5 = radians(-90)
    theta6 = radians(-90)

    #Find the grip coordinates, by subtracting from the coordinates the displacement of the base frame
    x_grip = x_wgrip
    y_grip = y_wgrip
    z_grip = z_wgrip - 0.043

    # #Find x_cen, y_cen and z_cen, the center of joint 6, to figure out the thetas
    # x_cen = x_grip - L09 * np.cos(yaw)
    # y_cen = y_grip - L09 * np.sin(yaw)
    # z_cen = z_grip
    x_cen = x_grip
    y_cen = y_grip
    z_cen = z_grip

    #Creating helper variables and using them to find theta1
    theta_x = np.arctan2(y_cen, x_cen)
    theta_y = np.arcsin(Lend/np.sqrt(x_cen**2 + y_cen**2))
    theta1 = theta_x - theta_y

    #Finding theta6 using theta1 and yaw
    #theta6 = radians(90) - yaw + theta1

    #Finding x_3end, y_3end and z_3end, coordinates of the extension of Link 6 
    #used as helper variables for finding theta2, theta3 and theta4
    x_3end = x_cen + Lend * np.sin(theta1) - L_link * np.cos(theta1)
    y_3end = y_cen - Lend * np.cos(theta1) - L_link * np.sin(theta1)
    z_3end = z_cen - L07

    #Finding theta2, theta3 and theta4
    #Using helper angles, theta_i and theta_j and helper length L_3endto find theta_2
    L_3end = np.sqrt(x_3end**2 + y_3end**2 + (z_3end - L01)**2)
    theta_i = np.arcsin((z_3end - L01)/L_3end)
    theta_j = np.arccos((L05**2 - L03**2 - L_3end**2)/(-2 * L03 * L_3end))

    #Calculate theta2, theta3 and theta4
    theta2 = (np.pi/2) - (theta_i + theta_j)
    theta3 = np.arccos((L_3end**2 - L03**2 - L05**2)/(2 * L03 * L05))
    theta4 = -(theta2 + theta3)
    
    return [theta1,theta2, theta3, theta4, theta5, theta6]

#Function for calculating outputs for verification
def main():
    x_w = 0.2
    y_w = 0.4
    z_w = 0.05
    #yaw = radians(45)

    theta_list = findJointAngles(x_w, y_w, z_w)
    print([np.round(degrees(theta),3) for theta in theta_list])

if __name__ == '__main__':
    main()