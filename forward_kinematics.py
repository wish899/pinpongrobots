import sympy as sym
import numpy as np
import math
from scipy.linalg import expm
from sympy import *

def main():
    #symbolic variables for the symbolic transformation matrix
    theta1 = Symbol('theta1')
    theta2 = Symbol('theta2')
    theta3 = Symbol('theta3')
    theta4 = Symbol('theta4')
    theta5 = Symbol('theta5')
    theta6 = Symbol('theta6')

    # matrices that hold the symbolic thetas
    sym_thetas = sym.Matrix([theta1, theta2, theta3, theta4, theta5, theta6])

    # change these values to test different verifications
    input = [-58.018, -84.526, 88.264, -3.738, -90.0, 76.982]

    # holds the real theta values for later subsitution
    real_thetas = sym.Matrix([radians(input[0]), radians(input[1]), radians(input[2]),
                              radians(input[3]), radians(input[4]), radians(input[5])])

    # stores the symbolic matrix into T_matrix
    T_matrix = symbolic_sol(sym_thetas)

    # matrix that holds the numerical value of the displacement
    Num_matrix = numeric_sol(T_matrix, real_thetas, sym_thetas)
    
    # debugging numerical values
    print("Answer to the verification values: ")
    print(Num_matrix)

def numeric_sol(T, r_thetas, s_thetas):
    # T is for some reason a 1D matrix, so the x,y,z values are stored at the 3,7,11 respectively
    # because the row is 4 wide
    x = T[3]
    y = T[7]
    z = T[11]

    # substituting the real theta values into the symbolic
    x = x.subs({s_thetas[0]: r_thetas[0], s_thetas[1]: r_thetas[1], s_thetas[2]: r_thetas[2], s_thetas[3]: r_thetas[3], s_thetas[4]: r_thetas[4], s_thetas[5]: r_thetas[5]})
    y = y.subs({s_thetas[0]: r_thetas[0], s_thetas[1]: r_thetas[1], s_thetas[2]: r_thetas[2], s_thetas[3]: r_thetas[3], s_thetas[4]: r_thetas[4], s_thetas[5]: r_thetas[5]})
    z = z.subs({s_thetas[0]: r_thetas[0], s_thetas[1]: r_thetas[1], s_thetas[2]: r_thetas[2], s_thetas[3]: r_thetas[3], s_thetas[4]: r_thetas[4], s_thetas[5]: r_thetas[5]})

    # stores x,y,z values into a single matrix
    numeric_sol = sym.Matrix([[x], [y], [z]])

    # evaluates any remaining sins and cosines
    ans = numeric_sol.evalf()

    return ans

def radians(angle):
    return angle * (math.pi/180)

def skew_symmetric(screw):
    # calculates the skew matrix * theta for easy exponential matrix
    s_k = sym.Matrix([[0, -screw[2], screw[1]],
                    [screw[2],    0,     -screw[0]],
                    [-screw[1], screw[0],   0]])
    return s_k

def exponential_matrix(w, v, theta):
    # Identity matrix
    I = sym.Matrix([[1,0,0],
                    [0,1,0],
                    [0,0,1]])
    # upper left term of the exponential screw matrix using rodriguez's formula
    ex_w = simplify(I + sin(theta)*skew_symmetric(w) + (1-cos(theta))*skew_symmetric(w)**2)

    # upper right term of the exponential screw matrix
    v_new = simplify(((I*theta) + (1-cos(theta))*skew_symmetric(w) + (theta - sin(theta))*skew_symmetric(w)**2)*v)

    # combining the two calculated terms into a homogenous 4x4 matrix
    ex_screw = sym.Matrix([[ex_w[0], ex_w[1], ex_w[2], v_new[0]],
                           [ex_w[3], ex_w[4], ex_w[5], v_new[1]],
                           [ex_w[6], ex_w[7], ex_w[8], v_new[2]],
                           [0,0,0,1]])
    return ex_screw

def symbolic_sol(theta_sym):
    # code is very redundant due to not being able to
    # figure out how to 2D cross multiply with sympy, so all the
    # joints have individual variables that store their zero position values
    w1 = sym.Matrix([[0,0,1]])
    w2 = sym.Matrix([[0,1,0]])
    w3 = sym.Matrix([[0,1,0]])
    w4 = sym.Matrix([[0,1,0]])
    w5 = sym.Matrix([[1,0,0]])
    w6 = sym.Matrix([[0,1,0]])

    q1 = sym.Matrix([[-0.15, 0.15, 0.01]])
    q2 = sym.Matrix([[-0.15, 0.27, 0.162]])
    q3 = sym.Matrix([[0.094, 0.27, 0.162]])
    q4 = sym.Matrix([[0.307, 0.177, 0.162]])
    q5 = sym.Matrix([[0.307, 0.260, 0.162]])
    q6 = sym.Matrix([[0.390, 0.260, 0.162]])

    # calculates v of (w,v) by v = -w X q
    v1 = -w1.cross(q1)
    v2 = -w2.cross(q2)
    v3 = -w3.cross(q3)
    v4 = -w4.cross(q4)
    v5 = -w5.cross(q5)
    v6 = -w6.cross(q6)
    # print("Velocities of calculations")
    # print(v1)
    # print(v2)
    # print(v3)
    # print(v4)
    # print(v5)
    # print(v6)
    # print("End of velocities")

    # stores the (w,v) components of each screw into a matrix for each screw
    s1 = sym.Matrix([[w1[0], w1[1], w1[2], v1[0], v1[1], v1[2]]])
    s2 = sym.Matrix([[w2[0], w2[1], w2[2], v2[0], v2[1], v2[2]]])
    s3 = sym.Matrix([[w3[0], w3[1], w3[2], v3[0], v3[1], v3[2]]])
    s4 = sym.Matrix([[w4[0], w4[1], w4[2], v4[0], v4[1], v4[2]]])
    s5 = sym.Matrix([[w5[0], w5[1], w5[2], v5[0], v5[1], v5[2]]])
    s6 = sym.Matrix([[w6[0], w6[1], w6[2], v6[0], v6[1], v6[2]]])

    # have to reorganize the v matrix from 1x3 to 3x1 so it will work with exponential matrix calculations
    v1 = sym.Matrix([[v1[0]],[v1[1]],[v1[2]]])
    v2 = sym.Matrix([[v2[0]],[v2[1]],[v2[2]]])
    v3 = sym.Matrix([[v3[0]],[v3[1]],[v3[2]]])
    v4 = sym.Matrix([[v4[0]],[v4[1]],[v4[2]]])
    v5 = sym.Matrix([[v5[0]],[v5[1]],[v5[2]]])
    v6 = sym.Matrix([[v6[0]],[v6[1]],[v6[2]]])

    # caclulates the symbolic exponential matrix of each screw * respective theta
    ex_1 = simplify(exponential_matrix(w1, v1, theta_sym[0]))
    ex_2 = simplify(exponential_matrix(w2, v2, theta_sym[1]))
    ex_3 = simplify(exponential_matrix(w3, v3, theta_sym[2]))
    ex_4 = simplify(exponential_matrix(w4, v4, theta_sym[3]))
    ex_5 = simplify(exponential_matrix(w5, v5, theta_sym[4]))
    ex_6 = simplify(exponential_matrix(w6, v6, theta_sym[5]))

    # print("Exponential 1")
    # print(ex_1)
    # print("Exponential 2")
    # print(ex_2)
    # print("Exponential 3")
    # print(ex_3)
    # print("Exponential 4")
    # print(ex_4)
    # print("Exponential 5")
    # print(ex_5)
    # print("Exponential 6")
    # print(ex_6)

    # homogenous transformation matrix of zero position
    M = sym.Matrix([[0, 0, 1, 0.390],
                    [-1, 0, 0, 0.401],
                    [0, -1, 0, 0.2155],
                    [0, 0, 0, 1]])

    # product of exponentials equation for calculation frame transformation matrix
    T = ex_1 * ex_2 * ex_3 * ex_4 * ex_5 * ex_6 * M

    return T


# call main to run the calculations
if __name__ == '__main__':
    main()
