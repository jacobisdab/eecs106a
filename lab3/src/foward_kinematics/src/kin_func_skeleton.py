#!/usr/bin/env python
"""
Kinematic function skeleton code for Lab 3 prelab.

Course: EE 106A, Fall 2021
Originally written by: Aaron Bestick, 9/10/14
Adapted for Fall 2020 by: Amay Saxena, 9/10/20

This Python file is a code skeleton for Lab 3 prelab. You should fill in 
the body of the five empty methods below so that they implement the kinematic 
functions described in the assignment.

When you think you have the methods implemented correctly, you can test your 
code by running "python kin_func_skeleton.py at the command line.
"""

import numpy as np

np.set_printoptions(precision=4,suppress=True)

#-----------------------------2D Examples---------------------------------------
#--(you don't need to modify anything here but you should take a look at them)--

def rotation_2d(theta):
    """
    Computes a 2D rotation matrix given the angle of rotation.
    
    Args:
    theta: the angle of rotation
    
    Returns:
    rot - (2,2) ndarray: the resulting rotation matrix
    """
    
    rot = np.zeros((2,2))
    rot[0,0] = np.cos(theta)
    rot[1,1] = np.cos(theta)
    rot[0,1] = -np.sin(theta)
    rot[1,0] = np.sin(theta)

    return rot

def hat_2d(xi):
    """
    Converts a 2D twist to its corresponding 3x3 matrix representation
    
    Args:
    xi - (3,) ndarray: the 2D twist
    
    Returns:
    xi_hat - (3,3) ndarray: the resulting 3x3 matrix
    """
    if not xi.shape == (3,):
        raise TypeError('omega must be a 3-vector')

    xi_hat = np.zeros((3,3))
    xi_hat[0,1] = -xi[2]
    xi_hat[1,0] =  xi[2]
    xi_hat[0:2,2] = xi[0:2]

    return xi_hat

def homog_2d(xi, theta):
    """
    Computes a 3x3 homogeneous transformation matrix given a 2D twist and a 
    joint displacement
    
    Args:
    xi - (3,) ndarray: the 2D twist
    theta: the joint displacement
    
    Returns:
    g - (3,3) ndarray: the resulting homogeneous transformation matrix
    """
    if not xi.shape == (3,):
        raise TypeError('xi must be a 3-vector')

    g = np.zeros((3,3))
    wtheta = xi[2]*theta
    R = rotation_2d(wtheta)
    p = np.dot(np.dot( \
        [[1 - np.cos(wtheta), np.sin(wtheta)],
        [-np.sin(wtheta), 1 - np.cos(wtheta)]], \
        [[0,-1],[1,0]]), \
        [[xi[0]/xi[2]],[xi[1]/xi[2]]])

    g[0:2,0:2] = R
    g[0:2,2:3] = p[0:2]
    g[2,2] = 1

    return g

#-----------------------------3D Functions--------------------------------------
#-------------(These are the functions you need to complete)--------------------

def skew_3d(omega):
    """
    Converts a rotation vector in 3D to its corresponding skew-symmetric matrix.
    
    Args:
    omega - (3,) ndarray: the rotation vector
    
    Returns:
    omega_hat - (3,3) ndarray: the corresponding skew symmetric matrix
    """

    omega_hat = np.array([[0, -omega[2], omega[1]],
                          [omega[2], 0, -omega[0]],
                          [-omega[1], omega[0], 0]])
    
    return omega_hat



def rotation_3d(omega, theta):
    """
    Computes a 3D rotation matrix given a rotation axis and angle of rotation.
    
    Args:
    omega - (3,) ndarray: the axis of rotation
    theta: the angle of rotation
    
    Returns:
    rot - (3,3) ndarray: the resulting rotation matrix
    """

    norm = np.linalg.norm(omega) #norm = ||w||
    
    # Get the skew-symmetric matrix of omega
    omega_hat = skew_3d(omega)
    
    
    # Identity matrix
    I = np.eye(3)
    
    # Rodrigues' rotation formula
    rot = I + ((omega_hat/norm) * np.sin(norm*theta)) + ( (np.dot(omega_hat, omega_hat)/(norm**2))* (1-np.cos(norm*theta)))
    
    return rot



def hat_3d(xi):
    """
    Converts a 3D twist to its corresponding 4x4 matrix representation
    
    Args:
    xi - (6,) ndarray: the 3D twist
    
    Returns:
    xi_hat - (4,4) ndarray: the corresponding 4x4 matrix
    """
    v = xi[:3]   # Angular velocity
    w = xi[3:]       # Linear velocity
    
    # Create the skew-symmetric matrix for omega
    omega_hat = skew_3d(w)
    
    # Construct the 4x4 matrix xi_hat
    xi_hat = np.zeros((4, 4))
    xi_hat[:3, :3] = omega_hat   # Top-left 3x3 is omega_hat
    xi_hat[:3, 3] = v           # Top-right 3x1 is v
    
    
    return xi_hat



def homog_3d(xi, theta):
    """
    Computes a 4x4 homogeneous transformation matrix given a 3D twist and a 
    joint displacement.
    
    Args:
    xi - (6,) ndarray: the 3D twist
    theta: the joint displacement
    Returns:
    g - (4,4) ndarary: the resulting homogeneous transformation matrix
    """
    v = xi[:3]  # Linear Velocity
    w = xi[3:]      # Angular velocity
    
    # Compute the norm of omega to check if it is a pure translation or rotation
    
    norm = np.linalg.norm(w)
    
    # If omega is not a zero vector (non-zero rotation)
    if norm != 0:
        omega_hat = skew_3d(w)
        R = rotation_3d(w,theta)
        
        
        t = (1/norm**2) * ((np.eye(3)-R)@(omega_hat@v) + ((np.outer(w,w.T))@(v*theta)))
    
    else:  # Pure translation (when omega is zero)
        R = np.eye(3)  
        t = v * theta  

    # Construct the homogeneous transformation matrix
    g = np.eye(4)
    g[:3, :3] = R  # Rotation part
    g[:3, 3] = t   # Translation part
    
    return g



def prod_exp(xi, theta):
    """
    Computes the product of exponentials for a kinematic chain, given 
    the twists and displacements for each joint.
    
    Args:
    xi - (6, N) ndarray: the twists for each joint
    theta - (N,) ndarray: the displacement of each joint
    
    Returns:
    g - (4,4) ndarray: the resulting homogeneous transformation matrix
    """

     # Ensure xi is a numpy array
    xi = np.asarray(xi)
    theta = np.asarray(theta)
    
    # Ensure xi is 6 x N and theta is N-dimensional
    if xi.shape[0] != 6:
        raise ValueError("Twists xi must have shape (6, N)")
    if xi.shape[1] != theta.shape[0]:
        raise ValueError("There must be the same number of twists and joint displacements.")
    
    # Initialize the homogeneous transformation matrix as the identity matrix
    g = np.eye(4)
    
    # Multiply the exponential of each twist for the given displacement
    for i in range(theta.shape[0]):
        g_i = homog_3d(xi[:, i], theta[i])  # Compute the transformation for joint i
        g = np.dot(g, g_i)  # Update the product of exponentials
    
    return g

    

#---------------------------------TESTING CODE---------------------------------
#-------------------------DO NOT MODIFY ANYTHING BELOW HERE--------------------

def array_func_test(func_name, args, ret_desired):
    ret_value = func_name(*args)
    if not isinstance(ret_value, np.ndarray):
        print('[FAIL] ' + func_name.__name__ + '() returned something other than a NumPy ndarray')
    elif ret_value.shape != ret_desired.shape:
        print('[FAIL] ' + func_name.__name__ + '() returned an ndarray with incorrect dimensions')
    elif not np.allclose(ret_value, ret_desired, rtol=1e-3):
        print('[FAIL] ' + func_name.__name__ + '() returned an incorrect value')
    else:
        print('[PASS] ' + func_name.__name__ + '() returned the correct value!')

if __name__ == "__main__":
    print('Testing...')

    #Test skew_3d()
    arg1 = np.array([1.0, 2, 3])
    func_args = (arg1,)
    ret_desired = np.array([[ 0., -3.,  2.],
                            [ 3., -0., -1.],
                            [-2.,  1.,  0.]])
    array_func_test(skew_3d, func_args, ret_desired)

    #Test rotation_3d()
    arg1 = np.array([2.0, 1, 3])
    arg2 = 0.587
    func_args = (arg1,arg2)
    ret_desired = np.array([[-0.1325, -0.4234,  0.8962],
                            [ 0.8765, -0.4723, -0.0935],
                            [ 0.4629,  0.7731,  0.4337]])
    array_func_test(rotation_3d, func_args, ret_desired)

    #Test hat_3d()
    arg1 = np.array([2.0, 1, 3, 5, 4, 2])
    func_args = (arg1,)
    ret_desired = np.array([[ 0., -2.,  4.,  2.],
                            [ 2., -0., -5.,  1.],
                            [-4.,  5.,  0.,  3.],
                            [ 0.,  0.,  0.,  0.]])
    array_func_test(hat_3d, func_args, ret_desired)

    #Test homog_3d()
    arg1 = np.array([2.0, 1, 3, 5, 4, 2])
    arg2 = 0.658
    func_args = (arg1,arg2)
    ret_desired = np.array([[ 0.4249,  0.8601, -0.2824,  1.7814],
                            [ 0.2901,  0.1661,  0.9425,  0.9643],
                            [ 0.8575, -0.4824, -0.179 ,  0.1978],
                            [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(homog_3d, func_args, ret_desired)

    #Test prod_exp()
    arg1 = np.array([[2.0, 1, 3, 5, 4, 6], [5, 3, 1, 1, 3, 2], [1, 3, 4, 5, 2, 4]]).T
    arg2 = np.array([0.658, 0.234, 1.345])
    func_args = (arg1,arg2)
    ret_desired = np.array([[ 0.4392,  0.4998,  0.7466,  7.6936],
                            [ 0.6599, -0.7434,  0.1095,  2.8849],
                            [ 0.6097,  0.4446, -0.6562,  3.3598],
                            [ 0.    ,  0.    ,  0.    ,  1.    ]])
    array_func_test(prod_exp, func_args, ret_desired)

    print('Done!')