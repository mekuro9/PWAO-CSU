print("Hello world! Hi\n")
import numpy as np

import sympy as sy

t = sy.symbols('t')
x = sy.symbols('x')
y = sy.symbols('y')
z = sy.symbols('z')
R = np.array([[[[sy.cos(t),0,sy.sin(-t),0],
               [0,1,0,0],
               [-sy.sin(t),0,sy.cos('t'),0],
               [0,0,0,1]]]])
T = np.array([[[[1,0,0,0],[0,1,0,0],[0,0,1,-10],[0,0,0,1]]]])
P = np.array([[[[x],[y],[0],[1]]]])
M = np.matmul(R,T)
P_ = np.matmul(M,P)
print(M)
print("\n")
print(P_)
# 1. convert to cartesian
# 2. Apply transformation matrix
# 3. Projection. normalize the 3d coordinates
# and project them to image plane
# x' = x/z, y' = y/z
# Use intrinsic parameters to find pixel coordinates
# p_x = fx*x' + cx
# p_y = fy*y' + cy
# 4. Overlay on image .. how to overlay (using opencv)
#
# The first step is to create nodes in ros2 (use python)
# Camera class -> stores cameras intrinsic parameters and provide methods
# for transforming 3D coordinates to 2D image coordinates
# 
# Lidar class -> converting 2D lidar polar coordinates to cartesian coordinates
# 
# Projection class -> overall process of projecting Lidar points on the
# camera image (uses methods from lidar and camera class)
# final ros2 node that uses methods from these classes to actually project the data on image
# and publish it


# I receive Laserscan message
# lets
