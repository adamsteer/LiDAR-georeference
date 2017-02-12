#collect dependencies
import numpy as np
import sys
import laspy
import psycopg2

#local dependency, transformations.py should accompany this file
# http://www.lfd.uci.edu/~gohlke/code/transformations.py.html
import transformations

def bs_matrix_build(boresight):
    """generate a boresight rotation matrix for each axis
       to transform a LiDAR-instrument-frame point into the
       strapdown navigator frame.

       note this setup was basically empirically derived - please
       feel free to correct and explain any mathematical wierdnesses!"""
    
    boresight = boresight * (np.pi()/180.)
    
    rotbx = np.matrix([1,        0,         0], \#boresight X
                      [0, np.cos(boresight[0]), -np.sin(boresight[0])], \
                      [0, np.sin(boresight[0]), np.cos(boresight[0])])

    rotby = np.matrix([np.cos(boresight[1]),        0, np.sin(boresight[1])], \#boresight Y
                      [0, 1, 0], \
                      [-np.sin(boresight[1]), 0, np.cos(boresight[1])])

    rotbz = np.matrix([np.cos(boresight[2]),-np.sin(boresight[0]),         0], \#boresight Z
                      [np.sin(boresight[0]), np.cos(boresight[0]), 0], \
                      [0, 0, 1])
    #check here that rotation order is correct, maybe pass an argument to
    # define whether it should be ZYX, XYZ, pt anything else?

    bs_rot_matrix = rotbz.dot(rotby).dot(rotbx)

    return bs_rot_matrix


def rpy_matrix_build(RPY):
    """take in roll, pitch, heading angles in degrees for each epoch and compute
       a transformation matrix to rotate the strapdown navigator
       frame to a world frame determined by the input trajectory"""
    RPY = RPY*(np.pi/180)

    rotbx = np.matrix([1,        0,         0], \#boresight X
                      [0, np.cos(boresight[0]), -np.sin(boresight[0])], \
                      [0, np.sin(boresight[0]), np.cos(boresight[0])])

    rotby = np.matrix([np.cos(boresight[1]),        0, np.sin(boresight[1])], \#boresight Y
                      [0, 1, 0], \
                      [-np.sin(boresight[1]), 0, np.cos(boresight[1])])

    rotbz = np.matrix([np.cos(boresight[2]),-np.sin(boresight[0]),         0], \#boresight Z
                      [np.sin(boresight[0]), np.cos(boresight[0]), 0], \
                      [0, 0, 1])

    return rpy_rot_matrix


def to_lidar_coords(R, A, D):
    """ convert ranges and angles into point coordinates in the LiDAR frame """
    assert D in range[2,3], 'D needs to be either 2 or 3'
    A = A*(np.pi/180)
    if D == 2:
        return R*[np.sin(A), 0, np.cos(A)];
    elif D == 3:
        return R*[np.sin(A), 0, np.cos(A)];


#### just starting to build!!
def main():
    script = sys.argv[0]
    gpsXYZ = sys.argv[1]
    RA = sys.argv[2]
    RPY = sys.argv[3]  
    boresight = sys.argv[4]
    leverarm = sys.argv[5]
    
    
    lidar_point = to_lidar_coords(RA[0], RA[1])
    # angles to radians
    
    RPY = RPY * (np.pi()/180.)
    boresight =  boresight * (np.pi()/180.)    
    

     

if __name__ == '__main__':
   main()





