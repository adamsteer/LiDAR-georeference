"""
A module for computing LiDAR point coordinates in some reference frame given raw LiDAR isntrument data, a vehicle trajectory, and some uncertainties.

:py:function:scannerconfig creates an instrument configuration object containing a boresight matrix, and lever arm offsets.

:py:function:bs_matrix computes a rotation matrix given a 3-element array of angles which describe the [X,Y,Z] rotations required to orient the scanning instrument reference frame and the navigation instrument reference frame.

:py:function:rpy_matrix_build computes a rotation matrix which describes rotations from the navigation instrument frame to a world frame, usually defined by [heading, pitch, roll].

:py:function:to_lidar_coords computes point coordinates in the 'LiDAR instrument reference frame' given a range and a set of angles


#just getting started
Adam Steer
2017

"""

#collect dependencies
import numpy as np
import sys

#local dependency, transformations.py should accompany this file
# http://www.lfd.uci.edu/~gohlke/code/transformations.py.html
#import transformations

def bs_matrix(boresight):

    print(boresight)

    boresight = np.array(boresight) * np.pi/180

    rotbx = np.matrix([[1.,        0.,         0.],
                      [0., np.cos(boresight[0]),-np.sin(boresight[0])],
                      [0., np.sin(boresight[0]),np.cos(boresight[0])]])

    rotby = np.matrix([[np.cos(boresight[1]),        0., np.sin(boresight[1])],
                      [0., 1., 0.],
                      [-np.sin(boresight[1]), 0., np.cos(boresight[1])]])

    rotbz = np.matrix([[np.cos(boresight[2]),-np.sin(boresight[0]),0.],
                      [np.sin(boresight[0]), np.cos(boresight[0]), 0.],
                      [0., 0., 1.]])

    #check here that rotation order is correct, maybe pass an argument to
    # define whether it should be ZYX, XYZ, pt anything else?

    bs_rot_matrix = rotbz.dot(rotby).dot(rotbx)

    return bs_rot_matrix


def rpy_matrix_build(RPY):
    """take in roll, pitch, heading angles in degrees for each epoch and compute
       a transformation matrix to rotate the strapdown navigator
       frame to a world frame determined by the input trajectory"""
    RPY = RPY*(np.pi/180)

    rotx = np.matrix([[1,        0,         0], \
                      [0, np.cos(RPY[0]), -np.sin(RPY[0])], \
                      [0, np.sin(RPY[0]), np.cos(RPY[0])]])

    roty = np.matrix([[np.cos(RPY[1]),        0, np.sin(RPY[1])], \
                      [0, 1, 0], \
                      [-np.sin(RPY[1]), 0, np.cos(RPY[1])]])

    rotz = np.matrix([[np.cos(RPY[2]),-np.sin(RPY[0]),         0], \
                      [np.sin(RPY[0]), np.cos(RPY[0]), 0], \
                      [0, 0, 1]])


    rpy_rot_matrix = rotz.dot(roty).dot(rotx)

    return rpy_rot_matrix


def to_lidar_coords(R, A, D):
    """ convert ranges and angles into point coordinates in the LiDAR frame """
    assert D in range[2,3], 'D needs to be either 2 or 3'
    A = A*(np.pi/180)
    if D == 2:
        return R*[np.sin(A), 0, np.cos(A)];
    elif D == 3:
        #what happens if we have a3D scanner?
        return R*[np.sin(A), 0, np.cos(A)];

def to_world_coords();
    #run the lidar georeferencing equation here...






#### just starting to build
#### scannercondig is a python dict containing a key-value pair for each of
#### required attributes in the georeferencing function
def main():
   # read processconfig.json



   # run to_world_coords







if __name__ == '__main__':
   main()
