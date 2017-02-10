#collect dependencies
import numpy as np
import sys


#### just starting to build!!
def main():
    script = sys.argv[0]
    gpsXYZ = sys.argv[1]
    RA = sys.argv[2]
    RPY = sys.argv[3]  
    boresight = sys.argv[4]
    leverarm = sys.argv[5]
    

    """
    Given a 
    """
    # generate a boresight rotation matrix for each axis
    rotbx = np.matrix([1,        0,         0], \#boresight X
                      [0, np.cos(boresight[0]), -np.sin(boresight[0])], \
                      [0, np.sin(boresight[0]), np.cos(boresight[0])])

    rotby = np.matrix([1,        0,         0], \#boresight X
                      [0, np.cos(boresight[0]), -np.sin(boresight[0])], \
                      [0, np.sin(boresight[0]), np.cos(boresight[0])])
     

if __name__ == '__main__':
   main()





