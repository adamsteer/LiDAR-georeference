"""
A coordinate transformation module. Made as a separate chunk of code to allow for easier implementation of newer/better reference frame translation methods.

Generally used to project a trajectory in ECEF coordinates (eg lat/lon) into a projected reference system.

##just getting started!

"""


#collect dependencies
import numpy as np
import sys
import pyproj as prj
