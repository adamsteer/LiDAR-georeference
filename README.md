# LiDAR_georeference
Open source code implementing the basic lidar georeferencing functions described in Baltsavias (1999),and more recently Glennie (2007) and Schaer et al (2006). 

Propagation of uncertanties for LiDAR points will also be implemented following Glennie (2007) and Schaer et al (2006). 

Based on MATLAB code developed for a PhD project - which can still be found in ./MATLAB. Future development will be in Python, unless someone wants to take up making a C version - although part of the point is to make the process fairly easy t understand.

Rough plan:

- implement the basic equations for 2D and 3D scanners in Python **underway**
- implement output to .LAS, or postgres-pointcloud (q2 2017). Las is trickier of user defined fields - see next point
- implement uncertainty propagation using SymPy to handle deriving the required partial derivatives (q2 2017)
- implement binary reading of Reigl LiDAR scanner output (q3 2017)

pipe dreams:

- read trajectory input from Novatel .bin files output from Inertial explorer
- read trajectory input from binary OxTS NCOM files, will need proj.4 to project (once) from an ECEF frame to some cartesian frame.
- Schaer (2006) Q-factors
- faster implementation for real-time point generation and QA.
