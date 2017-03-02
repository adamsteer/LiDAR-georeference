# LiDAR_georeference
Open source code implementing the basic lidar georeferencing functions described in Baltsavias (1999),and more recently Glennie (2007) and Schaer et al (2006). 

Propagation of uncertanties for LiDAR points will also be implemented following Glennie (2007) and Schaer et al (2006). 

Based on MATLAB code developed for a PhD project - which can still be found in ./MATLAB. Future development will be in Python, unless someone wants to take up making a C version - although part of the point is to make the process fairly easy to understand.

If you want to contribute, use what's in ./MATLAB as a base, and if you don't see it's equivalent in Python then give it a shot. Or, if you can make the Python code better/faster/more efficient/more functional please do so!

Rough plan:

- implement the basic equations for 2D and 3D scanners in Python **underway**
- implement uncertainty propagation using SymPy to handle deriving the required partial derivatives (q2 2017)
- implement output to .LAS, or postgres-pointcloud (q2 2017). Las is trickier because of user defined fields which will be required (see point above)
- ...and implement output to an Entwine index, skipping the standard output to LAS/LAZ tiles entirely. All these outputs will depend on PDAL (http://pdal.io)
- implement binary reading of Reigl LiDAR scanner output (q3 2017)

pipe dreams:

- read trajectory input from Novatel .bin files output from Inertial explorer
- read trajectory input from binary OxTS NCOM files, will need proj.4 to project (once) from an ECEF frame to some cartesian frame.
- Schaer (2007) Q-factors
- faster implementation for real-time point generation and QA.
- maybe even it's own kalman filter implementation to do tightly-coupled GPS+inertial trajectory determination, avoiding the proprietary blackbox step currently required (beyond my skills at present)

