# matlab_LIDAR
### MATLAB code for lidar georeferencing

Adam Steer
adam.d.steer@gmail.com

## Summary
A set of MATLAB scripts were developed by the author for geolocating airborne LiDAR –
specifically the AAD RAPPLS package. Code already exists in the form of the 
APPLS Java project (Peter Jansen, http://sourceforge.net/projects/appls/files/). The 
MATLAB code was developed because a need existed to:

* Investigate error propagation in the LiDAR system
* Employ arbitraty coordinate systems as georeferencing targets
* Modify aircraft rotations by a boresight misalignment parameter

...none of which were possible using the APPLS Java project.

The MATLAB code referred to in this document is able to:

* Geolocate LiDAR in an arbirtrary East-North-Up coordinate system
* Use VCV propagation to provide uncertainty estimates for every point in the resulting cloud
* Use an arbitrary set of instrument mounting angles
* Adjust mounting angles by boresight parameters
* Attempt to reduce noise in output point clouds.

Code shared by Luke Wallace, working with the UTAS TerraLuma group, was instrumental in figuring out how to achieve some of this for the RAPPLS system, specifically for comparing results.

It is not very user-friendly, but it is experimental software designed to be experimented with.

### 1. Data preparation
You need the MATLAB scripts:
* doit.m
* killit.m
* getdata.m
* makeswath.m
* lidargeo.m
* propagate_errs.m
...in your working directory or MATLAB path.

Next, assemble the data. Georeferencing LiDAR requires a LiDAR raw output file, and an aircraft trajectory. 

For the MATLAB code package, these must appear as follows:

LiDAR data in ASCII format with 7 space delimited columns, as follows:

TIME RANGE THETA X Y Z AMPLITUDE 
13966.314840 75.775 60.025 65.639 0.036 37.859 16191

Trajectory data as UTM coordinates, in 3dp format, as follows:

TimeOfWeek(GPS) PosLat(deg) PosLon(deg) PosHeight(m) AngleRoll(deg) AnglePitch(deg) Heading(deg) PosLatStdev(m) PosLonStdev(m) PosAltStdev(m) AnglePitchStdev(deg) AngleRollStdev(deg) AngleHeadingStdev(deg) 
359117.62000 538814.464318 5258045.999403 8.626000 0.771 0.610 286.002 0.386 0.380 0.596 0.036 0.035 0.122

A raw IMU rotational velocity file (.rot file). Experimental, ask Adam.

It is recommended to store each of these files in it's own meaningfully-named subdirectory of wherever you are working, like:

* working dir > trajectory
              > LIDAR
              > IMU
              > swaths

### 1.1 LiDAR - converting Riegl binary output to ASCII

Making the ASCII LiDAR data is straightforward. Rename a raw logged .q24 file to .2dd, and open it in SDCimport. 
Export the file in .sdc format. You might like to use a range gate, such that aerial returns are minimised. 
For most processing, 200 → 1500m range seems a good fit. You may also choose to use the time gate option here as 
well – but be cautious, LiDAR time is in 'seconds since last midnight'.
Open your new .sdc file in SDCview, and export it in ASCII format. The unix utility 'split' can segment 
this file into manageable chunks if needed.

### 1.2 Trajectory - converting OxTS IMU output to ASCII

The first task is to generate a .3dp file using OxTS's RT-post-process. This can be done by simply opening 
the relevant .ncom file and exporting as 3dp. The coordinates in this file need to be converted to UTM. 
If you're using a *nix variant (including cygwin), the script '3dp2utm.sh' will handle this for you, if 
passed some information about the relevant UTM zone. Use it like:

>bash 3dp2utm infile.3dp 50 south outfile.utm

Store the UTM file in your 'trajectory' folder.
Since the LIDAR processing code is coordinate-system-agnostic, It is possible to use an arbitrary cartesian
coordinate system here.

### 1.3 IMU rotations

Highly experimental, probably no use for systems other than AAD's APPLS platform.

### 2. Making a point cloud

Open doit.m. First, modify the file locations as appropriate in lines 4-15.
Then, specify a time window in GPS seconds-of-week if desired at lines 18-19.
Determine the second-of-week of your survey date – for example Tuesday, 23 October 2012 has
GPS second-of-week 172800. Set this at line 26:

gps_secofweek = 172800.

Sometimes an additional time offset is needed, for example if the LiDAR and IMU synced a second 
late or using UTC rather than GPS time. Account for this at line 30:

sync_offset = 0.0;

The script adds the start and stop time in GPS seconds-of-week to the file name, so output files 
would be something like:

file_prefix_185263_185460.xyz

A series of annotated options follows, read the details and if in doubt set all to -1.

Run doit.m. If all is well it will spew a log file and finish without errors. If not, MATLAB error 
messages should point to a solution. 

Be sure to run killit.m before re-running doit.m!

The output file is space-separated, with 13 fields are as follows:

Time (GPS second-of-week) | UTM X (m) | UTM Y (m) | UTM Z (m) | Intensity (unscaled) | Scan Angle (degrees) |
X uncertainty (m) | Y uncertainty (m) | Z uncertainty (m) | 3d uncertainty (m)

An extended format can be made which outputs trajectory uncertainty at each LiDAR point as well (see doit.m). Use for investigation but not production.

The result is best explored using CloudCompare. Here, ASCII clouds can be re-exported as some more efficient format (eg .las).

Importantly, heights are computed using whatever input coordinate system is used - so if you input orthometric heights you get orthometric heights. If you send in ellipsoidal heights, you get back ellipsoidal heights.

### Things to do:
# Port to Python, in the lidar_python branch
# Ingest binary files from laser scanners, skipping the huuuuge ASCII lidar input files
# send back binary files, or potentially HDF (looking ahead to emerging OGC standards)
