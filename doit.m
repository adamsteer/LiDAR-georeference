
%this file name is generally the only thing that changes
% often. All else is pretty much fixed per flight.
lidarfile = '20091122_prydzbay1_04095_aa'; %the file to process

%----------------------------------------------------------
%all below here is generally set once per flight

lidarfolder = '../lidar/'; %ascii lidar file directory
imufile = '../imu/mobile.rot'; %imu acceletation file
trajectoryfile = 'prydz_040647_utm.3dp'; %trajectory file
trajectoryfolder = '../trajectory/'; %trajectory file root

path_to = '../swaths/'; %where to place processed point clouds

%optional time windowing. Set to ridiculous so that an entire input
% file is processed.
t_start = 0;
t_stop = 10000000;

% time offset. For SIPEX 2007 data in particular, GPS leap seconds
% are not accounted for. generally use integer seconds here.
t_offset = 1.0;

%switch to turn trajectory modification using base station data on or off.
% -1 = off, >0 = on. REQUIRES a base station trajectory in UTM coordinates 
% if turned on - see ice_base_file.
tmod = -1;
ice_base_file = '';

%switch to add IMU acceleration noise to heading, pitch and roll. The 
% value used is a scalar multiplier, ie -1 = off,  1 = noise *1, 2 = noise*2.
% EXPERIMENTAL
addnoise = 2;

%switch to control whether we output a point cloud with Riegl XY coordinates
% from the liDAR or XYZ coordinates computed from ranges and angles.
% -1 = used computed, > 0 = use Riegl
makerxyz = -1;

%append scan line and trajectory XYZ uncertainty to the end of output files:
ext_out = -1;

%angle adjustments. If you've empirically determined angles from LiDAR swaths, apply
% them here. If you have boresight misalignment parameters, set these to 0 and 
% use the boresight misalignment matrix in makeswath.m
r_adj = 0.1598; p_adj = 0; y_adj = 0;
disp(['heading adjustment: ' num2str(y_adj)])
disp(['pitch adjustment: ' num2str(p_adj)])
disp(['roll adjustment: ' num2str(r_adj)] )

%range gate for liDAR
range_gate = [50,900];
disp(['range gate (m): ' num2str(range_gate)])

%end of config
%------------------------------------------------------------------------

%configured! now make LiDAR....

getdata; makeswath;
