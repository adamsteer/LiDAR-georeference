
%this file name is generally the only thing that changes
% often. All else is pretty much fixed per flight.
lidarfile = '20101106_Alpha_2mil_ad.rasc'; %the file to process

%----------------------------------------------------------
%all below here is generally set once per flight

lidarfolder = '../lidar/'; %ascii lidar file directory
imufile = '../imu/mobile.rot'; %imu acceletation file
trajectoryfile = '20101106_alpha_ppp_utm45.3dp'; %trajectory file
trajectoryfolder = '../trajectory/'; %trajectory file root

path_to = '../swaths/'; %where to place processed point clouds

%optional time windowing. Set to ridiculous so that an entire input
% file is processed.
t_start = 0;
t_stop = 10000000;

%GPS second of week to start the relevant day.
% required because lIDAR times are second-of-day, trajectory times
% are second of week. Compute by:
% flight-day-of-week * 86400, where sunday = 0 and saturday = 6.

gps_secofweek = 518400;

% time offset. For SIPEX 2007 data in particular, GPS leap seconds
% are not accounted for. generally use integer seconds here.
sync_offset = 1.0;

t_offset = sync_offset + gps_secofweek;

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
r_adj = 0; p_adj = 0; y_adj = 0;

%range gate for liDAR
range_gate = [50,900];

%------------some harward configuration---------------------%

% set up boresight angles, assuming no error (error can be added here later)
%These descirbe the relationship between the LIDAR coordinate system and the
%IMU coordinat system.
%----------DO NOT ALTER FOR RAPPLS PROCESSING---------------%
BSX = -90;
BSY = 90;
BSZ = 90; %mostly right

%-----------------------------------------------------------%

%next, boresight misalignment angles
%corrections from 2012 calibration run, in IMU frame
%bsc_x = 0.0019; %roll axis
%bsc_y = 0.0034; %pitch axis
%bsc_z = 0.0008; %yaw axis

%corrections from 2010 calibration, in IMU frame
bsc_x = 0.0043; %roll
bsc_y = 0.0035; %pitch
bsc_z = 0.0272; %heading

%add empirical offsets here if required
% note these are offsets in IMU coordinate frame
% between the IMU and the LiDAR. They should be
% pretty small! Please use EITHER these or
% attitiude adjustments in getdata.m, not both.
% bsc_x = 0.0043; %roll
% bsc_y = 0.0035; %pitch
% bsc_z = 0.0272; %heading


boresight = [BSX+bsc_x BSY+bsc_y BSZ+bsc_z] *  pi/180;
%------------end of boresight---------------------------------%


%Lever arm, X Y Z in IMU frame. 0 if output is already displaced in RTPP
%for RAPPLS, use:
%leverarm =[0.1405 -0.196 0.050]
leverarm =[0.0 0.0 0.0];
%------------end of lever arm---------------------------------%


%------------a priori uncertainties---------------------------------%
%position uncertainty comes as a per-epoch value from the 
% input trajectory. For experimental purposes a constant value
% can be set here.
%errGPS = [ 0.1, 0.1, 0.15 ]; 

%angular uncertainty comes as a per-epoch value from the 
% input trajectory. For experimental purposes a constant value
% can be set here.
%errPRH = [ 0.015 0.015 0.05] * pi/180; %Glennie 2007 - lowball
%errPRH = [ 0.03 0.03 0.1] * pi/180; %Glennie 2007, OxTS

%conservative boresight angular uncertainty - estimate from drawings and 
%some communications with terrasolid.
%errBS =[ deg0.001,0.001, 0.03].*(pi/180); %RAPPLS
errBS = [ 0.001, 0.001, 0.004].*(pi/180); %Glennie
%..about the pitch, roll and heading axes respectively

%from Riegl - liDAR uncertainties.
errLiDAR = [ 0.02, 0.039*pi/180];
% .039 includes the Riegl 0.005 degree figure and
% a beam divergence term from Glennie (2007), which is modelled as 1/4
% of beam divergence after Lichti and Gordon (2004).

%best guess at lever arm measurement uncertainty
% from engineering drawings/tape measure, in metres XYZ in the IMU frame.
% 1cm comes from an as-built engineering model.
errLA = [ 0.01, 0.01, 0.01]; %rappls

%end of config
%------------------------------------------------------------------------
%configured! now make LiDAR....

disp(['georeferencing lidar file ' lidarfile ' using trajectory ' trajectoryfile] )
disp('basic processing settings:' )
disp(['modify trajectory to local coordinates: ' num2str(tmod) ' (-1 = no, > 0 = yes)'])
disp(['add bandpassed noise to the trajectory: ' num2str(addnoise) ' (-1 = no, > 0 = yes)'])
disp(['use riegl or computed lidar Y coordinates: ' num2str(makerxyz) ' (-1 = riegl, > 0 = computed)'])
disp(['give extended output(trajectory uncertainty): ' num2str(ext_out) ' (-1 = no, > 0 = yes)'])
disp(['GPS second of week at day start: ' num2str(gps_secofweek)])
disp(['time sync (leap seconds) offset: ' num2str(sync_offset)])
disp(['range gate (m): ' num2str(range_gate)])

disp('instrument settings:' )
disp(['applied IMU-> LiDAR lever arm (X Y Z, metres): ' num2str(leverarm)])
disp(['boresight orientation X: ' num2str(BSX)])
disp(['boresight orientation Y: ' num2str(BSY)])
disp(['boresight orientation Z: ' num2str(BSZ)])

disp('angular adjustments, determined empirically:' )
disp(['heading adjustment: ' num2str(y_adj)])
disp(['pitch adjustment: ' num2str(p_adj)])
disp(['roll adjustment: ' num2str(r_adj)] )

disp('boresighting misalignments from calibration:' )
disp(['applied boresight misalignment X: ' num2str(bsc_x)])
disp(['applied boresight misalignment Y: ' num2str(bsc_y)])
disp(['applied boresight misalignment Z: ' num2str(bsc_z)])

disp('fixed uncertainties:' )
disp(['lidar range (m) and angular (degrees) uncertainty: ' num2str(errLiDAR(1)) ', ' num2str(errLiDAR(2)*180/pi)])
disp(['lever arm measurement uncertainty: ' num2str(errLA)])
disp(['boresight alignment uncertainty: ' num2str(errBS * 180/pi)])


getdata; 
makeswath;
