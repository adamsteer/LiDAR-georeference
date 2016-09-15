function [X_var,Y_var,Z_var] = propogate_errs(gpsXYZ, RPY, bsXYZ, RA, laXYZ, errGPS, errRPY, errBS, errLiDAR, errLA)

%to do:
% feed trajectory std errors to the error code as another term:
% GPSerr

%error propagation...
%{
so we need to propagate errors for 14 parameters, which are:

gpsX
gpsY
gpsZ

laserRange
laserAngle

roll
pitch
heading

boresightX
boresightY
boresightZ

leverarmX
leverarmY
leverarmZ

...through the model:

i. [X Y Z] = [gpsX gpsY gpsZ] + Ri_g*(Rl_i* laserRange*[0 sin(laserAngle) -cos(laserAngle)] + [leverarmX leverarmY leverarmZ])

...where Ri_g is the rotation from IMU frame to mapping frame (pitch, roll, heading) and Rl_i is the rotation matrix from the laser frame to the imu frame

%}

%unpack the function arguments
gpsX = gpsXYZ(1); gpsY = gpsXYZ(2); gpsZ = gpsXYZ(3);
roll = RPY(1); pitch = RPY(2); yaw = RPY(3);
bsX = bsXYZ(1); bsY = bsXYZ(2); bsZ = bsXYZ(3);
r = RA(1); a = RA(2);
laX = laXYZ(2); laY = laXYZ(2); laZ = laXYZ(3);

%errorparams
eGPSX = errGPS(1); eGPSY = errGPS(2); eGPSZ = errGPS(3); 

%from OxTS
%if no args, use these
eRoll = errRPY(1); ePitch = errRPY(2);  eYaw = errRPY(3);

%boresight errors
eBSx = errBS(1); %roll 
eBSy = errBS(2); %pitch
eBSz = errBS(3); %yaw - 

%from Riegl - including beam divergence (1/4 of laser spec as per Glennie 2007)
eR = errLiDAR(1); eA = errLiDAR(2);

%best guess at lever arm from engineering drawings/tape measure, in m
eLAx = errLA(1); eLAy = errLA(2); eLAz = errLA(3);

%---------------------------
%matrices below derived by the function 'gen_jacobians.m'. requires the matlab symbolic math toolbox.
% if rotations in the functional model (lidargeo.m) change, use
% 'make_rots.m' to make the required XYZ rotation matrices. Use the output
% of make_rots.m to replace the rotations in gen_jacobians.m, then run
% gen_jacobians to get the derivatives wrt to each parameter. use 'diary' to
% capture the output, then copy and past to here.

%GPS XYZ - always a unit matrix
%---------------------------
gpsXYZd = [ 1 0 0; ...
            0 1 0; ...
            0 0 1 ];

%ANGLES ROLL, PITCH, YAW
%---------------------------
angle_roll = [ (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(laZ - r*sin(a)*sin(bsY) + r*cos(a)*cos(bsX)*cos(bsY)) - (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(laX + r*cos(a)*(sin(bsX)*sin(bsZ) + cos(bsX)*cos(bsZ)*sin(bsY)) + r*cos(bsY)*cos(bsZ)*sin(a)); ...
               (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(laX + r*cos(a)*(sin(bsX)*sin(bsZ) + cos(bsX)*cos(bsZ)*sin(bsY)) + r*cos(bsY)*cos(bsZ)*sin(a)) - (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(laZ - r*sin(a)*sin(bsY) + r*cos(a)*cos(bsX)*cos(bsY)); ...
               -cos(pitch)*sin(roll)*(laZ - r*sin(a)*sin(bsY) + r*cos(a)*cos(bsX)*cos(bsY)) - cos(pitch)*cos(roll)*(laX + r*cos(a)*(sin(bsX)*sin(bsZ) + cos(bsX)*cos(bsZ)*sin(bsY)) + r*cos(bsY)*cos(bsZ)*sin(a)) ];
 
%----------------------------
angle_pitch = [ cos(pitch)*sin(roll)*sin(yaw)*(laX + r*cos(a)*(sin(bsX)*sin(bsZ) + cos(bsX)*cos(bsZ)*sin(bsY)) + r*cos(bsY)*cos(bsZ)*sin(a)) - cos(pitch)*cos(roll)*sin(yaw)*(laZ - r*sin(a)*sin(bsY) + r*cos(a)*cos(bsX)*cos(bsY)) - sin(pitch)*sin(yaw)*(laY - r*cos(a)*(cos(bsZ)*sin(bsX) - cos(bsX)*sin(bsY)*sin(bsZ)) + r*cos(bsY)*sin(a)*sin(bsZ)); ...
                cos(pitch)*cos(yaw)*sin(roll)*(laX + r*cos(a)*(sin(bsX)*sin(bsZ) + cos(bsX)*cos(bsZ)*sin(bsY)) + r*cos(bsY)*cos(bsZ)*sin(a)) - cos(pitch)*cos(roll)*cos(yaw)*(laZ - r*sin(a)*sin(bsY) + r*cos(a)*cos(bsX)*cos(bsY)) - cos(yaw)*sin(pitch)*(laY - r*cos(a)*(cos(bsZ)*sin(bsX) - cos(bsX)*sin(bsY)*sin(bsZ)) + r*cos(bsY)*sin(a)*sin(bsZ)); ...
                cos(pitch)*(laY - r*cos(a)*(cos(bsZ)*sin(bsX) - cos(bsX)*sin(bsY)*sin(bsZ)) + r*cos(bsY)*sin(a)*sin(bsZ)) - cos(roll)*sin(pitch)*(laZ - r*sin(a)*sin(bsY) + r*cos(a)*cos(bsX)*cos(bsY)) + sin(pitch)*sin(roll)*(laX + r*cos(a)*(sin(bsX)*sin(bsZ) + cos(bsX)*cos(bsZ)*sin(bsY)) + r*cos(bsY)*cos(bsZ)*sin(a)) ];
 
%---------------------------
angle_yaw =   [ cos(pitch)*cos(yaw)*(laY - r*cos(a)*(cos(bsZ)*sin(bsX) - cos(bsX)*sin(bsY)*sin(bsZ)) + r*cos(bsY)*sin(a)*sin(bsZ)) - (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(laX + r*cos(a)*(sin(bsX)*sin(bsZ) + cos(bsX)*cos(bsZ)*sin(bsY)) + r*cos(bsY)*cos(bsZ)*sin(a)) - (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(laZ - r*sin(a)*sin(bsY) + r*cos(a)*cos(bsX)*cos(bsY)); ...
                -(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(laZ - r*sin(a)*sin(bsY) + r*cos(a)*cos(bsX)*cos(bsY)) - (cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(laX + r*cos(a)*(sin(bsX)*sin(bsZ) + cos(bsX)*cos(bsZ)*sin(bsY)) + r*cos(bsY)*cos(bsZ)*sin(a)) - cos(pitch)*sin(yaw)*(laY - r*cos(a)*(cos(bsZ)*sin(bsX) - cos(bsX)*sin(bsY)*sin(bsZ)) + r*cos(bsY)*sin(a)*sin(bsZ)); ...
                0 ]; 
            
%---------------------------
% order here must match VCV matrix...
RPYd = [angle_roll, angle_pitch, angle_yaw];

%BORESIGHT*****
%---------------------------
bore_x = [ r*cos(a)*(cos(bsX)*sin(bsZ) - cos(bsZ)*sin(bsX)*sin(bsY))*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - r*cos(a)*cos(pitch)*sin(yaw)*(cos(bsX)*cos(bsZ) + sin(bsX)*sin(bsY)*sin(bsZ)) - r*cos(a)*cos(bsY)*sin(bsX)*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)); ...
           r*cos(a)*cos(bsY)*sin(bsX)*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) - r*cos(a)*cos(pitch)*cos(yaw)*(cos(bsX)*cos(bsZ) + sin(bsX)*sin(bsY)*sin(bsZ)) - r*cos(a)*(cos(bsX)*sin(bsZ) - cos(bsZ)*sin(bsX)*sin(bsY))*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)); ...
           -r*cos(a)*sin(pitch)*(cos(bsX)*cos(bsZ) + sin(bsX)*sin(bsY)*sin(bsZ)) - r*cos(a)*cos(pitch)*sin(roll)*(cos(bsX)*sin(bsZ) - cos(bsZ)*sin(bsX)*sin(bsY)) - r*cos(a)*cos(bsY)*cos(pitch)*sin(bsX)*cos(roll) ];
 
%---------------------------
bore_y = [ -(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw))*(r*cos(bsZ)*sin(a)*sin(bsY) - r*cos(a)*cos(bsX)*cos(bsY)*cos(bsZ)) - (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(r*cos(bsY)*sin(a) + r*cos(a)*cos(bsX)*sin(bsY)) - cos(pitch)*sin(yaw)*(r*sin(a)*sin(bsY)*sin(bsZ) - r*cos(a)*cos(bsX)*cos(bsY)*sin(bsZ)); ...
          (cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll))*(r*cos(bsZ)*sin(a)*sin(bsY) - r*cos(a)*cos(bsX)*cos(bsY)*cos(bsZ)) + (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(r*cos(bsY)*sin(a) + r*cos(a)*cos(bsX)*sin(bsY)) - cos(pitch)*cos(yaw)*(r*sin(a)*sin(bsY)*sin(bsZ) - r*cos(a)*cos(bsX)*cos(bsY)*sin(bsZ)); ...
          cos(pitch)*sin(roll)*(r*cos(bsZ)*sin(a)*sin(bsY) - r*cos(a)*cos(bsX)*cos(bsY)*cos(bsZ)) - cos(pitch)*cos(roll)*(r*cos(bsY)*sin(a) + r*cos(a)*cos(bsX)*sin(bsY)) - sin(pitch)*(r*sin(a)*sin(bsY)*sin(bsZ) - r*cos(a)*cos(bsX)*cos(bsY)*sin(bsZ)) ];
 
%---------------------------
bore_z = [ (r*cos(a)*(cos(bsZ)*sin(bsX) - cos(bsX)*sin(bsY)*sin(bsZ)) - r*cos(bsY)*sin(a)*sin(bsZ))*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) + cos(pitch)*sin(yaw)*(r*cos(a)*(sin(bsX)*sin(bsZ) + cos(bsX)*cos(bsZ)*sin(bsY)) + r*cos(bsY)*cos(bsZ)*sin(a)); ...
           cos(pitch)*cos(yaw)*(r*cos(a)*(sin(bsX)*sin(bsZ) + cos(bsX)*cos(bsZ)*sin(bsY)) + r*cos(bsY)*cos(bsZ)*sin(a)) - (r*cos(a)*(cos(bsZ)*sin(bsX) - cos(bsX)*sin(bsY)*sin(bsZ)) - r*cos(bsY)*sin(a)*sin(bsZ))*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)); ...
           sin(pitch)*(r*cos(a)*(sin(bsX)*sin(bsZ) + cos(bsX)*cos(bsZ)*sin(bsY)) + r*cos(bsY)*cos(bsZ)*sin(a)) - cos(pitch)*sin(roll)*(r*cos(a)*(cos(bsZ)*sin(bsX) - cos(bsX)*sin(bsY)*sin(bsZ)) - r*cos(bsY)*sin(a)*sin(bsZ)) ];

%---------------------------
bore_xyz = [bore_x, bore_y, bore_z];

%LASER SCANNER RANGE AND ANGLE
%---------------------------
laser_range = [ (cos(a)*(sin(bsX)*sin(bsZ) + cos(bsX)*cos(bsZ)*sin(bsY)) + cos(bsY)*cos(bsZ)*sin(a))*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)) - (sin(a)*sin(bsY) - cos(a)*cos(bsX)*cos(bsY))*(cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw)) - cos(pitch)*sin(yaw)*(cos(a)*(cos(bsZ)*sin(bsX) - cos(bsX)*sin(bsY)*sin(bsZ)) - cos(bsY)*sin(a)*sin(bsZ)); ...
                (sin(a)*sin(bsY) - cos(a)*cos(bsX)*cos(bsY))*(sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch)) - (cos(a)*(sin(bsX)*sin(bsZ) + cos(bsX)*cos(bsZ)*sin(bsY)) + cos(bsY)*cos(bsZ)*sin(a))*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) - cos(pitch)*cos(yaw)*(cos(a)*(cos(bsZ)*sin(bsX) - cos(bsX)*sin(bsY)*sin(bsZ)) - cos(bsY)*sin(a)*sin(bsZ)); ...
                -sin(pitch)*(cos(a)*(cos(bsZ)*sin(bsX) - cos(bsX)*sin(bsY)*sin(bsZ)) - cos(bsY)*sin(a)*sin(bsZ)) - cos(pitch)*cos(roll)*(sin(a)*sin(bsY) - cos(a)*cos(bsX)*cos(bsY)) - cos(pitch)*sin(roll)*(cos(a)*(sin(bsX)*sin(bsZ) + cos(bsX)*cos(bsZ)*sin(bsY)) + cos(bsY)*cos(bsZ)*sin(a)) ];
 
%---------------------------
laser_angle = [ cos(pitch)*sin(yaw)*(r*sin(a)*(cos(bsZ)*sin(bsX) - cos(bsX)*sin(bsY)*sin(bsZ)) + r*cos(a)*cos(bsY)*sin(bsZ)) - (cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw))*(r*cos(a)*sin(bsY) + r*cos(bsX)*cos(bsY)*sin(a)) - (r*sin(a)*(sin(bsX)*sin(bsZ) + cos(bsX)*cos(bsZ)*sin(bsY)) - r*cos(a)*cos(bsY)*cos(bsZ))*(cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw)); ...
               (r*sin(a)*(sin(bsX)*sin(bsZ) + cos(bsX)*cos(bsZ)*sin(bsY)) - r*cos(a)*cos(bsY)*cos(bsZ))*(cos(roll)*sin(yaw) - cos(yaw)*sin(pitch)*sin(roll)) + (sin(roll)*sin(yaw) + cos(roll)*cos(yaw)*sin(pitch))*(r*cos(a)*sin(bsY) + r*cos(bsX)*cos(bsY)*sin(a)) + cos(pitch)*cos(yaw)*(r*sin(a)*(cos(bsZ)*sin(bsX) - cos(bsX)*sin(bsY)*sin(bsZ)) + r*cos(a)*cos(bsY)*sin(bsZ)); ...
               sin(pitch)*(r*sin(a)*(cos(bsZ)*sin(bsX) - cos(bsX)*sin(bsY)*sin(bsZ)) + r*cos(a)*cos(bsY)*sin(bsZ)) - cos(pitch)*cos(roll)*(r*cos(a)*sin(bsY) + r*cos(bsX)*cos(bsY)*sin(a)) + cos(pitch)*sin(roll)*(r*sin(a)*(sin(bsX)*sin(bsZ) + cos(bsX)*cos(bsZ)*sin(bsY)) - r*cos(a)*cos(bsY)*cos(bsZ)) ];
 
%---------------------------
laser_RA = [laser_range, laser_angle];

%LEVER ARM
%---------------------------
lever_x = [ cos(roll)*cos(yaw) + sin(pitch)*sin(roll)*sin(yaw); ...
            cos(yaw)*sin(pitch)*sin(roll) - cos(roll)*sin(yaw); ...
                              -cos(pitch)*sin(roll) ];

%---------------------------
lever_y = [  cos(pitch)*sin(yaw); ...
             cos(pitch)*cos(yaw); ...
             sin(pitch)];

%---------------------------
lever_z = [ cos(yaw)*sin(roll) - cos(roll)*sin(pitch)*sin(yaw); ...
            - sin(roll)*sin(yaw) - cos(roll)*cos(yaw)*sin(pitch); ...
            cos(pitch)*cos(roll) ];

%---------------------------
laXYZd = [lever_x, lever_y, lever_z];

%generate the matrix of partial derivatives:
%---------------------------
J = [gpsXYZd, RPYd, bore_xyz, laser_RA, laXYZd];

%this follows schaer et al, usng the classical propagation of variances formula 
%make a diagonal matrix holding error variances (published standard deviations squared).
theerrors = diag([eGPSX, eGPSY, eGPSZ, eRoll, ePitch, eYaw, eBSx, eBSy, eBSz, eR, eA, eLAx, eLAy, eLAz]).^2;

%put it all together in the error model...
errorXYZ = J * theerrors * transpose(J);

%extract variances...
X_var = errorXYZ(1);
Y_var = errorXYZ(2,2);
Z_var = errorXYZ(3,3);



