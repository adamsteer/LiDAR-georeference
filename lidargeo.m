function [xyz] = lidargeo(gpsXYZ,R,A,RPY,boresight,leverarm,rieglxyz)

%generate a laser coordinate. X axis = (vertical distance from LiDAR ->
%point), Y axis = front/back of aircraft as installed
% Z axis is across-track, inverted to flip from LH to RH coordinates here.
% works better than the discussed x-axis inversion...
if length(rieglxyz) > 1
  laserxyz = rieglxyz';
else
  laserxyz =  R*[sin(A) 0 cos(A)]';
end

% next, boresight...

BSx = boresight(1);
BSy = boresight(2);
BSz = boresight(3);

%using 'textbook' matrices from here:
% http://planning.cs.uiuc.edu/node102.html
% (among other sources)

rotbx = [ 1,        0,         0; ...
          0, cos(BSx), -sin(BSx); ...
          0, sin(BSx), cos(BSx)];
      
rotby = [cos(BSy), 0, sin(BSy);...
                0, 1,        0; ...
         -sin(BSy), 0, cos(BSy)];
      
rotbz = [ cos(BSz), -sin(BSz), 0; ...
          sin(BSz),  cos(BSz), 0; ...
                 0,         0, 1];
             
las2imu = rotbz*rotby*rotbx; %nearly works...
%las2imu = rotbx*rotby*rotbz;

%OK, now I'm rotating an IMU coordinate set
%(XY and -Z) to world coords (NEU). X = roll axis; Y = pitch axis, Z = heading
%rotation axis in the IMU frame.
Roll = RPY(1);
Pitch = RPY(2);
Yaw = RPY(3);

%again, using 'textbook' rotation matrices
% OK, IMU -> world is a reverse rotation using
% the euler angles, NOT a forward rotation..
% so, sign-of-sines swapped here.

%{
rotR = [1,         0,          0; ...
        0, cos(Roll), -sin(Roll); ...
        0, sin(Roll), cos(Roll)];
    
rotP = [ cos(Pitch), 0, sin(Pitch); ...
                  0, 1,          0; ...
        -sin(Pitch), 0, cos(Pitch)];
%}    


rotP = [1,         0,          0; ...
        0, cos(Pitch), -sin(Pitch); ...
        0, sin(Pitch), cos(Pitch)];
    
rotR = [ cos(Roll), 0, sin(Roll); ...
                  0, 1,          0; ...
        -sin(Roll), 0, cos(Roll)];


rotYAW = [ cos(Yaw), sin(Yaw), 0; ...
         -sin(Yaw),  cos(Yaw), 0; ...
                0,         0, 1];


% Riegl/RAPPLS does R -> P -> Y;  Luke rotates Yaw -> Pitch -> Roll
% here, best order so far is R -> P -> Y (right to left) 
rot2world = rotYAW * rotP * rotR;

%this is the functional model - making a world XYZ point from
% all the components
xyz = gpsXYZ' + rot2world * (las2imu * laserxyz + leverarm');
