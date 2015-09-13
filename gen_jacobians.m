%make symbolic values for everything:
syms gpsX gpsY gpsZ r a roll pitch yaw bsX bsY bsZ laX laY laZ real;

%put the symbolic values into the functional model (from above)
f = [gpsX gpsY gpsZ]' ...
    - [  cos(pitch)*sin(yaw), sin(pitch)*sin(roll)*sin(yaw) - cos(roll)*cos(yaw), cos(yaw)*sin(roll) + cos(roll)*sin(pitch)*sin(yaw); ...
         cos(pitch)*cos(yaw), cos(roll)*sin(yaw) + cos(yaw)*sin(pitch)*sin(roll), cos(roll)*cos(yaw)*sin(pitch) - sin(roll)*sin(yaw); ...
         -sin(pitch),                               cos(pitch)*sin(roll),                               cos(pitch)*cos(roll) ...
      ]... % this is the rotation from IMU to mapping (ground) coordinates, should be 3 * 3
    * ...
       ([ cos(bsY)*cos(bsZ),                              cos(bsY)*sin(bsZ),          -sin(bsY); ...
          -cos(bsX)*sin(bsZ) - cos(bsZ)*sin(bsX)*sin(bsY), cos(bsX)*cos(bsZ) - sin(bsX)*sin(bsY)*sin(bsZ), -cos(bsY)*sin(bsX); ...
         cos(bsX)*cos(bsZ)*sin(bsY) - sin(bsX)*sin(bsZ), cos(bsZ)*sin(bsX) + cos(bsX)*sin(bsY)*sin(bsZ),  cos(bsX)*cos(bsY) ...
       ] ...% this is the rotation from laser coordinates to IMU coordinates, 3 * 3
        * ...
        (r*[-sin(a) 0 -cos(a)]')... % this is the determination of laser coordinates from range and angle
        +...
        [laX , laY , laZ ]' ) ... % and a lever arm adjustment 
    ;

%test the model... does it make sane LiDAR points?
%pointXYZ = subs(f, {gpsX gpsY gpsZ pitch roll yaw bsX bsY bsZ r a laX laY laZ}, {GPSX GPSY GPSZ Pitch Roll Yaw BSx BSy BSz R A LAx LAy LAz});

%now to attempt partial derivatives of the above function
% with respect to each parameter
%GPS first
gpsX_ = diff(f, gpsX)
gpsY_ = diff(f, gpsY)
gpsZ_ = diff(f, gpsZ)

%now IMU orientation - differentiate using symblic math
P_ = diff(f, pitch)
R_ = diff(f, roll)
Y_ = diff(f, yaw)

%now boresight misalignment - differentiate
bsX_ = diff(f, bsX)
bsY_ = diff(f, bsY)
bsZ_ = diff(f, bsZ)

%range and angle from the LIDAR -differentiate
lR_ = diff(f, r)
lA_ = diff(f, a)

%finally the lever arm measurement
laX_ = diff(f, laX)
laY_ = diff(f, laY)
laZ_ = diff(f, laZ)
