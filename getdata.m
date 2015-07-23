%data ingestion and preprocessing for LiDAR georeferencing
% Adam Steer
% adam.d.steer@gmail.com
% July 2015
%-----------------------------------------------------------------
% 21 July 2015: all configuration is done in 'doit.m'.
% 20 July 2015: logfiles are kept of basic processing settings and 
%               some maybe useful summary stats
%
%-----------------------------------------------------------------

%start logging, turned off at the end of makeswath.
diary(['../' lidarfile '_processing.log']);

disp(['georeferencing lidar file ' lidarfile ' using trajectory ' trajectoryfile] )
disp('basic processing settings:' )
disp(['modify trajectory to local coordinates: ' num2str(tmod) ' (-1 = no, > 0 = yes)'])
disp(['add bandpassed noise to the trajectory: ' num2str(addnoise) ' (-1 = no, > 0 = yes)'])
disp(['use riegl or computed lidar Y coordinates: ' num2str(makerxyz) ' (-1 = riegl, > 0 = computed)'])
disp(['give extended output(trajectory uncertainty): ' num2str(ext_out) ' (-1 = no, > 0 = yes)'])

%output file prefixes:
% to be automagicalised
file_prefix = lidarfile;

tic
%import trajectory data
%  - output from RTpost process as .3dp file, converted to UTM coords using
%  cs2cs via the 3dp2utm.sh script.
% format: sec-of-week, utm E, utm N, height (ellispoidal), roll, pitch,heading,
%         sd (lat), sd(lon), sd(up), sd(pitch), sd(roll), sd(heading)
% no line skip since the file is beheaded by 3dp2utm.
disp('reading trajectory')

traj=dlmread([trajectoryfolder trajectoryfile], ' ');

%ditch everything outside our time range

this = find(traj(:,1) >= t_start & traj(:,1) <= t_stop);
traj = traj(this,:);
clear this;

traj_time = traj(:,1); %get time from file
%echo time, quick sanity check
disp(['trajectory start time: ' num2str(min(traj_time))])
disp(['trajectory stop time: ' num2str(max(traj_time))])
disp(['flight seconds: ' num2str(max(traj_time) - min(traj_time))] )
toc

%## here's where we load the LiDAR ASCII files...
% - converted to ASCII from riegl .2dd 
tic
disp('reading LiDAR file')
lidar = dlmread([lidarfolder lidarfile], ' ', 0,0);

%implement a range gate
r_gate = find(lidar(:,2) > range_gate(1) & lidar(:,2) < range_gate(2));
lidar = lidar(r_gate, :);

lidar_time=lidar(:,1)+t_offset; %get time from
%sanity check pt 2, if this range is not close to the IMU
% range, we got trouble..

this = find(lidar_time > t_start & lidar_time < t_stop);
lidar = lidar(this,:);
lidar_time = lidar_time(this);

disp(['lidar start time: ' num2str(min(lidar_time))])
disp(['lidar stop time: ' num2str(max(lidar_time))])
disp(['flight seconds: ' num2str(max(lidar_time) - min(lidar_time))] )

%reduce the pos/imu data to just the window of laser obs;
j=find(traj_time >= min(lidar_time) & traj_time <= max(lidar_time));
if length(j) < 1
 j = find(traj_time > -1);
end

traj_subset_time=traj_time(j); %trim IMU time to laser time window
GPS = zeros(length(j), 3);

disp(['trimmed trajectory start time: ' num2str(min(traj_subset_time))])
disp(['trimmed trajectory stop time: ' num2str(max(traj_subset_time))])
disp(['flight seconds: ' num2str(max(traj_subset_time) - min(traj_subset_time))] )


clear this r_gate;
toc

tic
disp('scanline finding, and LiDAR preprocessing')

scan=find(abs(diff(lidar(:,3)))>1); %these are the scan lines

%select laser data 
D=lidar(:,2); %laser range
A=lidar(:,3); %scan angle
I=lidar(:,7); %intensity
lXYZ = lidar(:,4:6); %riegl XYZ 

% convert range and angle to XYZ
% in LiDAR coordinate frame
cX = D.*sin(A.*pi/180);
cZ = D.*cos(A.*pi/180);
cXYZ =[cX zeros(length(D),1) cZ]; %computed LiDAR XYZ

toc

%optional section to subtract ship motion from the trajectory
% prior to liDAR point generation
% optimally requires an ice-based GPS station
if tmod > 0
    file_prefix = [file_prefix '_basemod'];
    disp('reading base station file')
    %import GPS base, reduce GPS base pos to LiDAR obs:
    base_pos=dlmread(ice_base_file, ' ');

    disp('find base position at t0')
    e_o = base_pos(1,2);
    n_o = base_pos(1,3);
    u_o = base_pos(1,4);
    %using flight start time here, since we sometimes split
    %lidar into chunks but they need the same reference point.
    
    t_b = base_pos(:,1);
    disp('finding base data which match LiDAR timeframe')
    j_=find(t_b >= min(lidar_time) & t_b <= max(lidar_time));
    if length(j_) < 1
         j_ = find(t_b > -1);
    end
    t_b = base_pos(j_,1);
    b_pos(:,1) = base_pos(j_,2);
    b_pos(:,2) = base_pos(j_,3);
    b_pos(:,3) = base_pos(j_,4);

    disp('subtracting base at t0 from base trajectory to make base corrections')
    b_corr(:,1) = b_pos(:,1) - e_o;
    b_corr(:,2) = b_pos(:,2) - n_o;
    b_corr(:,3) = b_pos(:,3);% - u_o;
    
    disp('interpolating base data to aircraft data rate')
    bc_i(:,1)=interp1(t_b,b_corr(:,1),traj_time, 'linear', 'extrap');
    bc_i(:,2)=interp1(t_b,b_corr(:,2),traj_time, 'linear', 'extrap');
    bc_i(:,3)=interp1(t_b,b_corr(:,3),traj_time, 'linear', 'extrap');

    disp('adding base corrections to aircraft trajectory')
    GPS(:,1)=traj(j,2)+bc_i(j,1); 
    GPS(:,2)=traj(j,3)+bc_i(j,2); 
    GPS(:,3)=traj(j,4)+bc_i(j,3);
    disp('done')

else
    disp('finding trajectory data which match LiDAR timeframe')
    GPS(:,1)=traj(j,2); % these are already UTM in the relevant zone
    GPS(:,2)=traj(j,3); % NOT lat/lon. 
    GPS(:,3)=traj(j,4);
end

%apply angular adjustments
roll=traj(j,5)+r_adj; % and gyro angles
pitch=traj(j,6)+p_adj;
yaw=traj(j,7)+y_adj;

toc

if addnoise > 0
  file_prefix = [file_prefix '_nr' num2str(addnoise) '_'];
  tic
  %mess with the trajectory signal here... run 'find_passbands' first...
  disp('use raw IMU accelerations to attempt to mitigate unmodelled LiDAR motion')
  tic
  [dr, dp, dy] = hprwiggle(imufile,250);
  %generate 3 signals with passbands given in bracketed frequencies
  % guesstimate passbands from find_passbands
  dr_b = IMU_bandpass(dr, 250, [4 8], [12 25], [35 42]); dr_b = dr_b(j)';
  dp_b = IMU_bandpass(dp, 250, [4 8], [12 25], [32 53]); dp_b = dp_b(j)';
  dy_b = IMU_bandpass(dy, 250, [4 8], [12 25], [38 40]); dy_b = dy_b(j)';
  
  W = [dr,dp,dy];
  
  disp('adding noise to attitude signal')

  roll = roll + addnoise.*(dr_b(:,1) + dr_b(:,2) + dr_b(:,3));
  pitch = pitch + addnoise.*(dp_b(:,1) + dp_b(:,2) + dp_b(:,3));
  yaw = yaw + addnoise.*(dy_b(:,1) + dy_b(:,2) + dy_b(:,3));
  toc
end

%adjust yaw to remove jumps
tic
disp('adjusting heading to remove jumps from 360 to 0')
this = diff(yaw);
yawo = yaw;
J = find(abs(this)> 90);
if(length(J) > 1)
 yaw(J(1)+1:J(2)) = yaw(J(1)+1:J(2))-360;
elseif(length(J) == 1)
 yaw(J(1)+1:length(yaw)) = yaw(J(1)+1:length(yaw))-360;
end
clear this;
toc
disp('data read done')
%END DATA READ
%+++++++++++++++++++++++++++++++++++++++++++++++++++++++

tic
disp('interpolating IMU data (100 or 250 Hz) to laser frequency (10 kHz)')
%INTERPOLATION

%attitude interpolation
R=interp1(traj_subset_time,roll,lidar_time, 'linear', 'extrap');
P=interp1(traj_subset_time,pitch,lidar_time, 'linear', 'extrap');
YAW=interp1(traj_subset_time,yaw,lidar_time, 'linear', 'extrap');

%position interpolation
lon=interp1(traj_subset_time,GPS(:,1),lidar_time, 'linear', 'extrap');
lat=interp1(traj_subset_time,GPS(:,2),lidar_time, 'linear', 'extrap');
elev=interp1(traj_subset_time,GPS(:,3),lidar_time, 'linear', 'extrap');

%error interpolation
GPSXe = interp1(traj_subset_time, traj(j,9), lidar_time, 'linear', 'extrap');
GPSYe = interp1(traj_subset_time, traj(j,8), lidar_time, 'linear', 'extrap');
GPSZe = interp1(traj_subset_time, traj(j,10), lidar_time, 'linear', 'extrap');

eP = interp1(traj_subset_time, traj(j,11), lidar_time, 'linear', 'extrap') .* pi/180;
eR = interp1(traj_subset_time, traj(j,12), lidar_time, 'linear', 'extrap') .* pi/180;
eH = interp1(traj_subset_time, traj(j,13), lidar_time, 'linear', 'extrap') .* pi/180;

file_prefix = [file_prefix num2str(floor(min(lidar_time))) '_' num2str(floor(max(lidar_time)))];

%DONE.....
%clear GPS; clear roll; clear pitch; clear yaw; clear traj; clear lidar;
disp('done data read and ready to roll a LiDAR cloud')
disp(['lidar lines: ' num2str(length(scan))])
disp(['mean laser range: ' num2str(mean(D))])
toc