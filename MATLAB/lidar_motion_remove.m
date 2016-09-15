
ice_base_file = './ice1/ice1_297_utm_headless.txt';
lidarfile = './2mil00_nr_cxyz_199043_199245.xyz';
outfile = './shifted_lidar/199043_199245_nr2.xyz';


%import trajectory data
%  - output from GPS processing

gps=dlmread(ice_base_file, ' ');

t2 = gps(:,1); %get time from file
%echo time, quick sanity check
min(t2)
max(t2)

%## here's where we load the LiDAR ASCII files...
% - converted to ASCII from riegl .2dd 
% - timestamps from second-of-day to second-of-week (add GPS secofweek for the day)
% - awk filtered to time, range, angle, intensity (.trai)

%import a chunk of LiDAR - actually, strip it to timestamps using AWK first
lidar=dlmread(lidarfile, ' ');

t1=lidar(:,1); %get time from
%sanity check pt 2, if this range is not close to the IMU
% range, we got trouble..
min(t1)
max(t1)

lidar_e = lidar(:,2);
lidar_n = lidar(:,3);
lidar_u = lidar(:,4);

%reduce the pos/imu data to just the window of laser obs;
j=find(t2 >= min(t1) & t2 <= max(t1));
if length(j) < 1
 j = find(t2 > -1);
end

t3=t2(j); %trim IMU time to laser time window
basepos(:,1)=gps(j,2); %and positions, strictly these are already UTM in the relevant zone
basepos(:,2)=gps(j,3); % NOT lat/lon. 
basepos(:,3)=gps(j,4);

%END DATA READ
%+++++++++++++++++++++++++++++++++++++++++++++++++++++++

%INTERPOLATION

%position interpolation
gps_e=interp1(t3,basepos(:,1),t1, 'linear', 'extrap');
gps_n=interp1(t3,basepos(:,2),t1, 'linear', 'extrap');
gps_u=interp1(t3,basepos(:,3),t1, 'linear', 'extrap');

%maths...

lidar(:,2) = lidar_e - (lidar_e - gps_e);
lidar(:,3) = lidar_n - (lidar_n - gps_n);
lidar(:,4) = lidar_u + gps_u;

%reroll...

fid = fopen(outfile, 'w');
fprintf(fid, '%.5f %.6f %.6f %.6f %.6f %.4f %.4f %.4f %.4f\n', lidar');
fclose(fid);

%done





