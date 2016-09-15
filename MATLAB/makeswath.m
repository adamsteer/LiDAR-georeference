
tic
disp('setting up for georeferencing LiDAR')
%suck up some memory for coordinate arrays

xerr = zeros(length(elev),1);
yerr = zeros(length(elev),1);
zerr = zeros(length(elev),1);

%scan angle to radians
Ar = A * pi/180;

% set up vector of angles for IMU -> world rotation
Rr = R * pi/180; Pr = P * pi/180; YAWr = YAW * pi/180;

RPY = [Rr Pr YAWr];

toc
%make points here:
disp('running uncertainy computation')
tic
for i = 1:length(elev)
    %hard coded errors
    %[xerr(i), yerr(i), zerr(i)] = propogate_errs([lon(i), lat(i), elev(i)],
    %RPY(i,:), boresight, [D(i), A(i)], leverarm, errGPS, errPRH, errBS, errLiDAR, errLA);
    
    %input errors from 3dp file - testing heading input...
    [xerr(i), yerr(i), zerr(i)] = propogate_errs([lon(i), lat(i), elev(i)], [RPY(i,1), RPY(i,2), 0], boresight, [D(i), Ar(i)], leverarm, [GPSXe(i) GPSYe(i) GPSZe(i)], [eP(i) eR(i) eH(i)] , errBS, errLiDAR, errLA);

end
toc
disp('uncertainies computed')

%switch to write out LiDAR with riegl's computed lidar coordinates, 
% or liDAR XY coordinates computed using ranges and angles in getdata.m
if makerxyz > 0
    tic
    disp('geolocation using riegl computed XY lidar coordinates')
    file_prefix = [file_prefix '_r.xyz'];
    xyz = zeros(length(elev),3);

    for i = 1:length(elev)
    %using the function 'lidargeo' 
        xyz(i,:) = lidargeo([lon(i),lat(i),elev(i)],D(i),Ar(i),RPY(i,:),boresight,leverarm,lXYZ(i,:));
    end

    disp('writing LiDAR and error combined file')
    fid = fopen([path_to file_prefix], 'w');
    if ext_out < 0
        xyz = [lidar_time, xyz, I, A, sqrt(xerr), sqrt(yerr), sqrt(zerr), sqrt(xerr + yerr + zerr)];
        fprintf(fid, '%.5f %.6f %.6f %.6f %.6f %.6f %.4f %.4f %.4f %.4f\n', xyz');
    else
        xyz = [lidar_time, xyz, I, A, sqrt(xerr), sqrt(yerr), sqrt(zerr), sqrt(xerr + yerr + zerr), GPSXe(i), GPSYe(i), GPSZe(i)];
        fprintf(fid, '%.5f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n', xyz');
    end
    fclose(fid);
    toc
else
    tic
    file_prefix = [file_prefix '_c.xyz'];
    disp('geolocation using computed LiDAR coordinates based on range and angle')
    xyz = zeros(length(elev),3);
    for i = 1:length(elev)
    %using the function 'lidargeo' 
        xyz(i,:) = lidargeo([lon(i),lat(i),elev(i)],D(i),Ar(i),RPY(i,:),boresight,leverarm,0);
    end
    disp('writing LiDAR and error combined file')
    fid = fopen([path_to file_prefix], 'w');
    if ext_out < 0
        xyz = [lidar_time, xyz, I, A, sqrt(xerr), sqrt(yerr), sqrt(zerr), sqrt(xerr + yerr + zerr)];
        fprintf(fid, '%.5f %.6f %.6f %.6f %.6f %.6f %.4f %.4f %.4f %.4f\n', xyz');
    else
        xyz = [lidar_time, xyz, I, A, sqrt(xerr), sqrt(yerr), sqrt(zerr), sqrt(xerr + yerr + zerr), GPSXe(i), GPSYe(i), GPSZe(i)];
        fprintf(fid, '%.5f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f\n', xyz');
    end
    fclose(fid);
    toc
end

disp(['modal Z uncertainty for this swath: ' num2str(mode(sqrt(zerr))) ' m'])
disp(['min LiDAR point elevation: ' num2str(min(xyz(:,4))) ' m'])
disp(['max LiDAR point elevation: ' num2str(max(xyz(:,4))) ' m'])
disp(['modal LiDAR point elevation: ' num2str(mode(xyz(:,4))) ' m'])
disp(['1-sigma LiDAR point elevation: ' num2str(std(xyz(:,4))) ' m'])

%use cloudcompare to take a look, much quicker than matlab...
clear xyz xerr yerr zerr traj lidar lidar_time traj_time traj_subset_time D A I lXYZ cX cZ cXYZ roll pitch yaw;
clear GPSXe GPSYe GPSZe yawo i j fid Ar scan
disp('done - use CloudCompare or a similar tool to inspect the results')
diary off