
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

% set up boresight angles, assuming no error (error can be added here later)
%working last was -90 90 90
% 90 -90 -90 also worked - maybe...
BSX = -90;
BSY = 90;
BSZ = 90; %mostly right
disp(['boresight orientation X: ' num2str(BSX)])
disp(['boresight orientation Y: ' num2str(BSY)])
disp(['boresight orientation X: ' num2str(BSZ)])

%corrections from 2012 calibration run, in IMU frame
%bsc_x = 0.0019; %roll axis
%bsc_y = 0.0034; %pitch axis
%bsc_z = 0.0008; %yaw axis

%add empirical offsets here if required
% note these are offsets in IMU coordinate frame
% between the IMU and the LiDAR. They should be
% pretty small! Please use EITHER these or
% attitiude adjustments in getdata.m, not both.
bsc_x = 0;
bsc_y = 0;
bsc_z = 0;
disp(['applied boresight misalignment X: ' num2str(bsc_x)])
disp(['applied boresight misalignment Y: ' num2str(bsc_y)])
disp(['applied boresight misalignment Z: ' num2str(bsc_z)])

boresight = [BSX+bsc_x BSY+bsc_y BSZ+bsc_z] *  pi/180;


%and lever arm, X Y Z in IMU frame. 0 if output is already displaced in RTPP
%leverarm =[0.1405 -0.196 0.050]
leverarm =[0.0 0.0 0.0];
disp(['applied IMU-> LiDAR lever arm (X Y Z, metres): ' num2str(leverarm)])


%provide errors:
%conservative GPS errors in m
%eventually these will come from function args. if no input use these:
%errGPS = [ 0.1, 0.1, 0.15 ]; 

%from OxTS
%if no args, use these
%errPRH = [ 0.015 0.015 0.05] * pi/180; %Glennie
%errPRH = [ 0.03 0.03 0.1] * pi/180; %Glennie


%conservative boresight measurements - estimate from drawings and 
%some insight from terrasolid
%errBS =[ deg0.001,0.001, 0.03].*(pi/180); %RAPPLS
errBS = [ 0.001, 0.001, 0.004].*(pi/180); %Glennie
disp(['boresight alignment uncertainty: ' num2str(errBS * 180/pi)])
%..about the pitch, roll and heading axes respectively

%from Riegl - 
errLiDAR = [ 0.02, 0.039*pi/180];
disp(['lidar range (m) and angular (degrees) uncertainty: ' num2str(errLiDAR(1)) ', ' num2str(errLiDAR(2)*180/pi)])
% .039 includes the Riegl 0.005 degree figure and
% a beam divergence term from Glennie (2007)

%best guess at lever arm from engineering drawings/tape measure, in metres XYZ
% in the IMU frame
errLA = [ 0.01, 0.01, 0.01]; %rappls
disp(['lever arm measurement uncertainty: ' num2str(errLA)])

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
    %save xyzadam; clear xyzadam;
    toc
end

disp(['modal Z uncertainty for this swath: ' num2str(mode(sqrt(zerr))) ' m'])
disp(['min LiDAR point elevation: ' num2str(min(xyz(:,4))) ' m'])
disp(['max LiDAR point elevation: ' num2str(max(xyz(:,4))) ' m'])
disp(['modal LiDAR point elevation: ' num2str(mode(xyz(:,4))) ' m'])
disp(['1-sigma LiDAR point elevation: ' num2str(std(xyz(:,4))) ' m'])

%use cloudcompare to take a look, much quicker than matlab...
clear all;
disp('done - use CloudCompare or a similar tool to inspect the results')
diary off