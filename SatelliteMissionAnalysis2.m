clear
clc
close all


%% Extract data from LatLonHeight.txt file

clear
clc
close all
%jsonData = read_json_arrays('C:\Users\ronak\OneDrive\Documents\MAVERIC\Sgp4Prop\SampleCode\Python\DriverExamples\Sgp4Prop\output\LatLonHeight.json');
jsonData = read_json_arrays('C:\Users\Zargai\MATLAB\Projects\magnetorquer_control\output\LatLonHeight.json');
jsonData2 = read_json_arrays('C:\Users\Zargai\MATLAB\Projects\magnetorquer_control\output\OscState.json');
height = [jsonData.HT_KM_];
latitude = [jsonData.LAT_DEG_];
longitude = [jsonData.LON_DEG_];
X_position = [jsonData.X_KM_];
Y_position = [jsonData.Y_KM_];
Z_position = [jsonData.Z_KM_];
Tsince = [jsonData.TSINCE_MIN_];
year = 2022;
X_velocity = [jsonData2.XDOT_KM_S_];
Y_velocity = [jsonData2.YDOT_KM_S_];
Z_velocity = [jsonData2.ZDOT_KM_S_];

% Convert position data from km to meters
positions_m = [jsonData.X_KM_; jsonData.Y_KM_; jsonData.Z_KM_]' * 1e3;

% Convert velocity data from km to meters
velocities_m = [jsonData2.XDOT_KM_S_; jsonData2.YDOT_KM_S_; jsonData2.ZDOT_KM_S_]' * 1e3;

%find datetime for each row
%jan 1, 1950 + (TSINCE (mins)) Epoch time from TLE
baseDate = datetime(1950, 1, 1, 0, 0, 0);
daysSince1950 = 26477.76488163;

% Vectorized computation of datetime
time = baseDate + days(daysSince1950) + minutes(Tsince);

%% Compute magnetic field value at each XYZ point
N = length(latitude);

% Preallocate output arrays
XYZ = zeros(N, 3); H = zeros(N, 1); D = zeros(N, 1); I = zeros(N, 1); 
F = zeros(N, 1); DXDYDZ  = zeros(N, 3); DH = zeros(N, 1); DD = zeros(N, 1); 
DI  = zeros(N, 1); DF = zeros(N, 1);

% Calculate magnetic field at each long/lat/height point 
for i = 1:N
    [XYZ(i,:), H(i), D(i), I(i), F(i), ...
     DXDYDZ(i,:), DH(i), DD(i), DI(i), DF(i)] = ...
     igrfmagm(height(i)*1000, latitude(i), longitude(i), year);
end

%where XYZ is the magnetic field vector array in nT north-east-down frame

%combine datetimes with magnetic field values (divide by 1000 to get uTs)
magneticfield = [num2cell(time(:)),num2cell(XYZ(:,1)/1000),num2cell(XYZ(:,2)/1000),num2cell(XYZ(:,3)/1000)];

% want it in Teslas, rotation matrix to get into body frame

%save time + magnetic field to .mat file
matObj = matfile('MagField.mat')
save('MagField.mat','magneticfield')


%% Visualize satellite orbit using the position output data

% Create position timetable
positionTT = timetable(time', positions_m, 'VariableNames', {'Position'});
sc = satelliteScenario(time(1), time(end), 1);   % set 1 second sample time

%Add the observation satellite to the scenario.
sat3 = satellite(sc,positionTT,"Name","MAVERIC")

ax = coordinateAxes(sat3);
coordinateAxes(sat3,Scale=2);

%Add another satellite for the purpose of checking rotated magnetic field
%vector
% mag = satellite(sc,positionTT,"Name","Magnetic Field")
% ax1 = coordinateAxes(mag)
% coordinateAxes(mag,Scale=2.4)

%hard-coded plots of start and end points for checking
%gsStart = groundStation(sc, 44.3200862,81.3990870,"Altitude", 515012.6876, Name="Starting Point");
%gsEnd = groundStation(sc, 29.8097085,6.3974097,"Altitude", 511851.5139, Name="Ending Point");

%Assign a 3-D model to the satellite and visualize its coordinate axes. 
%The red, green and blue arrows point along the x, y and z axes of the body frame of the satellite.
% sat3.Visual3DModel = "simple_model_assembly.STL";
% sat3.Visual3DModelScale = 20;



%% Conversion to body frame

% for every long/lat, rotate magnetic field vector 
% find and apply rotation matrix for north frame and east frame so satellite 
% maintains fixed orientation in the inertial frame


quats = zeros(N, 4);
R_all = zeros(3, 3, N);
mag_ecef = zeros(N, 3);
mag_ecef_tip = zeros(N, 3);
mag_eci = zeros(N, 3);
mag_body = zeros(N, 3);
origin_ecef = zeros(N, 3);
origin_eci = zeros(N, 3);
ellipsoid = wgs84Ellipsoid('kilometer');

for i = 1:N
    % Build LVLH rotation matrix (inertial to body)
    r = positions_m(i,:)';
    v = velocities_m(i,:)';

    z_body = r / norm(r);  % points nadir
    y_body = -cross(r, v); 
    y_body = y_body / norm(y_body);
    x_body = cross(y_body, z_body);

    R = [x_body y_body z_body]; % inertial → body
    R = R * R;
    R_all(:,:,i) = R; %store rotation matrix

    quats(i,:) = rotm2quat(R); %store quaternion
    
    %Convert NED → ECEF
    h = height(i) * 1000; %convert from km to m

    % First find the origin in ECEF coordinates:
    [origin_ecefX,origin_ecefY, origin_ecefZ] = ned2ecef(0, 0, 0, ...
                                     latitude(i), longitude(i), h, ...
                                     ellipsoid);
    origin_ecef(i,:) = [origin_ecefX, origin_ecefY, origin_ecefZ];

    %Next find "tip" of magnetic field vector in ECEF
    [ecefX, ecefY, ecefZ] = ned2ecef(XYZ(i,1), XYZ(i,2), XYZ(i,3), ...
                                     latitude(i), longitude(i), h, ...
                                     ellipsoid);
    mag_ecef_tip(i,:) = [ecefX, ecefY, ecefZ];

    %Now subtract "tip" - origin to get magnetic field vector in ECEF
    mag_ecef(i,:) = mag_ecef_tip(i,:) - origin_ecef(i,:);


    %Convert ECEF → ECI

    %Origin in ECEF coordinates
    origin_eci(i,:) = ecef2eci(time(i), origin_ecef(i,:));

    % "Tip" of magnetic field vector in ECI
    mag_eci(i,:) = ecef2eci(time(i), mag_ecef_tip(i,:));

    % "Tip" - origin to get magnetic field vector in ECI
    mag_eci(i,:) = mag_eci(i,:) - origin_eci(i,:);


    %Convert Inertial → Body using rotation matrices
    mag_body(i,:) = (R * mag_eci(i,:)')'; 
end

%combine datetimes with magnetic field values (divide by 1000 to get uTs)
magneticfieldbody = [num2cell(time(:)),num2cell(mag_body(:,1)/1000),num2cell(mag_body(:,2)/1000),num2cell(mag_body(:,3)/1000)];
magneticfieldbodyteslas = [num2cell(time(:)),num2cell(mag_body(:,1)/1000),num2cell(mag_body(:,2)/1000),num2cell(mag_body(:,3)/1e9)];

%save time + body frame magnetic field to .mat file
matObjBody = matfile('MagFieldBody.mat')
save('MagFieldBody.mat','magneticfieldbody')


%save time + body frame magnetic field to .mat file
matObjBodyTeslas = matfile('MagFieldBodyTeslas.mat')
save('MagFieldBodyTeslas.mat','magneticfieldbody')



%%
%{
quats = zeros(N, 4);  


%apply rotations to quaternions that'll maintain a "fixed" position
R_all = zeros(3,3,N); %to store rotation matrices
for i = 1:N
    r = positions_m(i,:)';
    v = velocities_m(i,:)';

    z_body = r / norm(r);  % points to Earth center
    y_body = -cross(r, v);
    y_body = y_body / norm(y_body);
    x_body = cross(y_body, z_body);

    %Rotation matrix: inertial to body
    R = [x_body y_body z_body];
    R = R*R;
    R_all(:,:,i) = R;

    %Convert to quaternion
    q = rotm2quat(R);
    quats(i,:) = q;
end


%have inertial to body. need NED to ECEF to ECI/inertial to body

%convert ned to spherical coordinates
%[az,elev,slantRange] = ned2aer(XYZ(:,1),XYZ(:,2),XYZ(:,3),'degrees')

%convert magnetic ned to ecef coordinates
mag_ecef = zeros(N, 3);
for i = 1:N
  [ecefX, ecefY, ecefZ] = ned2ecef(XYZ(i,1), XYZ(i,2), XYZ(i,3), ...
                                   latitude(i), longitude(i), height(i), ...
                                   wgs84Ellipsoid('kilometer'));
  mag_ecef(i,:) = [ecefX, ecefY, ecefZ];
end


%convert ned to ecef coordinates. note ecef2eci only takes in scalar
%datetime
%r_ecef = [ecefX,ecefY,ecefZ];
mag_eci = zeros(N, 3);
for i = 1:N
    mag_eci(i,:) = ecef2eci(time(i),mag_ecef(i,:));
end

%convert inertial to body using rotation matrix
mag_body = zeros(N, 3);
for i = 1:N
   mag_body(i,:) = (R_all(:,:,i) * mag_eci(i,:)')'; 
end
%}


% Just use a constant attitude for all timesteps
quat_fixed = [1 0 0 0]; 
quats = repmat(quat_fixed, N, 1);
%can we extract the attitude quaternions from the visualizer? 
%need to convert from NED frame to body frame. entering [0 0 0 1]
%as the attitude only applies to the body frame


%2nd attempt at introducing body-fixed rotation 
totaltime = time(length(time)) - time(1);
dt = seconds(totaltime / length(time)); % time step between samples (e.g., 0.01s if 100 Hz)

omega_mag = pi/2; %rad/s
axis_b = [10 10 10]; %input spin about body x y or z-axis
axis_b = axis_b / norm(axis_b); %normalize
omega_b = omega_mag * axis_b; %rotation axis

q = quats(1,:);
q_act = zeros(N,4);
new_quats(1,:) = q;


for k=1:N
    t = (k-1)*dt;
    theta = omega_mag * t;
    w = cos(theta/2);
    v = axis_b * sin(theta/2);
    q_act(k,:) = [w v];
end

q_pass = [ q_act(:,1), -q_act(:,2), -q_act(:,3), -q_act(:,4) ];


%% apply same rotation to magneticfieldbody
% Inputs:
% time : Nx1 datetime (1-second spacing)
% q_pass : Nx4 quaternions used in pointAt (inertial -> body), scalar-first [w x y z]
% B_body_nospin : Nx3 magnetic vectors computed for the no-spin case (effectively inertial)

% Apply the same attitude to the vectors:
B_body_spin = quatrotate(q_pass, mag_eci);   % returns Nx3
%we use mag_eci since q_pass transforms from inertial to body

% sanity check: magnitudes unchanged
assert(all(abs(vecnorm(B_body_spin,2,2) - vecnorm(mag_eci,2,2)) < 1e-10))

%combine datetimes with magnetic field rotation values (divide by 1000 to get uTs)
magneticfieldbodyspin = [num2cell(time(:)),num2cell(B_body_spin(:,1)/1000),num2cell(B_body_spin(:,2)/1000),num2cell(B_body_spin(:,3)/1000)];

%save time + body frame magnetic field to .mat file
matObjBodySpin = matfile('MagFieldBodySpin.mat');
save('MagFieldBodySpin.mat','magneticfieldbodyspin')


%% Secondary satellite to visualize magnetic field vector along sat3 orbit

% Get main satellite states
%[positions, velocities, times] = states(sat3, "inertial");  % Nx3 each

positions = positions_m;
velocities = velocities_m;
times = time;

% Scale magnetic field vector for visibility (optional)
scale = 2;  % meters
mag_scaled = (mag_eci ./ vecnorm(mag_eci,2,2)) * scale;

% Offset positions along magnetic field for visibility
marker_pos = positions + mag_scaled;


% Create secondary satellite asset
%magSat = satellite(sc, positionTT, "Name", "MagneticFieldVector");
%magSat.Color = [1 0 0];  % red
%magSat.Diameter = 0.2;   % meters

% Create ephemeris timetable
% ephemTT = timetable(times', marker_pos, velocities, ...
%     'VariableNames', {'Position','Velocity'});
% magSat.Ephemeris = ephemTT;

% Convert scaled magnetic field vector to pure quaternions for attitude
B_quat = [zeros(N,1), mag_scaled];  % Nx4 [w x y z]

% Create attitude timetable
attitudeTTmag = timetable(times', B_quat, 'VariableNames', {'Attitude'});

% Attach attitude to secondary satellite
%pointAt(magSat, attitudeTTmag, "CoordinateFrame","inertial", ...
 %       "Format","quaternion","ExtrapolationMethod","nadir");



% Create attitude timetable
attitudeTT = timetable(time', q_pass, 'VariableNames', {'Attitude'});

% Use the pointAt method to associate the logged attitude timetable with the satellite. 
% Parameter ExtrapolationMethod controls the pointing behavior outside of the timetable range.
pointAt(sat3, attitudeTT, ...
  "CoordinateFrame", "inertial", "Format", "quaternion", "ExtrapolationMethod", "nadir");
% "inertial" — interprets the provided attitude values as the rotation from the GCRF to the body frame.

%Attitude for magnetic field vector
%pointAt(mag, attitudeTT, ...
  %"CoordinateFrame", "inertial", "Format", "quaternion", "ExtrapolationMethod", "nadir");

% Use a gimbal sensor attached to the satellite
%sensor = conicalSensor(sat3, "HalfAngle", deg2rad(5));  % narrow cone

% % Convert Nx3 vector to Nx4 quaternion (w=0)
% B_quat = [zeros(N,1), B_body_spin];  % Nx4 [w x y z]
% B_TT = timetable(time', B_quat, 'VariableNames', {'MagQuat'});
% 
% % Now attach it
% pointAt(sat3, B_TT, "CoordinateFrame","inertial");


%open scenario viewer
viewer1 = satelliteScenarioViewer(sc);



%{
%% plot magnetic field for one orbit
orbitalelements = orbitalElements(sat3); %retrieve orbital elements from satellite 1

camtarget(viewer1, sat3);

%cut to positions 1 to 5930 for one full orbit
magneticfieldx = magneticfield(:,2);
magneticfieldy = magneticfield(:,3);
magneticfieldz = magneticfield(:,4);
magneticfieldx = cell2mat(magneticfieldx(1:5930));
magneticfieldy = cell2mat(magneticfieldy(1:5930));
magneticfieldz = cell2mat(magneticfieldz(1:5930));

magneticfieldOneOrbit=[magneticfieldx,magneticfieldy,magneticfieldz];
magneticfieldOneOrbitVecNorm = vecnorm(magneticfieldOneOrbit');

figure
hold on
plot(1:length(magneticfieldOneOrbitVecNorm), magneticfieldOneOrbitVecNorm)
title('Magnetic field magnitude over one orbit')
xlabel('Index')
ylabel('Mag Field (nT)')
hold off

%}


%{
%% Create satelllite visualizer using .TLE file

tleFile = "C:\Users\ronak\OneDrive\Documents\MAVERIC\Sgp4Prop\SampleCode\Python\DriverExamples\Sgp4Prop\input\sat000052191.tle";

%jan 1, 1950 + (TSINCE (mins)) Epoch time from TLE

baseDate = datetime(1950, 1, 1, 0, 0, 0, 'TimeZone', 'UTC');
daysSince1950 = 26477.76488163;
dt = baseDate + days(daysSince1950);

%startTime = datetime(year(dt),month(dt),day(dt),hour(dt),minute(dt),floor(second(dt)),1000*(second(dt)-floor(second(dt)))) + minutes(698.5704528)  
%stopTime = datetime(year(dt),month(dt),day(dt),hour(dt),minute(dt),floor(second(dt)),1000*(second(dt)-floor(second(dt)))) + minutes(986.5704528) 

sampleTime = 60; %seconds
%sc = satelliteScenario(startTime,stopTime,sampleTime);
sc = satelliteScenario();

sat1 = satellite(sc,tleFile,"Name","Sat1");
gsStart = groundStation(sc, 44.3200862,81.3990870,"Altitude", 515012.6876, Name="Starting Point");
gsEnd = groundStation(sc, 29.8097085,6.3974097,"Altitude", 511851.5139, Name="Ending Point");

orbitalelements = orbitalElements(sat1); %retrieve orbital elements from satellite 1

%note that epoch time for sat 2 is the StartTime of the scenario
%but epoch time for sat 1 is September 13???
%can i manually set orbital parameters for the satellite? seems to be read only

Mdeg = orbitalelements.MeanAnomaly;
period = orbitalelements.Period; %seconds
semiMajorAxis = (3.98600*10^14*(period/(2*pi))^2)^(1/3);     %meters   
eccentricity = orbitalelements.Eccentricity;
inclination = orbitalelements.Inclination;                                      % degrees
rightAscensionOfAscendingNode = orbitalelements.RightAscensionOfAscendingNode;   % degrees
argumentOfPeriapsis = orbitalelements.ArgumentOfPeriapsis;     % degrees

%calculate true anomaly
Mrad = deg2rad(Mdeg);
E=Mrad; %initial guess
for i=1:10
    E = E- (E - eccentricity*sin(E)- Mrad)/(1-eccentricity*cos(E));
end

trueAnomalyRad = 2*atan2(sqrt(1+eccentricity)*sin(E/2), sqrt(1-eccentricity)*cos(E/2));
trueAnomaly = rad2deg(mod(trueAnomalyRad, 2*pi))  % keep it [0,360)  % degrees

%define satellite 2 using orbital parameters from satellite 1
sat2 = satellite(sc,semiMajorAxis,eccentricity, ...
       inclination,rightAscensionOfAscendingNode, ...
       argumentOfPeriapsis,trueAnomaly, ...
       "OrbitPropagator","sgp4", ...
       "Name","Sat2");
%RAAN should be around 85 to match sat 1's orbit

orbitalelements2 = orbitalElements(sat2)


%{
%change satellite 2's epoch to equal satellite 1's
orbitalelements2.Epoch = orbitalelements.Epoch 
orbitalelements2.Eccentricity = 1.2
%}


%open scenario viewer
viewer1 = satelliteScenarioViewer(sc);

%Assign a 3-D model to the satellite and visualize its coordinate axes. 
%The red, green and blue arrows point along the x, y and z axes of the body frame of the satellite.
sat.Visual3DModel = "SmallSat.glb";

%TLE data starts at September 13, 2022, but Lat/Long.txt data starts at June
%30, 20202, then we simply need to remove all of the Lat/Long data before
%September 13. Problem is, the Lat/Long data only lasts a few hours on June
%30. so we need to rather match satellite 2's epoch to satelitte 1's.


%try changing periapsis and true anomaly
%or to try simply defining a satellite using the position output data
%}

Ixx = 30469883.00/1000000;
Iyy = 60061378.64/1000000;
Izz = 67624489.73/1000000;
Ixy = -107508.48/1000000;
Iyz = -163163.91/1000000;
Izx = 692935.27/1000000;

Ib = [Ixx Ixy Izx;
    Ixy Iyy Iyz;
    Izx Iyz Izz];

x0 = [1; 1; 1];
dx0 = [1; 1; 1];

% Create new mag_eci_T from nT to T

mag_eci_T = mag_eci/1e9;
time_seconds = (0:k-1) * dt;
mag_eci_T_time_data = timeseries(mag_eci_T, time_seconds);
