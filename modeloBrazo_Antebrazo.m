clear;
clc;
close all;

% Xsens DOT UUID Constants
DOT_Measure_ServiceUUID = "15172000-4947-11E9-8646-D663BD873D93";
DOT_Control_CharacteristicUUID = "15172001-4947-11E9-8646-D663BD873D93";
DOT_ShortPayload_CharacteristicUUID = "15172004-4947-11E9-8646-D663BD873D93";
Heading_Reset_Control_CharacteristicUUID = "15172006-4947-11E9-8646-D663BD873D93";
Select_Orientation_Quaternion = [01, 01, 05];
Heading_Reset_Buffer = [01, 00];

% Function to connect one sensor (using BLE ones)
connectSensor = @(mac) localConnectSensor(mac, DOT_Measure_ServiceUUID, ...
    DOT_Control_CharacteristicUUID, ...
    DOT_ShortPayload_CharacteristicUUID, ...
    Heading_Reset_Control_CharacteristicUUID);

% Connect to the sensors with each specific MAC address
P01 = connectSensor("D422CD004D68"); % Hombro
P03 = connectSensor("D422CD004D93"); % Codo

% Configure sensors and reset heading
write(P01.control, Select_Orientation_Quaternion);
write(P03.control, Select_Orientation_Quaternion);
write(P01.heading, Heading_Reset_Buffer);
write(P03.heading, Heading_Reset_Buffer);

%automatic notification data sent to the device when something new exits
subscribe(P01.data);
subscribe(P03.data);

% Biomechanical parameters
L_brazo = 0.30;       % Upper arm length (meters)
L_antebrazo = 0.25;   % Forearm length (meters)

% 3D plot 
figure;
h = plot3([0 0 0], [0 0 0], [0 0 0], 'o-', 'LineWidth', 3); %  line of the limb 
hold on;

% Coordinate axes
quiver3(0,0,0,0.1,0,0,'r','LineWidth',2); % X-red
quiver3(0,0,0,0,0.1,0,'g','LineWidth',2); % Y-green
quiver3(0,0,0,0,0,0.1,'b','LineWidth',2); % Z-blue

% Static torso
P_torso = [0; 0; 0.1];
plot3([0 P_torso(1)], [0 P_torso(2)], [0 P_torso(3)], 'k-', 'LineWidth', 2);

axis equal;
grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
xlim([-0.6 0.6]); ylim([-0.6 0.6]); zlim([-0.6 0.6]);
title('Real-time 3D Arm Reconstruction');
view ([0 0 1]);
% Real-time acquisition and animation loop
count = 600; %time streaming

for i = 1:count
    % Read data from sensors-quaternion packet
    data01 = read(P01.data);
    data03 = read(P03.data);
    
    % Extract quaternions
    q01 = quatConvert(data01); % shoulder segment rotation
    q03 = quatConvert(data03); % forearm segment rotation
    
    % Conversion to  rotation matrices
    R01 = quat2rotm(q01); % shoulder
    R03 = quat2rotm(q03); % elbow
    
    % Direct kinematics (position calculation)
    P_hombro = [0; 0; 0];                           %shoulder is fixed
    P_codo = P_hombro + R01 * [0; 0; -L_brazo];     %Elbow position
    P_muneca = P_codo + R03 * [0; 0; -L_antebrazo]; %Wrist position
    
    % Update the plot-new positions
    set(h, 'XData', [P_hombro(1) P_codo(1) P_muneca(1)], ...
           'YData', [P_hombro(2) P_codo(2) P_muneca(2)], ...
           'ZData', [P_hombro(3) P_codo(3) P_muneca(3)]);
    drawnow; %refresh figure
end

fprintf("¡Animación terminada!\n");

% EXTRACTION QUATERNION
function q = quatConvert(data)
    q_bytes = data(5:20);                   % bytes 5-20: w, x, y, z (4 floats)
    q = typecast(uint8(q_bytes), 'single'); % convert raw bytes to float->return [w x y z]
end

% CONNECT AND CONFIGURATION A BLE SENSOR
function sensor = localConnectSensor(mac, svcUUID, ctrlUUID, dataUUID, headingUUID)
    d = ble(mac);
    sensor.device = d;
    sensor.control = characteristic(d, svcUUID, ctrlUUID);
    sensor.data = characteristic(d, svcUUID, dataUUID);
    sensor.heading = characteristic(d, svcUUID, headingUUID);
end
