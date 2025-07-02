clear; clc;

% UUIDs and BLE characteristics for sensors
DOT_Measure_ServiceUUID = "15172000-4947-11E9-8646-D663BD873D93";
DOT_Control_CharacteristicUUID = "15172001-4947-11E9-8646-D663BD873D93";
DOT_ShortPayload_CharacteristicUUID = "15172004-4947-11E9-8646-D663BD873D93";
Heading_Reset_Control_CharacteristicUUID = "15172006-4947-11E9-8646-D663BD873D93";

%Selection orientation of quaternions
Select_Orientation_Quaternion = [01, 01, 05];
Heading_Reset_Buffer = [01, 00];

% Connect to sensors
P01 = ble("D422CD004D68");%UPPER ARM
P03 = ble("D422CD004D93");%FOREARM

%Characteristics to control de sensors
EnableP01 = characteristic(P01, DOT_Measure_ServiceUUID, DOT_Control_CharacteristicUUID);
EnableP03 = characteristic(P03, DOT_Measure_ServiceUUID, DOT_Control_CharacteristicUUID);

%Send quaternion orientation command to both sensor
write(EnableP01, Select_Orientation_Quaternion);
write(EnableP03, Select_Orientation_Quaternion);

%for reading sensor data
P01Data = characteristic(P01, DOT_Measure_ServiceUUID, DOT_ShortPayload_CharacteristicUUID);
P03Data = characteristic(P03, DOT_Measure_ServiceUUID, DOT_ShortPayload_CharacteristicUUID);

%for receiving data from sensor
subscribe(P01Data);
subscribe(P03Data);

Heading_Reset_P01 = characteristic(P01, DOT_Measure_ServiceUUID, Heading_Reset_Control_CharacteristicUUID);
Heading_Reset_P03 = characteristic(P03, DOT_Measure_ServiceUUID, Heading_Reset_Control_CharacteristicUUID);

% Reset the orientation -->sensors
write(Heading_Reset_P01, Heading_Reset_Buffer);
write(Heading_Reset_P03, Heading_Reset_Buffer);

pause(1); % Waiting 1sec before the reset
fprintf("Reading real-time yaw...Keep your arm straight.\n");

% Loop for reading and display the differences
for i = 1:400
    q1 = quatConvert(read(P01Data)); %quaternion for sensor1 P01 upper arm
    q3 = quatConvert(read(P03Data)); %quaternion for sensor2 P03 forearm
    
    euler1 = quat2eul(q1, 'ZYX'); %euler angle conversion
    euler3 = quat2eul(q3, 'ZYX');
    
    yaw_diff = rad2deg(wrapToPi(euler3(1) - euler1(1)));  % yaw difference and conversion to degrees
    pitch_diff= rad2deg(wrapToPi(euler3(2) - euler1(2)));

    fprintf("Yaw P01: %6.2f°, Yaw P03: %6.2f°, Diferencia Yaw: %6.2f°, Pitch P01: %6.2f°, Pitch P03: %6.2f°,Diferencia Pitch:%6.2f°\n", ...
            rad2deg(euler1(1)), rad2deg(euler3(1)), yaw_diff, rad2deg(euler1(2)),rad2deg(euler3(2)), pitch_diff);

    angles(i, :) = [euler1(1), euler3(1), yaw_diff, euler1(2), euler3(2), pitch_diff];

    pause(0.2);%wait for next reading in the loop
end

fprintf("Fin de la comprobación.\n");

% Keep data en CSV
headers = {'Yaw_P01', 'Yaw_P03', 'Yaw_Diff', 'Pitch_P01', 'Pitch_P03', 'Pitch_Diff'};
T = array2table(angles, 'VariableNames', headers);
writetable(T, 'Angles_Orientation.csv');

% FUNCTION CONVERSION OF QUATERNIONS
function q = quatConvert(data)
    q_bytes = data(5:20);                     % [w x y z]
    q = double(typecast(uint8(q_bytes), 'single')); % in order [w x y z]
    q = [q(2), q(3), q(4), q(1)];             % conversion to  [x y z w] for Matlab
end
clear All