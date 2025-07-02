%Xsens_DOT_Matlab_PC
%Using Matlab BLEList(Matlab R2019b or later) to connect 5pcs Xsens DOT wearable sensors(IMU) 
%and real time plot Euler Angles.

%Change the MAC Address of Xsens DOT to the MAC addresses of your DOTs, and select the payload mode 
%that you want to get the data, please refer to Xsens DOT BLE Service Specifications for more details. 
%https://content.xsens.com/xsens-dot-ble-services-specifications
%
% --------------------------------------------------------------------------
clear
scan = blelist("Timeout", 5); %scan ble devices, set the timeout 10secs.
DOT_Measure_ServiceUUID = "15172000-4947-11E9-8646-D663BD873D93";
DOT_Control_CharacteristicUUID = "15172001-4947-11E9-8646-D663BD873D93";
DOT_ShortPayload_CharacteristicUUID = "15172004-4947-11E9-8646-D663BD873D93";
Heading_Reset_Control_CharacteristicUUID = "15172006-4947-11E9-8646-D663BD873D93";
Select_Orientation_Euler = [01, 01, 04 ]; %Select the data output type, 04 means Euler Angles(Roll, Pitch, Yaw).
Select_Orientation_Quaternion = [01, 01, 05 ]; %Select the data output type, 05 means Quaternions(w, x, y, z).
Heading_Reset_Buffer = [01,00]; % Reinicia la orientación inicial del sensor

%SENSOR húmero
P01 = ble("D422CD004D68"); %Change P01 to your own DOT's tag name, Change the MAC Address to your own sensor's MAC Address.
EnableP01 = characteristic(P01, DOT_Measure_ServiceUUID, DOT_Control_CharacteristicUUID);
write( EnableP01, Select_Orientation_Euler); % You can change to your desired payload mode.
P01Dataoutput = characteristic(P01, DOT_Measure_ServiceUUID, DOT_ShortPayload_CharacteristicUUID); % You can change to your desired payload mode.
subscribe(P01Dataoutput);
Heading_Reset_P01 = characteristic(P01, DOT_Measure_ServiceUUID, Heading_Reset_Control_CharacteristicUUID); % heading rest, make sure your sensors are physically heading aligned, for example, in the charger.
%write( Heading_Reset_P01, Heading_Reset_Buffer);

% SENSOR radio
P03 = ble("D422CD004D93");
EnableP03 = characteristic(P03, DOT_Measure_ServiceUUID, DOT_Control_CharacteristicUUID);
write( EnableP03, Select_Orientation_Euler);
P03Dataoutput = characteristic(P03, DOT_Measure_ServiceUUID,DOT_ShortPayload_CharacteristicUUID);
subscribe(P03Dataoutput);
Heading_Reset_P03 = characteristic(P03, DOT_Measure_ServiceUUID, Heading_Reset_Control_CharacteristicUUID);
%write( Heading_Reset_P03, Heading_Reset_Buffer);

fprintf('\n=== IMPORTANTE: Coloca al sujeto con pierna extendida (de pie o tumbado), sensores bien alineados en plano sagital ===\n');
fprintf('Tienes 5 segundos para alinear sensores antes del heading reset...\n');
pause(5);  % Tiempo para colocar bien los sensores físicamente

% Ejecutar reset de orientación (heading reset)
write( Heading_Reset_P01, Heading_Reset_Buffer);
write( Heading_Reset_P03, Heading_Reset_Buffer);

pause(3);
fprintf("\n=== CAPTURANDO OFFSET INICIAL DURANTE 2 SEGUNDOS... ===\n");
n_offset = 50;  % número de muestras para calcular offset (~2 segundos si sampling ~25Hz)
offsets_P01 = zeros(1, n_offset);
offsets_P03 = zeros(1, n_offset);

for i = 1:n_offset
    dataP01 = read(P01Dataoutput);
    dataP03 = read(P03Dataoutput);
    
    offsets_P01(i) = pitchConvert(dataP01);

    offsets_P03(i) = pitchConvert(dataP03);
    pause(0.04); % espera 40 ms para evitar saturación
end

pitch_P01_offset = mean(offsets_P01);
pitch_P03_offset = mean(offsets_P03);
pitch_diff_offset = pitch_P03_offset - pitch_P01_offset;

fprintf("Offset medio P01 (muslo): %.2f°\n", pitch_P01_offset);
fprintf("Offset medio P03 (tibia): %.2f°\n", pitch_P03_offset);
fprintf("Offset angular inicial (P03 - P01): %.2f°\n", pitch_diff_offset);


P01data = read(P01Dataoutput);
P03data = read(P03Dataoutput);


count = 200; %20 lecturas aprox 20-30 secs
P01Roll = [];
P03Roll = [];
% P05Roll = [];
%P04Roll = [];
% B64Roll = [];

P01Pitch = [];
P03Pitch = [];
% P05Pitch = [];
%P04Pitch = [];
% B64Pitch = [];

P01Yaw = [];
P03Yaw = [];
% P05Yaw = [];
%P04Yaw = [];
% B64Yaw = [];

fprintf('Empieza la medición en 5 segundos');
pause(5);
%medición del tiempo
tic

% FOR LOOP: adquisición de datos
for i=1:count
    tic
    
    P01data = read(P01Dataoutput);
    P03data = read(P03Dataoutput);
    timestamps(i) = toc;

    P01Roll(i)= rollConvert (P01data);
    P03Roll(i)= rollConvert (P03data);

    P01Pitch(i)= pitchConvert (P01data);
    P03Pitch(i)= pitchConvert (P03data);
    

    P01Yaw(i)= yawConvert (P01data);
    P03Yaw(i)= yawConvert (P03data);
end

time = toc;  % Mide el tiempo total
fprintf('\nTiempo total: %.2f segundos\n', time);
meanInterval = mean(timestamps);
fs_estimada = 1 / meanInterval;
fprintf("Frecuencia de muestreo estimada: %.2f Hz\n", fs_estimada);


% === CORRECCIÓN DE SALTOS EN LOS ÁNGULOS ===

P01Roll = unwrap(deg2rad(P01Roll)) * 360/pi;
P03Roll = unwrap(deg2rad(P03Roll)) * 360/pi;

P01Pitch = unwrap(deg2rad(P01Pitch)) * 360/pi-pitch_P01_offset;
P03Pitch = unwrap(deg2rad(P03Pitch)) * 360/pi-pitch_P03_offset;
angle_Pitch = -(P03Pitch - P01Pitch);


P01Yaw = unwrap(deg2rad(P01Yaw)) * 360/pi;
P03Yaw = unwrap(deg2rad(P03Yaw)) * 360/pi;

%SUAVIZAR señales
order=3;
window=23;

yaw_p01_smooth = sgolayfilt(P01Yaw, order, window);
yaw_p03_smooth = sgolayfilt(P03Yaw, order, window);
pitch_p01_smooth = sgolayfilt(P01Pitch, order, window);
pitch_p03_smooth = sgolayfilt(P03Pitch, order, window);
roll_p01_smooth = sgolayfilt(P01Roll, order, window);
roll_p03_smooth = sgolayfilt(P03Roll, order, window);

diffPitch_smooth = sgolayfilt(angle_Pitch, order, window);



% === PLOT CORREGIDO ===
tiledlayout(4,1)

ax1 = nexttile;
plot(ax1,roll_p01_smooth,'red')
hold on
plot(ax1,roll_p03_smooth,'green')
title('Roll: Red:P01, Green:P03')
hold off

ax2 = nexttile;
plot(ax2,pitch_p01_smooth,'red')
hold on
plot(ax2,pitch_p03_smooth,'green')
title('Pitch: Red:P01, Green:P03')
hold off

ax3 = nexttile;
plot(ax3,yaw_p01_smooth,'red')
hold on
plot(ax3,yaw_p03_smooth,'green')
title('Yaw: Red:P01, Green:P03')
hold off

ax4 = nexttile;
plot(ax4,-diffPitch_smooth*2,'blue')
hold on
title('Flexion/Extension Knee')
hold off

% === EXPORTACIÓN CSV CORREGIDO ===
IMU_table = table( (1:count)', roll_p01_smooth', pitch_p01_smooth', yaw_p01_smooth', roll_p03_smooth', pitch_p03_smooth', yaw_p03_smooth', diffPitch_smooth', ...
    'VariableNames', {'Sample', 'P01_Roll', 'P01_Pitch', 'P01_Yaw', 'P03_Roll', 'P03_Pitch', 'P03_Yaw', 'AnglePitch'});

writetable(IMU_table, 'IMU_data_RIGHTlegUNWRAP.csv');
fprintf('Datos guardados en IMU_data_RIGHTleg.csv\n');

fprintf('done');
clear All

function  timeData = timeStampConvert(data)

    t = data (1:4);
    t = uint8 (t);
    timeData = typecast ( t, 'uint32' );
end

function  rollData = rollConvert(data)

    r = data (5:8);
    r = uint8 (r(:));
    rollData = typecast ( r, 'single' );
end

function  pitchData = pitchConvert(data)
    p = data (9:12);
    p = uint8 (p(:));
    pitchData = typecast ( p, 'single' );
end

function  yawData = yawConvert(data)
    y = data (13:16);
    y = uint8 (y(:));
    yawData = typecast ( y, 'single' );
end


