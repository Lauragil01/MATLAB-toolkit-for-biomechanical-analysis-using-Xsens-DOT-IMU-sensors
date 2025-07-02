clear;
clc;

disp("Escaneando dispositivo BLE");
scan = blelist("Timeout", 5);
disp(scan);

mac="D422CD004D68";  %PRUEBA CON SENSOR P03-antebrazo
disp("Intentando conectar sensor...");

try
    device = ble(mac);
    
    if ~isempty(device)
        disp("Sensor conectado con éxito");
        conectado = 1;
    else
        disp("No se pudo conectar al sensor");
        conectado = 0;
    end

catch ME
    disp("Error al conectar con el sensor:");
    disp(ME.message);
    conectado = 0;
end

% Devolver resultado
disp("Resultado de la conexión:");
disp(conectado);