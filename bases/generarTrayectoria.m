function [track, radar, projection] = generarTrayectoria()
% FUNCION generarTrayectoria
% Crea un radar, una trayectoria y la proyección estereográfica

%% Datos de la Tierra y proyección estereográfica
geoide = referenceEllipsoid('wgs84', 'meter');

latP = 40.5; % Latitud del centro de proyección
longP = 0.5; % Longitud del centro de proyección

projection = defaultm('stereo');
projection.origin = [latP longP 0];
projection.geoid = geoide;
projection = defaultm(projection);

%% Definición del radar
[radar(1).posGeod(1), radar(1).posGeod(2)] = stereo(...
         projection,0,0,'surface','inverse');
radar(1).posGeod(3) = 0; % Altura 0 m
radar(1).id = 1;
radar(1).range = 400e3;
radar(1).resDist = 70;     % Error en distancia (m)
radar(1).resAzim = 0.08;   % Error en azimut (grados)
radar(1).Tr = 4;           % Tiempo de barrido radar (s)
radar(1).Tini = rand(1)*radar(1).Tr; % Tiempo inicial aleatorio
radar(1).VelMS = 0;
radar(1).StVel = 0;

%% Trayectoria inicial
Ts = 0.001; % Tiempo de muestreo muy fino (1 ms)

[yini, xini] = stereo(projection, -26.305e3, 150e3+26.305e3, 'surface', 'inverse');
zini = 10e3;    % 10 km de altura
vini = 155;     % velocidad inicial
rini = 135;     % rumbo inicial

tramos = [0 0 0 240; 0 4 0 98; 0 0 0 262]; % aceleraciones y tiempos
% mio para tener los tramos
track(1).tramos = tramos;
track(1).tramos_tiempos = cumsum([0; tramos(:,4)]);


[track(1).posGeod, track(1).tiempo, track(1).velocidad, track(1).rumbo, track(1).velascen] = ...
    trayectMia(tramos, [yini xini zini], vini, rini, 0, Ts, geoide);

%% Proyección a estereográficas
[radar(1).posStereo(1), radar(1).posStereo(2)] = stereo(...
    projection, radar(1).posGeod(1), radar(1).posGeod(2),'surface','forward');
radar(1).posStereo(3) = radar(1).posGeod(3);

[track(1).posStereo(:,1), track(1).posStereo(:,2)] = stereo(...
    projection, track(1).posGeod(:,1), track(1).posGeod(:,2), 'surface', 'forward');
track(1).posStereo(:,3) = track(1).posGeod(:,3);

end