clear; clc;
addpath(genpath('bases'));

% Generar una única trayectoria
[track, ~, ~] = generarTrayectoria();
track_test = track(1);

% Detectar transiciones
transiciones = detectarTransiciones(track_test);

% Mostrar resultados
for i = 1:length(transiciones)
    fprintf("Transición %d: %s\n", i, transiciones(i).tipo);
    fprintf("  Inicio: %.2f s | Fin: %.2f s | Duración: %.2f s\n\n", ...
        transiciones(i).t_inicio, transiciones(i).t_fin, transiciones(i).duracion);
end
