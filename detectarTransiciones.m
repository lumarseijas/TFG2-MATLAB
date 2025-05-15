function transiciones = detectarTransiciones(track)
% Detecta transiciones de tipo de movimiento por cambio de rumbo
% Usa diferencias angulares robustas (angdiff) y detecta duración real

tiempo = track.tiempo;
rumbo_deg = track.rumbo;
rumbo_rad = deg2rad(rumbo_deg);
tramos = track.tramos;
tiempos_tramos = track.tramos_tiempos;
n_tramos = size(tramos,1);

% Clasificación por tipo de movimiento
tipos = strings(n_tramos,1);
for j = 1:n_tramos
    acelL = tramos(j,1);
    acelT = tramos(j,2);
    Vvert = tramos(j,3);
    if acelL == 0 && acelT == 0 && Vvert == 0
        tipos(j) = "uniforme";
    elseif acelL ~= 0 && acelT == 0 && Vvert == 0
        tipos(j) = "acelerado";
    elseif acelL == 0 && acelT ~= 0 && Vvert == 0
        tipos(j) = "giro estándar";
    elseif acelL ~= 0 && acelT ~= 0 && Vvert == 0
        tipos(j) = "giro con aceleración";
    elseif Vvert ~= 0
        tipos(j) = "ascenso/descenso";
    else
        tipos(j) = "otro";
    end
end

transiciones = struct('tipo', {}, 't_inicio', {}, 't_fin', {}, 'duracion', {});

% Parámetros
tol = deg2rad(1);  % tolerancia angular (1 grado)
ventana = 60;      % segundos alrededor del cambio

for j = 1:n_tramos-1
    t_cambio = tiempos_tramos(j+1);
    idx_win = find(tiempo >= t_cambio - ventana & tiempo <= t_cambio + ventana);
    t_local = tiempo(idx_win);
    r_local = rumbo_rad(idx_win);

    % rumbo antes y después del cambio
    rumbo_ini = mean(r_local(1:5));
    rumbo_fin = mean(r_local(end-4:end));

    % Detectar inicio
    inicio_idx = find(abs(angdiff(r_local, rumbo_ini * ones(size(r_local)))) > tol, 1, 'first');
    
    % Detectar fin
    fin_idx = find(abs(angdiff(r_local, rumbo_fin * ones(size(r_local)))) < tol, 1, 'last');


    if isempty(inicio_idx) || isempty(fin_idx)
        t_ini = t_cambio;
        t_fin = t_cambio;
    else
        t_ini = t_local(inicio_idx);
        t_fin = t_local(fin_idx);
    end

    transiciones(j).tipo = tipos(j) + " → " + tipos(j+1);
    transiciones(j).t_inicio = t_ini;
    transiciones(j).t_fin = t_fin;
    transiciones(j).duracion = t_fin - t_ini;
end

end
