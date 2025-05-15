%% MONTECARLO CON KALMAN SIMPLE
addpath(genpath('bases'));

% Parámetros
N = 200;              % número de iteraciones Monte Carlo
sigma_a = 0.5;        % valor de sigma_a a evaluar

% Trayectoria base (misma en todos los ensayos)
[track, radar, projection] = generarTrayectoria();
track_base = track(1);
tramos = track_base.tramos;
n_tramos = size(tramos, 1);

% Clasificación de tipos de movimiento
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

% Inicializar estructuras para acumular errores por tramo
ECM_long = zeros(N, n_tramos);
ECM_trans = zeros(N, n_tramos);
ECM_vel = zeros(N, n_tramos);
ECM_rumbo = zeros(N, n_tramos);

% Bucle Monte Carlo
for i = 1:N
    targetIdeal = ideal_measurement(track, radar, projection);
    targetReal = real_measurement(targetIdeal, radar, true, true, false, 0, 0, projection);
    [xhat, ~] = kalman_cv(targetReal(1), radar(1), sigma_a);
    tiempo_est = targetReal(1).measure(targetReal(1).measure(:,1)==radar(1).id, 2);
    err = calcularErrores(xhat, tiempo_est, track_base);

    for j = 1:n_tramos
        t_start = track_base.tramos_tiempos(j);
        t_end = track_base.tramos_tiempos(j+1);
        idx = find(tiempo_est >= t_start & tiempo_est < t_end);

        if ~isempty(idx)
            ECM_long(i,j) = sqrt(mean(err.longitudinal(idx).^2));
            ECM_trans(i,j) = sqrt(mean(err.transversal(idx).^2));
            ECM_vel(i,j) = sqrt(mean(err.velocidad(idx).^2));
            ECM_rumbo(i,j) = sqrt(mean(err.rumbo(idx).^2));
        end
    end
end

% Resultados finales por tramo
fprintf('Resultados Monte Carlo (%d iteraciones) para sigma_a = %.2f\n', N, sigma_a);
for j = 1:n_tramos
    dur = diff(track_base.tramos_tiempos([j j+1]));
    fprintf('\nTramo %d (%.0f s): Movimiento %s\n', j, dur, tipos(j));
    fprintf('  ECM Longitudinal: %.2f m\n', mean(ECM_long(:,j)));
    fprintf('  ECM Transversal:  %.2f m\n', mean(ECM_trans(:,j)));
    fprintf('  ECM Velocidad:    %.2f m/s\n', mean(ECM_vel(:,j)));
    fprintf('  ECM Rumbo:        %.2f º\n', mean(ECM_rumbo(:,j)));
end

% Transiciones entre tramos
detallesTrans = detectarTransiciones(track_base);
fprintf('\nTransiciones:\n');
for j = 1:length(detallesTrans)
    all_long = [];
    all_trans = [];
    all_vel = [];
    all_rumbo = [];

    for i = 1:N
        targetIdeal = ideal_measurement(track, radar, projection);
        targetReal = real_measurement(targetIdeal, radar, true, true, false, 0, 0, projection);
        [xhat, ~] = kalman_cv(targetReal(1), radar(1), sigma_a);
        tiempo_est = targetReal(1).measure(targetReal(1).measure(:,1)==radar(1).id, 2);
        err = calcularErrores(xhat, tiempo_est, track_base);

        idx_trans = find(tiempo_est >= detallesTrans(j).t_inicio & tiempo_est <= detallesTrans(j).t_fin);

        all_long = [all_long; err.longitudinal(idx_trans)];
        all_trans = [all_trans; err.transversal(idx_trans)];
        all_vel = [all_vel; err.velocidad(idx_trans)];
        all_rumbo = [all_rumbo; err.rumbo(idx_trans)];
    end

    fprintf('\nTransición %d: %s\n', j, detallesTrans(j).tipo);
    fprintf('  Duración: %.1f s\n', detallesTrans(j).duracion);
    fprintf('  ECM Longitudinal: %.2f m\n', rms(all_long));
    fprintf('  ECM Transversal:  %.2f m\n', rms(all_trans));
    fprintf('  ECM Velocidad:    %.2f m/s\n', rms(all_vel));
    fprintf('  ECM Rumbo:        %.2f º\n', rms(all_rumbo));
end
