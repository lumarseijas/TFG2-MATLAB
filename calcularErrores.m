function errores = calcularErrores(xhat, tiempo_est, track)
% Calcula errores longitudinales, transversales, velocidad y rumbo
% xhat: estado estimado [x;y;vx;vy] (4xN)
% tiempo_est: tiempos de las estimaciones (Nx1)
% track: trayectoria ideal (estructura de generarTrayectoria)

% Interpolamos trayectoria real a los tiempos de estimación
x_real = interp1(track.tiempo, track.posStereo(:,1), tiempo_est);
y_real = interp1(track.tiempo, track.posStereo(:,2), tiempo_est);
v_real = interp1(track.tiempo, track.velocidad, tiempo_est);
r_real = interp1(track.tiempo, track.rumbo, tiempo_est);

% Componentes reales de velocidad
vx_real = v_real .* sin(deg2rad(r_real));
vy_real = v_real .* cos(deg2rad(r_real));

% Errores de posición
dx = xhat(1,:)' - x_real;
dy = xhat(2,:)' - y_real;

% Vector tangente unitario (rumbo)
v_unit = [vx_real'; vy_real'];
v_unit = v_unit ./ vecnorm(v_unit);

% Vector error de posición
e_pos = [dx'; dy'];

% Longitudinal: proyección sobre rumbo
err_long = dot(e_pos, v_unit);

% Transversal: proyección sobre vector perpendicular
v_perp = [-v_unit(2,:); v_unit(1,:)];
err_trans = dot(e_pos, v_perp);

% Error de velocidad escalar
vel_est = sqrt(xhat(3,:).^2 + xhat(4,:).^2);
vel_real = sqrt(vx_real.^2 + vy_real.^2);
err_vel = vel_est' - vel_real;

% Error de rumbo
rumbo_est = atan2d(xhat(3,:), xhat(4,:));
rumbo_est = mod(rumbo_est, 360);
err_rumbo = angdiff(deg2rad(r_real), deg2rad(rumbo_est'));
err_rumbo = rad2deg(err_rumbo);

% Guardamos
errores.longitudinal = err_long';
errores.transversal = err_trans';
errores.velocidad = err_vel;
errores.rumbo = err_rumbo;
end