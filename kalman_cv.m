function [xhat, P, errores] = kalman_cv(target, radar, sigma_a)
% Filtro de Kalman con modelo de velocidad constante (CV)
% target: estructura con las medidas reales
% radar: estructura con el radar (para acceder a la matriz mcov)
% sigma_a: desviación típica del ruido de aceleración (modelo)

% Extraemos medidas
medidas = target.measure;
t = medidas(:,2);
x = medidas(:,13);
y = medidas(:,14);

% Buscamos solo las medidas de este radar (id=1)
idRadar = radar.id;
ind = find(medidas(:,1) == idRadar);
t = t(ind);
x = x(ind);
y = y(ind);

N = length(t);

% Inicialización del estado con las dos primeras medidas
T = t(2) - t(1); % suponemos muestreo constante
vx0 = (x(2)-x(1)) / T;
vy0 = (y(2)-y(1)) / T;

xhat = zeros(4,N); % estado estimado
xhat(:,1) = [x(1); y(1); vx0; vy0];

% Matriz de transición (modelo CV)
F = [1 0 T 0;
     0 1 0 T;
     0 0 1 0;
     0 0 0 1];

% Matriz H (medimos solo posición)
H = [1 0 0 0;
     0 1 0 0];

% Matriz Q (ruido de proceso)
q = sigma_a^2;
Q = q * [T^4/4 0     T^3/2 0;
         0     T^4/4 0     T^3/2;
         T^3/2 0     T^2   0;
         0     T^3/2 0     T^2];

% Matriz de covarianza inicial basada en resolución del radar
pos_var = radar.resDist^2;         % Varianza en x, y por resolución del radar
vel_var = 2 * pos_var / T^2;       % Estimación conservadora de varianza en velocidad

P = diag([pos_var, pos_var, vel_var, vel_var]);


% Almacenamos errores si hay verdad (opcional)
errores = struct('pos', [], 'vel', []);

for k = 2:N
    % Predicción
    xhat_pred = F * xhat(:,k-1);
    P_pred = F * P * F' + Q;

    % Matriz de covarianza de medida en este instante
    R = target.mcov(:,:,ind(k));

    % Medida actual
    z = [x(k); y(k)];

    % Innovación
    ytilde = z - H * xhat_pred;

    % Matriz de innovación
    S = H * P_pred * H' + R;

    % Ganancia de Kalman
    K = P_pred * H' / S;

    % Actualización
    xhat(:,k) = xhat_pred + K * ytilde;
    P = (eye(4) - K * H) * P_pred;
end

end
