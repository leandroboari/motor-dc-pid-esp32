clc; clear all;

% Parâmetros do sistema
K = 384 / 4095; % Constante do sistema: rpm / V
tau = 0.1;        % Constante de tempo: segundos

num = K;        % Numerador da função de transferência
den = [tau, 1]; % Denominador da função de transferência

% Criando a função de transferência
sys = tf(num, den);
step(sys); % Visualizar a resposta do sistema
hold on;

% Parâmetros do PID
Kp = 2.0;  % Ganho proporcional
Ki = 7.0;  % Ganho integral
Kd = 0.2;  % Ganho derivativo

% Parâmetros de simulação
dt = 0.05;           % Intervalo de amostragem (50 ms)
t_sim = 5;           % Tempo total de simulação (segundos)
N = t_sim / dt;      % Número de iterações
time = (0:N-1) * dt; % Vetor de tempo

% Setpoint (referência de velocidade)
setpoint = 330; % Velocidade em rpm

% Inicializando variáveis
error = zeros(1, N);       % Erro
integral = 0;              % Integral acumulada
output = zeros(1, N);      % Sinal de controle (saída PID)
speed = zeros(1, N);       % Velocidade simulada do motor
previousError = 0;         % Erro anterior
penultimateError = 0;      % Erro penúltimo

% Simulação do controle PID
for k = 2:N
    % Calcula o erro
    error(k) = setpoint - speed(k-1);

    % Controle PID
    P = Kp * error(k);
    integral = integral + error(k) * dt;
    I = Ki * integral;
    D = Kd * (error(k) - 2 * previousError + penultimateError) / dt;

    output(k) = P + I + D;

    % Atualiza os erros anteriores
    penultimateError = previousError;
    previousError = error(k);

    % Atualiza a velocidade simulada (saída do sistema)
    speed(k) = lsim(sys, output(1:k), time(1:k), speed(1)); % Resposta do motor
end

% Plotando os resultados
figure;
subplot(3, 1, 1);
plot(time, setpoint * ones(1, N), 'r--', 'LineWidth', 1.5); hold on;
plot(time, speed, 'b', 'LineWidth', 1.5);
xlabel('Tempo (s)');
ylabel('Velocidade (RPM)');
title('Resposta do Controle PID');
legend('Setpoint', 'Velocidade');

subplot(3, 1, 2);
plot(time, error, 'k', 'LineWidth', 1.5);
xlabel('Tempo (s)');
ylabel('Erro (RPM)');
title('Erro ao Longo do Tempo');

subplot(3, 1, 3);
plot(time, output, 'g', 'LineWidth', 1.5);
xlabel('Tempo (s)');
ylabel('Saída do PID');
title('Sinal de Controle (Output)');
