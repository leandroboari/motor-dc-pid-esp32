% Parâmetros do sistema
K = 60;         % Ganho estático do sistema (RPM/V)
tau = 0.5;      % Constante de tempo (s)

% Função de transferência do sistema
numerator = K;                % Numerador: apenas o ganho
denominator = [tau, 1];       % Denominador: tau*s + 1
motor_tf = tf(numerator, denominator);

% Exibir a função de transferência
disp('Função de transferência do sistema:');
disp(motor_tf);

% Simulação da resposta ao degrau
step_amplitude = 5;           % Amplitude do degrau de entrada (V)
t = 0:0.01:5;                 % Vetor de tempo (0 a 5 segundos)

% Resposta ao degrau
figure;
step(step_amplitude * motor_tf, t);
title('Resposta do sistema a um degrau de entrada');
xlabel('Tempo (s)');
ylabel('Velocidade (RPM)');
grid on;

% Informações da resposta ao degrau
step_info = stepinfo(step_amplitude * motor_tf);
disp('Informações da resposta ao degrau:');
disp(step_info);

% Ajustar a constante de tempo (opcional)
disp(['Constante de tempo usada: ', num2str(tau), ' s']);
