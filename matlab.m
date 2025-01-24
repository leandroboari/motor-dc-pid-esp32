clc; clear all;

K = 500 / 4.6; % rpm / V
tau = 0.2;

num = K;               
den = [tau, 1];      
tf = tf(num, den)


Kp = 2.0;
Ki = 7.0;
Kd = 0.2;
pidc = pid(Kp, Ki, Kd);

pidTuner(tf, pidc);