% Application of P, PI, and PID to spring-mass-damper model.
% Model Equation: mx'' + cx' + kx = F
% Transfer Function (Laplace Transform of model): X(s)/F(s) = 1/(ms^2 + cs + k) 
% m = mass of block, c = damping constant, k = spring constant
% F is the applied force
% x is the resulting displacement of the block

%Main model parameters are:
m = 1;  % 1 kg
c = 10; % 10 N-s/m
k = 20; % 20 N/m



% PID Control Values:
Kp = 350; 
Kd = 50;
Ki = 300;

% Time-step 0.01 s
t  = 0:0.01:5;
hold on


% Open-Loop Response
s = tf('s');
P = 1/(m*s^2 + c*s + k) % Transfer function
step(P)


% Only Proportional Control (P-Control)
C1  = pid(Kp,0,0)
T1   = feedback(C1*P,1) % Transfer function with P-only controller
step(T1,t)

% Only Integral Control (I-Control)
C2  = pid(0,Ki,0)
T2   = feedback(C2*P,1) % Transfer function with P-only controller
step(T2,t)

% Only Derivative Control (D-Control)
C3  = pid(0,0,Kd)
T3   = feedback(C3*P,1) % Transfer function with P-only controller
step(T3,t)

% Proportional-Derivative Control (PD)
C4  = pid(Kp,0,Kd)
T4  = feedback(C4*P,1)
step(T4,t)

% Proportional-Integral Control (PI)
C5  = pid(Kp,Ki,0)
T5  = feedback(C5*P,1)
step(T5,t)


% Proportional-Integral-Derivative Control (PID)
C6  = pid(Kp,Ki,Kd)
T6  = feedback(C6*P,1)
step(T6,t)

legend('P only Control','I only Control','D only Control','PD Control','PI Control','PID Control')