clear all
close all
clc

%% Declaracion de los parametros fisicos

R = 0.75;
d=0.3;
r=0.15;
m_c=30;
m_w = 1;
I_c = 15.625;
I_w=0.005;
I_m = 0.0025;
R_a = eye(2,2) * 2.5;
L_a = eye(2,2) * 0.048;
K_e = eye(2,2) * 0.02;
N= eye(2,2) * 62.55;
K_t = eye(2,2) * 0.2613;
d_m1 = 0.5;
d_m2 = d_m1;

%% declaracion de las matrices y vectores necesarios



% estado inicial x1
x1 = [0;1;2]; % [x;y;theta]
x2 = [0.5*pi;0.5*pi]; %velocidad angular de las ruedas
x3 = [0,0];  % corrientes


% declaramos los limites de tiempo
t0 = 0;
tf = 10;
%declaracion de las condiciones iniciales
x10 = [1 2 pi/2]';
x20 = [1 1]';
x30 = [0 0]';

% declaracion de la estimacion inicial
x1e =[1 2]';
x2e =[2 1]';
x3e =[0.5 0]';

% incializamos los vectores
% valores reales
x1_hist = [];
x2_hist = [];
x3_hist = [];

% valores estimados
xe1_hist=[];
xe2_hist=[];
xe3_hist=[];


dt = 0.01; %definimos el periodo de tiempo

x1 = x10;
x2 = x20;
x3 = x30;

m=m_c + 2*m_w;
I = m_c*d^2+2*m_w*R^2+I_c+2*I_m;
m_1 = 0.25*R^-2*r^2*(m*R^2+I)+I_w;
m_2 = 0.25*R^-2*r^2*(m*R^2-I);
M = [m_1 m_2;m_2 m_1];
D = eye(2,2) * d_m1;
T_d = [0.001 0.001]';



%--------------------- variables de kalman
p1 = 1e2;
p2 = 1e2;
p3 = 1e2;
q1 = 5e-1;
q2 = 5e-1;
q3 = 5e-1;
R1 = 1e2;
R2 = 1e2;
R3 = 1e2;

g1=1;
g3=1;
g2=1;

W1 = [0 0]';
W2 = [1 3]';
W3 = [0 0]';
P1 = p1* eye(2,2);
P2 = p2* eye(2,2);
P3 = p3* eye(2,2);

Q1 = q1* eye(2,2);
Q2 = q2* eye(2,2);
Q3 = q3* eye(2,2);


A = eye(3); % Matriz de transición de estado
C_k = eye(3,3); % Matriz de observación

Q = eye(3) * 0.01; % Covarianza del ruido de proceso
R_k = eye(3) * 0.1; % Covarianza del ruido de medición

P1e = eye(3); % Estimación inicial de la covarianza

xe1_hist = [];
P1e_hist = [];

%comenzamos con la integracion de las variables

Z1 = [0 0 0]';

for t = t0:dt:tf
    Z1 = [tanh(x1(1)), tanh(x1(1))]';
    Z2 = [tanh(x1(2)), tanh(x1(3))]';
    Z3 = [tanh(x1(3)), tanh(x1(3))]';

    x1_hist = [x1_hist ; [t x1']];
    

    x2_hist = [x2_hist ; [t x2']];
    % xe2_hist=[x1e_hist [t x2e]];

    x3_hist = [x3_hist ; [t x3']];
    % xe3_hist=[x3e_hist [t x3e]];

    %integracion
    dx_dt1 = 0.5*r*[cos(x1(3)), cos(x1(3)); 
        sin(x1(3)), sin(x1(3)); 
        R^-1, -R^-1] * x2;
    x1 = x1 + dx_dt1*dt;
   

    C = 0.5*R^-1*r^2*m_c*d*[0, dx_dt1(3);
                            -dx_dt1(3), 0];

    dx_dt2 = M^-1*(-C*x2-D*x2-T_d+N*K_t*x3);
    x2 = x2 + dx_dt2*dt;

    u=  [4 3]';

    dx_dt3 = L_a^(-1)*(u-R_a*x3-N*K_e*x2);
    x3 = x3 + dx_dt3*dt;


    y_1 = C_k * x1; % Medición real
    % --------------------- Filtro de Kalman
    H1 = Z1;
    K1 = P1*H1*inv(R1+H1'*P1*H1);
    W1 = W1+g1*K1*(x1(1)-x1e(1));
    P1 = P1-K1*H1'*P1+Q1;

    H2 = Z2;
    K2 = P2*H2*inv(R2+H2'*P2*H2);
    W2 = W2+g2*K2*(x1(2)-x2e(1));
    P2 = P2-K2*H2'*P2+Q2;

    H3 = Z3;
    K3 = P3 * H3*inv(R3+H3'*P3*H3);
    W3 = W3+g3*K3*(x1(3)-x3e(1));
    P3 = P3-K3*H3'*P3+Q3;

    x1e = W1'*Z1+0.0001*u(1) + 0.0001*u(2);
    x2e = W2'*Z2+0.0001*u(1) + 0.0001*u(2);
    x3e = W3'*Z3+0.0001*u(1) + 0.0001*u(2);

    % x1e_pred = A * x1e;
    % P1e_pred = A * P1e * A' + Q;
    % 
    
    % K = P1e_pred * H1'*inv(H1 * P1e_pred * H1' + R_k); % Ganancia de Kalman
    % x1e = x1e_pred + K * (y_1 - (C_k * x1e_pred));
    % P1e = (eye(3) - K * H1) * P1e_pred;

    xe1_hist=[xe1_hist; [t x1e']];
    xe2_hist=[xe2_hist; [t x2e']];
    xe3_hist=[xe3_hist; [t x3e']];
    %P1e_hist = [P1e_hist, P1];


end
figure(1)
hold on
plot(x1_hist(:,1),x1_hist(:,2),'-b')
plot(xe1_hist(:,1),xe1_hist(:,2),'--b')

plot(x1_hist(:,1),x1_hist(:,3),'-g')
plot(xe2_hist(:,1),xe2_hist(:,2),'--g')

plot(x1_hist(:,1),x1_hist(:,4),'-r')
plot(xe3_hist(:,1),xe3_hist(:,2),'--r')

grid on
legend('posicion x','estimacion posicion x','posicion y','estimacion posicion y','theta','estimacion theta');
title('x11');

return
figure (2)
plot(x2_hist(:,1),x2_hist(:,2),'-b')
plot(x2_hist(:,1),x2_hist(:,3),'-r')
hold on
grid on
legend('velocidad angular derecha','velocidad angular izquierda');
title('x2');

figure (3)
plot(x3_hist(:,1),x3_hist(:,2),'-b')
hold on
plot(x3_hist(:,1),x3_hist(:,3),'-r')
hold on
grid on
legend('corriente derecha','corriente izquierda');
title('x3');
% haganlo funcionar para las otras dos x