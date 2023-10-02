% M-script for numerical integration of the attitude dynamics of a rigid 
% body represented by unit quaternions. The MSS m-files must be on your
% Matlab path in order to run the script.
%
% System:                      .
%                              q = T(q)w
%                              .
%                            I w - S(Iw)w = tau
% Control law:
%                            tau = constant
% 
% Definitions:             
%                            I = inertia matrix (3x3)
%                            S(w) = skew-symmetric matrix (3x3)
%                            T(q) = transformation matrix (4x3)
%                            tau = control input (3x1)
%                            w = angular velocity vector (3x1)
%                            q = unit quaternion vector (4x1)
%
% Author:                   2018-08-15 Thor I. Fossen and Håkon H. Helgesen

%% USER INPUTS
h = 0.1;                     % sample time (s)
N  = 15000;                    % number of samples. Should be adjusted

% model parameters
m = 180;
r = 2;
I = m*r^2*eye(3);            % inertia matrix
I_inv = inv(I);

% constants
deg2rad = pi/180;   
rad2deg = 180/pi;

phi = -5*deg2rad;            % initial Euler angles
theta = 10*deg2rad;
psi = -20*deg2rad;

q = euler2q(phi,theta,psi);   % transform initial Euler angles to q

w = [0 0 0]';                 % initial angular rates

table = zeros(N+1,14);        % memory allocation

% Control params
% control gains
kd = 400; 
kp = 20; 


% linearized system
Aol = [zeros(3,3), (1/2)*eye(3); zeros(3), zeros(3)]; 
B = [zeros(3,3); I_inv]; 
K = [kp*eye(3), kd*eye(3)]; 

Acl = Aol-B*K;

% eigenvavlues for linearized closed loop system
eig(Acl);

%% FOR-END LOOP
for i = 1:N+1
   t = (i-1)*h;  
   

   %time varying reference signals
   theta_freq = 0.1; 
   psi_freq = 0.05; 

   phi_d = 0;
   theta_d = 15*cos(theta_freq*t)*deg2rad;
   psi_d = 10*sin(psi_freq*t)*deg2rad; 


   q_d = euler2q(phi_d,theta_d,psi_d);

   phi_d_dot = 0; 
   theta_d_dot = -15*theta_freq*sin(0.1*t)*deg2rad; 
   psi_d_dot = 10*psi_freq*cos(0.05*t)*deg2rad; 


   w_d = Tzyx(phi_d, theta_d)\[phi_d_dot; theta_d_dot; psi_d_dot]; 

 
   %define error state
   q_d_conj = [q_d(1); -q_d(2:4)]; 
   q_err = quatprod(q_d_conj, q); 
   w_err = w - w_d; 
   x_err = [q_err(2:4); w_err];

   tau = -K*x_err;            % control law
   [J,J1,J2] = quatern(q);       % kinematic transformation matrices
   
   q_dot = J2*w;                        % quaternion kinematics
   w_dot = I_inv*(Smtrx(I*w)*w + tau);  % rigid-body kinetics

   %store values
   [phi,theta,psi] = q2euler(q); % transform q to Euler angles
   phi_e = phi - phi_d;
   theta_e = theta - theta_d;
   psi_e = psi - psi_d;
   table(i,:) = [t q' phi_e theta_e psi_e w_err' tau'];  % store data in table
   
   q = q + h*q_dot;	             % Euler integration
   w = w + h*w_dot;
   
   q  = q/norm(q);               % unit quaternion normalization
end 

%% PLOT FIGURES
t       = table(:,1);  
q       = table(:,2:5); 
phi     = rad2deg*table(:,6);
theta   = rad2deg*table(:,7);
psi     = rad2deg*table(:,8);
w       = rad2deg*table(:,9:11);  
tau     = table(:,12:14);

% Define the linewidth
linewidth = 1;

% Create a single figure with three subplots
figure;

% Subplot 1 - Euler angles
subplot(3, 1, 1);
hold on;
plot(t, phi, 'b', 'LineWidth', linewidth);
plot(t, theta, 'r', 'LineWidth', linewidth);
plot(t, psi, 'g', 'LineWidth', linewidth);
hold off;
grid on;
legend('\phi', '\theta', '\psi');
title('Error in euler angles');
xlabel('time [s]'); 
ylabel('angle [deg]');

% Subplot 2 - Angular velocities
subplot(3, 1, 2);
hold on;
plot(t, w(:,1), 'b', 'LineWidth', linewidth);
plot(t, w(:,2), 'r', 'LineWidth', linewidth);
plot(t, w(:,3), 'g', 'LineWidth', linewidth);
hold off;
grid on;
legend('p', 'q', 'r');
title('Angular velocities error');
xlabel('time [s]'); 
ylabel('angular rate [deg/s]');

% Subplot 3 - Control input
subplot(3, 1, 3);
hold on;
plot(t, tau(:,1), 'b', 'LineWidth', linewidth);
plot(t, tau(:,2), 'r', 'LineWidth', linewidth);
plot(t, tau(:,3), 'g', 'LineWidth', linewidth);
hold off;
grid on;
legend('x', 'y', 'z');
title('Control input');
xlabel('time [s]'); 
ylabel('input [Nm]');
