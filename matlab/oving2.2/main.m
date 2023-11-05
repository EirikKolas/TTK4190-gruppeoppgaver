% Project in TTK4190 Guidance, Navigation and Control of Vehicles 
%
% Author:           My name
% Study program:    My study program
close all;
clear;
% Add folder for 3-D visualization files
addpath(genpath('flypath3d_v2'))
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% USER INPUTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
h  = 0.1;    % sampling time [s]
Ns = 80000;    % no. of samples

% Set psi_ref to be 10 degrees for the first half of the simulation, and -30 for the second half
% psi_ref = [deg2rad(10)*ones(1,Ns/2 + 1) deg2rad(-20)*ones(1,Ns/2)]; % reference course angle
psi_ref = deg2rad(10)*ones(1,Ns + 1); % reference course angle

U_ref   = 9;            % desired surge speed (m/s)

% nu    = x(1:3);
% eta   = x(4:6);
% delta = x(7);
% n     = x(8); 
% Qm    = x(9);

% initial states
eta_0 = [0 0 0]';
nu_0  = [0 0 0]';
delta_0 = 0;
n_0 = 0;
Qm_0 = 0; 
x = [nu_0' eta_0' delta_0 n_0 Qm_0]';
xd = [0 0 0]';            % initial reference
e_int = 0;       % initial error integral


load('WP.mat');
wp_index = 1; 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% MAIN LOOP
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
simdata = zeros(Ns+1,14);       % table of simulation data
wait_bar = waitbar(0, 'Starting');
for i=1:Ns+1
    
    t = (i-1) * h;              % time (s)
    u     = x(1); % surge velocity, must be positive  (m/s)    
    v     = x(2); % sway velocity                     (m/s)
    r     = x(3); % yaw velocity                      (rad/s)
    % x     = x(4); % position in x-direction           (m)
    % y     = x(5); % position in y-direction           (m)
    psi   = x(6); % yaw angle                         (rad)
    delta = x(7); % actual rudder angle               (rad)
    n     = x(8); % actual shaft velocity             (rpm)
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 2, 1a) Add current disturbance here 49
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    Vc = 1;
    % Vc = 0;
    beta_Vc = deg2rad(45); 
    u_c = Vc*cos(beta_Vc - psi);
    v_c = Vc*sin(beta_Vc - psi);

    nu_c = [ u_c v_c 0 ]';
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 2, 1c) Add wind disturbance here 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    V_w = 10;
    beta_V_w = deg2rad(135);
    c_y = 0.95; 
    c_n = 0.15; 
    L = 161; 
    A_L_w = 10*L;
    rho_a = 1.247; 
    q = 0.5*rho_a*V_w^2; 
    gamma_w = psi-beta_V_w-pi;
    
    C_N_gamma_w = c_n*sin(2*gamma_w);
    C_Y_gamma_w = c_y*sin(gamma_w);

    if t >200
        Ywind = q*C_Y_gamma_w*A_L_w;
        Nwind = q*C_N_gamma_w*A_L_w*L; 
    else
        Ywind = 0; 
        Nwind = 0;
    end 

    tau_wind = [0 Ywind Nwind]';
    % tau_wind = zeros(3,1);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 2, 2d) Add a reference model here 
    % Define it as a function
    % check eq. (15.143) in (Fossen, 2021) for help
    %
    % The result should look like this:
    % xd_dot = ref_model(xd, psi_ref(i));
    % psi_d = xd(1);
    % r_d = xd(2);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

    %     Guidance law
    [xk1,yk1,xk,yk,wp_index] = wp_selector(x(4),x(5), wp_index, WP);
    [y_e, pi_p] = crossTrackError(xk1,yk1,xk,yk,x(4),x(5)); 
    chi_d = LOS_guidance(y_e,pi_p);
    psi_ref = chi_d;

    xd_dot = ref_model(xd, psi_ref);
    psi_d  = xd(1);
    r_d    = xd(2);
    u_d    = U_ref;

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 2, 2d) Add the heading controller here 
    % Define it as a function
    %
    % The result should look like this:
    % delta_c = PID_heading(e_psi,e_r,e_int);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %delta_c = 0.1;              % rudder angle command (rad)
    e_psi = psi - psi_d;
    e_r = r - r_d;
    e_int = e_int + e_psi*h;
    
    delta_c = PID_heading(e_psi, e_r, e_int); 
    
    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 3, 1e) Add open loop speed control here
    % Define it as a function
    %
    % The result should look like this:
    % n_c = open_loop_speed_control(U_ref);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     n_c = 10;                   % propeller speed (rps)
    n_c = open_loop_speed_control(U_ref);


    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Part 3, 1f) Replace the open loop speed controller, 
    % with a closed loop speed controller here 
    % Define it as a function
    %
    % The result should look like this:
    % n_c = closed_loop_speed_control(u_d,e_u,e_int_u);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % ship dynamics
    u = [delta_c n_c]';
    [xdot,u] = ship(x,u,nu_c,tau_wind);
    
    % store simulation data in a table (for testing)
    simdata(i,:) = [t x(1:3)' x(4:6)' x(7) x(8) u(1) u(2) u_d psi_d r_d];     
 
    % Euler integration
    x = euler2(xdot,x,h);  
    xd = euler2(xd_dot,xd,h);  

    waitbar(i/(Ns+1), wait_bar, sprintf('Progress: %d %%', floor(i/(Ns+1)*100)));
end
close(wait_bar);
simdata = simdata(1:i,:);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PLOTS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
t       = simdata(:,1);                 % s
u       = simdata(:,2);                 % m/s
v       = simdata(:,3);                 % m/s
r       = (180/pi) * simdata(:,4);      % deg/s
x       = simdata(:,5);                 % m
y       = simdata(:,6);                 % m
psi     = simdata(:,7);                 % rad
psi_deg = (180/pi) * psi;               % deg
delta_0 = (180/pi) * simdata(:,8);      % deg
n_0     = 60 * simdata(:,9);            % rpm
delta_c = (180/pi) * simdata(:,10);     % deg
n_c     = 60 * simdata(:,11);           % rpm
u_d     = simdata(:,12);                % m/s
psi_d   = (180/pi) * simdata(:,13);     % deg
r_d     = (180/pi) * simdata(:,14);     % deg/s

figure(2)
figure(gcf)
subplot(311)
plot(y,x,'linewidth',2); axis('equal')
title('North-East positions (m)'); xlabel('(m)'); ylabel('(m)'); 
subplot(312)
plot(t,psi_deg,t,psi_d,'linewidth',2);
title('Actual and desired yaw angles (deg)'); xlabel('time (s)');
legend(["Actual yaw angle", "Desired yaw angle"])
subplot(313)
plot(t,r,t,r_d,'linewidth',2);
title('Actual and desired yaw rates (deg/s)'); xlabel('time (s)');
legend(["Actual yaw rate", "Desired yaw rate"])

figure(3)
figure(gcf)
subplot(311)
plot(t,u,t,u_d,'linewidth',2);
title('Actual and desired surge velocities (m/s)'); xlabel('time (s)');
legend(["Actual surge velocity", "Desired surge velocity"])
subplot(312)
plot(t,n_0,t,n_c,'linewidth',2);
title('Actual and commanded propeller speed (rpm)'); xlabel('time (s)');
legend(["Actual propeller speed", "Commanded propeller speed"])
subplot(313)
plot(t,delta_0,t,delta_c,'linewidth',2);
title('Actual and commanded rudder angles (deg)'); xlabel('time (s)');
legend(["Actual rudder angle", "Commanded rudder angle"])

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Own plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Speed over ground
U = sqrt(u.^2 + v.^2);                      % m/s
% Relative speed
U_r = sqrt((u-u_c).^2 + (v-v_c).^2);        % m/s
% Sideslip angle
beta = atan((v-v_c)./U_r);                  % rad
beta_deg = (180/pi) * beta;                 % deg
% Crab angle
beta_c = atan(v./U);                        % rad
beta_c_deg = (180/pi) * beta_c;             % deg
figure(4)
figure(gcf)
plot(t,beta_deg,'linewidth',2); hold on;
plot(t,beta_c_deg,'linewidth',2);
title('Sideslip vs. Crab Angle'); xlabel('time (s)');
legend('Sideslip angle','Crab angle');
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%% Create objects for 3-D visualization 
% Since we only simulate 3-DOF we need to construct zero arrays for the 
% excluded dimensions, including height, roll and pitch
z = zeros(length(x),1);
phi = zeros(length(psi),1);
theta = zeros(length(psi),1);

figure(5)
figure(gcf)
% create object 1: ship (ship1.mat)
new_object('flypath3d_v2/ship1.mat',[x,y,z,phi,theta,psi],...
'model','royalNavy2.mat','scale',(max(max(abs(x)),max(abs(y)))/1000),...
'edge',[0 0 0],'face',[0 0 0],'alpha',1,...
'path','on','pathcolor',[.89 .0 .27],'pathwidth',2);

% Plot trajectories 
flypath('flypath3d_v2/ship1.mat',...
'animate','on','step',500,...
'axis','on','axiscolor',[0 0 0],'color',[1 1 1],...
'font','Georgia','fontsize',12,...
'view',[-25 35],'window',[900 900],...
'xlim', [min(y)-0.1*max(abs(y)),max(y)+0.1*max(abs(y))],... 
'ylim', [min(x)-0.1*max(abs(x)),max(x)+0.1*max(abs(x))], ...
'zlim', [-max(max(abs(x)),max(abs(y)))/100,max(max(abs(x)),max(abs(y)))/20]); 

pathplotter(x,y) 