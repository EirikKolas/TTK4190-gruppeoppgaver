function n_c = open_loop_speed_control(U_ref)

% parameters taken from ship.m
Dia = 3.3;              
rho = 1025;            
T1 = 20; 
Xudot = -8.9830e5;
m = 17.0677e6;         
Xu = -(m-Xudot)/T1;

% factor between 0.05 and 2 due to
% extra resitance on the hull caused by the propeller
t = 0.05; 

% compute desired torque based on simplified linear model for surge
Td = U_ref*Xu/(t-1);

% compute toque coefficient 
J_a = 0; 
PD = 1.5;   %pitch diameter ratio
BAR = 0.65; %blade area ratio
z = 4; %number of blades
[KT, KQ] = wageningen(J_a,PD,BAR,z);

% compute desired turtall based on eq 9.7 in Fossen
n_c = sign(Td)*sqrt(sign(Td)*Td/(rho*(Dia^4)*KT));

end