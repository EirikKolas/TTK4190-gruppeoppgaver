function delta_c = PID_heading(e_psi,e_r,e_int)

% Controller gains computed in task d.
% kp = -6.5853e+04; 
% kd = -2.1174e+06; 
% ki = -613.9226; 

kp = -196.6580; 
kd = -4.0855e+03; 
ki = -1.8334; 

delta_c = kp*e_psi + kd*e_r + ki*e_int;

end