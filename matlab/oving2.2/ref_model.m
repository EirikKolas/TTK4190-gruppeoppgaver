%% Reference Model


function xd_dot = ref_model(xd, psi_ref)
    % See equations 15.145-15.147 in Fossen
    psi_d = xd(1);
    r_d = xd(2);
    a_d = xd(3);

    omega_n = 0.03;
    zeta = 0.7;
    
    psi_d_dot = r_d;
    r_d_dot = a_d;
    a_d_dot = -(2*zeta+1)*omega_n*a_d - (2*zeta+1)*omega_n^2*r_d + omega_n^3*(psi_ref-psi_d);

    xd_dot = [psi_d_dot; r_d_dot; a_d_dot];
end