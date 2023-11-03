function chi_d = LOS_guidance(e_y, pi_p)  %Find desired course from position and path
    K_p = 1;
    delta = 1/K_p;
    chi_d = pi_p - atan(e_y/delta);     

end