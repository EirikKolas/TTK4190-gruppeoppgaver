function chi_d = LOS_guidance(e_y, pi_p)  %Find desired course from position and path
    delta = 1e04;
    chi_d = pi_p - atan(e_y/delta);  % use atan2 ??   

end