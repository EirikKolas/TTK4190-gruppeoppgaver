function chi_d = LOS_guidance(e_y,pi_p, crab_angle)  %Find desired course from position and path
    delta = 1e04;
    chi_d = pi_p - atan(e_y/delta) - crab_angle;
end