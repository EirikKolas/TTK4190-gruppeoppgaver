function chi_d = LOS_guidance(e_y,pi_p, crab_angle, compensate_for_crab)  %Find desired course from position and path
    delta = 1e04;

    if compensate_for_crab
        chi_d = pi_p - atan(e_y/delta) - crab_angle;
    else
        chi_d = pi_p - atan(e_y/delta);
    end

end