function [chi_d, y_int_dot] = ILOS_guidance(y_e, pi_p, y_int)
    look_ahead_dist = 1e04; 
    kappa = 10; 
    Kp = 1/look_ahead_dist; 
    Ki = kappa*Kp; 

    chi_d = pi_p - atan(Kp*y_e + Ki*y_int); 
    y_int_dot = look_ahead_dist*y_e/(look_ahead_dist^2 + (y_e + kappa*y_int)^2); 

end