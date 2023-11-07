function crab_angle = compute_crab_angle(nu)

v = nu(2); 
U = norm(nu); 
if abs(U) < 0.00001
    crab_angle = 0; 
else
    crab_angle = asin(v/U); 
end

end