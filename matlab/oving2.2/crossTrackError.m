function [y_e, pi_p] = crossTrackError(xk1,yk1,xk,yk,x,y)

[x_p, y_p, y_e] = crosstrack(xk1,yk1,xk,yk,x,y);
pi_p = atan2(yk1 - yk, xk1 - xk);

end