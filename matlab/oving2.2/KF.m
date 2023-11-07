% Kalman Filter
function [x_pst,P_pst,x_prd,P_prd] = KF(x_hat,P_hat,Ad,Bd,Ed,Cd,Qd,Rd,psi_meas,delta)
    % Measurement: y[k]
    y = psi_meas;
    % Input: u[k]
    u = delta;

    % Predictor: x_prd[k+1] and P_prd[k+1]
    x_prd = Ad * x_hat + Bd * u;
    P_prd = Ad * P_hat * Ad' + Ed * Qd * Ed';
    y_prd = Cd * x_prd;
    % Innovation
    nu = y - y_prd;
    S = Cd * P_prd * Cd' + Rd;
    % KF gain: K[k]
    K = P_prd * Cd' / S;
    IKC = eye(5) - K * Cd;
    
    % Posteriori: x_pst[k+1] and P_pst[k+1]
    x_pst = x_prd + K * nu;
    P_pst = IKC * P_prd * IKC' + K * Rd * K';
end