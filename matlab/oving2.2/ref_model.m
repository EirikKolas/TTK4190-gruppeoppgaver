%% Reference Model


function xd_dot = ref_model(xd, psi_ref)
    % xd = [x y psi u v r]'
s = tf('s');
omega_n = 0.03;
zeta = 0.7;

% Third order reference model between psi_ref and psi_d
sys = omega_n^3/((s + omega_n)*(s^2 + 2*zeta*omega_n*s + omega_n^2));

[A, B] = tf2ss(sys.Numerator{1}, sys.Denominator{1});

xd_dot = A*xd + B*psi_ref;



end