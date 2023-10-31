%% Reference Model


function xd_dot = ref_model(xd, psi_ref, A, B)
xd_dot = A*xd + B*psi_ref;

end