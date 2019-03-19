%% Setup simulink

ground_W = diag([10 1 10 0.1 0.1]);
ground_WN = diag([20 10 10]);
ground_xInit = [0; 0; 0];
ground_zInit = [];
ground_uInit= [0; 0];
ground_bValues = [];