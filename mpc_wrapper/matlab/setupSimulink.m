%% Setup simulink

%% Hardcoded params
% If you change these you have to rebuild MPCs

% Horizon samples
N = 10;             % WARNING: when you change this you have also to change N in ref. generation fcn
% Sample time
Ts = 0.1;

load ('hardcoded.mat');

if (N ~= N_h) | (Ts ~= Ts_h)
    N_h = N;
    Ts_h = Ts;
    save ('hardcoded.mat','N_h','Ts_h');
    setupAerialMPC
    setupGroundMPC
end

%% Params

ground_l1_val = 3;
ground_l2_val = 2;
ground_l3_val = 1;
ground_lox_val = 1;
ground_loy_val = 0;
ground_loz_val = 0;

groundParams = [ground_l1_val; ...
                ground_l2_val; ...
                ground_l3_val; ...
                ground_lox_val; ...
                ground_loy_val; ...
                ground_loz_val];
            
%% Simulation setup

ground_j0= [-0.8765    0.7933   -1.0707   -0.6416   -0.3887    5.2268   -0.6201];
ground_p_o_0 = getGroundKinematics (ground_j0, groundParams)';
ground_x0 = ground_p_o_0;
R_w_g = getGroundAttitude(ground_j0);

% MPC initial configuration
ground_xInit = [ground_x0.';  R_w_g(:); ground_j0']';
ground_zInit = [];
ground_uInit = zeros(7,1);

% Object desired position
object_ref = ground_x0 + [1 0 0];
object_ref_rpy = tr2rpy2 (R_w_g) + [0 0 0]';

%% Control tuning

% Weights
NR = 9;
posOnes = ones(1,3);
rotOnes = ones(1,NR);
groundJointCtrOnes = ones(1,4);
groundVehicleCtrOnes = ones(1,3);

ground_W = diag([posOnes 70*rotOnes 0.1*groundJointCtrOnes 1*groundVehicleCtrOnes]);
ground_WN = 1 * diag([5*posOnes 10*rotOnes]);

% Params
ground_bValues = [];  % bound values are fixed in the generated code
