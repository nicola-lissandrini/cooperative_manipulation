%% Setup simulink

            
%% Simulation setup


% Initial configuration
aerial_j0 = [pi/6 pi/6 0 0 5 0];
aerial_p_o_0 = getAerialKinematics (aerial_j0, aerialParams)';
aerial_x0 = aerial_p_o_0;
aerial_R0 = getAerialAttitude(aerial_j0);
aerial_rpy = tr2rpy2 (aerial_R0);

% MPC initial configuration
aerial_xInit = [aerial_x0'; aerial_R0(:); aerial_j0']; % aerial_x0.'; 
aerial_zInit = [];
aerial_uInit = zeros(6,1);

% Object desired position
aerial_rpy_des = aerial_rpy';
object_ref = [0 -2 0];
object_ref_rpy = [aerial_rpy(1), aerial_rpy(2) -1];

%% Control tuning

% Weights
NR = 9;
posOnes = ones(1,3);
rotOnes = ones(1,NR);
jointCtrOnes = ones(1,2);
vehicleCtrOnes = ones(1,4);
aerial_W = diag([rotOnes*1 jointCtrOnes*0.5 vehicleCtrOnes*0.5]);
aerial_WN = 10 * diag([rotOnes]);

% Params
aerial_bValues = [];  % bound values are fixed in the generated code
