%% Setup Ground MPC

clear all
close all

EXPORT = 1;
COMPILE = 1;

% Horizon samples
N = 10;
% Keypoints count
K = 1;
% Sample time
Ts = 0.1;

% Common definitions
setupSimulink

getP2 = @(q, params)[q(5)+params(7).*cos(q(7))-params(8).*sin(q(7))+cos(q(7)+q(1)).*(params(1).*cos(q(2))+params(4).*sin(q(2)));q(6)+params(8).*cos(q(7))+params(7).*sin(q(7))+sin(q(7)+q(1)).*(params(1).*cos(q(2))+params(4).*sin(q(2)));params(9)-params(4).*cos(q(2))+params(1).*sin(q(2))]
getP3 = @(q, params)[q(5)+params(7).*cos(q(7))-params(8).*sin(q(7))+cos(q(7)+q(1)).*(params(1).*cos(q(2))+params(4).*sin(q(2)))+params(2).*cos(q(7)+q(1)).*sin(q(2)+q(3));q(6)+params(8).*cos(q(7))+params(7).*sin(q(7))+sin(q(7)+q(1)).*(params(1).*cos(q(2))+params(4).*sin(q(2)))+params(2).*sin(q(7)+q(1)).*sin(q(2)+q(3));params(9)-params(2).*cos(q(2)+q(3))-params(4).*cos(q(2))+params(1).*sin(q(2))]
getP4 = @(q, params)[q(5)+params(7).*cos(q(7))-params(8).*sin(q(7))+cos(q(7)+q(1)).*(params(1).*cos(q(2))+params(4).*sin(q(2)))+params(2).*cos(q(7)+q(1)).*sin(q(2)+q(3))+params(3).*cos(q(7)+q(1)).*cos(q(2)+q(3)).*sin(q(4))+params(3).*cos(q(7)+q(1)).*sin(q(2)+q(3)).*cos(q(4));q(6)+params(8).*cos(q(7))+params(7).*sin(q(7))+sin(q(7)+q(1)).*(params(1).*cos(q(2))+params(4).*sin(q(2)))+params(2).*sin(q(7)+q(1)).*sin(q(2)+q(3))+params(3).*cos(q(2)+q(3)).*sin(q(7)+q(1)).*sin(q(4))+params(3).*sin(q(7)+q(1)).*sin(q(2)+q(3)).*cos(q(4));params(9)-params(2).*cos(q(2)+q(3))-params(4).*cos(q(2))+params(1).*sin(q(2))-params(3).*cos(q(2)+q(3)+q(4))]
getFR = @(q, params)[q(5)+params(10); q(6)-params(11); params(9)/2];
getFL = @(q, params)[q(5)+params(10); q(6)+params(11); params(9)/2];
getRR = @(q, params)[q(5)-params(10); q(6)-params(11); params(9)/2];
getRL = @(q, params)[q(5)-params(10); q(6)+params(11); params(9)/2];

%% OCP definitions

DifferentialState p_o(3) R_e_vec(9);
DifferentialState th1 th2 th3 th4 x_v y_v psi_v;
Control uth1 uth2 uth3 uth4 ux uy upsi;

% Take reference rotation as online data to process it
OnlineData R_w_des_vec(9)
% Obstacle avoidance1
OnlineData v_o_1(3) a_1 b_1 c_1
OnlineData v_o_2(3) a_2 b_2 c_2
% Obstacle avoidance keypoints
OnlineData keypts(3*K)

% Joint variables
q = [th1; th2; th3; th4; x_v; y_v; psi_v];
% Shorthand
u = controls;
u_joints = u(1:4);
u_base = u(5:end);
joints = [th1; th2; th3; th4];

% Get Jacobian
J = getGroundJacobian (q, groundParams);
% Translational part
J_P = J(1:3,:);
% Orientational part
J_O = J(4:6,:);
% Angular velocity vector
omega_o = J_O * u;

B_base = [cos(psi_v), -sin(psi_v), 0;
     sin(psi_v),  cos(psi_v), 0;
              0,           0, 1];
B = blkdiag(eye(4), B_base);

% Dynamics on rotation matrix
R_e = reshape(R_e_vec,3,3);
R_e_dot = skew(omega_o)*R_e;
R_e_dot_vec = reshape(R_e_dot, 9, 1);

f = [dot(p_o)==J_P*u;...
     dot(R_e_vec)==R_e_dot_vec;...
     dot(q)==B*u];

% Compute distance with ref rotation
R_w_des = reshape (R_w_des_vec,3,3);
R_e_err = R_e' * R_w_des;
R_err_meas = R_e_err - eye(3);

p_v = [x_v; y_v; 0];
p_2 = getP2 (q, groundParams);
p_3 = getP3 (q, groundParams);
w_fr = getFR (q, groundParams);
w_fl = getFL (q, groundParams);
w_rr = getRR (q, groundParams);
w_rl = getRL (q, groundParams);

delta_eef = 0.15;
delta_base = 0.07;
delta_joints = 0.1;

% Cost matrix: rows - obstacles; columns - joints
cost(1,1) = costCollide (p_o, v_o_1, a_1, delta_eef);
cost(1,2) = costCollide (p_v, v_o_1, a_1, 0.2);
cost(1,3) = costCollide (p_3, v_o_1, a_1, delta_joints);
cost(1,4) = 0*costCollide (keypts(1:3), v_o_1, a_1, delta_eef);
% base
cost(1,5) = costCollide (w_fr, v_o_1, a_1, delta_base);
cost(1,6) = costCollide (w_fl, v_o_1, a_1, delta_base);
cost(1,7) = costCollide (w_rr, v_o_1, a_1, delta_base);
cost(1,8) = costCollide (w_rl, v_o_1, a_1, delta_base);

% Second obstacle
cost(2,1) = costCollide (p_o, v_o_2, a_2, delta_eef);
cost(2,2) = costCollide (p_v, v_o_2, a_2, 0.2);
cost(2,3) = costCollide (p_3, v_o_2, a_2, delta_joints);
cost(2,4) = 0*costCollide (keypts(1:3), v_o_2, a_2, delta_eef);
% base
cost(2,5) = costCollide (w_fr, v_o_2, a_2, delta_base);
cost(2,6) = costCollide (w_fl, v_o_2, a_2, delta_base);
cost(2,7) = costCollide (w_rr, v_o_2, a_2, delta_base);
cost(2,8) = costCollide (w_rl, v_o_2, a_2, delta_base);

totalCost = sum (cost(:));

h = [p_o; totalCost; R_err_meas(:); controls];
hN = [p_o; totalCost; R_err_meas(:)];

%% MPCexport
acadoSet ('problemname','mpc');

% N and Ts are defined in setupSimulink
ocp = acado.OCP (0.0, N*Ts, N);

W = acado.BMatrix (eye (length(h)));
WN = acado.BMatrix (eye (length(hN)));

ocp.minimizeLSQ (W, h);
ocp.minimizeLSQEndTerm (WN, hN);
ocp.setModel (f);
ocp.subjectTo (-0.25 <= u_base <= 0.25);
ocp.subjectTo (-0.07 <= psi_v <= 0.07);
ocp.subjectTo (-0.07 <= uth1 <= 0.07);
ocp.subjectTo (-0.3 <= u_joints <= 0.3);
ocp.subjectTo (-2.517 <= th1 <= 2.517);
ocp.subjectTo (-pi/3 <= th2 <= pi/2 + pi/8);
ocp.subjectTo (-pi/2 <= th3 <= pi/2);
ocp.subjectTo (-1.745 <= th4 <= 1.745);
ocp.subjectTo (-0.6 <= x_v);

mpc = acado.OCPexport (ocp);
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL2'       );
mpc.set( 'NUM_INTEGRATOR_STEPS',        2*N                 );
mpc.set( 'QP_SOLVER',                   'QP_QPOASES3'    	);
mpc.set( 'LEVENBERG_MARQUARDT',         1e-4                );


%% Export
path = '../src/export_MPC';
mpc.exportCode (path);
global ACADO_;
copyfile([ACADO_.pwd '/../../external_packages/qpoases3'], [path '/qpoases3']);






