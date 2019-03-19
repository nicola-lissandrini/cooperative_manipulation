%% Setup Aerial MPC

clear all
close all

EXPORT = 1;
COMPILE = 1;

% Horizon samples
N =10;             % WARNING: when you change this you have also to change N in ref. generation fcn
% Sample time
Ts = 0.1;

% Common definitions
aerialParams = csvread('../config.sml/parameters.csv');

%% OCP definitions

DifferentialState p_o(3) R_e_vec(9);
DifferentialState th1 th2 x_v y_v z_v psi_v;
Control uth1 uth2 ux uy uz upsi;
OnlineData R_w_des_vec(9)

%%

% Joint variables
q = [th1; th2; x_v; y_v; z_v; psi_v];
% Shorthand
u = [uth1 uth2 ux uy uz upsi]';

p_v = [x_v; y_v; z_v; psi_v];

% Get total jacobian
J = getAerialJacobian (q, aerialParams);
% Translational part
J_P = J(1:3,:);
% Orientational part
J_O = J(4:6,:);
% Angular velocity vector
omega_o = J_O * u;

% Input matrix for the motion constraint
%B = reshape([cos(psi_v),sin(psi_v),0.0,0.0,-sin(psi_v),cos(psi_v),0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0],[4,4]);
B = getAerialInputMatrix (q)';
% Dynamics on rotation matrix
R_e = reshape(R_e_vec,3,3);
R_e_dot = skew(omega_o) * R_e;
R_e_dot_vec = reshape(R_e_dot, 9, 1);

% Model definition
f = [dot(p_o)==J_P*u; ...
     dot(R_e_vec)==R_e_dot_vec;...
     dot(q)==u];
 
% Compute distance with ref rotation
R_w_des = reshape (R_w_des_vec,3,3);
R_e_err = R_e' * R_w_des;
R_err_meas = R_e_err - eye(3);


h = [p_o(1:3);0; R_err_meas(:); controls];
hN = [p_o(1:3);0; R_err_meas(:)];


%% MPCexport

acadoSet ('problemname','aerialMPC');

% N and Ts defined in setupSimulink
ocp = acado.OCP (0.0, N*Ts, N);

W = acado.BMatrix (eye (length(h)));
WN = acado.BMatrix (eye (length(hN)));

ocp.minimizeLSQ (W, h);
ocp.minimizeLSQEndTerm (WN, hN);
ocp.setModel (f);
ocp.subjectTo (-0.65 <= controls <= 0.65);
ocp.subjectTo (-1.571 <= th1 <= 1.571);
ocp.subjectTo (-1.571 <= th2 <= 1.571);

mpc = acado.OCPexport (ocp);
mpc.set( 'HESSIAN_APPROXIMATION',       'GAUSS_NEWTON'      );
mpc.set( 'DISCRETIZATION_TYPE',         'MULTIPLE_SHOOTING' );
mpc.set( 'SPARSE_QP_SOLUTION',          'FULL_CONDENSING_N2');
mpc.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL2'       );
mpc.set( 'NUM_INTEGRATOR_STEPS',         4*N                );
mpc.set( 'QP_SOLVER',                   'QP_QPOASES3'    	);
mpc.set( 'LEVENBERG_MARQUARDT',          1e-4                );

%% Compile
path = '../src/export_MPC';
mpc.exportCode (path);
global ACADO_;
copyfile([ACADO_.pwd '/../../external_packages/qpoases3'], [path '/qpoases3']);






