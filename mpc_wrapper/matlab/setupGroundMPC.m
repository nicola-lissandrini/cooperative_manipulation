%% Setup Ground MPC

clear all
close all

EXPORT = 1;
COMPILE = 1;

% Common definitions
setupSimulink

%% OCP definitions

DifferentialState p_o(3) R_e_vec(9);
DifferentialState th1 th2 th3 th4 x_v y_v psi_v;
Control uth1 uth2 uth3 uth4 ux uy upsi;

% Obstacle avoidance
OnlineData R_w_des_vec(9)


% Joint variables
q = [th1; th2; th3; th4; x_v; y_v; psi_v];
% Shorthand
u = controls;

% Get Jacobian
J = getGroundJacobian (q, groundParams);
% Translational part
J_P = J(1:3,:);
% Orientational part
J_O = J(4:6,:);
% Angular velocity vector
omega_o = J_O * u;

% Dynamics on rotation matrix
R_e = reshape(R_e_vec,3,3);
R_e_dot = skew(omega_o)*R_e;
R_e_dot_vec = reshape(R_e_dot, 9, 1);

f = [dot(p_o)==J_P*u;...
     dot(R_e_vec)==R_e_dot_vec;...
     dot(q)==u];
   
% Compute distance with ref rotation
R_w_des = reshape (R_w_des_vec,3,3);
R_e_err = R_e' * R_w_des;
R_err_meas = R_e_err - eye(3);

h = [p_o; R_err_meas(:); controls];
hN = [p_o; R_err_meas(:)];


%% MPCexport
acadoSet ('problemname','mpc');

% N and Ts are defined in setupSimulink
ocp = acado.OCP (0.0, N*Ts, N);

W = acado.BMatrix (eye (length(h)));
WN = acado.BMatrix (eye (length(hN)));

ocp.minimizeLSQ (W, h);
ocp.minimizeLSQEndTerm (WN, hN);
ocp.setModel (f);
UMAX = 0.1;
%ocp.subjectTo (-UMAX <= controls <= UMAX);

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






