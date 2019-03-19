%% Simple control


clear all
close all

EXPORT = 1;
COMPILE = 1;

N = 10;
Ts = 0.1;

%% Problem definition

DifferentialState x v th;
Control u uth;

OnlineData unused;

f = [dot(x)==v;
     dot(v)==u;
     dot(th)==uth];
 
h = [x;v; th; controls];
hN = [x; v; th];


%% MPCexport
acadoSet ('problemname','groundMPC');

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


%% Compile

if EXPORT
    mpc.exportCode( 'ground_MPC' );
end
if COMPILE
    global ACADO_;
    copyfile([ACADO_.pwd '/../../external_packages/qpoases3'], 'ground_MPC/qpoases3')
    
    make_ground
end


