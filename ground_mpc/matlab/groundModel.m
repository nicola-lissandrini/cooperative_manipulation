% Note: everything is referred in the world frame
% Notation:
%   T_a_b = T^a_b coords of Frame b wrt Frame a
clearvars
close all
syms th1 th2 th3 th4 xb yb psi 
syms l1 l2 l3 lox loy loz lbx lby lbz

x = [1; 0; 0];
y = [0; 1; 0];
z = [0; 0; 1];
zero = [0; 0; 0];

%% Vehicle

% From World to Body
R_w_b = rotz(psi);
o_w_b = [xb; yb; 0];
T_w_b = rt2tr (R_w_b, o_w_b);

%% Arm

% From body to Frame 0

o_b_0 = [lbx; lby; lbz];
R_b_0 = eye(3);
T_b_0 = rt2tr (R_b_0, o_b_0);

% From Frame 0 to Frame 1

R_0_1p = rotz (th1);
R_1p_1 = rotx (pi/2);
R_0_1 = R_0_1p * R_1p_1;
o_0_1 = zero;
T_0_1 = rt2tr (R_0_1, o_0_1);

% From Frame 1 to Frame 2

% Account for first segment
R_1_2b = rotz(th2);
o_1_2b = l1 * R_1_2b * x;
T_1_2b = rt2tr (R_1_2b, o_1_2b);
% Account for second segment
R_2b_2 = rotz(-pi/2);
o_2b_2 = lox * R_2b_2 * x;
T_2b_2 = rt2tr (R_2b_2, o_2b_2);
T_1_2 = T_1_2b * T_2b_2;

% From F2 to F3

R_2_3 = rotz (th3);
o_2_3 = l2 * R_2_3 * x;
T_2_3 = rt2tr (R_2_3, o_2_3);

% From F3 to F4

R_3_4 = rotz (th4);
o_3_4 = l3 * R_3_4 * x;
T_3_4 = rt2tr (R_3_4, o_3_4);


% From F4 to end effector
R_4_e = eye(3);
o_4_e = zero;
T_4_e = rt2tr (R_4_e, o_4_e);

% From EEF to object - deprecated
R_e_o = eye(3);
o_e_o = [0; 0; 0];
T_e_o = rt2tr (R_e_o, o_e_o);

% Compute total tf's from W

T_b = T_w_b;
T_0 = T_b * T_b_0;
T_1 = T_0 * T_0_1;
T_2 = T_1 * T_1_2;
T_3 = T_2 * T_2_3;
T_4 = T_3 * T_3_4;
T_e = T_4 * T_4_e;
T_o = T_e * T_e_o;
T_2b = T_1 * T_1_2b;

% R,t

[R_b, p_b] = tr2rt (T_b);
[R_0, p_0] = tr2rt (T_0);
[R_1, p_1] = tr2rt (T_1);
[R_2, p_2] = tr2rt (T_2);
[R_3, p_3] = tr2rt (T_3);
[R_4, p_4] = tr2rt (T_4);
[R_e, p_e] = tr2rt (T_e);
[R_o, p_o] = tr2rt (T_o);
[R_2b, p_2b] = tr2rt (T_2b);

%% Differential model
% In the world frame

% Control inputs
syms uth1 uth2 uth3 uth4 ux uy upsi;

%% Linear velocity

% Body frame velocity
% dot p_o = J_L_v [ux; uy] + (*)
J_L_v = R_b * [x, y];

% Yaw velocity
% dot p_o = z_b \times (p_o - p_b) * upsi  + (*)
z_b = R_b * z;
J_L_psi = cross (z_b, p_o - p_0);

% Joint 1 velocity
% dot p_o = z_0 \times (p_o - p_0) * uth1 + (*)
z_0 =  R_0 * z;
J_L_1 = cross (z_0, p_o - p_0);

% Joint 2 velocity
z_1 = R_1 * z;
J_L_2 = cross (z_1, p_o - p_1);

% Joint 3 velocity
z_2 = R_2 * z;
J_L_3 = cross (z_2, p_o - p_2);

% Joint 4 velocity
z_3 = R_3 * z;
J_L_4 = cross (z_3, p_o - p_3);

% Linear Jacobian
J_L = simplify ([J_L_1, J_L_2, J_L_3, J_L_4, J_L_v, J_L_psi]);

%% Angular velocity

J_O_vx = zero;
J_O_vy = zero;
J_O_psi = z_b;
J_O_1 = z_0;
J_O_2 = z_1;
J_O_3 = z_2;
J_O_4 = z_3;

J_O = simplify ([J_O_1, J_O_2, J_O_3, J_O_4, J_O_vx, J_O_vy, J_O_psi]);

%% Total Jacobian

J_tot = [J_L; J_O];

%% Input to joints matrix

B = blkdiag (eye(4),R_b(1:2,1:2), 1);

%% For direct kinematics

 makeItWork (simplify (p_o))

%% For differential model

 makeItWork (simplify (J_tot))

 %% Test

groundParams = csvread ('../config.sml/parameters.csv');
state = [0.38531816396543805, -0.984961276578673, 0.3419236385022637, -0.0015799210088931481, -8.5186960050247e-05, -0.9999987482956104, -2.497139651863191e-05, 0.9999999960631566, -8.514761346053963e-05, 0.9999987516122383, 2.4836838758461832e-05, -0.001579923129910732, -2.0257484578678486e-10, 1.570796326718341, -1.0630998303895467e-09, 7.30251414893246e-11, 4.013880100031805e-05, -0.9999978401824063, 2.4837611153601877e-05];
q = state(13:end);
symbols = [l1 l2 l3 lox loy loz lbx lby lbz  th1 th2 th3 th4 xb yb psi];
vals = [groundParams(1:9), q];

pp = [p_1, p_2b, p_2, p_3, p_4, p_e];
pp_val = eval (subs (pp, symbols, vals));

axis equal
hold on
plot(pp_val(1,:),pp_val(3,:));
scatter(pp_val(1,1), pp_val(3,1), 'filled');
scatter(pp_val(1,2), pp_val(3,2), 'filled');
scatter(pp_val(1,3), pp_val(3,3), 'filled');
scatter(pp_val(1,4), pp_val(3,4), 'filled');
scatter(pp_val(1,5), pp_val(3,5), 'filled');
grid on
grid minor








