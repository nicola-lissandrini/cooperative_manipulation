%% UAV Model
clear all
close all
syms xv yv zv psi th1 th2
syms l1 l2 lbx lby lbz lox loy loz

x = [1; 0; 0];
y = [0; 1; 0];
z = [0; 0; 1];
zero = [0; 0; 0];

%% Frames

% From world to vehicle

R_w_v = rotz (psi);
o_w_v = [xv; yv; zv];
T_w_v = rt2tr (R_w_v, o_w_v);

% From vehicle to body
R_v_b = eye(3);
o_v_b = [lbx; lby; lbz];
T_v_b = rt2tr (R_v_b, o_v_b);

% From body to F0
R_b_0 = rotx(pi/2)*rotz(-pi/2);
o_b_0 = zero;
T_b_0 = rt2tr (R_b_0, o_b_0);

% From F0 to F1
% First link
% account for the first segment

R_0_j = rotz (th1);
o_0_j = l1 * R_0_j * x;
T_0_j = rt2tr (R_0_j, o_0_j);
% Account for second segment
R_j_1 = rotz(pi/2);
o_j_1 = lox * R_j_1 * x;
T_j_1 = rt2tr (R_j_1, o_j_1);
T_0_1 = T_0_j * T_j_1;

% From F1 to F2
% Second link
R_1_2 = rotz (th2);
o_1_2 = l2 * R_1_2 * x;
T_1_2 = rt2tr (R_1_2, o_1_2);

% From F2 to EEF
R_2_e = eye(3);
o_2_e = zero;
T_2_e = rt2tr (R_2_e, o_2_e);

% From EEF to object
R_e_o = eye(3);
o_e_o = [0; 0; 0];
T_e_o = rt2tr (R_e_o, o_e_o);

% Compute total tf's from W

T_v = T_w_v;
T_b = T_v * T_v_b;
T_0 = T_b * T_b_0;
T_1 = T_0 * T_0_1;
T_2 = T_1 * T_1_2;
T_e = T_2 * T_2_e;
T_o = T_e * T_e_o;
T_j = T_0 * T_0_j;

% R,t
[R_v, p_v] = tr2rt (T_v);
[R_b, p_b] = tr2rt (T_b);
[R_0, p_0] = tr2rt (T_0);
[R_1, p_1] = tr2rt (T_1);
[R_2, p_2] = tr2rt (T_2);
[R_e, p_e] = tr2rt (T_e);
[R_o, p_o] = tr2rt (T_o);
[R_0b, p_0b] = tr2rt (T_j);

%% Differential model
% In the world frame

% Control inputs
syms ux uy uz upsi uth1 uth2;

%% Linear velocity

% Vehicle frame velocity
% dot p_o = J_p_v [ux; uy; uz] + (*)
J_L_v = eye(3);

% Yaw velocity
% dot p_o =  z_v \times (p_o - p_v) * psi + (*)
z_v = eye(3) * z;
J_L_psi = cross (z_v, p_o - p_b);

% Joint 1 velocity
% dot p_o = z_0 \times (p_o - p_0) * uth1 + (*)
z_0 = R_0 * z;
J_L_1 = cross (z_0, p_o - p_0);

% Joint 2 velocity
% dot p_o = z_0 \times p_o * uth1 + (*)
z_1 = R_1 * z;
J_L_2 = cross (z_1,p_o - p_1);

% Linear jacobian 
J_L = simplify ([J_L_1, J_L_2, J_L_v, J_L_psi]);

%% Angular velocity

J_O_x = zero;
J_O_y = zero;
J_O_z = zero;
J_O_psi = z_v;
J_O_1 = z_0;
J_O_2 = z_1; 

J_O = simplify ([J_O_1, J_O_2, J_O_x, J_O_y, J_O_z, J_O_psi]);

% Total Jacobian
J_tot = [J_L; J_O];

%% Input to joints matrix

% outdated
B = blkdiag (eye(2), R_v, 1);


%% For direct kinematics
 makeItWork (p_o)

%% For the differential model
 makeItWork (J_tot)



 %% Test

aerialParams = csvread ('../config.sml/parameters.csv');
%state = [0.7913659547051668, -0.00031602107327960774, 0.2899724535570237, 0.0002896530534102215, -0.0021072447252018997, 0.9999977378078295, -0.0029197340204915505, 0.9999935155517119, 0.002108081539642183, -0.9999956956179152, -0.0029203380277469052, 0.0002834985810562518, -1.563797529967684, -0.004408345942823466, 0.9219059415523078, -0.00040206173386014035, 0.511533810692369, -3.138681194454324];
q = [ 0, 0, 0, 0 ,0, pi];
symbols = [l1 l2 lbx lby lbz lox loy loz  th1 th2 xv yv zv psi];
vals = [aerialParams(1:8), q];

pp = [p_b, p_0, p_0b, p_1, p_2, p_e];
pp_val = eval (subs (pp, symbols, vals));
R_b_val =  eval (subs (R_b, symbols, vals));
R_0_val =  eval (subs (R_0, symbols, vals));
R_0b_val =  eval (subs (R_0b, symbols, vals));
R_1_val =  eval (subs (R_1, symbols, vals));
R_2_val =  eval (subs (R_2, symbols, vals));

hold on
plot3(pp_val(1,:),pp_val(2,:),pp_val(3,:));
scatter3(pp_val(1,1), pp_val(2,1), pp_val(3,1),'r', 'filled');
scatter3(pp_val(1,2), pp_val(2,2),pp_val(3,2),'r', 'filled');
scatter3(pp_val(1,3), pp_val(2,3),pp_val(3,3),'g', 'filled');
scatter3(pp_val(1,4), pp_val(2,4),pp_val(3,4),'b', 'filled');
scatter3(pp_val(1,5), pp_val(2,5),pp_val(3,5),'k', 'filled');

refGraph ([pp_val(1,1), pp_val(2,1),pp_val(3,1)], R_b_val)
refGraph ([pp_val(1,2), pp_val(2,2),pp_val(3,2)], R_0_val)
refGraph ([pp_val(1,3), pp_val(2,3),pp_val(3,3)], R_0b_val)
refGraph ([pp_val(1,4), pp_val(2,4),pp_val(3,4)], R_1_val)
refGraph ([pp_val(1,5), pp_val(2,5),pp_val(3,5)], R_2_val)


axis equal
grid on
grid minor





function refGraph (p, rotm)
    l = 0.05;
    
    rx = quiver3 (0,0,0,0,0,0,'Autoscale','off','Color','green');
    ry = quiver3 (0,0,0,0,0,0,'Autoscale','off','Color','red');
    rz = quiver3 (0,0,0,0,0,0,'Autoscale','off','Color','black');

    
    
    rx.XData = p(1);
    rx.YData = p(2);
    rx.ZData = p(3);
    rx.UData = l*rotm(1,1);
    rx.VData = l*rotm(2,1);
    rx.WData = l*rotm(3,1);
    
    ry.XData = p(1);
    ry.YData = p(2);
    ry.ZData = p(3);
    ry.UData = l*rotm(1,2);
    ry.VData = l*rotm(2,2);
    ry.WData = l*rotm(3,2);
    
    rz.XData = p(1);
    rz.YData = p(2);
    rz.ZData = p(3);
    rz.UData = l*rotm(1,3);
    rz.VData = l*rotm(2,3);
    rz.WData = l*rotm(3,3);
end




