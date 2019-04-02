/*
*    This file is part of ACADO Toolkit.
*
*    ACADO Toolkit -- A Toolkit for Automatic Control and Dynamic Optimization.
*    Copyright (C) 2008-2009 by Boris Houska and Hans Joachim Ferreau, K.U.Leuven.
*    Developed within the Optimization in Engineering Center (OPTEC) under
*    supervision of Moritz Diehl. All rights reserved.
*
*    ACADO Toolkit is free software; you can redistribute it and/or
*    modify it under the terms of the GNU Lesser General Public
*    License as published by the Free Software Foundation; either
*    version 3 of the License, or (at your option) any later version.
*
*    ACADO Toolkit is distributed in the hope that it will be useful,
*    but WITHOUT ANY WARRANTY; without even the implied warranty of
*    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
*    Lesser General Public License for more details.
*
*    You should have received a copy of the GNU Lesser General Public
*    License along with ACADO Toolkit; if not, write to the Free Software
*    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*
*/


/**
*    Author David Ariens, Rien Quirynen
*    Date 2009-2013
*    http://www.acadotoolkit.org/matlab 
*/

#include <acado_optimal_control.hpp>
#include <acado_toolkit.hpp>
#include <acado/utils/matlab_acado_utils.hpp>

USING_NAMESPACE_ACADO

#include <mex.h>


void mexFunction( int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[] ) 
 { 
 
    MatlabConsoleStreamBuf mybuf;
    RedirectStream redirect(std::cout, mybuf);
    clearAllStaticCounters( ); 
 
    mexPrintf("\nACADO Toolkit for Matlab - Developed by David Ariens and Rien Quirynen, 2009-2013 \n"); 
    mexPrintf("Support available at http://www.acadotoolkit.org/matlab \n \n"); 

    if (nrhs != 0){ 
      mexErrMsgTxt("This problem expects 0 right hand side argument(s) since you have defined 0 MexInput(s)");
    } 
 
    TIME autotime;
    DifferentialState p_o1;
    DifferentialState p_o2;
    DifferentialState p_o3;
    DifferentialState R_e_vec1;
    DifferentialState R_e_vec2;
    DifferentialState R_e_vec3;
    DifferentialState R_e_vec4;
    DifferentialState R_e_vec5;
    DifferentialState R_e_vec6;
    DifferentialState R_e_vec7;
    DifferentialState R_e_vec8;
    DifferentialState R_e_vec9;
    DifferentialState th1;
    DifferentialState th2;
    DifferentialState th3;
    DifferentialState th4;
    DifferentialState x_v;
    DifferentialState y_v;
    DifferentialState psi_v;
    Control uth1;
    Control uth2;
    Control uth3;
    Control uth4;
    Control ux;
    Control uy;
    Control upsi;
    OnlineData R_w_des_vec1; 
    OnlineData R_w_des_vec2; 
    OnlineData R_w_des_vec3; 
    OnlineData R_w_des_vec4; 
    OnlineData R_w_des_vec5; 
    OnlineData R_w_des_vec6; 
    OnlineData R_w_des_vec7; 
    OnlineData R_w_des_vec8; 
    OnlineData R_w_des_vec9; 
    OnlineData v_o_11; 
    OnlineData v_o_12; 
    OnlineData v_o_13; 
    OnlineData a_1; 
    OnlineData b_1; 
    OnlineData c_1; 
    OnlineData v_o_21; 
    OnlineData v_o_22; 
    OnlineData v_o_23; 
    OnlineData a_2; 
    OnlineData b_2; 
    OnlineData c_2; 
    OnlineData keypts1; 
    OnlineData keypts2; 
    OnlineData keypts3; 
    BMatrix acadodata_M1;
    acadodata_M1.read( "mpc_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "mpc_data_acadodata_M2.txt" );
    Function acadodata_f1;
    acadodata_f1 << p_o1;
    acadodata_f1 << p_o2;
    acadodata_f1 << p_o3;
    acadodata_f1 << (1.00000000000000000000e+02*exp((-2.30258509299404536819e+01)*(-a_1+sqrt((pow((-v_o_11+x_v),2.00000000000000000000e+00)+pow((-v_o_12+y_v),2.00000000000000000000e+00)+pow((-v_o_13),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-2.30258509299404536819e+01)*(-a_2+sqrt((pow((-v_o_21+x_v),2.00000000000000000000e+00)+pow((-v_o_22+y_v),2.00000000000000000000e+00)+pow((-v_o_23),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-3.07011345732539417952e+01)*(-a_1+sqrt((pow((p_o1-v_o_11),2.00000000000000000000e+00)+pow((p_o2-v_o_12),2.00000000000000000000e+00)+pow((p_o3-v_o_13),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-3.07011345732539417952e+01)*(-a_2+sqrt((pow((p_o1-v_o_21),2.00000000000000000000e+00)+pow((p_o2-v_o_22),2.00000000000000000000e+00)+pow((p_o3-v_o_23),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-4.60517018598809073637e+01)*(-a_1+sqrt((pow(((1.41999999999999987343e-01*cos(th2)+4.83000000000000026534e-02*sin(th2))*cos((psi_v+th1))+1.41999999999999987343e-01*cos((psi_v+th1))*sin((th2+th3))-1.49999999999999994449e-02*sin(psi_v)+8.00000000000000016653e-02*cos(psi_v)-v_o_11+x_v),2.00000000000000000000e+00)+pow(((1.41999999999999987343e-01*cos(th2)+4.83000000000000026534e-02*sin(th2))*sin((psi_v+th1))+1.41999999999999987343e-01*sin((psi_v+th1))*sin((th2+th3))+1.49999999999999994449e-02*cos(psi_v)+8.00000000000000016653e-02*sin(psi_v)-v_o_12+y_v),2.00000000000000000000e+00)+pow((-1.41999999999999987343e-01*cos((th2+th3))+1.41999999999999987343e-01*sin(th2)+2.00000000000000011102e-01-4.83000000000000026534e-02*cos(th2)-v_o_13),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-4.60517018598809073637e+01)*(-a_2+sqrt((pow(((1.41999999999999987343e-01*cos(th2)+4.83000000000000026534e-02*sin(th2))*cos((psi_v+th1))+1.41999999999999987343e-01*cos((psi_v+th1))*sin((th2+th3))-1.49999999999999994449e-02*sin(psi_v)+8.00000000000000016653e-02*cos(psi_v)-v_o_21+x_v),2.00000000000000000000e+00)+pow(((1.41999999999999987343e-01*cos(th2)+4.83000000000000026534e-02*sin(th2))*sin((psi_v+th1))+1.41999999999999987343e-01*sin((psi_v+th1))*sin((th2+th3))+1.49999999999999994449e-02*cos(psi_v)+8.00000000000000016653e-02*sin(psi_v)-v_o_22+y_v),2.00000000000000000000e+00)+pow((-1.41999999999999987343e-01*cos((th2+th3))+1.41999999999999987343e-01*sin(th2)+2.00000000000000011102e-01-4.83000000000000026534e-02*cos(th2)-v_o_23),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-6.57881455141155839783e+01)*(-a_1+sqrt((pow(((-1.30000000000000004441e-01)-v_o_12+y_v),2.00000000000000000000e+00)+pow((-1.15000000000000004996e-01-v_o_11+x_v),2.00000000000000000000e+00)+pow((1.00000000000000005551e-01-v_o_13),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-6.57881455141155839783e+01)*(-a_1+sqrt((pow(((-1.30000000000000004441e-01)-v_o_12+y_v),2.00000000000000000000e+00)+pow((1.00000000000000005551e-01-v_o_13),2.00000000000000000000e+00)+pow((1.15000000000000004996e-01-v_o_11+x_v),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-6.57881455141155839783e+01)*(-a_1+sqrt((pow((-(-1.30000000000000004441e-01)-v_o_12+y_v),2.00000000000000000000e+00)+pow((-1.15000000000000004996e-01-v_o_11+x_v),2.00000000000000000000e+00)+pow((1.00000000000000005551e-01-v_o_13),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-6.57881455141155839783e+01)*(-a_1+sqrt((pow((-(-1.30000000000000004441e-01)-v_o_12+y_v),2.00000000000000000000e+00)+pow((1.00000000000000005551e-01-v_o_13),2.00000000000000000000e+00)+pow((1.15000000000000004996e-01-v_o_11+x_v),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-6.57881455141155839783e+01)*(-a_2+sqrt((pow(((-1.30000000000000004441e-01)-v_o_22+y_v),2.00000000000000000000e+00)+pow((-1.15000000000000004996e-01-v_o_21+x_v),2.00000000000000000000e+00)+pow((1.00000000000000005551e-01-v_o_23),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-6.57881455141155839783e+01)*(-a_2+sqrt((pow(((-1.30000000000000004441e-01)-v_o_22+y_v),2.00000000000000000000e+00)+pow((1.00000000000000005551e-01-v_o_23),2.00000000000000000000e+00)+pow((1.15000000000000004996e-01-v_o_21+x_v),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-6.57881455141155839783e+01)*(-a_2+sqrt((pow((-(-1.30000000000000004441e-01)-v_o_22+y_v),2.00000000000000000000e+00)+pow((-1.15000000000000004996e-01-v_o_21+x_v),2.00000000000000000000e+00)+pow((1.00000000000000005551e-01-v_o_23),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-6.57881455141155839783e+01)*(-a_2+sqrt((pow((-(-1.30000000000000004441e-01)-v_o_22+y_v),2.00000000000000000000e+00)+pow((1.00000000000000005551e-01-v_o_23),2.00000000000000000000e+00)+pow((1.15000000000000004996e-01-v_o_21+x_v),2.00000000000000000000e+00))))));
    acadodata_f1 << (-1.00000000000000000000e+00+R_e_vec1*R_w_des_vec1+R_e_vec2*R_w_des_vec2+R_e_vec3*R_w_des_vec3);
    acadodata_f1 << (R_e_vec4*R_w_des_vec1+R_e_vec5*R_w_des_vec2+R_e_vec6*R_w_des_vec3);
    acadodata_f1 << (R_e_vec7*R_w_des_vec1+R_e_vec8*R_w_des_vec2+R_e_vec9*R_w_des_vec3);
    acadodata_f1 << (R_e_vec1*R_w_des_vec4+R_e_vec2*R_w_des_vec5+R_e_vec3*R_w_des_vec6);
    acadodata_f1 << (-1.00000000000000000000e+00+R_e_vec4*R_w_des_vec4+R_e_vec5*R_w_des_vec5+R_e_vec6*R_w_des_vec6);
    acadodata_f1 << (R_e_vec7*R_w_des_vec4+R_e_vec8*R_w_des_vec5+R_e_vec9*R_w_des_vec6);
    acadodata_f1 << (R_e_vec1*R_w_des_vec7+R_e_vec2*R_w_des_vec8+R_e_vec3*R_w_des_vec9);
    acadodata_f1 << (R_e_vec4*R_w_des_vec7+R_e_vec5*R_w_des_vec8+R_e_vec6*R_w_des_vec9);
    acadodata_f1 << (-1.00000000000000000000e+00+R_e_vec7*R_w_des_vec7+R_e_vec8*R_w_des_vec8+R_e_vec9*R_w_des_vec9);
    acadodata_f1 << uth1;
    acadodata_f1 << uth2;
    acadodata_f1 << uth3;
    acadodata_f1 << uth4;
    acadodata_f1 << ux;
    acadodata_f1 << uy;
    acadodata_f1 << upsi;
    Function acadodata_f2;
    acadodata_f2 << p_o1;
    acadodata_f2 << p_o2;
    acadodata_f2 << p_o3;
    acadodata_f2 << (1.00000000000000000000e+02*exp((-2.30258509299404536819e+01)*(-a_1+sqrt((pow((-v_o_11+x_v),2.00000000000000000000e+00)+pow((-v_o_12+y_v),2.00000000000000000000e+00)+pow((-v_o_13),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-2.30258509299404536819e+01)*(-a_2+sqrt((pow((-v_o_21+x_v),2.00000000000000000000e+00)+pow((-v_o_22+y_v),2.00000000000000000000e+00)+pow((-v_o_23),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-3.07011345732539417952e+01)*(-a_1+sqrt((pow((p_o1-v_o_11),2.00000000000000000000e+00)+pow((p_o2-v_o_12),2.00000000000000000000e+00)+pow((p_o3-v_o_13),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-3.07011345732539417952e+01)*(-a_2+sqrt((pow((p_o1-v_o_21),2.00000000000000000000e+00)+pow((p_o2-v_o_22),2.00000000000000000000e+00)+pow((p_o3-v_o_23),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-4.60517018598809073637e+01)*(-a_1+sqrt((pow(((1.41999999999999987343e-01*cos(th2)+4.83000000000000026534e-02*sin(th2))*cos((psi_v+th1))+1.41999999999999987343e-01*cos((psi_v+th1))*sin((th2+th3))-1.49999999999999994449e-02*sin(psi_v)+8.00000000000000016653e-02*cos(psi_v)-v_o_11+x_v),2.00000000000000000000e+00)+pow(((1.41999999999999987343e-01*cos(th2)+4.83000000000000026534e-02*sin(th2))*sin((psi_v+th1))+1.41999999999999987343e-01*sin((psi_v+th1))*sin((th2+th3))+1.49999999999999994449e-02*cos(psi_v)+8.00000000000000016653e-02*sin(psi_v)-v_o_12+y_v),2.00000000000000000000e+00)+pow((-1.41999999999999987343e-01*cos((th2+th3))+1.41999999999999987343e-01*sin(th2)+2.00000000000000011102e-01-4.83000000000000026534e-02*cos(th2)-v_o_13),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-4.60517018598809073637e+01)*(-a_2+sqrt((pow(((1.41999999999999987343e-01*cos(th2)+4.83000000000000026534e-02*sin(th2))*cos((psi_v+th1))+1.41999999999999987343e-01*cos((psi_v+th1))*sin((th2+th3))-1.49999999999999994449e-02*sin(psi_v)+8.00000000000000016653e-02*cos(psi_v)-v_o_21+x_v),2.00000000000000000000e+00)+pow(((1.41999999999999987343e-01*cos(th2)+4.83000000000000026534e-02*sin(th2))*sin((psi_v+th1))+1.41999999999999987343e-01*sin((psi_v+th1))*sin((th2+th3))+1.49999999999999994449e-02*cos(psi_v)+8.00000000000000016653e-02*sin(psi_v)-v_o_22+y_v),2.00000000000000000000e+00)+pow((-1.41999999999999987343e-01*cos((th2+th3))+1.41999999999999987343e-01*sin(th2)+2.00000000000000011102e-01-4.83000000000000026534e-02*cos(th2)-v_o_23),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-6.57881455141155839783e+01)*(-a_1+sqrt((pow(((-1.30000000000000004441e-01)-v_o_12+y_v),2.00000000000000000000e+00)+pow((-1.15000000000000004996e-01-v_o_11+x_v),2.00000000000000000000e+00)+pow((1.00000000000000005551e-01-v_o_13),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-6.57881455141155839783e+01)*(-a_1+sqrt((pow(((-1.30000000000000004441e-01)-v_o_12+y_v),2.00000000000000000000e+00)+pow((1.00000000000000005551e-01-v_o_13),2.00000000000000000000e+00)+pow((1.15000000000000004996e-01-v_o_11+x_v),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-6.57881455141155839783e+01)*(-a_1+sqrt((pow((-(-1.30000000000000004441e-01)-v_o_12+y_v),2.00000000000000000000e+00)+pow((-1.15000000000000004996e-01-v_o_11+x_v),2.00000000000000000000e+00)+pow((1.00000000000000005551e-01-v_o_13),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-6.57881455141155839783e+01)*(-a_1+sqrt((pow((-(-1.30000000000000004441e-01)-v_o_12+y_v),2.00000000000000000000e+00)+pow((1.00000000000000005551e-01-v_o_13),2.00000000000000000000e+00)+pow((1.15000000000000004996e-01-v_o_11+x_v),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-6.57881455141155839783e+01)*(-a_2+sqrt((pow(((-1.30000000000000004441e-01)-v_o_22+y_v),2.00000000000000000000e+00)+pow((-1.15000000000000004996e-01-v_o_21+x_v),2.00000000000000000000e+00)+pow((1.00000000000000005551e-01-v_o_23),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-6.57881455141155839783e+01)*(-a_2+sqrt((pow(((-1.30000000000000004441e-01)-v_o_22+y_v),2.00000000000000000000e+00)+pow((1.00000000000000005551e-01-v_o_23),2.00000000000000000000e+00)+pow((1.15000000000000004996e-01-v_o_21+x_v),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-6.57881455141155839783e+01)*(-a_2+sqrt((pow((-(-1.30000000000000004441e-01)-v_o_22+y_v),2.00000000000000000000e+00)+pow((-1.15000000000000004996e-01-v_o_21+x_v),2.00000000000000000000e+00)+pow((1.00000000000000005551e-01-v_o_23),2.00000000000000000000e+00)))))+1.00000000000000000000e+02*exp((-6.57881455141155839783e+01)*(-a_2+sqrt((pow((-(-1.30000000000000004441e-01)-v_o_22+y_v),2.00000000000000000000e+00)+pow((1.00000000000000005551e-01-v_o_23),2.00000000000000000000e+00)+pow((1.15000000000000004996e-01-v_o_21+x_v),2.00000000000000000000e+00))))));
    acadodata_f2 << (-1.00000000000000000000e+00+R_e_vec1*R_w_des_vec1+R_e_vec2*R_w_des_vec2+R_e_vec3*R_w_des_vec3);
    acadodata_f2 << (R_e_vec4*R_w_des_vec1+R_e_vec5*R_w_des_vec2+R_e_vec6*R_w_des_vec3);
    acadodata_f2 << (R_e_vec7*R_w_des_vec1+R_e_vec8*R_w_des_vec2+R_e_vec9*R_w_des_vec3);
    acadodata_f2 << (R_e_vec1*R_w_des_vec4+R_e_vec2*R_w_des_vec5+R_e_vec3*R_w_des_vec6);
    acadodata_f2 << (-1.00000000000000000000e+00+R_e_vec4*R_w_des_vec4+R_e_vec5*R_w_des_vec5+R_e_vec6*R_w_des_vec6);
    acadodata_f2 << (R_e_vec7*R_w_des_vec4+R_e_vec8*R_w_des_vec5+R_e_vec9*R_w_des_vec6);
    acadodata_f2 << (R_e_vec1*R_w_des_vec7+R_e_vec2*R_w_des_vec8+R_e_vec3*R_w_des_vec9);
    acadodata_f2 << (R_e_vec4*R_w_des_vec7+R_e_vec5*R_w_des_vec8+R_e_vec6*R_w_des_vec9);
    acadodata_f2 << (-1.00000000000000000000e+00+R_e_vec7*R_w_des_vec7+R_e_vec8*R_w_des_vec8+R_e_vec9*R_w_des_vec9);
    OCP ocp1(0, 1, 10);
    ocp1.minimizeLSQ(acadodata_M1, acadodata_f1);
    ocp1.minimizeLSQEndTerm(acadodata_M2, acadodata_f2);
    ocp1.subjectTo((-2.50000000000000000000e-01) <= ux <= 2.50000000000000000000e-01);
    ocp1.subjectTo((-2.50000000000000000000e-01) <= uy <= 2.50000000000000000000e-01);
    ocp1.subjectTo((-2.50000000000000000000e-01) <= upsi <= 2.50000000000000000000e-01);
    ocp1.subjectTo((-7.00000000000000066613e-02) <= psi_v <= 7.00000000000000066613e-02);
    ocp1.subjectTo((-7.00000000000000066613e-02) <= uth1 <= 7.00000000000000066613e-02);
    ocp1.subjectTo((-2.99999999999999988898e-01) <= uth1 <= 2.99999999999999988898e-01);
    ocp1.subjectTo((-2.99999999999999988898e-01) <= uth2 <= 2.99999999999999988898e-01);
    ocp1.subjectTo((-2.99999999999999988898e-01) <= uth3 <= 2.99999999999999988898e-01);
    ocp1.subjectTo((-2.99999999999999988898e-01) <= uth4 <= 2.99999999999999988898e-01);
    ocp1.subjectTo((-2.51699999999999990408e+00) <= th1 <= 2.51699999999999990408e+00);
    ocp1.subjectTo((-1.04719755119659763132e+00) <= th2 <= 1.96349540849362069750e+00);
    ocp1.subjectTo((-1.57079632679489655800e+00) <= th3 <= 1.57079632679489655800e+00);
    ocp1.subjectTo((-1.74500000000000010658e+00) <= th4 <= 1.74500000000000010658e+00);
    ocp1.subjectTo((-5.99999999999999977796e-01) <= x_v);
    DifferentialEquation acadodata_f3;
    acadodata_f3 << dot(p_o1) == ((-sin((psi_v+th1)))*(4.83000000000000026534e-02*sin(th2)+1.14500000000000004552e-01*sin((th2+th3+th4))+1.41999999999999987343e-01*cos(th2)+1.41999999999999987343e-01*sin((th2+th3)))*upsi+(-sin((psi_v+th1)))*(4.83000000000000026534e-02*sin(th2)+1.14500000000000004552e-01*sin((th2+th3+th4))+1.41999999999999987343e-01*cos(th2)+1.41999999999999987343e-01*sin((th2+th3)))*uth1+(-sin(psi_v))*uy+(1.14500000000000004552e-01*cos((th2+th3+th4))+1.41999999999999987343e-01*cos((th2+th3)))*cos((psi_v+th1))*uth3+(4.83000000000000026534e-02*cos(th2)+1.14500000000000004552e-01*cos((th2+th3+th4))+1.41999999999999987343e-01*cos((th2+th3))-1.41999999999999987343e-01*sin(th2))*cos((psi_v+th1))*uth2+1.14500000000000004552e-01*cos((psi_v+th1))*cos((th2+th3+th4))*uth4+cos(psi_v)*ux);
    acadodata_f3 << dot(p_o2) == ((1.14500000000000004552e-01*cos((th2+th3+th4))+1.41999999999999987343e-01*cos((th2+th3)))*sin((psi_v+th1))*uth3+(4.83000000000000026534e-02*cos(th2)+1.14500000000000004552e-01*cos((th2+th3+th4))+1.41999999999999987343e-01*cos((th2+th3))-1.41999999999999987343e-01*sin(th2))*sin((psi_v+th1))*uth2+(4.83000000000000026534e-02*sin(th2)+1.14500000000000004552e-01*sin((th2+th3+th4))+1.41999999999999987343e-01*cos(th2)+1.41999999999999987343e-01*sin((th2+th3)))*cos((psi_v+th1))*upsi+(4.83000000000000026534e-02*sin(th2)+1.14500000000000004552e-01*sin((th2+th3+th4))+1.41999999999999987343e-01*cos(th2)+1.41999999999999987343e-01*sin((th2+th3)))*cos((psi_v+th1))*uth1+1.14500000000000004552e-01*cos((th2+th3+th4))*sin((psi_v+th1))*uth4+cos(psi_v)*uy+sin(psi_v)*ux);
    acadodata_f3 << dot(p_o3) == ((1.14500000000000004552e-01*sin((th2+th3+th4))+1.41999999999999987343e-01*sin((th2+th3)))*uth3+(4.83000000000000026534e-02*sin(th2)+1.14500000000000004552e-01*sin((th2+th3+th4))+1.41999999999999987343e-01*cos(th2)+1.41999999999999987343e-01*sin((th2+th3)))*uth2+1.14500000000000004552e-01*sin((th2+th3+th4))*uth4);
    acadodata_f3 << dot(R_e_vec1) == (((-cos((psi_v+th1)))*uth2+(-cos((psi_v+th1)))*uth3+(-cos((psi_v+th1)))*uth4)*R_e_vec3+(-upsi-uth1)*R_e_vec2);
    acadodata_f3 << dot(R_e_vec2) == ((-sin((psi_v+th1))*uth2-sin((psi_v+th1))*uth3-sin((psi_v+th1))*uth4)*R_e_vec3+(upsi+uth1)*R_e_vec1);
    acadodata_f3 << dot(R_e_vec3) == ((-(-cos((psi_v+th1)))*uth2-(-cos((psi_v+th1)))*uth3-(-cos((psi_v+th1)))*uth4)*R_e_vec1+(sin((psi_v+th1))*uth2+sin((psi_v+th1))*uth3+sin((psi_v+th1))*uth4)*R_e_vec2);
    acadodata_f3 << dot(R_e_vec4) == (((-cos((psi_v+th1)))*uth2+(-cos((psi_v+th1)))*uth3+(-cos((psi_v+th1)))*uth4)*R_e_vec6+(-upsi-uth1)*R_e_vec5);
    acadodata_f3 << dot(R_e_vec5) == ((-sin((psi_v+th1))*uth2-sin((psi_v+th1))*uth3-sin((psi_v+th1))*uth4)*R_e_vec6+(upsi+uth1)*R_e_vec4);
    acadodata_f3 << dot(R_e_vec6) == ((-(-cos((psi_v+th1)))*uth2-(-cos((psi_v+th1)))*uth3-(-cos((psi_v+th1)))*uth4)*R_e_vec4+(sin((psi_v+th1))*uth2+sin((psi_v+th1))*uth3+sin((psi_v+th1))*uth4)*R_e_vec5);
    acadodata_f3 << dot(R_e_vec7) == (((-cos((psi_v+th1)))*uth2+(-cos((psi_v+th1)))*uth3+(-cos((psi_v+th1)))*uth4)*R_e_vec9+(-upsi-uth1)*R_e_vec8);
    acadodata_f3 << dot(R_e_vec8) == ((-sin((psi_v+th1))*uth2-sin((psi_v+th1))*uth3-sin((psi_v+th1))*uth4)*R_e_vec9+(upsi+uth1)*R_e_vec7);
    acadodata_f3 << dot(R_e_vec9) == ((-(-cos((psi_v+th1)))*uth2-(-cos((psi_v+th1)))*uth3-(-cos((psi_v+th1)))*uth4)*R_e_vec7+(sin((psi_v+th1))*uth2+sin((psi_v+th1))*uth3+sin((psi_v+th1))*uth4)*R_e_vec8);
    acadodata_f3 << dot(th1) == uth1;
    acadodata_f3 << dot(th2) == uth2;
    acadodata_f3 << dot(th3) == uth3;
    acadodata_f3 << dot(th4) == uth4;
    acadodata_f3 << dot(x_v) == ((-sin(psi_v))*uy+cos(psi_v)*ux);
    acadodata_f3 << dot(y_v) == (cos(psi_v)*uy+sin(psi_v)*ux);
    acadodata_f3 << dot(psi_v) == upsi;

    ocp1.setModel( acadodata_f3 );


    ocp1.setNU( 7 );
    ocp1.setNP( 0 );
    ocp1.setNOD( 24 );
    OCPexport ExportModule1( ocp1 );
    ExportModule1.set( GENERATE_MATLAB_INTERFACE, 1 );
    uint options_flag;
    options_flag = ExportModule1.set( HESSIAN_APPROXIMATION, GAUSS_NEWTON );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: HESSIAN_APPROXIMATION");
    options_flag = ExportModule1.set( DISCRETIZATION_TYPE, MULTIPLE_SHOOTING );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: DISCRETIZATION_TYPE");
    options_flag = ExportModule1.set( SPARSE_QP_SOLUTION, FULL_CONDENSING_N2 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: SPARSE_QP_SOLUTION");
    options_flag = ExportModule1.set( INTEGRATOR_TYPE, INT_IRK_GL2 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: INTEGRATOR_TYPE");
    options_flag = ExportModule1.set( NUM_INTEGRATOR_STEPS, 20 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: NUM_INTEGRATOR_STEPS");
    options_flag = ExportModule1.set( QP_SOLVER, QP_QPOASES3 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: QP_SOLVER");
    options_flag = ExportModule1.set( LEVENBERG_MARQUARDT, 1.000000E-04 );
    if(options_flag != 0) mexErrMsgTxt("ACADO export failed when setting the following option: LEVENBERG_MARQUARDT");
    uint export_flag;
    export_flag = ExportModule1.exportCode( "../src/export_MPC" );
    if(export_flag != 0) mexErrMsgTxt("ACADO export failed because of the above error(s)!");


    clearAllStaticCounters( ); 
 
} 

