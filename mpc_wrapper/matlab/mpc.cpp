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
    BMatrix acadodata_M1;
    acadodata_M1.read( "mpc_data_acadodata_M1.txt" );
    BMatrix acadodata_M2;
    acadodata_M2.read( "mpc_data_acadodata_M2.txt" );
    Function acadodata_f1;
    acadodata_f1 << p_o1;
    acadodata_f1 << p_o2;
    acadodata_f1 << p_o3;
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
    DifferentialEquation acadodata_f3;
    acadodata_f3 << dot(p_o1) == ((-(cos((th2+th3))*cos(th4)*sin((psi_v+th1))-sin((psi_v+th1))*sin((th2+th3))*sin(th4))-2.00000000000000000000e+00*cos((th2+th3))*sin((psi_v+th1))-3.00000000000000000000e+00*cos(th2)*sin((psi_v+th1))-cos((th2+th3))*cos(th4)*sin((psi_v+th1))+sin((psi_v+th1))*sin((th2+th3))*sin(th4))*upsi+(-(cos((th2+th3))*cos(th4)*sin((psi_v+th1))-sin((psi_v+th1))*sin((th2+th3))*sin(th4))-2.00000000000000000000e+00*cos((th2+th3))*sin((psi_v+th1))-3.00000000000000000000e+00*cos(th2)*sin((psi_v+th1))-cos((th2+th3))*cos(th4)*sin((psi_v+th1))+sin((psi_v+th1))*sin((th2+th3))*sin(th4))*uth1+(-cos((psi_v+th1)))*(2.00000000000000000000e+00*sin((th2+th3))+3.00000000000000000000e+00*sin(th2)+sin((th2+th3+th4))+sin((th2+th3+th4)))*uth2+(-cos((psi_v+th1)))*(2.00000000000000000000e+00*sin((th2+th3))+sin((th2+th3+th4))+sin((th2+th3+th4)))*uth3+(-cos((psi_v+th1)))*(sin((th2+th3+th4))+sin((th2+th3+th4)))*uth4+ux);
    acadodata_f3 << dot(p_o2) == (((cos((psi_v+th1))*cos((th2+th3))*cos(th4)-cos((psi_v+th1))*sin((th2+th3))*sin(th4))+2.00000000000000000000e+00*cos((psi_v+th1))*cos((th2+th3))+3.00000000000000000000e+00*cos((psi_v+th1))*cos(th2)+cos((psi_v+th1))*cos((th2+th3))*cos(th4)-cos((psi_v+th1))*sin((th2+th3))*sin(th4))*upsi+((cos((psi_v+th1))*cos((th2+th3))*cos(th4)-cos((psi_v+th1))*sin((th2+th3))*sin(th4))+2.00000000000000000000e+00*cos((psi_v+th1))*cos((th2+th3))+3.00000000000000000000e+00*cos((psi_v+th1))*cos(th2)+cos((psi_v+th1))*cos((th2+th3))*cos(th4)-cos((psi_v+th1))*sin((th2+th3))*sin(th4))*uth1+(-sin((psi_v+th1)))*(2.00000000000000000000e+00*sin((th2+th3))+3.00000000000000000000e+00*sin(th2)+sin((th2+th3+th4))+sin((th2+th3+th4)))*uth2+(-sin((psi_v+th1)))*(2.00000000000000000000e+00*sin((th2+th3))+sin((th2+th3+th4))+sin((th2+th3+th4)))*uth3+(-sin((psi_v+th1)))*(sin((th2+th3+th4))+sin((th2+th3+th4)))*uth4+uy);
    acadodata_f3 << dot(p_o3) == ((2.00000000000000000000e+00*cos((th2+th3))+3.00000000000000000000e+00*cos(th2)+cos((th2+th3+th4))+cos((th2+th3+th4)))*uth2+(2.00000000000000000000e+00*cos((th2+th3))+cos((th2+th3+th4))+cos((th2+th3+th4)))*uth3+(cos((th2+th3+th4))+cos((th2+th3+th4)))*uth4);
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
    acadodata_f3 << dot(x_v) == ux;
    acadodata_f3 << dot(y_v) == uy;
    acadodata_f3 << dot(psi_v) == upsi;

    ocp1.setModel( acadodata_f3 );


    ocp1.setNU( 7 );
    ocp1.setNP( 0 );
    ocp1.setNOD( 9 );
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

