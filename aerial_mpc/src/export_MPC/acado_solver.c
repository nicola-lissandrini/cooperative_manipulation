/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"




/******************************************************************************/
/*                                                                            */
/* ACADO code generation                                                      */
/*                                                                            */
/******************************************************************************/


int acado_modelSimulation(  )
{
int ret;

int lRun1;
int lRun2;
ret = 0;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.state[0] = acadoVariables.x[lRun1 * 18];
acadoWorkspace.state[1] = acadoVariables.x[lRun1 * 18 + 1];
acadoWorkspace.state[2] = acadoVariables.x[lRun1 * 18 + 2];
acadoWorkspace.state[3] = acadoVariables.x[lRun1 * 18 + 3];
acadoWorkspace.state[4] = acadoVariables.x[lRun1 * 18 + 4];
acadoWorkspace.state[5] = acadoVariables.x[lRun1 * 18 + 5];
acadoWorkspace.state[6] = acadoVariables.x[lRun1 * 18 + 6];
acadoWorkspace.state[7] = acadoVariables.x[lRun1 * 18 + 7];
acadoWorkspace.state[8] = acadoVariables.x[lRun1 * 18 + 8];
acadoWorkspace.state[9] = acadoVariables.x[lRun1 * 18 + 9];
acadoWorkspace.state[10] = acadoVariables.x[lRun1 * 18 + 10];
acadoWorkspace.state[11] = acadoVariables.x[lRun1 * 18 + 11];
acadoWorkspace.state[12] = acadoVariables.x[lRun1 * 18 + 12];
acadoWorkspace.state[13] = acadoVariables.x[lRun1 * 18 + 13];
acadoWorkspace.state[14] = acadoVariables.x[lRun1 * 18 + 14];
acadoWorkspace.state[15] = acadoVariables.x[lRun1 * 18 + 15];
acadoWorkspace.state[16] = acadoVariables.x[lRun1 * 18 + 16];
acadoWorkspace.state[17] = acadoVariables.x[lRun1 * 18 + 17];

acadoWorkspace.state[450] = acadoVariables.u[lRun1 * 6];
acadoWorkspace.state[451] = acadoVariables.u[lRun1 * 6 + 1];
acadoWorkspace.state[452] = acadoVariables.u[lRun1 * 6 + 2];
acadoWorkspace.state[453] = acadoVariables.u[lRun1 * 6 + 3];
acadoWorkspace.state[454] = acadoVariables.u[lRun1 * 6 + 4];
acadoWorkspace.state[455] = acadoVariables.u[lRun1 * 6 + 5];
acadoWorkspace.state[456] = acadoVariables.od[lRun1 * 9];
acadoWorkspace.state[457] = acadoVariables.od[lRun1 * 9 + 1];
acadoWorkspace.state[458] = acadoVariables.od[lRun1 * 9 + 2];
acadoWorkspace.state[459] = acadoVariables.od[lRun1 * 9 + 3];
acadoWorkspace.state[460] = acadoVariables.od[lRun1 * 9 + 4];
acadoWorkspace.state[461] = acadoVariables.od[lRun1 * 9 + 5];
acadoWorkspace.state[462] = acadoVariables.od[lRun1 * 9 + 6];
acadoWorkspace.state[463] = acadoVariables.od[lRun1 * 9 + 7];
acadoWorkspace.state[464] = acadoVariables.od[lRun1 * 9 + 8];

ret = acado_integrate(acadoWorkspace.state, 1);

acadoWorkspace.d[lRun1 * 18] = acadoWorkspace.state[0] - acadoVariables.x[lRun1 * 18 + 18];
acadoWorkspace.d[lRun1 * 18 + 1] = acadoWorkspace.state[1] - acadoVariables.x[lRun1 * 18 + 19];
acadoWorkspace.d[lRun1 * 18 + 2] = acadoWorkspace.state[2] - acadoVariables.x[lRun1 * 18 + 20];
acadoWorkspace.d[lRun1 * 18 + 3] = acadoWorkspace.state[3] - acadoVariables.x[lRun1 * 18 + 21];
acadoWorkspace.d[lRun1 * 18 + 4] = acadoWorkspace.state[4] - acadoVariables.x[lRun1 * 18 + 22];
acadoWorkspace.d[lRun1 * 18 + 5] = acadoWorkspace.state[5] - acadoVariables.x[lRun1 * 18 + 23];
acadoWorkspace.d[lRun1 * 18 + 6] = acadoWorkspace.state[6] - acadoVariables.x[lRun1 * 18 + 24];
acadoWorkspace.d[lRun1 * 18 + 7] = acadoWorkspace.state[7] - acadoVariables.x[lRun1 * 18 + 25];
acadoWorkspace.d[lRun1 * 18 + 8] = acadoWorkspace.state[8] - acadoVariables.x[lRun1 * 18 + 26];
acadoWorkspace.d[lRun1 * 18 + 9] = acadoWorkspace.state[9] - acadoVariables.x[lRun1 * 18 + 27];
acadoWorkspace.d[lRun1 * 18 + 10] = acadoWorkspace.state[10] - acadoVariables.x[lRun1 * 18 + 28];
acadoWorkspace.d[lRun1 * 18 + 11] = acadoWorkspace.state[11] - acadoVariables.x[lRun1 * 18 + 29];
acadoWorkspace.d[lRun1 * 18 + 12] = acadoWorkspace.state[12] - acadoVariables.x[lRun1 * 18 + 30];
acadoWorkspace.d[lRun1 * 18 + 13] = acadoWorkspace.state[13] - acadoVariables.x[lRun1 * 18 + 31];
acadoWorkspace.d[lRun1 * 18 + 14] = acadoWorkspace.state[14] - acadoVariables.x[lRun1 * 18 + 32];
acadoWorkspace.d[lRun1 * 18 + 15] = acadoWorkspace.state[15] - acadoVariables.x[lRun1 * 18 + 33];
acadoWorkspace.d[lRun1 * 18 + 16] = acadoWorkspace.state[16] - acadoVariables.x[lRun1 * 18 + 34];
acadoWorkspace.d[lRun1 * 18 + 17] = acadoWorkspace.state[17] - acadoVariables.x[lRun1 * 18 + 35];

for (lRun2 = 0; lRun2 < 324; ++lRun2)
acadoWorkspace.evGx[(0) + ((lRun2) + (lRun1 * 324))] = acadoWorkspace.state[lRun2 + 18];


acadoWorkspace.evGu[lRun1 * 108] = acadoWorkspace.state[342];
acadoWorkspace.evGu[lRun1 * 108 + 1] = acadoWorkspace.state[343];
acadoWorkspace.evGu[lRun1 * 108 + 2] = acadoWorkspace.state[344];
acadoWorkspace.evGu[lRun1 * 108 + 3] = acadoWorkspace.state[345];
acadoWorkspace.evGu[lRun1 * 108 + 4] = acadoWorkspace.state[346];
acadoWorkspace.evGu[lRun1 * 108 + 5] = acadoWorkspace.state[347];
acadoWorkspace.evGu[lRun1 * 108 + 6] = acadoWorkspace.state[348];
acadoWorkspace.evGu[lRun1 * 108 + 7] = acadoWorkspace.state[349];
acadoWorkspace.evGu[lRun1 * 108 + 8] = acadoWorkspace.state[350];
acadoWorkspace.evGu[lRun1 * 108 + 9] = acadoWorkspace.state[351];
acadoWorkspace.evGu[lRun1 * 108 + 10] = acadoWorkspace.state[352];
acadoWorkspace.evGu[lRun1 * 108 + 11] = acadoWorkspace.state[353];
acadoWorkspace.evGu[lRun1 * 108 + 12] = acadoWorkspace.state[354];
acadoWorkspace.evGu[lRun1 * 108 + 13] = acadoWorkspace.state[355];
acadoWorkspace.evGu[lRun1 * 108 + 14] = acadoWorkspace.state[356];
acadoWorkspace.evGu[lRun1 * 108 + 15] = acadoWorkspace.state[357];
acadoWorkspace.evGu[lRun1 * 108 + 16] = acadoWorkspace.state[358];
acadoWorkspace.evGu[lRun1 * 108 + 17] = acadoWorkspace.state[359];
acadoWorkspace.evGu[lRun1 * 108 + 18] = acadoWorkspace.state[360];
acadoWorkspace.evGu[lRun1 * 108 + 19] = acadoWorkspace.state[361];
acadoWorkspace.evGu[lRun1 * 108 + 20] = acadoWorkspace.state[362];
acadoWorkspace.evGu[lRun1 * 108 + 21] = acadoWorkspace.state[363];
acadoWorkspace.evGu[lRun1 * 108 + 22] = acadoWorkspace.state[364];
acadoWorkspace.evGu[lRun1 * 108 + 23] = acadoWorkspace.state[365];
acadoWorkspace.evGu[lRun1 * 108 + 24] = acadoWorkspace.state[366];
acadoWorkspace.evGu[lRun1 * 108 + 25] = acadoWorkspace.state[367];
acadoWorkspace.evGu[lRun1 * 108 + 26] = acadoWorkspace.state[368];
acadoWorkspace.evGu[lRun1 * 108 + 27] = acadoWorkspace.state[369];
acadoWorkspace.evGu[lRun1 * 108 + 28] = acadoWorkspace.state[370];
acadoWorkspace.evGu[lRun1 * 108 + 29] = acadoWorkspace.state[371];
acadoWorkspace.evGu[lRun1 * 108 + 30] = acadoWorkspace.state[372];
acadoWorkspace.evGu[lRun1 * 108 + 31] = acadoWorkspace.state[373];
acadoWorkspace.evGu[lRun1 * 108 + 32] = acadoWorkspace.state[374];
acadoWorkspace.evGu[lRun1 * 108 + 33] = acadoWorkspace.state[375];
acadoWorkspace.evGu[lRun1 * 108 + 34] = acadoWorkspace.state[376];
acadoWorkspace.evGu[lRun1 * 108 + 35] = acadoWorkspace.state[377];
acadoWorkspace.evGu[lRun1 * 108 + 36] = acadoWorkspace.state[378];
acadoWorkspace.evGu[lRun1 * 108 + 37] = acadoWorkspace.state[379];
acadoWorkspace.evGu[lRun1 * 108 + 38] = acadoWorkspace.state[380];
acadoWorkspace.evGu[lRun1 * 108 + 39] = acadoWorkspace.state[381];
acadoWorkspace.evGu[lRun1 * 108 + 40] = acadoWorkspace.state[382];
acadoWorkspace.evGu[lRun1 * 108 + 41] = acadoWorkspace.state[383];
acadoWorkspace.evGu[lRun1 * 108 + 42] = acadoWorkspace.state[384];
acadoWorkspace.evGu[lRun1 * 108 + 43] = acadoWorkspace.state[385];
acadoWorkspace.evGu[lRun1 * 108 + 44] = acadoWorkspace.state[386];
acadoWorkspace.evGu[lRun1 * 108 + 45] = acadoWorkspace.state[387];
acadoWorkspace.evGu[lRun1 * 108 + 46] = acadoWorkspace.state[388];
acadoWorkspace.evGu[lRun1 * 108 + 47] = acadoWorkspace.state[389];
acadoWorkspace.evGu[lRun1 * 108 + 48] = acadoWorkspace.state[390];
acadoWorkspace.evGu[lRun1 * 108 + 49] = acadoWorkspace.state[391];
acadoWorkspace.evGu[lRun1 * 108 + 50] = acadoWorkspace.state[392];
acadoWorkspace.evGu[lRun1 * 108 + 51] = acadoWorkspace.state[393];
acadoWorkspace.evGu[lRun1 * 108 + 52] = acadoWorkspace.state[394];
acadoWorkspace.evGu[lRun1 * 108 + 53] = acadoWorkspace.state[395];
acadoWorkspace.evGu[lRun1 * 108 + 54] = acadoWorkspace.state[396];
acadoWorkspace.evGu[lRun1 * 108 + 55] = acadoWorkspace.state[397];
acadoWorkspace.evGu[lRun1 * 108 + 56] = acadoWorkspace.state[398];
acadoWorkspace.evGu[lRun1 * 108 + 57] = acadoWorkspace.state[399];
acadoWorkspace.evGu[lRun1 * 108 + 58] = acadoWorkspace.state[400];
acadoWorkspace.evGu[lRun1 * 108 + 59] = acadoWorkspace.state[401];
acadoWorkspace.evGu[lRun1 * 108 + 60] = acadoWorkspace.state[402];
acadoWorkspace.evGu[lRun1 * 108 + 61] = acadoWorkspace.state[403];
acadoWorkspace.evGu[lRun1 * 108 + 62] = acadoWorkspace.state[404];
acadoWorkspace.evGu[lRun1 * 108 + 63] = acadoWorkspace.state[405];
acadoWorkspace.evGu[lRun1 * 108 + 64] = acadoWorkspace.state[406];
acadoWorkspace.evGu[lRun1 * 108 + 65] = acadoWorkspace.state[407];
acadoWorkspace.evGu[lRun1 * 108 + 66] = acadoWorkspace.state[408];
acadoWorkspace.evGu[lRun1 * 108 + 67] = acadoWorkspace.state[409];
acadoWorkspace.evGu[lRun1 * 108 + 68] = acadoWorkspace.state[410];
acadoWorkspace.evGu[lRun1 * 108 + 69] = acadoWorkspace.state[411];
acadoWorkspace.evGu[lRun1 * 108 + 70] = acadoWorkspace.state[412];
acadoWorkspace.evGu[lRun1 * 108 + 71] = acadoWorkspace.state[413];
acadoWorkspace.evGu[lRun1 * 108 + 72] = acadoWorkspace.state[414];
acadoWorkspace.evGu[lRun1 * 108 + 73] = acadoWorkspace.state[415];
acadoWorkspace.evGu[lRun1 * 108 + 74] = acadoWorkspace.state[416];
acadoWorkspace.evGu[lRun1 * 108 + 75] = acadoWorkspace.state[417];
acadoWorkspace.evGu[lRun1 * 108 + 76] = acadoWorkspace.state[418];
acadoWorkspace.evGu[lRun1 * 108 + 77] = acadoWorkspace.state[419];
acadoWorkspace.evGu[lRun1 * 108 + 78] = acadoWorkspace.state[420];
acadoWorkspace.evGu[lRun1 * 108 + 79] = acadoWorkspace.state[421];
acadoWorkspace.evGu[lRun1 * 108 + 80] = acadoWorkspace.state[422];
acadoWorkspace.evGu[lRun1 * 108 + 81] = acadoWorkspace.state[423];
acadoWorkspace.evGu[lRun1 * 108 + 82] = acadoWorkspace.state[424];
acadoWorkspace.evGu[lRun1 * 108 + 83] = acadoWorkspace.state[425];
acadoWorkspace.evGu[lRun1 * 108 + 84] = acadoWorkspace.state[426];
acadoWorkspace.evGu[lRun1 * 108 + 85] = acadoWorkspace.state[427];
acadoWorkspace.evGu[lRun1 * 108 + 86] = acadoWorkspace.state[428];
acadoWorkspace.evGu[lRun1 * 108 + 87] = acadoWorkspace.state[429];
acadoWorkspace.evGu[lRun1 * 108 + 88] = acadoWorkspace.state[430];
acadoWorkspace.evGu[lRun1 * 108 + 89] = acadoWorkspace.state[431];
acadoWorkspace.evGu[lRun1 * 108 + 90] = acadoWorkspace.state[432];
acadoWorkspace.evGu[lRun1 * 108 + 91] = acadoWorkspace.state[433];
acadoWorkspace.evGu[lRun1 * 108 + 92] = acadoWorkspace.state[434];
acadoWorkspace.evGu[lRun1 * 108 + 93] = acadoWorkspace.state[435];
acadoWorkspace.evGu[lRun1 * 108 + 94] = acadoWorkspace.state[436];
acadoWorkspace.evGu[lRun1 * 108 + 95] = acadoWorkspace.state[437];
acadoWorkspace.evGu[lRun1 * 108 + 96] = acadoWorkspace.state[438];
acadoWorkspace.evGu[lRun1 * 108 + 97] = acadoWorkspace.state[439];
acadoWorkspace.evGu[lRun1 * 108 + 98] = acadoWorkspace.state[440];
acadoWorkspace.evGu[lRun1 * 108 + 99] = acadoWorkspace.state[441];
acadoWorkspace.evGu[lRun1 * 108 + 100] = acadoWorkspace.state[442];
acadoWorkspace.evGu[lRun1 * 108 + 101] = acadoWorkspace.state[443];
acadoWorkspace.evGu[lRun1 * 108 + 102] = acadoWorkspace.state[444];
acadoWorkspace.evGu[lRun1 * 108 + 103] = acadoWorkspace.state[445];
acadoWorkspace.evGu[lRun1 * 108 + 104] = acadoWorkspace.state[446];
acadoWorkspace.evGu[lRun1 * 108 + 105] = acadoWorkspace.state[447];
acadoWorkspace.evGu[lRun1 * 108 + 106] = acadoWorkspace.state[448];
acadoWorkspace.evGu[lRun1 * 108 + 107] = acadoWorkspace.state[449];
}
return ret;
}

void acado_evaluateLSQ(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 18;
const real_t* od = in + 24;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = (real_t)(0.0000000000000000e+00);
out[4] = ((((real_t)(-1.0000000000000000e+00)+(xd[3]*od[0]))+(xd[4]*od[1]))+(xd[5]*od[2]));
out[5] = (((xd[6]*od[0])+(xd[7]*od[1]))+(xd[8]*od[2]));
out[6] = (((xd[9]*od[0])+(xd[10]*od[1]))+(xd[11]*od[2]));
out[7] = (((xd[3]*od[3])+(xd[4]*od[4]))+(xd[5]*od[5]));
out[8] = ((((real_t)(-1.0000000000000000e+00)+(xd[6]*od[3]))+(xd[7]*od[4]))+(xd[8]*od[5]));
out[9] = (((xd[9]*od[3])+(xd[10]*od[4]))+(xd[11]*od[5]));
out[10] = (((xd[3]*od[6])+(xd[4]*od[7]))+(xd[5]*od[8]));
out[11] = (((xd[6]*od[6])+(xd[7]*od[7]))+(xd[8]*od[8]));
out[12] = ((((real_t)(-1.0000000000000000e+00)+(xd[9]*od[6]))+(xd[10]*od[7]))+(xd[11]*od[8]));
out[13] = u[0];
out[14] = u[1];
out[15] = u[2];
out[16] = u[3];
out[17] = u[4];
out[18] = u[5];
out[19] = (real_t)(1.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(1.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = (real_t)(0.0000000000000000e+00);
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = (real_t)(1.0000000000000000e+00);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(0.0000000000000000e+00);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(0.0000000000000000e+00);
out[69] = (real_t)(0.0000000000000000e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = (real_t)(0.0000000000000000e+00);
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = (real_t)(0.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = (real_t)(0.0000000000000000e+00);
out[78] = (real_t)(0.0000000000000000e+00);
out[79] = (real_t)(0.0000000000000000e+00);
out[80] = (real_t)(0.0000000000000000e+00);
out[81] = (real_t)(0.0000000000000000e+00);
out[82] = (real_t)(0.0000000000000000e+00);
out[83] = (real_t)(0.0000000000000000e+00);
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(0.0000000000000000e+00);
out[86] = (real_t)(0.0000000000000000e+00);
out[87] = (real_t)(0.0000000000000000e+00);
out[88] = (real_t)(0.0000000000000000e+00);
out[89] = (real_t)(0.0000000000000000e+00);
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = (real_t)(0.0000000000000000e+00);
out[92] = (real_t)(0.0000000000000000e+00);
out[93] = (real_t)(0.0000000000000000e+00);
out[94] = od[0];
out[95] = od[1];
out[96] = od[2];
out[97] = (real_t)(0.0000000000000000e+00);
out[98] = (real_t)(0.0000000000000000e+00);
out[99] = (real_t)(0.0000000000000000e+00);
out[100] = (real_t)(0.0000000000000000e+00);
out[101] = (real_t)(0.0000000000000000e+00);
out[102] = (real_t)(0.0000000000000000e+00);
out[103] = (real_t)(0.0000000000000000e+00);
out[104] = (real_t)(0.0000000000000000e+00);
out[105] = (real_t)(0.0000000000000000e+00);
out[106] = (real_t)(0.0000000000000000e+00);
out[107] = (real_t)(0.0000000000000000e+00);
out[108] = (real_t)(0.0000000000000000e+00);
out[109] = (real_t)(0.0000000000000000e+00);
out[110] = (real_t)(0.0000000000000000e+00);
out[111] = (real_t)(0.0000000000000000e+00);
out[112] = (real_t)(0.0000000000000000e+00);
out[113] = (real_t)(0.0000000000000000e+00);
out[114] = (real_t)(0.0000000000000000e+00);
out[115] = od[0];
out[116] = od[1];
out[117] = od[2];
out[118] = (real_t)(0.0000000000000000e+00);
out[119] = (real_t)(0.0000000000000000e+00);
out[120] = (real_t)(0.0000000000000000e+00);
out[121] = (real_t)(0.0000000000000000e+00);
out[122] = (real_t)(0.0000000000000000e+00);
out[123] = (real_t)(0.0000000000000000e+00);
out[124] = (real_t)(0.0000000000000000e+00);
out[125] = (real_t)(0.0000000000000000e+00);
out[126] = (real_t)(0.0000000000000000e+00);
out[127] = (real_t)(0.0000000000000000e+00);
out[128] = (real_t)(0.0000000000000000e+00);
out[129] = (real_t)(0.0000000000000000e+00);
out[130] = (real_t)(0.0000000000000000e+00);
out[131] = (real_t)(0.0000000000000000e+00);
out[132] = (real_t)(0.0000000000000000e+00);
out[133] = (real_t)(0.0000000000000000e+00);
out[134] = (real_t)(0.0000000000000000e+00);
out[135] = (real_t)(0.0000000000000000e+00);
out[136] = od[0];
out[137] = od[1];
out[138] = od[2];
out[139] = (real_t)(0.0000000000000000e+00);
out[140] = (real_t)(0.0000000000000000e+00);
out[141] = (real_t)(0.0000000000000000e+00);
out[142] = (real_t)(0.0000000000000000e+00);
out[143] = (real_t)(0.0000000000000000e+00);
out[144] = (real_t)(0.0000000000000000e+00);
out[145] = (real_t)(0.0000000000000000e+00);
out[146] = (real_t)(0.0000000000000000e+00);
out[147] = (real_t)(0.0000000000000000e+00);
out[148] = od[3];
out[149] = od[4];
out[150] = od[5];
out[151] = (real_t)(0.0000000000000000e+00);
out[152] = (real_t)(0.0000000000000000e+00);
out[153] = (real_t)(0.0000000000000000e+00);
out[154] = (real_t)(0.0000000000000000e+00);
out[155] = (real_t)(0.0000000000000000e+00);
out[156] = (real_t)(0.0000000000000000e+00);
out[157] = (real_t)(0.0000000000000000e+00);
out[158] = (real_t)(0.0000000000000000e+00);
out[159] = (real_t)(0.0000000000000000e+00);
out[160] = (real_t)(0.0000000000000000e+00);
out[161] = (real_t)(0.0000000000000000e+00);
out[162] = (real_t)(0.0000000000000000e+00);
out[163] = (real_t)(0.0000000000000000e+00);
out[164] = (real_t)(0.0000000000000000e+00);
out[165] = (real_t)(0.0000000000000000e+00);
out[166] = (real_t)(0.0000000000000000e+00);
out[167] = (real_t)(0.0000000000000000e+00);
out[168] = (real_t)(0.0000000000000000e+00);
out[169] = od[3];
out[170] = od[4];
out[171] = od[5];
out[172] = (real_t)(0.0000000000000000e+00);
out[173] = (real_t)(0.0000000000000000e+00);
out[174] = (real_t)(0.0000000000000000e+00);
out[175] = (real_t)(0.0000000000000000e+00);
out[176] = (real_t)(0.0000000000000000e+00);
out[177] = (real_t)(0.0000000000000000e+00);
out[178] = (real_t)(0.0000000000000000e+00);
out[179] = (real_t)(0.0000000000000000e+00);
out[180] = (real_t)(0.0000000000000000e+00);
out[181] = (real_t)(0.0000000000000000e+00);
out[182] = (real_t)(0.0000000000000000e+00);
out[183] = (real_t)(0.0000000000000000e+00);
out[184] = (real_t)(0.0000000000000000e+00);
out[185] = (real_t)(0.0000000000000000e+00);
out[186] = (real_t)(0.0000000000000000e+00);
out[187] = (real_t)(0.0000000000000000e+00);
out[188] = (real_t)(0.0000000000000000e+00);
out[189] = (real_t)(0.0000000000000000e+00);
out[190] = od[3];
out[191] = od[4];
out[192] = od[5];
out[193] = (real_t)(0.0000000000000000e+00);
out[194] = (real_t)(0.0000000000000000e+00);
out[195] = (real_t)(0.0000000000000000e+00);
out[196] = (real_t)(0.0000000000000000e+00);
out[197] = (real_t)(0.0000000000000000e+00);
out[198] = (real_t)(0.0000000000000000e+00);
out[199] = (real_t)(0.0000000000000000e+00);
out[200] = (real_t)(0.0000000000000000e+00);
out[201] = (real_t)(0.0000000000000000e+00);
out[202] = od[6];
out[203] = od[7];
out[204] = od[8];
out[205] = (real_t)(0.0000000000000000e+00);
out[206] = (real_t)(0.0000000000000000e+00);
out[207] = (real_t)(0.0000000000000000e+00);
out[208] = (real_t)(0.0000000000000000e+00);
out[209] = (real_t)(0.0000000000000000e+00);
out[210] = (real_t)(0.0000000000000000e+00);
out[211] = (real_t)(0.0000000000000000e+00);
out[212] = (real_t)(0.0000000000000000e+00);
out[213] = (real_t)(0.0000000000000000e+00);
out[214] = (real_t)(0.0000000000000000e+00);
out[215] = (real_t)(0.0000000000000000e+00);
out[216] = (real_t)(0.0000000000000000e+00);
out[217] = (real_t)(0.0000000000000000e+00);
out[218] = (real_t)(0.0000000000000000e+00);
out[219] = (real_t)(0.0000000000000000e+00);
out[220] = (real_t)(0.0000000000000000e+00);
out[221] = (real_t)(0.0000000000000000e+00);
out[222] = (real_t)(0.0000000000000000e+00);
out[223] = od[6];
out[224] = od[7];
out[225] = od[8];
out[226] = (real_t)(0.0000000000000000e+00);
out[227] = (real_t)(0.0000000000000000e+00);
out[228] = (real_t)(0.0000000000000000e+00);
out[229] = (real_t)(0.0000000000000000e+00);
out[230] = (real_t)(0.0000000000000000e+00);
out[231] = (real_t)(0.0000000000000000e+00);
out[232] = (real_t)(0.0000000000000000e+00);
out[233] = (real_t)(0.0000000000000000e+00);
out[234] = (real_t)(0.0000000000000000e+00);
out[235] = (real_t)(0.0000000000000000e+00);
out[236] = (real_t)(0.0000000000000000e+00);
out[237] = (real_t)(0.0000000000000000e+00);
out[238] = (real_t)(0.0000000000000000e+00);
out[239] = (real_t)(0.0000000000000000e+00);
out[240] = (real_t)(0.0000000000000000e+00);
out[241] = (real_t)(0.0000000000000000e+00);
out[242] = (real_t)(0.0000000000000000e+00);
out[243] = (real_t)(0.0000000000000000e+00);
out[244] = od[6];
out[245] = od[7];
out[246] = od[8];
out[247] = (real_t)(0.0000000000000000e+00);
out[248] = (real_t)(0.0000000000000000e+00);
out[249] = (real_t)(0.0000000000000000e+00);
out[250] = (real_t)(0.0000000000000000e+00);
out[251] = (real_t)(0.0000000000000000e+00);
out[252] = (real_t)(0.0000000000000000e+00);
out[253] = (real_t)(0.0000000000000000e+00);
out[254] = (real_t)(0.0000000000000000e+00);
out[255] = (real_t)(0.0000000000000000e+00);
out[256] = (real_t)(0.0000000000000000e+00);
out[257] = (real_t)(0.0000000000000000e+00);
out[258] = (real_t)(0.0000000000000000e+00);
out[259] = (real_t)(0.0000000000000000e+00);
out[260] = (real_t)(0.0000000000000000e+00);
out[261] = (real_t)(0.0000000000000000e+00);
out[262] = (real_t)(0.0000000000000000e+00);
out[263] = (real_t)(0.0000000000000000e+00);
out[264] = (real_t)(0.0000000000000000e+00);
out[265] = (real_t)(0.0000000000000000e+00);
out[266] = (real_t)(0.0000000000000000e+00);
out[267] = (real_t)(0.0000000000000000e+00);
out[268] = (real_t)(0.0000000000000000e+00);
out[269] = (real_t)(0.0000000000000000e+00);
out[270] = (real_t)(0.0000000000000000e+00);
out[271] = (real_t)(0.0000000000000000e+00);
out[272] = (real_t)(0.0000000000000000e+00);
out[273] = (real_t)(0.0000000000000000e+00);
out[274] = (real_t)(0.0000000000000000e+00);
out[275] = (real_t)(0.0000000000000000e+00);
out[276] = (real_t)(0.0000000000000000e+00);
out[277] = (real_t)(0.0000000000000000e+00);
out[278] = (real_t)(0.0000000000000000e+00);
out[279] = (real_t)(0.0000000000000000e+00);
out[280] = (real_t)(0.0000000000000000e+00);
out[281] = (real_t)(0.0000000000000000e+00);
out[282] = (real_t)(0.0000000000000000e+00);
out[283] = (real_t)(0.0000000000000000e+00);
out[284] = (real_t)(0.0000000000000000e+00);
out[285] = (real_t)(0.0000000000000000e+00);
out[286] = (real_t)(0.0000000000000000e+00);
out[287] = (real_t)(0.0000000000000000e+00);
out[288] = (real_t)(0.0000000000000000e+00);
out[289] = (real_t)(0.0000000000000000e+00);
out[290] = (real_t)(0.0000000000000000e+00);
out[291] = (real_t)(0.0000000000000000e+00);
out[292] = (real_t)(0.0000000000000000e+00);
out[293] = (real_t)(0.0000000000000000e+00);
out[294] = (real_t)(0.0000000000000000e+00);
out[295] = (real_t)(0.0000000000000000e+00);
out[296] = (real_t)(0.0000000000000000e+00);
out[297] = (real_t)(0.0000000000000000e+00);
out[298] = (real_t)(0.0000000000000000e+00);
out[299] = (real_t)(0.0000000000000000e+00);
out[300] = (real_t)(0.0000000000000000e+00);
out[301] = (real_t)(0.0000000000000000e+00);
out[302] = (real_t)(0.0000000000000000e+00);
out[303] = (real_t)(0.0000000000000000e+00);
out[304] = (real_t)(0.0000000000000000e+00);
out[305] = (real_t)(0.0000000000000000e+00);
out[306] = (real_t)(0.0000000000000000e+00);
out[307] = (real_t)(0.0000000000000000e+00);
out[308] = (real_t)(0.0000000000000000e+00);
out[309] = (real_t)(0.0000000000000000e+00);
out[310] = (real_t)(0.0000000000000000e+00);
out[311] = (real_t)(0.0000000000000000e+00);
out[312] = (real_t)(0.0000000000000000e+00);
out[313] = (real_t)(0.0000000000000000e+00);
out[314] = (real_t)(0.0000000000000000e+00);
out[315] = (real_t)(0.0000000000000000e+00);
out[316] = (real_t)(0.0000000000000000e+00);
out[317] = (real_t)(0.0000000000000000e+00);
out[318] = (real_t)(0.0000000000000000e+00);
out[319] = (real_t)(0.0000000000000000e+00);
out[320] = (real_t)(0.0000000000000000e+00);
out[321] = (real_t)(0.0000000000000000e+00);
out[322] = (real_t)(0.0000000000000000e+00);
out[323] = (real_t)(0.0000000000000000e+00);
out[324] = (real_t)(0.0000000000000000e+00);
out[325] = (real_t)(0.0000000000000000e+00);
out[326] = (real_t)(0.0000000000000000e+00);
out[327] = (real_t)(0.0000000000000000e+00);
out[328] = (real_t)(0.0000000000000000e+00);
out[329] = (real_t)(0.0000000000000000e+00);
out[330] = (real_t)(0.0000000000000000e+00);
out[331] = (real_t)(0.0000000000000000e+00);
out[332] = (real_t)(0.0000000000000000e+00);
out[333] = (real_t)(0.0000000000000000e+00);
out[334] = (real_t)(0.0000000000000000e+00);
out[335] = (real_t)(0.0000000000000000e+00);
out[336] = (real_t)(0.0000000000000000e+00);
out[337] = (real_t)(0.0000000000000000e+00);
out[338] = (real_t)(0.0000000000000000e+00);
out[339] = (real_t)(0.0000000000000000e+00);
out[340] = (real_t)(0.0000000000000000e+00);
out[341] = (real_t)(0.0000000000000000e+00);
out[342] = (real_t)(0.0000000000000000e+00);
out[343] = (real_t)(0.0000000000000000e+00);
out[344] = (real_t)(0.0000000000000000e+00);
out[345] = (real_t)(0.0000000000000000e+00);
out[346] = (real_t)(0.0000000000000000e+00);
out[347] = (real_t)(0.0000000000000000e+00);
out[348] = (real_t)(0.0000000000000000e+00);
out[349] = (real_t)(0.0000000000000000e+00);
out[350] = (real_t)(0.0000000000000000e+00);
out[351] = (real_t)(0.0000000000000000e+00);
out[352] = (real_t)(0.0000000000000000e+00);
out[353] = (real_t)(0.0000000000000000e+00);
out[354] = (real_t)(0.0000000000000000e+00);
out[355] = (real_t)(0.0000000000000000e+00);
out[356] = (real_t)(0.0000000000000000e+00);
out[357] = (real_t)(0.0000000000000000e+00);
out[358] = (real_t)(0.0000000000000000e+00);
out[359] = (real_t)(0.0000000000000000e+00);
out[360] = (real_t)(0.0000000000000000e+00);
}

void acado_evaluateLSQEndTerm(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* od = in + 18;

/* Compute outputs: */
out[0] = xd[0];
out[1] = xd[1];
out[2] = xd[2];
out[3] = (real_t)(0.0000000000000000e+00);
out[4] = ((((real_t)(-1.0000000000000000e+00)+(xd[3]*od[0]))+(xd[4]*od[1]))+(xd[5]*od[2]));
out[5] = (((xd[6]*od[0])+(xd[7]*od[1]))+(xd[8]*od[2]));
out[6] = (((xd[9]*od[0])+(xd[10]*od[1]))+(xd[11]*od[2]));
out[7] = (((xd[3]*od[3])+(xd[4]*od[4]))+(xd[5]*od[5]));
out[8] = ((((real_t)(-1.0000000000000000e+00)+(xd[6]*od[3]))+(xd[7]*od[4]))+(xd[8]*od[5]));
out[9] = (((xd[9]*od[3])+(xd[10]*od[4]))+(xd[11]*od[5]));
out[10] = (((xd[3]*od[6])+(xd[4]*od[7]))+(xd[5]*od[8]));
out[11] = (((xd[6]*od[6])+(xd[7]*od[7]))+(xd[8]*od[8]));
out[12] = ((((real_t)(-1.0000000000000000e+00)+(xd[9]*od[6]))+(xd[10]*od[7]))+(xd[11]*od[8]));
out[13] = (real_t)(1.0000000000000000e+00);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(0.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(0.0000000000000000e+00);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(0.0000000000000000e+00);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(0.0000000000000000e+00);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(1.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(0.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(0.0000000000000000e+00);
out[38] = (real_t)(0.0000000000000000e+00);
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(0.0000000000000000e+00);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(1.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = (real_t)(0.0000000000000000e+00);
out[56] = (real_t)(0.0000000000000000e+00);
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(0.0000000000000000e+00);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(0.0000000000000000e+00);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(0.0000000000000000e+00);
out[69] = (real_t)(0.0000000000000000e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = (real_t)(0.0000000000000000e+00);
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = (real_t)(0.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = (real_t)(0.0000000000000000e+00);
out[78] = (real_t)(0.0000000000000000e+00);
out[79] = (real_t)(0.0000000000000000e+00);
out[80] = (real_t)(0.0000000000000000e+00);
out[81] = (real_t)(0.0000000000000000e+00);
out[82] = (real_t)(0.0000000000000000e+00);
out[83] = (real_t)(0.0000000000000000e+00);
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(0.0000000000000000e+00);
out[86] = (real_t)(0.0000000000000000e+00);
out[87] = (real_t)(0.0000000000000000e+00);
out[88] = od[0];
out[89] = od[1];
out[90] = od[2];
out[91] = (real_t)(0.0000000000000000e+00);
out[92] = (real_t)(0.0000000000000000e+00);
out[93] = (real_t)(0.0000000000000000e+00);
out[94] = (real_t)(0.0000000000000000e+00);
out[95] = (real_t)(0.0000000000000000e+00);
out[96] = (real_t)(0.0000000000000000e+00);
out[97] = (real_t)(0.0000000000000000e+00);
out[98] = (real_t)(0.0000000000000000e+00);
out[99] = (real_t)(0.0000000000000000e+00);
out[100] = (real_t)(0.0000000000000000e+00);
out[101] = (real_t)(0.0000000000000000e+00);
out[102] = (real_t)(0.0000000000000000e+00);
out[103] = (real_t)(0.0000000000000000e+00);
out[104] = (real_t)(0.0000000000000000e+00);
out[105] = (real_t)(0.0000000000000000e+00);
out[106] = (real_t)(0.0000000000000000e+00);
out[107] = (real_t)(0.0000000000000000e+00);
out[108] = (real_t)(0.0000000000000000e+00);
out[109] = od[0];
out[110] = od[1];
out[111] = od[2];
out[112] = (real_t)(0.0000000000000000e+00);
out[113] = (real_t)(0.0000000000000000e+00);
out[114] = (real_t)(0.0000000000000000e+00);
out[115] = (real_t)(0.0000000000000000e+00);
out[116] = (real_t)(0.0000000000000000e+00);
out[117] = (real_t)(0.0000000000000000e+00);
out[118] = (real_t)(0.0000000000000000e+00);
out[119] = (real_t)(0.0000000000000000e+00);
out[120] = (real_t)(0.0000000000000000e+00);
out[121] = (real_t)(0.0000000000000000e+00);
out[122] = (real_t)(0.0000000000000000e+00);
out[123] = (real_t)(0.0000000000000000e+00);
out[124] = (real_t)(0.0000000000000000e+00);
out[125] = (real_t)(0.0000000000000000e+00);
out[126] = (real_t)(0.0000000000000000e+00);
out[127] = (real_t)(0.0000000000000000e+00);
out[128] = (real_t)(0.0000000000000000e+00);
out[129] = (real_t)(0.0000000000000000e+00);
out[130] = od[0];
out[131] = od[1];
out[132] = od[2];
out[133] = (real_t)(0.0000000000000000e+00);
out[134] = (real_t)(0.0000000000000000e+00);
out[135] = (real_t)(0.0000000000000000e+00);
out[136] = (real_t)(0.0000000000000000e+00);
out[137] = (real_t)(0.0000000000000000e+00);
out[138] = (real_t)(0.0000000000000000e+00);
out[139] = (real_t)(0.0000000000000000e+00);
out[140] = (real_t)(0.0000000000000000e+00);
out[141] = (real_t)(0.0000000000000000e+00);
out[142] = od[3];
out[143] = od[4];
out[144] = od[5];
out[145] = (real_t)(0.0000000000000000e+00);
out[146] = (real_t)(0.0000000000000000e+00);
out[147] = (real_t)(0.0000000000000000e+00);
out[148] = (real_t)(0.0000000000000000e+00);
out[149] = (real_t)(0.0000000000000000e+00);
out[150] = (real_t)(0.0000000000000000e+00);
out[151] = (real_t)(0.0000000000000000e+00);
out[152] = (real_t)(0.0000000000000000e+00);
out[153] = (real_t)(0.0000000000000000e+00);
out[154] = (real_t)(0.0000000000000000e+00);
out[155] = (real_t)(0.0000000000000000e+00);
out[156] = (real_t)(0.0000000000000000e+00);
out[157] = (real_t)(0.0000000000000000e+00);
out[158] = (real_t)(0.0000000000000000e+00);
out[159] = (real_t)(0.0000000000000000e+00);
out[160] = (real_t)(0.0000000000000000e+00);
out[161] = (real_t)(0.0000000000000000e+00);
out[162] = (real_t)(0.0000000000000000e+00);
out[163] = od[3];
out[164] = od[4];
out[165] = od[5];
out[166] = (real_t)(0.0000000000000000e+00);
out[167] = (real_t)(0.0000000000000000e+00);
out[168] = (real_t)(0.0000000000000000e+00);
out[169] = (real_t)(0.0000000000000000e+00);
out[170] = (real_t)(0.0000000000000000e+00);
out[171] = (real_t)(0.0000000000000000e+00);
out[172] = (real_t)(0.0000000000000000e+00);
out[173] = (real_t)(0.0000000000000000e+00);
out[174] = (real_t)(0.0000000000000000e+00);
out[175] = (real_t)(0.0000000000000000e+00);
out[176] = (real_t)(0.0000000000000000e+00);
out[177] = (real_t)(0.0000000000000000e+00);
out[178] = (real_t)(0.0000000000000000e+00);
out[179] = (real_t)(0.0000000000000000e+00);
out[180] = (real_t)(0.0000000000000000e+00);
out[181] = (real_t)(0.0000000000000000e+00);
out[182] = (real_t)(0.0000000000000000e+00);
out[183] = (real_t)(0.0000000000000000e+00);
out[184] = od[3];
out[185] = od[4];
out[186] = od[5];
out[187] = (real_t)(0.0000000000000000e+00);
out[188] = (real_t)(0.0000000000000000e+00);
out[189] = (real_t)(0.0000000000000000e+00);
out[190] = (real_t)(0.0000000000000000e+00);
out[191] = (real_t)(0.0000000000000000e+00);
out[192] = (real_t)(0.0000000000000000e+00);
out[193] = (real_t)(0.0000000000000000e+00);
out[194] = (real_t)(0.0000000000000000e+00);
out[195] = (real_t)(0.0000000000000000e+00);
out[196] = od[6];
out[197] = od[7];
out[198] = od[8];
out[199] = (real_t)(0.0000000000000000e+00);
out[200] = (real_t)(0.0000000000000000e+00);
out[201] = (real_t)(0.0000000000000000e+00);
out[202] = (real_t)(0.0000000000000000e+00);
out[203] = (real_t)(0.0000000000000000e+00);
out[204] = (real_t)(0.0000000000000000e+00);
out[205] = (real_t)(0.0000000000000000e+00);
out[206] = (real_t)(0.0000000000000000e+00);
out[207] = (real_t)(0.0000000000000000e+00);
out[208] = (real_t)(0.0000000000000000e+00);
out[209] = (real_t)(0.0000000000000000e+00);
out[210] = (real_t)(0.0000000000000000e+00);
out[211] = (real_t)(0.0000000000000000e+00);
out[212] = (real_t)(0.0000000000000000e+00);
out[213] = (real_t)(0.0000000000000000e+00);
out[214] = (real_t)(0.0000000000000000e+00);
out[215] = (real_t)(0.0000000000000000e+00);
out[216] = (real_t)(0.0000000000000000e+00);
out[217] = od[6];
out[218] = od[7];
out[219] = od[8];
out[220] = (real_t)(0.0000000000000000e+00);
out[221] = (real_t)(0.0000000000000000e+00);
out[222] = (real_t)(0.0000000000000000e+00);
out[223] = (real_t)(0.0000000000000000e+00);
out[224] = (real_t)(0.0000000000000000e+00);
out[225] = (real_t)(0.0000000000000000e+00);
out[226] = (real_t)(0.0000000000000000e+00);
out[227] = (real_t)(0.0000000000000000e+00);
out[228] = (real_t)(0.0000000000000000e+00);
out[229] = (real_t)(0.0000000000000000e+00);
out[230] = (real_t)(0.0000000000000000e+00);
out[231] = (real_t)(0.0000000000000000e+00);
out[232] = (real_t)(0.0000000000000000e+00);
out[233] = (real_t)(0.0000000000000000e+00);
out[234] = (real_t)(0.0000000000000000e+00);
out[235] = (real_t)(0.0000000000000000e+00);
out[236] = (real_t)(0.0000000000000000e+00);
out[237] = (real_t)(0.0000000000000000e+00);
out[238] = od[6];
out[239] = od[7];
out[240] = od[8];
out[241] = (real_t)(0.0000000000000000e+00);
out[242] = (real_t)(0.0000000000000000e+00);
out[243] = (real_t)(0.0000000000000000e+00);
out[244] = (real_t)(0.0000000000000000e+00);
out[245] = (real_t)(0.0000000000000000e+00);
out[246] = (real_t)(0.0000000000000000e+00);
}

void acado_setObjQ1Q2( real_t* const tmpFx, real_t* const tmpObjS, real_t* const tmpQ1, real_t* const tmpQ2 )
{
int lRun1;
int lRun2;
int lRun3;
for (lRun1 = 0; lRun1 < 18; ++lRun1)
{
for (lRun2 = 0; lRun2 < 19; ++lRun2)
{
real_t t = 0.0;
for (lRun3 = 0; lRun3 < 19; ++lRun3)
{
t += + tmpFx[(lRun3 * 18) + (lRun1)]*tmpObjS[(lRun3 * 19) + (lRun2)];
}
tmpQ2[(lRun1 * 19) + (lRun2)] = t;
}
}
for (lRun1 = 0; lRun1 < 18; ++lRun1)
{
for (lRun2 = 0; lRun2 < 18; ++lRun2)
{
real_t t = 0.0;
for (lRun3 = 0; lRun3 < 19; ++lRun3)
{
t += + tmpQ2[(lRun1 * 19) + (lRun3)]*tmpFx[(lRun3 * 18) + (lRun2)];
}
tmpQ1[(lRun1 * 18) + (lRun2)] = t;
}
}
}

void acado_setObjR1R2( real_t* const tmpObjS, real_t* const tmpR1, real_t* const tmpR2 )
{
tmpR2[0] = +tmpObjS[247];
tmpR2[1] = +tmpObjS[248];
tmpR2[2] = +tmpObjS[249];
tmpR2[3] = +tmpObjS[250];
tmpR2[4] = +tmpObjS[251];
tmpR2[5] = +tmpObjS[252];
tmpR2[6] = +tmpObjS[253];
tmpR2[7] = +tmpObjS[254];
tmpR2[8] = +tmpObjS[255];
tmpR2[9] = +tmpObjS[256];
tmpR2[10] = +tmpObjS[257];
tmpR2[11] = +tmpObjS[258];
tmpR2[12] = +tmpObjS[259];
tmpR2[13] = +tmpObjS[260];
tmpR2[14] = +tmpObjS[261];
tmpR2[15] = +tmpObjS[262];
tmpR2[16] = +tmpObjS[263];
tmpR2[17] = +tmpObjS[264];
tmpR2[18] = +tmpObjS[265];
tmpR2[19] = +tmpObjS[266];
tmpR2[20] = +tmpObjS[267];
tmpR2[21] = +tmpObjS[268];
tmpR2[22] = +tmpObjS[269];
tmpR2[23] = +tmpObjS[270];
tmpR2[24] = +tmpObjS[271];
tmpR2[25] = +tmpObjS[272];
tmpR2[26] = +tmpObjS[273];
tmpR2[27] = +tmpObjS[274];
tmpR2[28] = +tmpObjS[275];
tmpR2[29] = +tmpObjS[276];
tmpR2[30] = +tmpObjS[277];
tmpR2[31] = +tmpObjS[278];
tmpR2[32] = +tmpObjS[279];
tmpR2[33] = +tmpObjS[280];
tmpR2[34] = +tmpObjS[281];
tmpR2[35] = +tmpObjS[282];
tmpR2[36] = +tmpObjS[283];
tmpR2[37] = +tmpObjS[284];
tmpR2[38] = +tmpObjS[285];
tmpR2[39] = +tmpObjS[286];
tmpR2[40] = +tmpObjS[287];
tmpR2[41] = +tmpObjS[288];
tmpR2[42] = +tmpObjS[289];
tmpR2[43] = +tmpObjS[290];
tmpR2[44] = +tmpObjS[291];
tmpR2[45] = +tmpObjS[292];
tmpR2[46] = +tmpObjS[293];
tmpR2[47] = +tmpObjS[294];
tmpR2[48] = +tmpObjS[295];
tmpR2[49] = +tmpObjS[296];
tmpR2[50] = +tmpObjS[297];
tmpR2[51] = +tmpObjS[298];
tmpR2[52] = +tmpObjS[299];
tmpR2[53] = +tmpObjS[300];
tmpR2[54] = +tmpObjS[301];
tmpR2[55] = +tmpObjS[302];
tmpR2[56] = +tmpObjS[303];
tmpR2[57] = +tmpObjS[304];
tmpR2[58] = +tmpObjS[305];
tmpR2[59] = +tmpObjS[306];
tmpR2[60] = +tmpObjS[307];
tmpR2[61] = +tmpObjS[308];
tmpR2[62] = +tmpObjS[309];
tmpR2[63] = +tmpObjS[310];
tmpR2[64] = +tmpObjS[311];
tmpR2[65] = +tmpObjS[312];
tmpR2[66] = +tmpObjS[313];
tmpR2[67] = +tmpObjS[314];
tmpR2[68] = +tmpObjS[315];
tmpR2[69] = +tmpObjS[316];
tmpR2[70] = +tmpObjS[317];
tmpR2[71] = +tmpObjS[318];
tmpR2[72] = +tmpObjS[319];
tmpR2[73] = +tmpObjS[320];
tmpR2[74] = +tmpObjS[321];
tmpR2[75] = +tmpObjS[322];
tmpR2[76] = +tmpObjS[323];
tmpR2[77] = +tmpObjS[324];
tmpR2[78] = +tmpObjS[325];
tmpR2[79] = +tmpObjS[326];
tmpR2[80] = +tmpObjS[327];
tmpR2[81] = +tmpObjS[328];
tmpR2[82] = +tmpObjS[329];
tmpR2[83] = +tmpObjS[330];
tmpR2[84] = +tmpObjS[331];
tmpR2[85] = +tmpObjS[332];
tmpR2[86] = +tmpObjS[333];
tmpR2[87] = +tmpObjS[334];
tmpR2[88] = +tmpObjS[335];
tmpR2[89] = +tmpObjS[336];
tmpR2[90] = +tmpObjS[337];
tmpR2[91] = +tmpObjS[338];
tmpR2[92] = +tmpObjS[339];
tmpR2[93] = +tmpObjS[340];
tmpR2[94] = +tmpObjS[341];
tmpR2[95] = +tmpObjS[342];
tmpR2[96] = +tmpObjS[343];
tmpR2[97] = +tmpObjS[344];
tmpR2[98] = +tmpObjS[345];
tmpR2[99] = +tmpObjS[346];
tmpR2[100] = +tmpObjS[347];
tmpR2[101] = +tmpObjS[348];
tmpR2[102] = +tmpObjS[349];
tmpR2[103] = +tmpObjS[350];
tmpR2[104] = +tmpObjS[351];
tmpR2[105] = +tmpObjS[352];
tmpR2[106] = +tmpObjS[353];
tmpR2[107] = +tmpObjS[354];
tmpR2[108] = +tmpObjS[355];
tmpR2[109] = +tmpObjS[356];
tmpR2[110] = +tmpObjS[357];
tmpR2[111] = +tmpObjS[358];
tmpR2[112] = +tmpObjS[359];
tmpR2[113] = +tmpObjS[360];
tmpR1[0] = + tmpR2[13];
tmpR1[1] = + tmpR2[14];
tmpR1[2] = + tmpR2[15];
tmpR1[3] = + tmpR2[16];
tmpR1[4] = + tmpR2[17];
tmpR1[5] = + tmpR2[18];
tmpR1[6] = + tmpR2[32];
tmpR1[7] = + tmpR2[33];
tmpR1[8] = + tmpR2[34];
tmpR1[9] = + tmpR2[35];
tmpR1[10] = + tmpR2[36];
tmpR1[11] = + tmpR2[37];
tmpR1[12] = + tmpR2[51];
tmpR1[13] = + tmpR2[52];
tmpR1[14] = + tmpR2[53];
tmpR1[15] = + tmpR2[54];
tmpR1[16] = + tmpR2[55];
tmpR1[17] = + tmpR2[56];
tmpR1[18] = + tmpR2[70];
tmpR1[19] = + tmpR2[71];
tmpR1[20] = + tmpR2[72];
tmpR1[21] = + tmpR2[73];
tmpR1[22] = + tmpR2[74];
tmpR1[23] = + tmpR2[75];
tmpR1[24] = + tmpR2[89];
tmpR1[25] = + tmpR2[90];
tmpR1[26] = + tmpR2[91];
tmpR1[27] = + tmpR2[92];
tmpR1[28] = + tmpR2[93];
tmpR1[29] = + tmpR2[94];
tmpR1[30] = + tmpR2[108];
tmpR1[31] = + tmpR2[109];
tmpR1[32] = + tmpR2[110];
tmpR1[33] = + tmpR2[111];
tmpR1[34] = + tmpR2[112];
tmpR1[35] = + tmpR2[113];
}

void acado_setObjQN1QN2( real_t* const tmpFx, real_t* const tmpObjSEndTerm, real_t* const tmpQN1, real_t* const tmpQN2 )
{
int lRun1;
int lRun2;
int lRun3;
tmpQN2[0] = + tmpFx[0]*tmpObjSEndTerm[0] + tmpFx[18]*tmpObjSEndTerm[13] + tmpFx[36]*tmpObjSEndTerm[26] + tmpFx[54]*tmpObjSEndTerm[39] + tmpFx[72]*tmpObjSEndTerm[52] + tmpFx[90]*tmpObjSEndTerm[65] + tmpFx[108]*tmpObjSEndTerm[78] + tmpFx[126]*tmpObjSEndTerm[91] + tmpFx[144]*tmpObjSEndTerm[104] + tmpFx[162]*tmpObjSEndTerm[117] + tmpFx[180]*tmpObjSEndTerm[130] + tmpFx[198]*tmpObjSEndTerm[143] + tmpFx[216]*tmpObjSEndTerm[156];
tmpQN2[1] = + tmpFx[0]*tmpObjSEndTerm[1] + tmpFx[18]*tmpObjSEndTerm[14] + tmpFx[36]*tmpObjSEndTerm[27] + tmpFx[54]*tmpObjSEndTerm[40] + tmpFx[72]*tmpObjSEndTerm[53] + tmpFx[90]*tmpObjSEndTerm[66] + tmpFx[108]*tmpObjSEndTerm[79] + tmpFx[126]*tmpObjSEndTerm[92] + tmpFx[144]*tmpObjSEndTerm[105] + tmpFx[162]*tmpObjSEndTerm[118] + tmpFx[180]*tmpObjSEndTerm[131] + tmpFx[198]*tmpObjSEndTerm[144] + tmpFx[216]*tmpObjSEndTerm[157];
tmpQN2[2] = + tmpFx[0]*tmpObjSEndTerm[2] + tmpFx[18]*tmpObjSEndTerm[15] + tmpFx[36]*tmpObjSEndTerm[28] + tmpFx[54]*tmpObjSEndTerm[41] + tmpFx[72]*tmpObjSEndTerm[54] + tmpFx[90]*tmpObjSEndTerm[67] + tmpFx[108]*tmpObjSEndTerm[80] + tmpFx[126]*tmpObjSEndTerm[93] + tmpFx[144]*tmpObjSEndTerm[106] + tmpFx[162]*tmpObjSEndTerm[119] + tmpFx[180]*tmpObjSEndTerm[132] + tmpFx[198]*tmpObjSEndTerm[145] + tmpFx[216]*tmpObjSEndTerm[158];
tmpQN2[3] = + tmpFx[0]*tmpObjSEndTerm[3] + tmpFx[18]*tmpObjSEndTerm[16] + tmpFx[36]*tmpObjSEndTerm[29] + tmpFx[54]*tmpObjSEndTerm[42] + tmpFx[72]*tmpObjSEndTerm[55] + tmpFx[90]*tmpObjSEndTerm[68] + tmpFx[108]*tmpObjSEndTerm[81] + tmpFx[126]*tmpObjSEndTerm[94] + tmpFx[144]*tmpObjSEndTerm[107] + tmpFx[162]*tmpObjSEndTerm[120] + tmpFx[180]*tmpObjSEndTerm[133] + tmpFx[198]*tmpObjSEndTerm[146] + tmpFx[216]*tmpObjSEndTerm[159];
tmpQN2[4] = + tmpFx[0]*tmpObjSEndTerm[4] + tmpFx[18]*tmpObjSEndTerm[17] + tmpFx[36]*tmpObjSEndTerm[30] + tmpFx[54]*tmpObjSEndTerm[43] + tmpFx[72]*tmpObjSEndTerm[56] + tmpFx[90]*tmpObjSEndTerm[69] + tmpFx[108]*tmpObjSEndTerm[82] + tmpFx[126]*tmpObjSEndTerm[95] + tmpFx[144]*tmpObjSEndTerm[108] + tmpFx[162]*tmpObjSEndTerm[121] + tmpFx[180]*tmpObjSEndTerm[134] + tmpFx[198]*tmpObjSEndTerm[147] + tmpFx[216]*tmpObjSEndTerm[160];
tmpQN2[5] = + tmpFx[0]*tmpObjSEndTerm[5] + tmpFx[18]*tmpObjSEndTerm[18] + tmpFx[36]*tmpObjSEndTerm[31] + tmpFx[54]*tmpObjSEndTerm[44] + tmpFx[72]*tmpObjSEndTerm[57] + tmpFx[90]*tmpObjSEndTerm[70] + tmpFx[108]*tmpObjSEndTerm[83] + tmpFx[126]*tmpObjSEndTerm[96] + tmpFx[144]*tmpObjSEndTerm[109] + tmpFx[162]*tmpObjSEndTerm[122] + tmpFx[180]*tmpObjSEndTerm[135] + tmpFx[198]*tmpObjSEndTerm[148] + tmpFx[216]*tmpObjSEndTerm[161];
tmpQN2[6] = + tmpFx[0]*tmpObjSEndTerm[6] + tmpFx[18]*tmpObjSEndTerm[19] + tmpFx[36]*tmpObjSEndTerm[32] + tmpFx[54]*tmpObjSEndTerm[45] + tmpFx[72]*tmpObjSEndTerm[58] + tmpFx[90]*tmpObjSEndTerm[71] + tmpFx[108]*tmpObjSEndTerm[84] + tmpFx[126]*tmpObjSEndTerm[97] + tmpFx[144]*tmpObjSEndTerm[110] + tmpFx[162]*tmpObjSEndTerm[123] + tmpFx[180]*tmpObjSEndTerm[136] + tmpFx[198]*tmpObjSEndTerm[149] + tmpFx[216]*tmpObjSEndTerm[162];
tmpQN2[7] = + tmpFx[0]*tmpObjSEndTerm[7] + tmpFx[18]*tmpObjSEndTerm[20] + tmpFx[36]*tmpObjSEndTerm[33] + tmpFx[54]*tmpObjSEndTerm[46] + tmpFx[72]*tmpObjSEndTerm[59] + tmpFx[90]*tmpObjSEndTerm[72] + tmpFx[108]*tmpObjSEndTerm[85] + tmpFx[126]*tmpObjSEndTerm[98] + tmpFx[144]*tmpObjSEndTerm[111] + tmpFx[162]*tmpObjSEndTerm[124] + tmpFx[180]*tmpObjSEndTerm[137] + tmpFx[198]*tmpObjSEndTerm[150] + tmpFx[216]*tmpObjSEndTerm[163];
tmpQN2[8] = + tmpFx[0]*tmpObjSEndTerm[8] + tmpFx[18]*tmpObjSEndTerm[21] + tmpFx[36]*tmpObjSEndTerm[34] + tmpFx[54]*tmpObjSEndTerm[47] + tmpFx[72]*tmpObjSEndTerm[60] + tmpFx[90]*tmpObjSEndTerm[73] + tmpFx[108]*tmpObjSEndTerm[86] + tmpFx[126]*tmpObjSEndTerm[99] + tmpFx[144]*tmpObjSEndTerm[112] + tmpFx[162]*tmpObjSEndTerm[125] + tmpFx[180]*tmpObjSEndTerm[138] + tmpFx[198]*tmpObjSEndTerm[151] + tmpFx[216]*tmpObjSEndTerm[164];
tmpQN2[9] = + tmpFx[0]*tmpObjSEndTerm[9] + tmpFx[18]*tmpObjSEndTerm[22] + tmpFx[36]*tmpObjSEndTerm[35] + tmpFx[54]*tmpObjSEndTerm[48] + tmpFx[72]*tmpObjSEndTerm[61] + tmpFx[90]*tmpObjSEndTerm[74] + tmpFx[108]*tmpObjSEndTerm[87] + tmpFx[126]*tmpObjSEndTerm[100] + tmpFx[144]*tmpObjSEndTerm[113] + tmpFx[162]*tmpObjSEndTerm[126] + tmpFx[180]*tmpObjSEndTerm[139] + tmpFx[198]*tmpObjSEndTerm[152] + tmpFx[216]*tmpObjSEndTerm[165];
tmpQN2[10] = + tmpFx[0]*tmpObjSEndTerm[10] + tmpFx[18]*tmpObjSEndTerm[23] + tmpFx[36]*tmpObjSEndTerm[36] + tmpFx[54]*tmpObjSEndTerm[49] + tmpFx[72]*tmpObjSEndTerm[62] + tmpFx[90]*tmpObjSEndTerm[75] + tmpFx[108]*tmpObjSEndTerm[88] + tmpFx[126]*tmpObjSEndTerm[101] + tmpFx[144]*tmpObjSEndTerm[114] + tmpFx[162]*tmpObjSEndTerm[127] + tmpFx[180]*tmpObjSEndTerm[140] + tmpFx[198]*tmpObjSEndTerm[153] + tmpFx[216]*tmpObjSEndTerm[166];
tmpQN2[11] = + tmpFx[0]*tmpObjSEndTerm[11] + tmpFx[18]*tmpObjSEndTerm[24] + tmpFx[36]*tmpObjSEndTerm[37] + tmpFx[54]*tmpObjSEndTerm[50] + tmpFx[72]*tmpObjSEndTerm[63] + tmpFx[90]*tmpObjSEndTerm[76] + tmpFx[108]*tmpObjSEndTerm[89] + tmpFx[126]*tmpObjSEndTerm[102] + tmpFx[144]*tmpObjSEndTerm[115] + tmpFx[162]*tmpObjSEndTerm[128] + tmpFx[180]*tmpObjSEndTerm[141] + tmpFx[198]*tmpObjSEndTerm[154] + tmpFx[216]*tmpObjSEndTerm[167];
tmpQN2[12] = + tmpFx[0]*tmpObjSEndTerm[12] + tmpFx[18]*tmpObjSEndTerm[25] + tmpFx[36]*tmpObjSEndTerm[38] + tmpFx[54]*tmpObjSEndTerm[51] + tmpFx[72]*tmpObjSEndTerm[64] + tmpFx[90]*tmpObjSEndTerm[77] + tmpFx[108]*tmpObjSEndTerm[90] + tmpFx[126]*tmpObjSEndTerm[103] + tmpFx[144]*tmpObjSEndTerm[116] + tmpFx[162]*tmpObjSEndTerm[129] + tmpFx[180]*tmpObjSEndTerm[142] + tmpFx[198]*tmpObjSEndTerm[155] + tmpFx[216]*tmpObjSEndTerm[168];
tmpQN2[13] = + tmpFx[1]*tmpObjSEndTerm[0] + tmpFx[19]*tmpObjSEndTerm[13] + tmpFx[37]*tmpObjSEndTerm[26] + tmpFx[55]*tmpObjSEndTerm[39] + tmpFx[73]*tmpObjSEndTerm[52] + tmpFx[91]*tmpObjSEndTerm[65] + tmpFx[109]*tmpObjSEndTerm[78] + tmpFx[127]*tmpObjSEndTerm[91] + tmpFx[145]*tmpObjSEndTerm[104] + tmpFx[163]*tmpObjSEndTerm[117] + tmpFx[181]*tmpObjSEndTerm[130] + tmpFx[199]*tmpObjSEndTerm[143] + tmpFx[217]*tmpObjSEndTerm[156];
tmpQN2[14] = + tmpFx[1]*tmpObjSEndTerm[1] + tmpFx[19]*tmpObjSEndTerm[14] + tmpFx[37]*tmpObjSEndTerm[27] + tmpFx[55]*tmpObjSEndTerm[40] + tmpFx[73]*tmpObjSEndTerm[53] + tmpFx[91]*tmpObjSEndTerm[66] + tmpFx[109]*tmpObjSEndTerm[79] + tmpFx[127]*tmpObjSEndTerm[92] + tmpFx[145]*tmpObjSEndTerm[105] + tmpFx[163]*tmpObjSEndTerm[118] + tmpFx[181]*tmpObjSEndTerm[131] + tmpFx[199]*tmpObjSEndTerm[144] + tmpFx[217]*tmpObjSEndTerm[157];
tmpQN2[15] = + tmpFx[1]*tmpObjSEndTerm[2] + tmpFx[19]*tmpObjSEndTerm[15] + tmpFx[37]*tmpObjSEndTerm[28] + tmpFx[55]*tmpObjSEndTerm[41] + tmpFx[73]*tmpObjSEndTerm[54] + tmpFx[91]*tmpObjSEndTerm[67] + tmpFx[109]*tmpObjSEndTerm[80] + tmpFx[127]*tmpObjSEndTerm[93] + tmpFx[145]*tmpObjSEndTerm[106] + tmpFx[163]*tmpObjSEndTerm[119] + tmpFx[181]*tmpObjSEndTerm[132] + tmpFx[199]*tmpObjSEndTerm[145] + tmpFx[217]*tmpObjSEndTerm[158];
tmpQN2[16] = + tmpFx[1]*tmpObjSEndTerm[3] + tmpFx[19]*tmpObjSEndTerm[16] + tmpFx[37]*tmpObjSEndTerm[29] + tmpFx[55]*tmpObjSEndTerm[42] + tmpFx[73]*tmpObjSEndTerm[55] + tmpFx[91]*tmpObjSEndTerm[68] + tmpFx[109]*tmpObjSEndTerm[81] + tmpFx[127]*tmpObjSEndTerm[94] + tmpFx[145]*tmpObjSEndTerm[107] + tmpFx[163]*tmpObjSEndTerm[120] + tmpFx[181]*tmpObjSEndTerm[133] + tmpFx[199]*tmpObjSEndTerm[146] + tmpFx[217]*tmpObjSEndTerm[159];
tmpQN2[17] = + tmpFx[1]*tmpObjSEndTerm[4] + tmpFx[19]*tmpObjSEndTerm[17] + tmpFx[37]*tmpObjSEndTerm[30] + tmpFx[55]*tmpObjSEndTerm[43] + tmpFx[73]*tmpObjSEndTerm[56] + tmpFx[91]*tmpObjSEndTerm[69] + tmpFx[109]*tmpObjSEndTerm[82] + tmpFx[127]*tmpObjSEndTerm[95] + tmpFx[145]*tmpObjSEndTerm[108] + tmpFx[163]*tmpObjSEndTerm[121] + tmpFx[181]*tmpObjSEndTerm[134] + tmpFx[199]*tmpObjSEndTerm[147] + tmpFx[217]*tmpObjSEndTerm[160];
tmpQN2[18] = + tmpFx[1]*tmpObjSEndTerm[5] + tmpFx[19]*tmpObjSEndTerm[18] + tmpFx[37]*tmpObjSEndTerm[31] + tmpFx[55]*tmpObjSEndTerm[44] + tmpFx[73]*tmpObjSEndTerm[57] + tmpFx[91]*tmpObjSEndTerm[70] + tmpFx[109]*tmpObjSEndTerm[83] + tmpFx[127]*tmpObjSEndTerm[96] + tmpFx[145]*tmpObjSEndTerm[109] + tmpFx[163]*tmpObjSEndTerm[122] + tmpFx[181]*tmpObjSEndTerm[135] + tmpFx[199]*tmpObjSEndTerm[148] + tmpFx[217]*tmpObjSEndTerm[161];
tmpQN2[19] = + tmpFx[1]*tmpObjSEndTerm[6] + tmpFx[19]*tmpObjSEndTerm[19] + tmpFx[37]*tmpObjSEndTerm[32] + tmpFx[55]*tmpObjSEndTerm[45] + tmpFx[73]*tmpObjSEndTerm[58] + tmpFx[91]*tmpObjSEndTerm[71] + tmpFx[109]*tmpObjSEndTerm[84] + tmpFx[127]*tmpObjSEndTerm[97] + tmpFx[145]*tmpObjSEndTerm[110] + tmpFx[163]*tmpObjSEndTerm[123] + tmpFx[181]*tmpObjSEndTerm[136] + tmpFx[199]*tmpObjSEndTerm[149] + tmpFx[217]*tmpObjSEndTerm[162];
tmpQN2[20] = + tmpFx[1]*tmpObjSEndTerm[7] + tmpFx[19]*tmpObjSEndTerm[20] + tmpFx[37]*tmpObjSEndTerm[33] + tmpFx[55]*tmpObjSEndTerm[46] + tmpFx[73]*tmpObjSEndTerm[59] + tmpFx[91]*tmpObjSEndTerm[72] + tmpFx[109]*tmpObjSEndTerm[85] + tmpFx[127]*tmpObjSEndTerm[98] + tmpFx[145]*tmpObjSEndTerm[111] + tmpFx[163]*tmpObjSEndTerm[124] + tmpFx[181]*tmpObjSEndTerm[137] + tmpFx[199]*tmpObjSEndTerm[150] + tmpFx[217]*tmpObjSEndTerm[163];
tmpQN2[21] = + tmpFx[1]*tmpObjSEndTerm[8] + tmpFx[19]*tmpObjSEndTerm[21] + tmpFx[37]*tmpObjSEndTerm[34] + tmpFx[55]*tmpObjSEndTerm[47] + tmpFx[73]*tmpObjSEndTerm[60] + tmpFx[91]*tmpObjSEndTerm[73] + tmpFx[109]*tmpObjSEndTerm[86] + tmpFx[127]*tmpObjSEndTerm[99] + tmpFx[145]*tmpObjSEndTerm[112] + tmpFx[163]*tmpObjSEndTerm[125] + tmpFx[181]*tmpObjSEndTerm[138] + tmpFx[199]*tmpObjSEndTerm[151] + tmpFx[217]*tmpObjSEndTerm[164];
tmpQN2[22] = + tmpFx[1]*tmpObjSEndTerm[9] + tmpFx[19]*tmpObjSEndTerm[22] + tmpFx[37]*tmpObjSEndTerm[35] + tmpFx[55]*tmpObjSEndTerm[48] + tmpFx[73]*tmpObjSEndTerm[61] + tmpFx[91]*tmpObjSEndTerm[74] + tmpFx[109]*tmpObjSEndTerm[87] + tmpFx[127]*tmpObjSEndTerm[100] + tmpFx[145]*tmpObjSEndTerm[113] + tmpFx[163]*tmpObjSEndTerm[126] + tmpFx[181]*tmpObjSEndTerm[139] + tmpFx[199]*tmpObjSEndTerm[152] + tmpFx[217]*tmpObjSEndTerm[165];
tmpQN2[23] = + tmpFx[1]*tmpObjSEndTerm[10] + tmpFx[19]*tmpObjSEndTerm[23] + tmpFx[37]*tmpObjSEndTerm[36] + tmpFx[55]*tmpObjSEndTerm[49] + tmpFx[73]*tmpObjSEndTerm[62] + tmpFx[91]*tmpObjSEndTerm[75] + tmpFx[109]*tmpObjSEndTerm[88] + tmpFx[127]*tmpObjSEndTerm[101] + tmpFx[145]*tmpObjSEndTerm[114] + tmpFx[163]*tmpObjSEndTerm[127] + tmpFx[181]*tmpObjSEndTerm[140] + tmpFx[199]*tmpObjSEndTerm[153] + tmpFx[217]*tmpObjSEndTerm[166];
tmpQN2[24] = + tmpFx[1]*tmpObjSEndTerm[11] + tmpFx[19]*tmpObjSEndTerm[24] + tmpFx[37]*tmpObjSEndTerm[37] + tmpFx[55]*tmpObjSEndTerm[50] + tmpFx[73]*tmpObjSEndTerm[63] + tmpFx[91]*tmpObjSEndTerm[76] + tmpFx[109]*tmpObjSEndTerm[89] + tmpFx[127]*tmpObjSEndTerm[102] + tmpFx[145]*tmpObjSEndTerm[115] + tmpFx[163]*tmpObjSEndTerm[128] + tmpFx[181]*tmpObjSEndTerm[141] + tmpFx[199]*tmpObjSEndTerm[154] + tmpFx[217]*tmpObjSEndTerm[167];
tmpQN2[25] = + tmpFx[1]*tmpObjSEndTerm[12] + tmpFx[19]*tmpObjSEndTerm[25] + tmpFx[37]*tmpObjSEndTerm[38] + tmpFx[55]*tmpObjSEndTerm[51] + tmpFx[73]*tmpObjSEndTerm[64] + tmpFx[91]*tmpObjSEndTerm[77] + tmpFx[109]*tmpObjSEndTerm[90] + tmpFx[127]*tmpObjSEndTerm[103] + tmpFx[145]*tmpObjSEndTerm[116] + tmpFx[163]*tmpObjSEndTerm[129] + tmpFx[181]*tmpObjSEndTerm[142] + tmpFx[199]*tmpObjSEndTerm[155] + tmpFx[217]*tmpObjSEndTerm[168];
tmpQN2[26] = + tmpFx[2]*tmpObjSEndTerm[0] + tmpFx[20]*tmpObjSEndTerm[13] + tmpFx[38]*tmpObjSEndTerm[26] + tmpFx[56]*tmpObjSEndTerm[39] + tmpFx[74]*tmpObjSEndTerm[52] + tmpFx[92]*tmpObjSEndTerm[65] + tmpFx[110]*tmpObjSEndTerm[78] + tmpFx[128]*tmpObjSEndTerm[91] + tmpFx[146]*tmpObjSEndTerm[104] + tmpFx[164]*tmpObjSEndTerm[117] + tmpFx[182]*tmpObjSEndTerm[130] + tmpFx[200]*tmpObjSEndTerm[143] + tmpFx[218]*tmpObjSEndTerm[156];
tmpQN2[27] = + tmpFx[2]*tmpObjSEndTerm[1] + tmpFx[20]*tmpObjSEndTerm[14] + tmpFx[38]*tmpObjSEndTerm[27] + tmpFx[56]*tmpObjSEndTerm[40] + tmpFx[74]*tmpObjSEndTerm[53] + tmpFx[92]*tmpObjSEndTerm[66] + tmpFx[110]*tmpObjSEndTerm[79] + tmpFx[128]*tmpObjSEndTerm[92] + tmpFx[146]*tmpObjSEndTerm[105] + tmpFx[164]*tmpObjSEndTerm[118] + tmpFx[182]*tmpObjSEndTerm[131] + tmpFx[200]*tmpObjSEndTerm[144] + tmpFx[218]*tmpObjSEndTerm[157];
tmpQN2[28] = + tmpFx[2]*tmpObjSEndTerm[2] + tmpFx[20]*tmpObjSEndTerm[15] + tmpFx[38]*tmpObjSEndTerm[28] + tmpFx[56]*tmpObjSEndTerm[41] + tmpFx[74]*tmpObjSEndTerm[54] + tmpFx[92]*tmpObjSEndTerm[67] + tmpFx[110]*tmpObjSEndTerm[80] + tmpFx[128]*tmpObjSEndTerm[93] + tmpFx[146]*tmpObjSEndTerm[106] + tmpFx[164]*tmpObjSEndTerm[119] + tmpFx[182]*tmpObjSEndTerm[132] + tmpFx[200]*tmpObjSEndTerm[145] + tmpFx[218]*tmpObjSEndTerm[158];
tmpQN2[29] = + tmpFx[2]*tmpObjSEndTerm[3] + tmpFx[20]*tmpObjSEndTerm[16] + tmpFx[38]*tmpObjSEndTerm[29] + tmpFx[56]*tmpObjSEndTerm[42] + tmpFx[74]*tmpObjSEndTerm[55] + tmpFx[92]*tmpObjSEndTerm[68] + tmpFx[110]*tmpObjSEndTerm[81] + tmpFx[128]*tmpObjSEndTerm[94] + tmpFx[146]*tmpObjSEndTerm[107] + tmpFx[164]*tmpObjSEndTerm[120] + tmpFx[182]*tmpObjSEndTerm[133] + tmpFx[200]*tmpObjSEndTerm[146] + tmpFx[218]*tmpObjSEndTerm[159];
tmpQN2[30] = + tmpFx[2]*tmpObjSEndTerm[4] + tmpFx[20]*tmpObjSEndTerm[17] + tmpFx[38]*tmpObjSEndTerm[30] + tmpFx[56]*tmpObjSEndTerm[43] + tmpFx[74]*tmpObjSEndTerm[56] + tmpFx[92]*tmpObjSEndTerm[69] + tmpFx[110]*tmpObjSEndTerm[82] + tmpFx[128]*tmpObjSEndTerm[95] + tmpFx[146]*tmpObjSEndTerm[108] + tmpFx[164]*tmpObjSEndTerm[121] + tmpFx[182]*tmpObjSEndTerm[134] + tmpFx[200]*tmpObjSEndTerm[147] + tmpFx[218]*tmpObjSEndTerm[160];
tmpQN2[31] = + tmpFx[2]*tmpObjSEndTerm[5] + tmpFx[20]*tmpObjSEndTerm[18] + tmpFx[38]*tmpObjSEndTerm[31] + tmpFx[56]*tmpObjSEndTerm[44] + tmpFx[74]*tmpObjSEndTerm[57] + tmpFx[92]*tmpObjSEndTerm[70] + tmpFx[110]*tmpObjSEndTerm[83] + tmpFx[128]*tmpObjSEndTerm[96] + tmpFx[146]*tmpObjSEndTerm[109] + tmpFx[164]*tmpObjSEndTerm[122] + tmpFx[182]*tmpObjSEndTerm[135] + tmpFx[200]*tmpObjSEndTerm[148] + tmpFx[218]*tmpObjSEndTerm[161];
tmpQN2[32] = + tmpFx[2]*tmpObjSEndTerm[6] + tmpFx[20]*tmpObjSEndTerm[19] + tmpFx[38]*tmpObjSEndTerm[32] + tmpFx[56]*tmpObjSEndTerm[45] + tmpFx[74]*tmpObjSEndTerm[58] + tmpFx[92]*tmpObjSEndTerm[71] + tmpFx[110]*tmpObjSEndTerm[84] + tmpFx[128]*tmpObjSEndTerm[97] + tmpFx[146]*tmpObjSEndTerm[110] + tmpFx[164]*tmpObjSEndTerm[123] + tmpFx[182]*tmpObjSEndTerm[136] + tmpFx[200]*tmpObjSEndTerm[149] + tmpFx[218]*tmpObjSEndTerm[162];
tmpQN2[33] = + tmpFx[2]*tmpObjSEndTerm[7] + tmpFx[20]*tmpObjSEndTerm[20] + tmpFx[38]*tmpObjSEndTerm[33] + tmpFx[56]*tmpObjSEndTerm[46] + tmpFx[74]*tmpObjSEndTerm[59] + tmpFx[92]*tmpObjSEndTerm[72] + tmpFx[110]*tmpObjSEndTerm[85] + tmpFx[128]*tmpObjSEndTerm[98] + tmpFx[146]*tmpObjSEndTerm[111] + tmpFx[164]*tmpObjSEndTerm[124] + tmpFx[182]*tmpObjSEndTerm[137] + tmpFx[200]*tmpObjSEndTerm[150] + tmpFx[218]*tmpObjSEndTerm[163];
tmpQN2[34] = + tmpFx[2]*tmpObjSEndTerm[8] + tmpFx[20]*tmpObjSEndTerm[21] + tmpFx[38]*tmpObjSEndTerm[34] + tmpFx[56]*tmpObjSEndTerm[47] + tmpFx[74]*tmpObjSEndTerm[60] + tmpFx[92]*tmpObjSEndTerm[73] + tmpFx[110]*tmpObjSEndTerm[86] + tmpFx[128]*tmpObjSEndTerm[99] + tmpFx[146]*tmpObjSEndTerm[112] + tmpFx[164]*tmpObjSEndTerm[125] + tmpFx[182]*tmpObjSEndTerm[138] + tmpFx[200]*tmpObjSEndTerm[151] + tmpFx[218]*tmpObjSEndTerm[164];
tmpQN2[35] = + tmpFx[2]*tmpObjSEndTerm[9] + tmpFx[20]*tmpObjSEndTerm[22] + tmpFx[38]*tmpObjSEndTerm[35] + tmpFx[56]*tmpObjSEndTerm[48] + tmpFx[74]*tmpObjSEndTerm[61] + tmpFx[92]*tmpObjSEndTerm[74] + tmpFx[110]*tmpObjSEndTerm[87] + tmpFx[128]*tmpObjSEndTerm[100] + tmpFx[146]*tmpObjSEndTerm[113] + tmpFx[164]*tmpObjSEndTerm[126] + tmpFx[182]*tmpObjSEndTerm[139] + tmpFx[200]*tmpObjSEndTerm[152] + tmpFx[218]*tmpObjSEndTerm[165];
tmpQN2[36] = + tmpFx[2]*tmpObjSEndTerm[10] + tmpFx[20]*tmpObjSEndTerm[23] + tmpFx[38]*tmpObjSEndTerm[36] + tmpFx[56]*tmpObjSEndTerm[49] + tmpFx[74]*tmpObjSEndTerm[62] + tmpFx[92]*tmpObjSEndTerm[75] + tmpFx[110]*tmpObjSEndTerm[88] + tmpFx[128]*tmpObjSEndTerm[101] + tmpFx[146]*tmpObjSEndTerm[114] + tmpFx[164]*tmpObjSEndTerm[127] + tmpFx[182]*tmpObjSEndTerm[140] + tmpFx[200]*tmpObjSEndTerm[153] + tmpFx[218]*tmpObjSEndTerm[166];
tmpQN2[37] = + tmpFx[2]*tmpObjSEndTerm[11] + tmpFx[20]*tmpObjSEndTerm[24] + tmpFx[38]*tmpObjSEndTerm[37] + tmpFx[56]*tmpObjSEndTerm[50] + tmpFx[74]*tmpObjSEndTerm[63] + tmpFx[92]*tmpObjSEndTerm[76] + tmpFx[110]*tmpObjSEndTerm[89] + tmpFx[128]*tmpObjSEndTerm[102] + tmpFx[146]*tmpObjSEndTerm[115] + tmpFx[164]*tmpObjSEndTerm[128] + tmpFx[182]*tmpObjSEndTerm[141] + tmpFx[200]*tmpObjSEndTerm[154] + tmpFx[218]*tmpObjSEndTerm[167];
tmpQN2[38] = + tmpFx[2]*tmpObjSEndTerm[12] + tmpFx[20]*tmpObjSEndTerm[25] + tmpFx[38]*tmpObjSEndTerm[38] + tmpFx[56]*tmpObjSEndTerm[51] + tmpFx[74]*tmpObjSEndTerm[64] + tmpFx[92]*tmpObjSEndTerm[77] + tmpFx[110]*tmpObjSEndTerm[90] + tmpFx[128]*tmpObjSEndTerm[103] + tmpFx[146]*tmpObjSEndTerm[116] + tmpFx[164]*tmpObjSEndTerm[129] + tmpFx[182]*tmpObjSEndTerm[142] + tmpFx[200]*tmpObjSEndTerm[155] + tmpFx[218]*tmpObjSEndTerm[168];
tmpQN2[39] = + tmpFx[3]*tmpObjSEndTerm[0] + tmpFx[21]*tmpObjSEndTerm[13] + tmpFx[39]*tmpObjSEndTerm[26] + tmpFx[57]*tmpObjSEndTerm[39] + tmpFx[75]*tmpObjSEndTerm[52] + tmpFx[93]*tmpObjSEndTerm[65] + tmpFx[111]*tmpObjSEndTerm[78] + tmpFx[129]*tmpObjSEndTerm[91] + tmpFx[147]*tmpObjSEndTerm[104] + tmpFx[165]*tmpObjSEndTerm[117] + tmpFx[183]*tmpObjSEndTerm[130] + tmpFx[201]*tmpObjSEndTerm[143] + tmpFx[219]*tmpObjSEndTerm[156];
tmpQN2[40] = + tmpFx[3]*tmpObjSEndTerm[1] + tmpFx[21]*tmpObjSEndTerm[14] + tmpFx[39]*tmpObjSEndTerm[27] + tmpFx[57]*tmpObjSEndTerm[40] + tmpFx[75]*tmpObjSEndTerm[53] + tmpFx[93]*tmpObjSEndTerm[66] + tmpFx[111]*tmpObjSEndTerm[79] + tmpFx[129]*tmpObjSEndTerm[92] + tmpFx[147]*tmpObjSEndTerm[105] + tmpFx[165]*tmpObjSEndTerm[118] + tmpFx[183]*tmpObjSEndTerm[131] + tmpFx[201]*tmpObjSEndTerm[144] + tmpFx[219]*tmpObjSEndTerm[157];
tmpQN2[41] = + tmpFx[3]*tmpObjSEndTerm[2] + tmpFx[21]*tmpObjSEndTerm[15] + tmpFx[39]*tmpObjSEndTerm[28] + tmpFx[57]*tmpObjSEndTerm[41] + tmpFx[75]*tmpObjSEndTerm[54] + tmpFx[93]*tmpObjSEndTerm[67] + tmpFx[111]*tmpObjSEndTerm[80] + tmpFx[129]*tmpObjSEndTerm[93] + tmpFx[147]*tmpObjSEndTerm[106] + tmpFx[165]*tmpObjSEndTerm[119] + tmpFx[183]*tmpObjSEndTerm[132] + tmpFx[201]*tmpObjSEndTerm[145] + tmpFx[219]*tmpObjSEndTerm[158];
tmpQN2[42] = + tmpFx[3]*tmpObjSEndTerm[3] + tmpFx[21]*tmpObjSEndTerm[16] + tmpFx[39]*tmpObjSEndTerm[29] + tmpFx[57]*tmpObjSEndTerm[42] + tmpFx[75]*tmpObjSEndTerm[55] + tmpFx[93]*tmpObjSEndTerm[68] + tmpFx[111]*tmpObjSEndTerm[81] + tmpFx[129]*tmpObjSEndTerm[94] + tmpFx[147]*tmpObjSEndTerm[107] + tmpFx[165]*tmpObjSEndTerm[120] + tmpFx[183]*tmpObjSEndTerm[133] + tmpFx[201]*tmpObjSEndTerm[146] + tmpFx[219]*tmpObjSEndTerm[159];
tmpQN2[43] = + tmpFx[3]*tmpObjSEndTerm[4] + tmpFx[21]*tmpObjSEndTerm[17] + tmpFx[39]*tmpObjSEndTerm[30] + tmpFx[57]*tmpObjSEndTerm[43] + tmpFx[75]*tmpObjSEndTerm[56] + tmpFx[93]*tmpObjSEndTerm[69] + tmpFx[111]*tmpObjSEndTerm[82] + tmpFx[129]*tmpObjSEndTerm[95] + tmpFx[147]*tmpObjSEndTerm[108] + tmpFx[165]*tmpObjSEndTerm[121] + tmpFx[183]*tmpObjSEndTerm[134] + tmpFx[201]*tmpObjSEndTerm[147] + tmpFx[219]*tmpObjSEndTerm[160];
tmpQN2[44] = + tmpFx[3]*tmpObjSEndTerm[5] + tmpFx[21]*tmpObjSEndTerm[18] + tmpFx[39]*tmpObjSEndTerm[31] + tmpFx[57]*tmpObjSEndTerm[44] + tmpFx[75]*tmpObjSEndTerm[57] + tmpFx[93]*tmpObjSEndTerm[70] + tmpFx[111]*tmpObjSEndTerm[83] + tmpFx[129]*tmpObjSEndTerm[96] + tmpFx[147]*tmpObjSEndTerm[109] + tmpFx[165]*tmpObjSEndTerm[122] + tmpFx[183]*tmpObjSEndTerm[135] + tmpFx[201]*tmpObjSEndTerm[148] + tmpFx[219]*tmpObjSEndTerm[161];
tmpQN2[45] = + tmpFx[3]*tmpObjSEndTerm[6] + tmpFx[21]*tmpObjSEndTerm[19] + tmpFx[39]*tmpObjSEndTerm[32] + tmpFx[57]*tmpObjSEndTerm[45] + tmpFx[75]*tmpObjSEndTerm[58] + tmpFx[93]*tmpObjSEndTerm[71] + tmpFx[111]*tmpObjSEndTerm[84] + tmpFx[129]*tmpObjSEndTerm[97] + tmpFx[147]*tmpObjSEndTerm[110] + tmpFx[165]*tmpObjSEndTerm[123] + tmpFx[183]*tmpObjSEndTerm[136] + tmpFx[201]*tmpObjSEndTerm[149] + tmpFx[219]*tmpObjSEndTerm[162];
tmpQN2[46] = + tmpFx[3]*tmpObjSEndTerm[7] + tmpFx[21]*tmpObjSEndTerm[20] + tmpFx[39]*tmpObjSEndTerm[33] + tmpFx[57]*tmpObjSEndTerm[46] + tmpFx[75]*tmpObjSEndTerm[59] + tmpFx[93]*tmpObjSEndTerm[72] + tmpFx[111]*tmpObjSEndTerm[85] + tmpFx[129]*tmpObjSEndTerm[98] + tmpFx[147]*tmpObjSEndTerm[111] + tmpFx[165]*tmpObjSEndTerm[124] + tmpFx[183]*tmpObjSEndTerm[137] + tmpFx[201]*tmpObjSEndTerm[150] + tmpFx[219]*tmpObjSEndTerm[163];
tmpQN2[47] = + tmpFx[3]*tmpObjSEndTerm[8] + tmpFx[21]*tmpObjSEndTerm[21] + tmpFx[39]*tmpObjSEndTerm[34] + tmpFx[57]*tmpObjSEndTerm[47] + tmpFx[75]*tmpObjSEndTerm[60] + tmpFx[93]*tmpObjSEndTerm[73] + tmpFx[111]*tmpObjSEndTerm[86] + tmpFx[129]*tmpObjSEndTerm[99] + tmpFx[147]*tmpObjSEndTerm[112] + tmpFx[165]*tmpObjSEndTerm[125] + tmpFx[183]*tmpObjSEndTerm[138] + tmpFx[201]*tmpObjSEndTerm[151] + tmpFx[219]*tmpObjSEndTerm[164];
tmpQN2[48] = + tmpFx[3]*tmpObjSEndTerm[9] + tmpFx[21]*tmpObjSEndTerm[22] + tmpFx[39]*tmpObjSEndTerm[35] + tmpFx[57]*tmpObjSEndTerm[48] + tmpFx[75]*tmpObjSEndTerm[61] + tmpFx[93]*tmpObjSEndTerm[74] + tmpFx[111]*tmpObjSEndTerm[87] + tmpFx[129]*tmpObjSEndTerm[100] + tmpFx[147]*tmpObjSEndTerm[113] + tmpFx[165]*tmpObjSEndTerm[126] + tmpFx[183]*tmpObjSEndTerm[139] + tmpFx[201]*tmpObjSEndTerm[152] + tmpFx[219]*tmpObjSEndTerm[165];
tmpQN2[49] = + tmpFx[3]*tmpObjSEndTerm[10] + tmpFx[21]*tmpObjSEndTerm[23] + tmpFx[39]*tmpObjSEndTerm[36] + tmpFx[57]*tmpObjSEndTerm[49] + tmpFx[75]*tmpObjSEndTerm[62] + tmpFx[93]*tmpObjSEndTerm[75] + tmpFx[111]*tmpObjSEndTerm[88] + tmpFx[129]*tmpObjSEndTerm[101] + tmpFx[147]*tmpObjSEndTerm[114] + tmpFx[165]*tmpObjSEndTerm[127] + tmpFx[183]*tmpObjSEndTerm[140] + tmpFx[201]*tmpObjSEndTerm[153] + tmpFx[219]*tmpObjSEndTerm[166];
tmpQN2[50] = + tmpFx[3]*tmpObjSEndTerm[11] + tmpFx[21]*tmpObjSEndTerm[24] + tmpFx[39]*tmpObjSEndTerm[37] + tmpFx[57]*tmpObjSEndTerm[50] + tmpFx[75]*tmpObjSEndTerm[63] + tmpFx[93]*tmpObjSEndTerm[76] + tmpFx[111]*tmpObjSEndTerm[89] + tmpFx[129]*tmpObjSEndTerm[102] + tmpFx[147]*tmpObjSEndTerm[115] + tmpFx[165]*tmpObjSEndTerm[128] + tmpFx[183]*tmpObjSEndTerm[141] + tmpFx[201]*tmpObjSEndTerm[154] + tmpFx[219]*tmpObjSEndTerm[167];
tmpQN2[51] = + tmpFx[3]*tmpObjSEndTerm[12] + tmpFx[21]*tmpObjSEndTerm[25] + tmpFx[39]*tmpObjSEndTerm[38] + tmpFx[57]*tmpObjSEndTerm[51] + tmpFx[75]*tmpObjSEndTerm[64] + tmpFx[93]*tmpObjSEndTerm[77] + tmpFx[111]*tmpObjSEndTerm[90] + tmpFx[129]*tmpObjSEndTerm[103] + tmpFx[147]*tmpObjSEndTerm[116] + tmpFx[165]*tmpObjSEndTerm[129] + tmpFx[183]*tmpObjSEndTerm[142] + tmpFx[201]*tmpObjSEndTerm[155] + tmpFx[219]*tmpObjSEndTerm[168];
tmpQN2[52] = + tmpFx[4]*tmpObjSEndTerm[0] + tmpFx[22]*tmpObjSEndTerm[13] + tmpFx[40]*tmpObjSEndTerm[26] + tmpFx[58]*tmpObjSEndTerm[39] + tmpFx[76]*tmpObjSEndTerm[52] + tmpFx[94]*tmpObjSEndTerm[65] + tmpFx[112]*tmpObjSEndTerm[78] + tmpFx[130]*tmpObjSEndTerm[91] + tmpFx[148]*tmpObjSEndTerm[104] + tmpFx[166]*tmpObjSEndTerm[117] + tmpFx[184]*tmpObjSEndTerm[130] + tmpFx[202]*tmpObjSEndTerm[143] + tmpFx[220]*tmpObjSEndTerm[156];
tmpQN2[53] = + tmpFx[4]*tmpObjSEndTerm[1] + tmpFx[22]*tmpObjSEndTerm[14] + tmpFx[40]*tmpObjSEndTerm[27] + tmpFx[58]*tmpObjSEndTerm[40] + tmpFx[76]*tmpObjSEndTerm[53] + tmpFx[94]*tmpObjSEndTerm[66] + tmpFx[112]*tmpObjSEndTerm[79] + tmpFx[130]*tmpObjSEndTerm[92] + tmpFx[148]*tmpObjSEndTerm[105] + tmpFx[166]*tmpObjSEndTerm[118] + tmpFx[184]*tmpObjSEndTerm[131] + tmpFx[202]*tmpObjSEndTerm[144] + tmpFx[220]*tmpObjSEndTerm[157];
tmpQN2[54] = + tmpFx[4]*tmpObjSEndTerm[2] + tmpFx[22]*tmpObjSEndTerm[15] + tmpFx[40]*tmpObjSEndTerm[28] + tmpFx[58]*tmpObjSEndTerm[41] + tmpFx[76]*tmpObjSEndTerm[54] + tmpFx[94]*tmpObjSEndTerm[67] + tmpFx[112]*tmpObjSEndTerm[80] + tmpFx[130]*tmpObjSEndTerm[93] + tmpFx[148]*tmpObjSEndTerm[106] + tmpFx[166]*tmpObjSEndTerm[119] + tmpFx[184]*tmpObjSEndTerm[132] + tmpFx[202]*tmpObjSEndTerm[145] + tmpFx[220]*tmpObjSEndTerm[158];
tmpQN2[55] = + tmpFx[4]*tmpObjSEndTerm[3] + tmpFx[22]*tmpObjSEndTerm[16] + tmpFx[40]*tmpObjSEndTerm[29] + tmpFx[58]*tmpObjSEndTerm[42] + tmpFx[76]*tmpObjSEndTerm[55] + tmpFx[94]*tmpObjSEndTerm[68] + tmpFx[112]*tmpObjSEndTerm[81] + tmpFx[130]*tmpObjSEndTerm[94] + tmpFx[148]*tmpObjSEndTerm[107] + tmpFx[166]*tmpObjSEndTerm[120] + tmpFx[184]*tmpObjSEndTerm[133] + tmpFx[202]*tmpObjSEndTerm[146] + tmpFx[220]*tmpObjSEndTerm[159];
tmpQN2[56] = + tmpFx[4]*tmpObjSEndTerm[4] + tmpFx[22]*tmpObjSEndTerm[17] + tmpFx[40]*tmpObjSEndTerm[30] + tmpFx[58]*tmpObjSEndTerm[43] + tmpFx[76]*tmpObjSEndTerm[56] + tmpFx[94]*tmpObjSEndTerm[69] + tmpFx[112]*tmpObjSEndTerm[82] + tmpFx[130]*tmpObjSEndTerm[95] + tmpFx[148]*tmpObjSEndTerm[108] + tmpFx[166]*tmpObjSEndTerm[121] + tmpFx[184]*tmpObjSEndTerm[134] + tmpFx[202]*tmpObjSEndTerm[147] + tmpFx[220]*tmpObjSEndTerm[160];
tmpQN2[57] = + tmpFx[4]*tmpObjSEndTerm[5] + tmpFx[22]*tmpObjSEndTerm[18] + tmpFx[40]*tmpObjSEndTerm[31] + tmpFx[58]*tmpObjSEndTerm[44] + tmpFx[76]*tmpObjSEndTerm[57] + tmpFx[94]*tmpObjSEndTerm[70] + tmpFx[112]*tmpObjSEndTerm[83] + tmpFx[130]*tmpObjSEndTerm[96] + tmpFx[148]*tmpObjSEndTerm[109] + tmpFx[166]*tmpObjSEndTerm[122] + tmpFx[184]*tmpObjSEndTerm[135] + tmpFx[202]*tmpObjSEndTerm[148] + tmpFx[220]*tmpObjSEndTerm[161];
tmpQN2[58] = + tmpFx[4]*tmpObjSEndTerm[6] + tmpFx[22]*tmpObjSEndTerm[19] + tmpFx[40]*tmpObjSEndTerm[32] + tmpFx[58]*tmpObjSEndTerm[45] + tmpFx[76]*tmpObjSEndTerm[58] + tmpFx[94]*tmpObjSEndTerm[71] + tmpFx[112]*tmpObjSEndTerm[84] + tmpFx[130]*tmpObjSEndTerm[97] + tmpFx[148]*tmpObjSEndTerm[110] + tmpFx[166]*tmpObjSEndTerm[123] + tmpFx[184]*tmpObjSEndTerm[136] + tmpFx[202]*tmpObjSEndTerm[149] + tmpFx[220]*tmpObjSEndTerm[162];
tmpQN2[59] = + tmpFx[4]*tmpObjSEndTerm[7] + tmpFx[22]*tmpObjSEndTerm[20] + tmpFx[40]*tmpObjSEndTerm[33] + tmpFx[58]*tmpObjSEndTerm[46] + tmpFx[76]*tmpObjSEndTerm[59] + tmpFx[94]*tmpObjSEndTerm[72] + tmpFx[112]*tmpObjSEndTerm[85] + tmpFx[130]*tmpObjSEndTerm[98] + tmpFx[148]*tmpObjSEndTerm[111] + tmpFx[166]*tmpObjSEndTerm[124] + tmpFx[184]*tmpObjSEndTerm[137] + tmpFx[202]*tmpObjSEndTerm[150] + tmpFx[220]*tmpObjSEndTerm[163];
tmpQN2[60] = + tmpFx[4]*tmpObjSEndTerm[8] + tmpFx[22]*tmpObjSEndTerm[21] + tmpFx[40]*tmpObjSEndTerm[34] + tmpFx[58]*tmpObjSEndTerm[47] + tmpFx[76]*tmpObjSEndTerm[60] + tmpFx[94]*tmpObjSEndTerm[73] + tmpFx[112]*tmpObjSEndTerm[86] + tmpFx[130]*tmpObjSEndTerm[99] + tmpFx[148]*tmpObjSEndTerm[112] + tmpFx[166]*tmpObjSEndTerm[125] + tmpFx[184]*tmpObjSEndTerm[138] + tmpFx[202]*tmpObjSEndTerm[151] + tmpFx[220]*tmpObjSEndTerm[164];
tmpQN2[61] = + tmpFx[4]*tmpObjSEndTerm[9] + tmpFx[22]*tmpObjSEndTerm[22] + tmpFx[40]*tmpObjSEndTerm[35] + tmpFx[58]*tmpObjSEndTerm[48] + tmpFx[76]*tmpObjSEndTerm[61] + tmpFx[94]*tmpObjSEndTerm[74] + tmpFx[112]*tmpObjSEndTerm[87] + tmpFx[130]*tmpObjSEndTerm[100] + tmpFx[148]*tmpObjSEndTerm[113] + tmpFx[166]*tmpObjSEndTerm[126] + tmpFx[184]*tmpObjSEndTerm[139] + tmpFx[202]*tmpObjSEndTerm[152] + tmpFx[220]*tmpObjSEndTerm[165];
tmpQN2[62] = + tmpFx[4]*tmpObjSEndTerm[10] + tmpFx[22]*tmpObjSEndTerm[23] + tmpFx[40]*tmpObjSEndTerm[36] + tmpFx[58]*tmpObjSEndTerm[49] + tmpFx[76]*tmpObjSEndTerm[62] + tmpFx[94]*tmpObjSEndTerm[75] + tmpFx[112]*tmpObjSEndTerm[88] + tmpFx[130]*tmpObjSEndTerm[101] + tmpFx[148]*tmpObjSEndTerm[114] + tmpFx[166]*tmpObjSEndTerm[127] + tmpFx[184]*tmpObjSEndTerm[140] + tmpFx[202]*tmpObjSEndTerm[153] + tmpFx[220]*tmpObjSEndTerm[166];
tmpQN2[63] = + tmpFx[4]*tmpObjSEndTerm[11] + tmpFx[22]*tmpObjSEndTerm[24] + tmpFx[40]*tmpObjSEndTerm[37] + tmpFx[58]*tmpObjSEndTerm[50] + tmpFx[76]*tmpObjSEndTerm[63] + tmpFx[94]*tmpObjSEndTerm[76] + tmpFx[112]*tmpObjSEndTerm[89] + tmpFx[130]*tmpObjSEndTerm[102] + tmpFx[148]*tmpObjSEndTerm[115] + tmpFx[166]*tmpObjSEndTerm[128] + tmpFx[184]*tmpObjSEndTerm[141] + tmpFx[202]*tmpObjSEndTerm[154] + tmpFx[220]*tmpObjSEndTerm[167];
tmpQN2[64] = + tmpFx[4]*tmpObjSEndTerm[12] + tmpFx[22]*tmpObjSEndTerm[25] + tmpFx[40]*tmpObjSEndTerm[38] + tmpFx[58]*tmpObjSEndTerm[51] + tmpFx[76]*tmpObjSEndTerm[64] + tmpFx[94]*tmpObjSEndTerm[77] + tmpFx[112]*tmpObjSEndTerm[90] + tmpFx[130]*tmpObjSEndTerm[103] + tmpFx[148]*tmpObjSEndTerm[116] + tmpFx[166]*tmpObjSEndTerm[129] + tmpFx[184]*tmpObjSEndTerm[142] + tmpFx[202]*tmpObjSEndTerm[155] + tmpFx[220]*tmpObjSEndTerm[168];
tmpQN2[65] = + tmpFx[5]*tmpObjSEndTerm[0] + tmpFx[23]*tmpObjSEndTerm[13] + tmpFx[41]*tmpObjSEndTerm[26] + tmpFx[59]*tmpObjSEndTerm[39] + tmpFx[77]*tmpObjSEndTerm[52] + tmpFx[95]*tmpObjSEndTerm[65] + tmpFx[113]*tmpObjSEndTerm[78] + tmpFx[131]*tmpObjSEndTerm[91] + tmpFx[149]*tmpObjSEndTerm[104] + tmpFx[167]*tmpObjSEndTerm[117] + tmpFx[185]*tmpObjSEndTerm[130] + tmpFx[203]*tmpObjSEndTerm[143] + tmpFx[221]*tmpObjSEndTerm[156];
tmpQN2[66] = + tmpFx[5]*tmpObjSEndTerm[1] + tmpFx[23]*tmpObjSEndTerm[14] + tmpFx[41]*tmpObjSEndTerm[27] + tmpFx[59]*tmpObjSEndTerm[40] + tmpFx[77]*tmpObjSEndTerm[53] + tmpFx[95]*tmpObjSEndTerm[66] + tmpFx[113]*tmpObjSEndTerm[79] + tmpFx[131]*tmpObjSEndTerm[92] + tmpFx[149]*tmpObjSEndTerm[105] + tmpFx[167]*tmpObjSEndTerm[118] + tmpFx[185]*tmpObjSEndTerm[131] + tmpFx[203]*tmpObjSEndTerm[144] + tmpFx[221]*tmpObjSEndTerm[157];
tmpQN2[67] = + tmpFx[5]*tmpObjSEndTerm[2] + tmpFx[23]*tmpObjSEndTerm[15] + tmpFx[41]*tmpObjSEndTerm[28] + tmpFx[59]*tmpObjSEndTerm[41] + tmpFx[77]*tmpObjSEndTerm[54] + tmpFx[95]*tmpObjSEndTerm[67] + tmpFx[113]*tmpObjSEndTerm[80] + tmpFx[131]*tmpObjSEndTerm[93] + tmpFx[149]*tmpObjSEndTerm[106] + tmpFx[167]*tmpObjSEndTerm[119] + tmpFx[185]*tmpObjSEndTerm[132] + tmpFx[203]*tmpObjSEndTerm[145] + tmpFx[221]*tmpObjSEndTerm[158];
tmpQN2[68] = + tmpFx[5]*tmpObjSEndTerm[3] + tmpFx[23]*tmpObjSEndTerm[16] + tmpFx[41]*tmpObjSEndTerm[29] + tmpFx[59]*tmpObjSEndTerm[42] + tmpFx[77]*tmpObjSEndTerm[55] + tmpFx[95]*tmpObjSEndTerm[68] + tmpFx[113]*tmpObjSEndTerm[81] + tmpFx[131]*tmpObjSEndTerm[94] + tmpFx[149]*tmpObjSEndTerm[107] + tmpFx[167]*tmpObjSEndTerm[120] + tmpFx[185]*tmpObjSEndTerm[133] + tmpFx[203]*tmpObjSEndTerm[146] + tmpFx[221]*tmpObjSEndTerm[159];
tmpQN2[69] = + tmpFx[5]*tmpObjSEndTerm[4] + tmpFx[23]*tmpObjSEndTerm[17] + tmpFx[41]*tmpObjSEndTerm[30] + tmpFx[59]*tmpObjSEndTerm[43] + tmpFx[77]*tmpObjSEndTerm[56] + tmpFx[95]*tmpObjSEndTerm[69] + tmpFx[113]*tmpObjSEndTerm[82] + tmpFx[131]*tmpObjSEndTerm[95] + tmpFx[149]*tmpObjSEndTerm[108] + tmpFx[167]*tmpObjSEndTerm[121] + tmpFx[185]*tmpObjSEndTerm[134] + tmpFx[203]*tmpObjSEndTerm[147] + tmpFx[221]*tmpObjSEndTerm[160];
tmpQN2[70] = + tmpFx[5]*tmpObjSEndTerm[5] + tmpFx[23]*tmpObjSEndTerm[18] + tmpFx[41]*tmpObjSEndTerm[31] + tmpFx[59]*tmpObjSEndTerm[44] + tmpFx[77]*tmpObjSEndTerm[57] + tmpFx[95]*tmpObjSEndTerm[70] + tmpFx[113]*tmpObjSEndTerm[83] + tmpFx[131]*tmpObjSEndTerm[96] + tmpFx[149]*tmpObjSEndTerm[109] + tmpFx[167]*tmpObjSEndTerm[122] + tmpFx[185]*tmpObjSEndTerm[135] + tmpFx[203]*tmpObjSEndTerm[148] + tmpFx[221]*tmpObjSEndTerm[161];
tmpQN2[71] = + tmpFx[5]*tmpObjSEndTerm[6] + tmpFx[23]*tmpObjSEndTerm[19] + tmpFx[41]*tmpObjSEndTerm[32] + tmpFx[59]*tmpObjSEndTerm[45] + tmpFx[77]*tmpObjSEndTerm[58] + tmpFx[95]*tmpObjSEndTerm[71] + tmpFx[113]*tmpObjSEndTerm[84] + tmpFx[131]*tmpObjSEndTerm[97] + tmpFx[149]*tmpObjSEndTerm[110] + tmpFx[167]*tmpObjSEndTerm[123] + tmpFx[185]*tmpObjSEndTerm[136] + tmpFx[203]*tmpObjSEndTerm[149] + tmpFx[221]*tmpObjSEndTerm[162];
tmpQN2[72] = + tmpFx[5]*tmpObjSEndTerm[7] + tmpFx[23]*tmpObjSEndTerm[20] + tmpFx[41]*tmpObjSEndTerm[33] + tmpFx[59]*tmpObjSEndTerm[46] + tmpFx[77]*tmpObjSEndTerm[59] + tmpFx[95]*tmpObjSEndTerm[72] + tmpFx[113]*tmpObjSEndTerm[85] + tmpFx[131]*tmpObjSEndTerm[98] + tmpFx[149]*tmpObjSEndTerm[111] + tmpFx[167]*tmpObjSEndTerm[124] + tmpFx[185]*tmpObjSEndTerm[137] + tmpFx[203]*tmpObjSEndTerm[150] + tmpFx[221]*tmpObjSEndTerm[163];
tmpQN2[73] = + tmpFx[5]*tmpObjSEndTerm[8] + tmpFx[23]*tmpObjSEndTerm[21] + tmpFx[41]*tmpObjSEndTerm[34] + tmpFx[59]*tmpObjSEndTerm[47] + tmpFx[77]*tmpObjSEndTerm[60] + tmpFx[95]*tmpObjSEndTerm[73] + tmpFx[113]*tmpObjSEndTerm[86] + tmpFx[131]*tmpObjSEndTerm[99] + tmpFx[149]*tmpObjSEndTerm[112] + tmpFx[167]*tmpObjSEndTerm[125] + tmpFx[185]*tmpObjSEndTerm[138] + tmpFx[203]*tmpObjSEndTerm[151] + tmpFx[221]*tmpObjSEndTerm[164];
tmpQN2[74] = + tmpFx[5]*tmpObjSEndTerm[9] + tmpFx[23]*tmpObjSEndTerm[22] + tmpFx[41]*tmpObjSEndTerm[35] + tmpFx[59]*tmpObjSEndTerm[48] + tmpFx[77]*tmpObjSEndTerm[61] + tmpFx[95]*tmpObjSEndTerm[74] + tmpFx[113]*tmpObjSEndTerm[87] + tmpFx[131]*tmpObjSEndTerm[100] + tmpFx[149]*tmpObjSEndTerm[113] + tmpFx[167]*tmpObjSEndTerm[126] + tmpFx[185]*tmpObjSEndTerm[139] + tmpFx[203]*tmpObjSEndTerm[152] + tmpFx[221]*tmpObjSEndTerm[165];
tmpQN2[75] = + tmpFx[5]*tmpObjSEndTerm[10] + tmpFx[23]*tmpObjSEndTerm[23] + tmpFx[41]*tmpObjSEndTerm[36] + tmpFx[59]*tmpObjSEndTerm[49] + tmpFx[77]*tmpObjSEndTerm[62] + tmpFx[95]*tmpObjSEndTerm[75] + tmpFx[113]*tmpObjSEndTerm[88] + tmpFx[131]*tmpObjSEndTerm[101] + tmpFx[149]*tmpObjSEndTerm[114] + tmpFx[167]*tmpObjSEndTerm[127] + tmpFx[185]*tmpObjSEndTerm[140] + tmpFx[203]*tmpObjSEndTerm[153] + tmpFx[221]*tmpObjSEndTerm[166];
tmpQN2[76] = + tmpFx[5]*tmpObjSEndTerm[11] + tmpFx[23]*tmpObjSEndTerm[24] + tmpFx[41]*tmpObjSEndTerm[37] + tmpFx[59]*tmpObjSEndTerm[50] + tmpFx[77]*tmpObjSEndTerm[63] + tmpFx[95]*tmpObjSEndTerm[76] + tmpFx[113]*tmpObjSEndTerm[89] + tmpFx[131]*tmpObjSEndTerm[102] + tmpFx[149]*tmpObjSEndTerm[115] + tmpFx[167]*tmpObjSEndTerm[128] + tmpFx[185]*tmpObjSEndTerm[141] + tmpFx[203]*tmpObjSEndTerm[154] + tmpFx[221]*tmpObjSEndTerm[167];
tmpQN2[77] = + tmpFx[5]*tmpObjSEndTerm[12] + tmpFx[23]*tmpObjSEndTerm[25] + tmpFx[41]*tmpObjSEndTerm[38] + tmpFx[59]*tmpObjSEndTerm[51] + tmpFx[77]*tmpObjSEndTerm[64] + tmpFx[95]*tmpObjSEndTerm[77] + tmpFx[113]*tmpObjSEndTerm[90] + tmpFx[131]*tmpObjSEndTerm[103] + tmpFx[149]*tmpObjSEndTerm[116] + tmpFx[167]*tmpObjSEndTerm[129] + tmpFx[185]*tmpObjSEndTerm[142] + tmpFx[203]*tmpObjSEndTerm[155] + tmpFx[221]*tmpObjSEndTerm[168];
tmpQN2[78] = + tmpFx[6]*tmpObjSEndTerm[0] + tmpFx[24]*tmpObjSEndTerm[13] + tmpFx[42]*tmpObjSEndTerm[26] + tmpFx[60]*tmpObjSEndTerm[39] + tmpFx[78]*tmpObjSEndTerm[52] + tmpFx[96]*tmpObjSEndTerm[65] + tmpFx[114]*tmpObjSEndTerm[78] + tmpFx[132]*tmpObjSEndTerm[91] + tmpFx[150]*tmpObjSEndTerm[104] + tmpFx[168]*tmpObjSEndTerm[117] + tmpFx[186]*tmpObjSEndTerm[130] + tmpFx[204]*tmpObjSEndTerm[143] + tmpFx[222]*tmpObjSEndTerm[156];
tmpQN2[79] = + tmpFx[6]*tmpObjSEndTerm[1] + tmpFx[24]*tmpObjSEndTerm[14] + tmpFx[42]*tmpObjSEndTerm[27] + tmpFx[60]*tmpObjSEndTerm[40] + tmpFx[78]*tmpObjSEndTerm[53] + tmpFx[96]*tmpObjSEndTerm[66] + tmpFx[114]*tmpObjSEndTerm[79] + tmpFx[132]*tmpObjSEndTerm[92] + tmpFx[150]*tmpObjSEndTerm[105] + tmpFx[168]*tmpObjSEndTerm[118] + tmpFx[186]*tmpObjSEndTerm[131] + tmpFx[204]*tmpObjSEndTerm[144] + tmpFx[222]*tmpObjSEndTerm[157];
tmpQN2[80] = + tmpFx[6]*tmpObjSEndTerm[2] + tmpFx[24]*tmpObjSEndTerm[15] + tmpFx[42]*tmpObjSEndTerm[28] + tmpFx[60]*tmpObjSEndTerm[41] + tmpFx[78]*tmpObjSEndTerm[54] + tmpFx[96]*tmpObjSEndTerm[67] + tmpFx[114]*tmpObjSEndTerm[80] + tmpFx[132]*tmpObjSEndTerm[93] + tmpFx[150]*tmpObjSEndTerm[106] + tmpFx[168]*tmpObjSEndTerm[119] + tmpFx[186]*tmpObjSEndTerm[132] + tmpFx[204]*tmpObjSEndTerm[145] + tmpFx[222]*tmpObjSEndTerm[158];
tmpQN2[81] = + tmpFx[6]*tmpObjSEndTerm[3] + tmpFx[24]*tmpObjSEndTerm[16] + tmpFx[42]*tmpObjSEndTerm[29] + tmpFx[60]*tmpObjSEndTerm[42] + tmpFx[78]*tmpObjSEndTerm[55] + tmpFx[96]*tmpObjSEndTerm[68] + tmpFx[114]*tmpObjSEndTerm[81] + tmpFx[132]*tmpObjSEndTerm[94] + tmpFx[150]*tmpObjSEndTerm[107] + tmpFx[168]*tmpObjSEndTerm[120] + tmpFx[186]*tmpObjSEndTerm[133] + tmpFx[204]*tmpObjSEndTerm[146] + tmpFx[222]*tmpObjSEndTerm[159];
tmpQN2[82] = + tmpFx[6]*tmpObjSEndTerm[4] + tmpFx[24]*tmpObjSEndTerm[17] + tmpFx[42]*tmpObjSEndTerm[30] + tmpFx[60]*tmpObjSEndTerm[43] + tmpFx[78]*tmpObjSEndTerm[56] + tmpFx[96]*tmpObjSEndTerm[69] + tmpFx[114]*tmpObjSEndTerm[82] + tmpFx[132]*tmpObjSEndTerm[95] + tmpFx[150]*tmpObjSEndTerm[108] + tmpFx[168]*tmpObjSEndTerm[121] + tmpFx[186]*tmpObjSEndTerm[134] + tmpFx[204]*tmpObjSEndTerm[147] + tmpFx[222]*tmpObjSEndTerm[160];
tmpQN2[83] = + tmpFx[6]*tmpObjSEndTerm[5] + tmpFx[24]*tmpObjSEndTerm[18] + tmpFx[42]*tmpObjSEndTerm[31] + tmpFx[60]*tmpObjSEndTerm[44] + tmpFx[78]*tmpObjSEndTerm[57] + tmpFx[96]*tmpObjSEndTerm[70] + tmpFx[114]*tmpObjSEndTerm[83] + tmpFx[132]*tmpObjSEndTerm[96] + tmpFx[150]*tmpObjSEndTerm[109] + tmpFx[168]*tmpObjSEndTerm[122] + tmpFx[186]*tmpObjSEndTerm[135] + tmpFx[204]*tmpObjSEndTerm[148] + tmpFx[222]*tmpObjSEndTerm[161];
tmpQN2[84] = + tmpFx[6]*tmpObjSEndTerm[6] + tmpFx[24]*tmpObjSEndTerm[19] + tmpFx[42]*tmpObjSEndTerm[32] + tmpFx[60]*tmpObjSEndTerm[45] + tmpFx[78]*tmpObjSEndTerm[58] + tmpFx[96]*tmpObjSEndTerm[71] + tmpFx[114]*tmpObjSEndTerm[84] + tmpFx[132]*tmpObjSEndTerm[97] + tmpFx[150]*tmpObjSEndTerm[110] + tmpFx[168]*tmpObjSEndTerm[123] + tmpFx[186]*tmpObjSEndTerm[136] + tmpFx[204]*tmpObjSEndTerm[149] + tmpFx[222]*tmpObjSEndTerm[162];
tmpQN2[85] = + tmpFx[6]*tmpObjSEndTerm[7] + tmpFx[24]*tmpObjSEndTerm[20] + tmpFx[42]*tmpObjSEndTerm[33] + tmpFx[60]*tmpObjSEndTerm[46] + tmpFx[78]*tmpObjSEndTerm[59] + tmpFx[96]*tmpObjSEndTerm[72] + tmpFx[114]*tmpObjSEndTerm[85] + tmpFx[132]*tmpObjSEndTerm[98] + tmpFx[150]*tmpObjSEndTerm[111] + tmpFx[168]*tmpObjSEndTerm[124] + tmpFx[186]*tmpObjSEndTerm[137] + tmpFx[204]*tmpObjSEndTerm[150] + tmpFx[222]*tmpObjSEndTerm[163];
tmpQN2[86] = + tmpFx[6]*tmpObjSEndTerm[8] + tmpFx[24]*tmpObjSEndTerm[21] + tmpFx[42]*tmpObjSEndTerm[34] + tmpFx[60]*tmpObjSEndTerm[47] + tmpFx[78]*tmpObjSEndTerm[60] + tmpFx[96]*tmpObjSEndTerm[73] + tmpFx[114]*tmpObjSEndTerm[86] + tmpFx[132]*tmpObjSEndTerm[99] + tmpFx[150]*tmpObjSEndTerm[112] + tmpFx[168]*tmpObjSEndTerm[125] + tmpFx[186]*tmpObjSEndTerm[138] + tmpFx[204]*tmpObjSEndTerm[151] + tmpFx[222]*tmpObjSEndTerm[164];
tmpQN2[87] = + tmpFx[6]*tmpObjSEndTerm[9] + tmpFx[24]*tmpObjSEndTerm[22] + tmpFx[42]*tmpObjSEndTerm[35] + tmpFx[60]*tmpObjSEndTerm[48] + tmpFx[78]*tmpObjSEndTerm[61] + tmpFx[96]*tmpObjSEndTerm[74] + tmpFx[114]*tmpObjSEndTerm[87] + tmpFx[132]*tmpObjSEndTerm[100] + tmpFx[150]*tmpObjSEndTerm[113] + tmpFx[168]*tmpObjSEndTerm[126] + tmpFx[186]*tmpObjSEndTerm[139] + tmpFx[204]*tmpObjSEndTerm[152] + tmpFx[222]*tmpObjSEndTerm[165];
tmpQN2[88] = + tmpFx[6]*tmpObjSEndTerm[10] + tmpFx[24]*tmpObjSEndTerm[23] + tmpFx[42]*tmpObjSEndTerm[36] + tmpFx[60]*tmpObjSEndTerm[49] + tmpFx[78]*tmpObjSEndTerm[62] + tmpFx[96]*tmpObjSEndTerm[75] + tmpFx[114]*tmpObjSEndTerm[88] + tmpFx[132]*tmpObjSEndTerm[101] + tmpFx[150]*tmpObjSEndTerm[114] + tmpFx[168]*tmpObjSEndTerm[127] + tmpFx[186]*tmpObjSEndTerm[140] + tmpFx[204]*tmpObjSEndTerm[153] + tmpFx[222]*tmpObjSEndTerm[166];
tmpQN2[89] = + tmpFx[6]*tmpObjSEndTerm[11] + tmpFx[24]*tmpObjSEndTerm[24] + tmpFx[42]*tmpObjSEndTerm[37] + tmpFx[60]*tmpObjSEndTerm[50] + tmpFx[78]*tmpObjSEndTerm[63] + tmpFx[96]*tmpObjSEndTerm[76] + tmpFx[114]*tmpObjSEndTerm[89] + tmpFx[132]*tmpObjSEndTerm[102] + tmpFx[150]*tmpObjSEndTerm[115] + tmpFx[168]*tmpObjSEndTerm[128] + tmpFx[186]*tmpObjSEndTerm[141] + tmpFx[204]*tmpObjSEndTerm[154] + tmpFx[222]*tmpObjSEndTerm[167];
tmpQN2[90] = + tmpFx[6]*tmpObjSEndTerm[12] + tmpFx[24]*tmpObjSEndTerm[25] + tmpFx[42]*tmpObjSEndTerm[38] + tmpFx[60]*tmpObjSEndTerm[51] + tmpFx[78]*tmpObjSEndTerm[64] + tmpFx[96]*tmpObjSEndTerm[77] + tmpFx[114]*tmpObjSEndTerm[90] + tmpFx[132]*tmpObjSEndTerm[103] + tmpFx[150]*tmpObjSEndTerm[116] + tmpFx[168]*tmpObjSEndTerm[129] + tmpFx[186]*tmpObjSEndTerm[142] + tmpFx[204]*tmpObjSEndTerm[155] + tmpFx[222]*tmpObjSEndTerm[168];
tmpQN2[91] = + tmpFx[7]*tmpObjSEndTerm[0] + tmpFx[25]*tmpObjSEndTerm[13] + tmpFx[43]*tmpObjSEndTerm[26] + tmpFx[61]*tmpObjSEndTerm[39] + tmpFx[79]*tmpObjSEndTerm[52] + tmpFx[97]*tmpObjSEndTerm[65] + tmpFx[115]*tmpObjSEndTerm[78] + tmpFx[133]*tmpObjSEndTerm[91] + tmpFx[151]*tmpObjSEndTerm[104] + tmpFx[169]*tmpObjSEndTerm[117] + tmpFx[187]*tmpObjSEndTerm[130] + tmpFx[205]*tmpObjSEndTerm[143] + tmpFx[223]*tmpObjSEndTerm[156];
tmpQN2[92] = + tmpFx[7]*tmpObjSEndTerm[1] + tmpFx[25]*tmpObjSEndTerm[14] + tmpFx[43]*tmpObjSEndTerm[27] + tmpFx[61]*tmpObjSEndTerm[40] + tmpFx[79]*tmpObjSEndTerm[53] + tmpFx[97]*tmpObjSEndTerm[66] + tmpFx[115]*tmpObjSEndTerm[79] + tmpFx[133]*tmpObjSEndTerm[92] + tmpFx[151]*tmpObjSEndTerm[105] + tmpFx[169]*tmpObjSEndTerm[118] + tmpFx[187]*tmpObjSEndTerm[131] + tmpFx[205]*tmpObjSEndTerm[144] + tmpFx[223]*tmpObjSEndTerm[157];
tmpQN2[93] = + tmpFx[7]*tmpObjSEndTerm[2] + tmpFx[25]*tmpObjSEndTerm[15] + tmpFx[43]*tmpObjSEndTerm[28] + tmpFx[61]*tmpObjSEndTerm[41] + tmpFx[79]*tmpObjSEndTerm[54] + tmpFx[97]*tmpObjSEndTerm[67] + tmpFx[115]*tmpObjSEndTerm[80] + tmpFx[133]*tmpObjSEndTerm[93] + tmpFx[151]*tmpObjSEndTerm[106] + tmpFx[169]*tmpObjSEndTerm[119] + tmpFx[187]*tmpObjSEndTerm[132] + tmpFx[205]*tmpObjSEndTerm[145] + tmpFx[223]*tmpObjSEndTerm[158];
tmpQN2[94] = + tmpFx[7]*tmpObjSEndTerm[3] + tmpFx[25]*tmpObjSEndTerm[16] + tmpFx[43]*tmpObjSEndTerm[29] + tmpFx[61]*tmpObjSEndTerm[42] + tmpFx[79]*tmpObjSEndTerm[55] + tmpFx[97]*tmpObjSEndTerm[68] + tmpFx[115]*tmpObjSEndTerm[81] + tmpFx[133]*tmpObjSEndTerm[94] + tmpFx[151]*tmpObjSEndTerm[107] + tmpFx[169]*tmpObjSEndTerm[120] + tmpFx[187]*tmpObjSEndTerm[133] + tmpFx[205]*tmpObjSEndTerm[146] + tmpFx[223]*tmpObjSEndTerm[159];
tmpQN2[95] = + tmpFx[7]*tmpObjSEndTerm[4] + tmpFx[25]*tmpObjSEndTerm[17] + tmpFx[43]*tmpObjSEndTerm[30] + tmpFx[61]*tmpObjSEndTerm[43] + tmpFx[79]*tmpObjSEndTerm[56] + tmpFx[97]*tmpObjSEndTerm[69] + tmpFx[115]*tmpObjSEndTerm[82] + tmpFx[133]*tmpObjSEndTerm[95] + tmpFx[151]*tmpObjSEndTerm[108] + tmpFx[169]*tmpObjSEndTerm[121] + tmpFx[187]*tmpObjSEndTerm[134] + tmpFx[205]*tmpObjSEndTerm[147] + tmpFx[223]*tmpObjSEndTerm[160];
tmpQN2[96] = + tmpFx[7]*tmpObjSEndTerm[5] + tmpFx[25]*tmpObjSEndTerm[18] + tmpFx[43]*tmpObjSEndTerm[31] + tmpFx[61]*tmpObjSEndTerm[44] + tmpFx[79]*tmpObjSEndTerm[57] + tmpFx[97]*tmpObjSEndTerm[70] + tmpFx[115]*tmpObjSEndTerm[83] + tmpFx[133]*tmpObjSEndTerm[96] + tmpFx[151]*tmpObjSEndTerm[109] + tmpFx[169]*tmpObjSEndTerm[122] + tmpFx[187]*tmpObjSEndTerm[135] + tmpFx[205]*tmpObjSEndTerm[148] + tmpFx[223]*tmpObjSEndTerm[161];
tmpQN2[97] = + tmpFx[7]*tmpObjSEndTerm[6] + tmpFx[25]*tmpObjSEndTerm[19] + tmpFx[43]*tmpObjSEndTerm[32] + tmpFx[61]*tmpObjSEndTerm[45] + tmpFx[79]*tmpObjSEndTerm[58] + tmpFx[97]*tmpObjSEndTerm[71] + tmpFx[115]*tmpObjSEndTerm[84] + tmpFx[133]*tmpObjSEndTerm[97] + tmpFx[151]*tmpObjSEndTerm[110] + tmpFx[169]*tmpObjSEndTerm[123] + tmpFx[187]*tmpObjSEndTerm[136] + tmpFx[205]*tmpObjSEndTerm[149] + tmpFx[223]*tmpObjSEndTerm[162];
tmpQN2[98] = + tmpFx[7]*tmpObjSEndTerm[7] + tmpFx[25]*tmpObjSEndTerm[20] + tmpFx[43]*tmpObjSEndTerm[33] + tmpFx[61]*tmpObjSEndTerm[46] + tmpFx[79]*tmpObjSEndTerm[59] + tmpFx[97]*tmpObjSEndTerm[72] + tmpFx[115]*tmpObjSEndTerm[85] + tmpFx[133]*tmpObjSEndTerm[98] + tmpFx[151]*tmpObjSEndTerm[111] + tmpFx[169]*tmpObjSEndTerm[124] + tmpFx[187]*tmpObjSEndTerm[137] + tmpFx[205]*tmpObjSEndTerm[150] + tmpFx[223]*tmpObjSEndTerm[163];
tmpQN2[99] = + tmpFx[7]*tmpObjSEndTerm[8] + tmpFx[25]*tmpObjSEndTerm[21] + tmpFx[43]*tmpObjSEndTerm[34] + tmpFx[61]*tmpObjSEndTerm[47] + tmpFx[79]*tmpObjSEndTerm[60] + tmpFx[97]*tmpObjSEndTerm[73] + tmpFx[115]*tmpObjSEndTerm[86] + tmpFx[133]*tmpObjSEndTerm[99] + tmpFx[151]*tmpObjSEndTerm[112] + tmpFx[169]*tmpObjSEndTerm[125] + tmpFx[187]*tmpObjSEndTerm[138] + tmpFx[205]*tmpObjSEndTerm[151] + tmpFx[223]*tmpObjSEndTerm[164];
tmpQN2[100] = + tmpFx[7]*tmpObjSEndTerm[9] + tmpFx[25]*tmpObjSEndTerm[22] + tmpFx[43]*tmpObjSEndTerm[35] + tmpFx[61]*tmpObjSEndTerm[48] + tmpFx[79]*tmpObjSEndTerm[61] + tmpFx[97]*tmpObjSEndTerm[74] + tmpFx[115]*tmpObjSEndTerm[87] + tmpFx[133]*tmpObjSEndTerm[100] + tmpFx[151]*tmpObjSEndTerm[113] + tmpFx[169]*tmpObjSEndTerm[126] + tmpFx[187]*tmpObjSEndTerm[139] + tmpFx[205]*tmpObjSEndTerm[152] + tmpFx[223]*tmpObjSEndTerm[165];
tmpQN2[101] = + tmpFx[7]*tmpObjSEndTerm[10] + tmpFx[25]*tmpObjSEndTerm[23] + tmpFx[43]*tmpObjSEndTerm[36] + tmpFx[61]*tmpObjSEndTerm[49] + tmpFx[79]*tmpObjSEndTerm[62] + tmpFx[97]*tmpObjSEndTerm[75] + tmpFx[115]*tmpObjSEndTerm[88] + tmpFx[133]*tmpObjSEndTerm[101] + tmpFx[151]*tmpObjSEndTerm[114] + tmpFx[169]*tmpObjSEndTerm[127] + tmpFx[187]*tmpObjSEndTerm[140] + tmpFx[205]*tmpObjSEndTerm[153] + tmpFx[223]*tmpObjSEndTerm[166];
tmpQN2[102] = + tmpFx[7]*tmpObjSEndTerm[11] + tmpFx[25]*tmpObjSEndTerm[24] + tmpFx[43]*tmpObjSEndTerm[37] + tmpFx[61]*tmpObjSEndTerm[50] + tmpFx[79]*tmpObjSEndTerm[63] + tmpFx[97]*tmpObjSEndTerm[76] + tmpFx[115]*tmpObjSEndTerm[89] + tmpFx[133]*tmpObjSEndTerm[102] + tmpFx[151]*tmpObjSEndTerm[115] + tmpFx[169]*tmpObjSEndTerm[128] + tmpFx[187]*tmpObjSEndTerm[141] + tmpFx[205]*tmpObjSEndTerm[154] + tmpFx[223]*tmpObjSEndTerm[167];
tmpQN2[103] = + tmpFx[7]*tmpObjSEndTerm[12] + tmpFx[25]*tmpObjSEndTerm[25] + tmpFx[43]*tmpObjSEndTerm[38] + tmpFx[61]*tmpObjSEndTerm[51] + tmpFx[79]*tmpObjSEndTerm[64] + tmpFx[97]*tmpObjSEndTerm[77] + tmpFx[115]*tmpObjSEndTerm[90] + tmpFx[133]*tmpObjSEndTerm[103] + tmpFx[151]*tmpObjSEndTerm[116] + tmpFx[169]*tmpObjSEndTerm[129] + tmpFx[187]*tmpObjSEndTerm[142] + tmpFx[205]*tmpObjSEndTerm[155] + tmpFx[223]*tmpObjSEndTerm[168];
tmpQN2[104] = + tmpFx[8]*tmpObjSEndTerm[0] + tmpFx[26]*tmpObjSEndTerm[13] + tmpFx[44]*tmpObjSEndTerm[26] + tmpFx[62]*tmpObjSEndTerm[39] + tmpFx[80]*tmpObjSEndTerm[52] + tmpFx[98]*tmpObjSEndTerm[65] + tmpFx[116]*tmpObjSEndTerm[78] + tmpFx[134]*tmpObjSEndTerm[91] + tmpFx[152]*tmpObjSEndTerm[104] + tmpFx[170]*tmpObjSEndTerm[117] + tmpFx[188]*tmpObjSEndTerm[130] + tmpFx[206]*tmpObjSEndTerm[143] + tmpFx[224]*tmpObjSEndTerm[156];
tmpQN2[105] = + tmpFx[8]*tmpObjSEndTerm[1] + tmpFx[26]*tmpObjSEndTerm[14] + tmpFx[44]*tmpObjSEndTerm[27] + tmpFx[62]*tmpObjSEndTerm[40] + tmpFx[80]*tmpObjSEndTerm[53] + tmpFx[98]*tmpObjSEndTerm[66] + tmpFx[116]*tmpObjSEndTerm[79] + tmpFx[134]*tmpObjSEndTerm[92] + tmpFx[152]*tmpObjSEndTerm[105] + tmpFx[170]*tmpObjSEndTerm[118] + tmpFx[188]*tmpObjSEndTerm[131] + tmpFx[206]*tmpObjSEndTerm[144] + tmpFx[224]*tmpObjSEndTerm[157];
tmpQN2[106] = + tmpFx[8]*tmpObjSEndTerm[2] + tmpFx[26]*tmpObjSEndTerm[15] + tmpFx[44]*tmpObjSEndTerm[28] + tmpFx[62]*tmpObjSEndTerm[41] + tmpFx[80]*tmpObjSEndTerm[54] + tmpFx[98]*tmpObjSEndTerm[67] + tmpFx[116]*tmpObjSEndTerm[80] + tmpFx[134]*tmpObjSEndTerm[93] + tmpFx[152]*tmpObjSEndTerm[106] + tmpFx[170]*tmpObjSEndTerm[119] + tmpFx[188]*tmpObjSEndTerm[132] + tmpFx[206]*tmpObjSEndTerm[145] + tmpFx[224]*tmpObjSEndTerm[158];
tmpQN2[107] = + tmpFx[8]*tmpObjSEndTerm[3] + tmpFx[26]*tmpObjSEndTerm[16] + tmpFx[44]*tmpObjSEndTerm[29] + tmpFx[62]*tmpObjSEndTerm[42] + tmpFx[80]*tmpObjSEndTerm[55] + tmpFx[98]*tmpObjSEndTerm[68] + tmpFx[116]*tmpObjSEndTerm[81] + tmpFx[134]*tmpObjSEndTerm[94] + tmpFx[152]*tmpObjSEndTerm[107] + tmpFx[170]*tmpObjSEndTerm[120] + tmpFx[188]*tmpObjSEndTerm[133] + tmpFx[206]*tmpObjSEndTerm[146] + tmpFx[224]*tmpObjSEndTerm[159];
tmpQN2[108] = + tmpFx[8]*tmpObjSEndTerm[4] + tmpFx[26]*tmpObjSEndTerm[17] + tmpFx[44]*tmpObjSEndTerm[30] + tmpFx[62]*tmpObjSEndTerm[43] + tmpFx[80]*tmpObjSEndTerm[56] + tmpFx[98]*tmpObjSEndTerm[69] + tmpFx[116]*tmpObjSEndTerm[82] + tmpFx[134]*tmpObjSEndTerm[95] + tmpFx[152]*tmpObjSEndTerm[108] + tmpFx[170]*tmpObjSEndTerm[121] + tmpFx[188]*tmpObjSEndTerm[134] + tmpFx[206]*tmpObjSEndTerm[147] + tmpFx[224]*tmpObjSEndTerm[160];
tmpQN2[109] = + tmpFx[8]*tmpObjSEndTerm[5] + tmpFx[26]*tmpObjSEndTerm[18] + tmpFx[44]*tmpObjSEndTerm[31] + tmpFx[62]*tmpObjSEndTerm[44] + tmpFx[80]*tmpObjSEndTerm[57] + tmpFx[98]*tmpObjSEndTerm[70] + tmpFx[116]*tmpObjSEndTerm[83] + tmpFx[134]*tmpObjSEndTerm[96] + tmpFx[152]*tmpObjSEndTerm[109] + tmpFx[170]*tmpObjSEndTerm[122] + tmpFx[188]*tmpObjSEndTerm[135] + tmpFx[206]*tmpObjSEndTerm[148] + tmpFx[224]*tmpObjSEndTerm[161];
tmpQN2[110] = + tmpFx[8]*tmpObjSEndTerm[6] + tmpFx[26]*tmpObjSEndTerm[19] + tmpFx[44]*tmpObjSEndTerm[32] + tmpFx[62]*tmpObjSEndTerm[45] + tmpFx[80]*tmpObjSEndTerm[58] + tmpFx[98]*tmpObjSEndTerm[71] + tmpFx[116]*tmpObjSEndTerm[84] + tmpFx[134]*tmpObjSEndTerm[97] + tmpFx[152]*tmpObjSEndTerm[110] + tmpFx[170]*tmpObjSEndTerm[123] + tmpFx[188]*tmpObjSEndTerm[136] + tmpFx[206]*tmpObjSEndTerm[149] + tmpFx[224]*tmpObjSEndTerm[162];
tmpQN2[111] = + tmpFx[8]*tmpObjSEndTerm[7] + tmpFx[26]*tmpObjSEndTerm[20] + tmpFx[44]*tmpObjSEndTerm[33] + tmpFx[62]*tmpObjSEndTerm[46] + tmpFx[80]*tmpObjSEndTerm[59] + tmpFx[98]*tmpObjSEndTerm[72] + tmpFx[116]*tmpObjSEndTerm[85] + tmpFx[134]*tmpObjSEndTerm[98] + tmpFx[152]*tmpObjSEndTerm[111] + tmpFx[170]*tmpObjSEndTerm[124] + tmpFx[188]*tmpObjSEndTerm[137] + tmpFx[206]*tmpObjSEndTerm[150] + tmpFx[224]*tmpObjSEndTerm[163];
tmpQN2[112] = + tmpFx[8]*tmpObjSEndTerm[8] + tmpFx[26]*tmpObjSEndTerm[21] + tmpFx[44]*tmpObjSEndTerm[34] + tmpFx[62]*tmpObjSEndTerm[47] + tmpFx[80]*tmpObjSEndTerm[60] + tmpFx[98]*tmpObjSEndTerm[73] + tmpFx[116]*tmpObjSEndTerm[86] + tmpFx[134]*tmpObjSEndTerm[99] + tmpFx[152]*tmpObjSEndTerm[112] + tmpFx[170]*tmpObjSEndTerm[125] + tmpFx[188]*tmpObjSEndTerm[138] + tmpFx[206]*tmpObjSEndTerm[151] + tmpFx[224]*tmpObjSEndTerm[164];
tmpQN2[113] = + tmpFx[8]*tmpObjSEndTerm[9] + tmpFx[26]*tmpObjSEndTerm[22] + tmpFx[44]*tmpObjSEndTerm[35] + tmpFx[62]*tmpObjSEndTerm[48] + tmpFx[80]*tmpObjSEndTerm[61] + tmpFx[98]*tmpObjSEndTerm[74] + tmpFx[116]*tmpObjSEndTerm[87] + tmpFx[134]*tmpObjSEndTerm[100] + tmpFx[152]*tmpObjSEndTerm[113] + tmpFx[170]*tmpObjSEndTerm[126] + tmpFx[188]*tmpObjSEndTerm[139] + tmpFx[206]*tmpObjSEndTerm[152] + tmpFx[224]*tmpObjSEndTerm[165];
tmpQN2[114] = + tmpFx[8]*tmpObjSEndTerm[10] + tmpFx[26]*tmpObjSEndTerm[23] + tmpFx[44]*tmpObjSEndTerm[36] + tmpFx[62]*tmpObjSEndTerm[49] + tmpFx[80]*tmpObjSEndTerm[62] + tmpFx[98]*tmpObjSEndTerm[75] + tmpFx[116]*tmpObjSEndTerm[88] + tmpFx[134]*tmpObjSEndTerm[101] + tmpFx[152]*tmpObjSEndTerm[114] + tmpFx[170]*tmpObjSEndTerm[127] + tmpFx[188]*tmpObjSEndTerm[140] + tmpFx[206]*tmpObjSEndTerm[153] + tmpFx[224]*tmpObjSEndTerm[166];
tmpQN2[115] = + tmpFx[8]*tmpObjSEndTerm[11] + tmpFx[26]*tmpObjSEndTerm[24] + tmpFx[44]*tmpObjSEndTerm[37] + tmpFx[62]*tmpObjSEndTerm[50] + tmpFx[80]*tmpObjSEndTerm[63] + tmpFx[98]*tmpObjSEndTerm[76] + tmpFx[116]*tmpObjSEndTerm[89] + tmpFx[134]*tmpObjSEndTerm[102] + tmpFx[152]*tmpObjSEndTerm[115] + tmpFx[170]*tmpObjSEndTerm[128] + tmpFx[188]*tmpObjSEndTerm[141] + tmpFx[206]*tmpObjSEndTerm[154] + tmpFx[224]*tmpObjSEndTerm[167];
tmpQN2[116] = + tmpFx[8]*tmpObjSEndTerm[12] + tmpFx[26]*tmpObjSEndTerm[25] + tmpFx[44]*tmpObjSEndTerm[38] + tmpFx[62]*tmpObjSEndTerm[51] + tmpFx[80]*tmpObjSEndTerm[64] + tmpFx[98]*tmpObjSEndTerm[77] + tmpFx[116]*tmpObjSEndTerm[90] + tmpFx[134]*tmpObjSEndTerm[103] + tmpFx[152]*tmpObjSEndTerm[116] + tmpFx[170]*tmpObjSEndTerm[129] + tmpFx[188]*tmpObjSEndTerm[142] + tmpFx[206]*tmpObjSEndTerm[155] + tmpFx[224]*tmpObjSEndTerm[168];
tmpQN2[117] = + tmpFx[9]*tmpObjSEndTerm[0] + tmpFx[27]*tmpObjSEndTerm[13] + tmpFx[45]*tmpObjSEndTerm[26] + tmpFx[63]*tmpObjSEndTerm[39] + tmpFx[81]*tmpObjSEndTerm[52] + tmpFx[99]*tmpObjSEndTerm[65] + tmpFx[117]*tmpObjSEndTerm[78] + tmpFx[135]*tmpObjSEndTerm[91] + tmpFx[153]*tmpObjSEndTerm[104] + tmpFx[171]*tmpObjSEndTerm[117] + tmpFx[189]*tmpObjSEndTerm[130] + tmpFx[207]*tmpObjSEndTerm[143] + tmpFx[225]*tmpObjSEndTerm[156];
tmpQN2[118] = + tmpFx[9]*tmpObjSEndTerm[1] + tmpFx[27]*tmpObjSEndTerm[14] + tmpFx[45]*tmpObjSEndTerm[27] + tmpFx[63]*tmpObjSEndTerm[40] + tmpFx[81]*tmpObjSEndTerm[53] + tmpFx[99]*tmpObjSEndTerm[66] + tmpFx[117]*tmpObjSEndTerm[79] + tmpFx[135]*tmpObjSEndTerm[92] + tmpFx[153]*tmpObjSEndTerm[105] + tmpFx[171]*tmpObjSEndTerm[118] + tmpFx[189]*tmpObjSEndTerm[131] + tmpFx[207]*tmpObjSEndTerm[144] + tmpFx[225]*tmpObjSEndTerm[157];
tmpQN2[119] = + tmpFx[9]*tmpObjSEndTerm[2] + tmpFx[27]*tmpObjSEndTerm[15] + tmpFx[45]*tmpObjSEndTerm[28] + tmpFx[63]*tmpObjSEndTerm[41] + tmpFx[81]*tmpObjSEndTerm[54] + tmpFx[99]*tmpObjSEndTerm[67] + tmpFx[117]*tmpObjSEndTerm[80] + tmpFx[135]*tmpObjSEndTerm[93] + tmpFx[153]*tmpObjSEndTerm[106] + tmpFx[171]*tmpObjSEndTerm[119] + tmpFx[189]*tmpObjSEndTerm[132] + tmpFx[207]*tmpObjSEndTerm[145] + tmpFx[225]*tmpObjSEndTerm[158];
tmpQN2[120] = + tmpFx[9]*tmpObjSEndTerm[3] + tmpFx[27]*tmpObjSEndTerm[16] + tmpFx[45]*tmpObjSEndTerm[29] + tmpFx[63]*tmpObjSEndTerm[42] + tmpFx[81]*tmpObjSEndTerm[55] + tmpFx[99]*tmpObjSEndTerm[68] + tmpFx[117]*tmpObjSEndTerm[81] + tmpFx[135]*tmpObjSEndTerm[94] + tmpFx[153]*tmpObjSEndTerm[107] + tmpFx[171]*tmpObjSEndTerm[120] + tmpFx[189]*tmpObjSEndTerm[133] + tmpFx[207]*tmpObjSEndTerm[146] + tmpFx[225]*tmpObjSEndTerm[159];
tmpQN2[121] = + tmpFx[9]*tmpObjSEndTerm[4] + tmpFx[27]*tmpObjSEndTerm[17] + tmpFx[45]*tmpObjSEndTerm[30] + tmpFx[63]*tmpObjSEndTerm[43] + tmpFx[81]*tmpObjSEndTerm[56] + tmpFx[99]*tmpObjSEndTerm[69] + tmpFx[117]*tmpObjSEndTerm[82] + tmpFx[135]*tmpObjSEndTerm[95] + tmpFx[153]*tmpObjSEndTerm[108] + tmpFx[171]*tmpObjSEndTerm[121] + tmpFx[189]*tmpObjSEndTerm[134] + tmpFx[207]*tmpObjSEndTerm[147] + tmpFx[225]*tmpObjSEndTerm[160];
tmpQN2[122] = + tmpFx[9]*tmpObjSEndTerm[5] + tmpFx[27]*tmpObjSEndTerm[18] + tmpFx[45]*tmpObjSEndTerm[31] + tmpFx[63]*tmpObjSEndTerm[44] + tmpFx[81]*tmpObjSEndTerm[57] + tmpFx[99]*tmpObjSEndTerm[70] + tmpFx[117]*tmpObjSEndTerm[83] + tmpFx[135]*tmpObjSEndTerm[96] + tmpFx[153]*tmpObjSEndTerm[109] + tmpFx[171]*tmpObjSEndTerm[122] + tmpFx[189]*tmpObjSEndTerm[135] + tmpFx[207]*tmpObjSEndTerm[148] + tmpFx[225]*tmpObjSEndTerm[161];
tmpQN2[123] = + tmpFx[9]*tmpObjSEndTerm[6] + tmpFx[27]*tmpObjSEndTerm[19] + tmpFx[45]*tmpObjSEndTerm[32] + tmpFx[63]*tmpObjSEndTerm[45] + tmpFx[81]*tmpObjSEndTerm[58] + tmpFx[99]*tmpObjSEndTerm[71] + tmpFx[117]*tmpObjSEndTerm[84] + tmpFx[135]*tmpObjSEndTerm[97] + tmpFx[153]*tmpObjSEndTerm[110] + tmpFx[171]*tmpObjSEndTerm[123] + tmpFx[189]*tmpObjSEndTerm[136] + tmpFx[207]*tmpObjSEndTerm[149] + tmpFx[225]*tmpObjSEndTerm[162];
tmpQN2[124] = + tmpFx[9]*tmpObjSEndTerm[7] + tmpFx[27]*tmpObjSEndTerm[20] + tmpFx[45]*tmpObjSEndTerm[33] + tmpFx[63]*tmpObjSEndTerm[46] + tmpFx[81]*tmpObjSEndTerm[59] + tmpFx[99]*tmpObjSEndTerm[72] + tmpFx[117]*tmpObjSEndTerm[85] + tmpFx[135]*tmpObjSEndTerm[98] + tmpFx[153]*tmpObjSEndTerm[111] + tmpFx[171]*tmpObjSEndTerm[124] + tmpFx[189]*tmpObjSEndTerm[137] + tmpFx[207]*tmpObjSEndTerm[150] + tmpFx[225]*tmpObjSEndTerm[163];
tmpQN2[125] = + tmpFx[9]*tmpObjSEndTerm[8] + tmpFx[27]*tmpObjSEndTerm[21] + tmpFx[45]*tmpObjSEndTerm[34] + tmpFx[63]*tmpObjSEndTerm[47] + tmpFx[81]*tmpObjSEndTerm[60] + tmpFx[99]*tmpObjSEndTerm[73] + tmpFx[117]*tmpObjSEndTerm[86] + tmpFx[135]*tmpObjSEndTerm[99] + tmpFx[153]*tmpObjSEndTerm[112] + tmpFx[171]*tmpObjSEndTerm[125] + tmpFx[189]*tmpObjSEndTerm[138] + tmpFx[207]*tmpObjSEndTerm[151] + tmpFx[225]*tmpObjSEndTerm[164];
tmpQN2[126] = + tmpFx[9]*tmpObjSEndTerm[9] + tmpFx[27]*tmpObjSEndTerm[22] + tmpFx[45]*tmpObjSEndTerm[35] + tmpFx[63]*tmpObjSEndTerm[48] + tmpFx[81]*tmpObjSEndTerm[61] + tmpFx[99]*tmpObjSEndTerm[74] + tmpFx[117]*tmpObjSEndTerm[87] + tmpFx[135]*tmpObjSEndTerm[100] + tmpFx[153]*tmpObjSEndTerm[113] + tmpFx[171]*tmpObjSEndTerm[126] + tmpFx[189]*tmpObjSEndTerm[139] + tmpFx[207]*tmpObjSEndTerm[152] + tmpFx[225]*tmpObjSEndTerm[165];
tmpQN2[127] = + tmpFx[9]*tmpObjSEndTerm[10] + tmpFx[27]*tmpObjSEndTerm[23] + tmpFx[45]*tmpObjSEndTerm[36] + tmpFx[63]*tmpObjSEndTerm[49] + tmpFx[81]*tmpObjSEndTerm[62] + tmpFx[99]*tmpObjSEndTerm[75] + tmpFx[117]*tmpObjSEndTerm[88] + tmpFx[135]*tmpObjSEndTerm[101] + tmpFx[153]*tmpObjSEndTerm[114] + tmpFx[171]*tmpObjSEndTerm[127] + tmpFx[189]*tmpObjSEndTerm[140] + tmpFx[207]*tmpObjSEndTerm[153] + tmpFx[225]*tmpObjSEndTerm[166];
tmpQN2[128] = + tmpFx[9]*tmpObjSEndTerm[11] + tmpFx[27]*tmpObjSEndTerm[24] + tmpFx[45]*tmpObjSEndTerm[37] + tmpFx[63]*tmpObjSEndTerm[50] + tmpFx[81]*tmpObjSEndTerm[63] + tmpFx[99]*tmpObjSEndTerm[76] + tmpFx[117]*tmpObjSEndTerm[89] + tmpFx[135]*tmpObjSEndTerm[102] + tmpFx[153]*tmpObjSEndTerm[115] + tmpFx[171]*tmpObjSEndTerm[128] + tmpFx[189]*tmpObjSEndTerm[141] + tmpFx[207]*tmpObjSEndTerm[154] + tmpFx[225]*tmpObjSEndTerm[167];
tmpQN2[129] = + tmpFx[9]*tmpObjSEndTerm[12] + tmpFx[27]*tmpObjSEndTerm[25] + tmpFx[45]*tmpObjSEndTerm[38] + tmpFx[63]*tmpObjSEndTerm[51] + tmpFx[81]*tmpObjSEndTerm[64] + tmpFx[99]*tmpObjSEndTerm[77] + tmpFx[117]*tmpObjSEndTerm[90] + tmpFx[135]*tmpObjSEndTerm[103] + tmpFx[153]*tmpObjSEndTerm[116] + tmpFx[171]*tmpObjSEndTerm[129] + tmpFx[189]*tmpObjSEndTerm[142] + tmpFx[207]*tmpObjSEndTerm[155] + tmpFx[225]*tmpObjSEndTerm[168];
tmpQN2[130] = + tmpFx[10]*tmpObjSEndTerm[0] + tmpFx[28]*tmpObjSEndTerm[13] + tmpFx[46]*tmpObjSEndTerm[26] + tmpFx[64]*tmpObjSEndTerm[39] + tmpFx[82]*tmpObjSEndTerm[52] + tmpFx[100]*tmpObjSEndTerm[65] + tmpFx[118]*tmpObjSEndTerm[78] + tmpFx[136]*tmpObjSEndTerm[91] + tmpFx[154]*tmpObjSEndTerm[104] + tmpFx[172]*tmpObjSEndTerm[117] + tmpFx[190]*tmpObjSEndTerm[130] + tmpFx[208]*tmpObjSEndTerm[143] + tmpFx[226]*tmpObjSEndTerm[156];
tmpQN2[131] = + tmpFx[10]*tmpObjSEndTerm[1] + tmpFx[28]*tmpObjSEndTerm[14] + tmpFx[46]*tmpObjSEndTerm[27] + tmpFx[64]*tmpObjSEndTerm[40] + tmpFx[82]*tmpObjSEndTerm[53] + tmpFx[100]*tmpObjSEndTerm[66] + tmpFx[118]*tmpObjSEndTerm[79] + tmpFx[136]*tmpObjSEndTerm[92] + tmpFx[154]*tmpObjSEndTerm[105] + tmpFx[172]*tmpObjSEndTerm[118] + tmpFx[190]*tmpObjSEndTerm[131] + tmpFx[208]*tmpObjSEndTerm[144] + tmpFx[226]*tmpObjSEndTerm[157];
tmpQN2[132] = + tmpFx[10]*tmpObjSEndTerm[2] + tmpFx[28]*tmpObjSEndTerm[15] + tmpFx[46]*tmpObjSEndTerm[28] + tmpFx[64]*tmpObjSEndTerm[41] + tmpFx[82]*tmpObjSEndTerm[54] + tmpFx[100]*tmpObjSEndTerm[67] + tmpFx[118]*tmpObjSEndTerm[80] + tmpFx[136]*tmpObjSEndTerm[93] + tmpFx[154]*tmpObjSEndTerm[106] + tmpFx[172]*tmpObjSEndTerm[119] + tmpFx[190]*tmpObjSEndTerm[132] + tmpFx[208]*tmpObjSEndTerm[145] + tmpFx[226]*tmpObjSEndTerm[158];
tmpQN2[133] = + tmpFx[10]*tmpObjSEndTerm[3] + tmpFx[28]*tmpObjSEndTerm[16] + tmpFx[46]*tmpObjSEndTerm[29] + tmpFx[64]*tmpObjSEndTerm[42] + tmpFx[82]*tmpObjSEndTerm[55] + tmpFx[100]*tmpObjSEndTerm[68] + tmpFx[118]*tmpObjSEndTerm[81] + tmpFx[136]*tmpObjSEndTerm[94] + tmpFx[154]*tmpObjSEndTerm[107] + tmpFx[172]*tmpObjSEndTerm[120] + tmpFx[190]*tmpObjSEndTerm[133] + tmpFx[208]*tmpObjSEndTerm[146] + tmpFx[226]*tmpObjSEndTerm[159];
tmpQN2[134] = + tmpFx[10]*tmpObjSEndTerm[4] + tmpFx[28]*tmpObjSEndTerm[17] + tmpFx[46]*tmpObjSEndTerm[30] + tmpFx[64]*tmpObjSEndTerm[43] + tmpFx[82]*tmpObjSEndTerm[56] + tmpFx[100]*tmpObjSEndTerm[69] + tmpFx[118]*tmpObjSEndTerm[82] + tmpFx[136]*tmpObjSEndTerm[95] + tmpFx[154]*tmpObjSEndTerm[108] + tmpFx[172]*tmpObjSEndTerm[121] + tmpFx[190]*tmpObjSEndTerm[134] + tmpFx[208]*tmpObjSEndTerm[147] + tmpFx[226]*tmpObjSEndTerm[160];
tmpQN2[135] = + tmpFx[10]*tmpObjSEndTerm[5] + tmpFx[28]*tmpObjSEndTerm[18] + tmpFx[46]*tmpObjSEndTerm[31] + tmpFx[64]*tmpObjSEndTerm[44] + tmpFx[82]*tmpObjSEndTerm[57] + tmpFx[100]*tmpObjSEndTerm[70] + tmpFx[118]*tmpObjSEndTerm[83] + tmpFx[136]*tmpObjSEndTerm[96] + tmpFx[154]*tmpObjSEndTerm[109] + tmpFx[172]*tmpObjSEndTerm[122] + tmpFx[190]*tmpObjSEndTerm[135] + tmpFx[208]*tmpObjSEndTerm[148] + tmpFx[226]*tmpObjSEndTerm[161];
tmpQN2[136] = + tmpFx[10]*tmpObjSEndTerm[6] + tmpFx[28]*tmpObjSEndTerm[19] + tmpFx[46]*tmpObjSEndTerm[32] + tmpFx[64]*tmpObjSEndTerm[45] + tmpFx[82]*tmpObjSEndTerm[58] + tmpFx[100]*tmpObjSEndTerm[71] + tmpFx[118]*tmpObjSEndTerm[84] + tmpFx[136]*tmpObjSEndTerm[97] + tmpFx[154]*tmpObjSEndTerm[110] + tmpFx[172]*tmpObjSEndTerm[123] + tmpFx[190]*tmpObjSEndTerm[136] + tmpFx[208]*tmpObjSEndTerm[149] + tmpFx[226]*tmpObjSEndTerm[162];
tmpQN2[137] = + tmpFx[10]*tmpObjSEndTerm[7] + tmpFx[28]*tmpObjSEndTerm[20] + tmpFx[46]*tmpObjSEndTerm[33] + tmpFx[64]*tmpObjSEndTerm[46] + tmpFx[82]*tmpObjSEndTerm[59] + tmpFx[100]*tmpObjSEndTerm[72] + tmpFx[118]*tmpObjSEndTerm[85] + tmpFx[136]*tmpObjSEndTerm[98] + tmpFx[154]*tmpObjSEndTerm[111] + tmpFx[172]*tmpObjSEndTerm[124] + tmpFx[190]*tmpObjSEndTerm[137] + tmpFx[208]*tmpObjSEndTerm[150] + tmpFx[226]*tmpObjSEndTerm[163];
tmpQN2[138] = + tmpFx[10]*tmpObjSEndTerm[8] + tmpFx[28]*tmpObjSEndTerm[21] + tmpFx[46]*tmpObjSEndTerm[34] + tmpFx[64]*tmpObjSEndTerm[47] + tmpFx[82]*tmpObjSEndTerm[60] + tmpFx[100]*tmpObjSEndTerm[73] + tmpFx[118]*tmpObjSEndTerm[86] + tmpFx[136]*tmpObjSEndTerm[99] + tmpFx[154]*tmpObjSEndTerm[112] + tmpFx[172]*tmpObjSEndTerm[125] + tmpFx[190]*tmpObjSEndTerm[138] + tmpFx[208]*tmpObjSEndTerm[151] + tmpFx[226]*tmpObjSEndTerm[164];
tmpQN2[139] = + tmpFx[10]*tmpObjSEndTerm[9] + tmpFx[28]*tmpObjSEndTerm[22] + tmpFx[46]*tmpObjSEndTerm[35] + tmpFx[64]*tmpObjSEndTerm[48] + tmpFx[82]*tmpObjSEndTerm[61] + tmpFx[100]*tmpObjSEndTerm[74] + tmpFx[118]*tmpObjSEndTerm[87] + tmpFx[136]*tmpObjSEndTerm[100] + tmpFx[154]*tmpObjSEndTerm[113] + tmpFx[172]*tmpObjSEndTerm[126] + tmpFx[190]*tmpObjSEndTerm[139] + tmpFx[208]*tmpObjSEndTerm[152] + tmpFx[226]*tmpObjSEndTerm[165];
tmpQN2[140] = + tmpFx[10]*tmpObjSEndTerm[10] + tmpFx[28]*tmpObjSEndTerm[23] + tmpFx[46]*tmpObjSEndTerm[36] + tmpFx[64]*tmpObjSEndTerm[49] + tmpFx[82]*tmpObjSEndTerm[62] + tmpFx[100]*tmpObjSEndTerm[75] + tmpFx[118]*tmpObjSEndTerm[88] + tmpFx[136]*tmpObjSEndTerm[101] + tmpFx[154]*tmpObjSEndTerm[114] + tmpFx[172]*tmpObjSEndTerm[127] + tmpFx[190]*tmpObjSEndTerm[140] + tmpFx[208]*tmpObjSEndTerm[153] + tmpFx[226]*tmpObjSEndTerm[166];
tmpQN2[141] = + tmpFx[10]*tmpObjSEndTerm[11] + tmpFx[28]*tmpObjSEndTerm[24] + tmpFx[46]*tmpObjSEndTerm[37] + tmpFx[64]*tmpObjSEndTerm[50] + tmpFx[82]*tmpObjSEndTerm[63] + tmpFx[100]*tmpObjSEndTerm[76] + tmpFx[118]*tmpObjSEndTerm[89] + tmpFx[136]*tmpObjSEndTerm[102] + tmpFx[154]*tmpObjSEndTerm[115] + tmpFx[172]*tmpObjSEndTerm[128] + tmpFx[190]*tmpObjSEndTerm[141] + tmpFx[208]*tmpObjSEndTerm[154] + tmpFx[226]*tmpObjSEndTerm[167];
tmpQN2[142] = + tmpFx[10]*tmpObjSEndTerm[12] + tmpFx[28]*tmpObjSEndTerm[25] + tmpFx[46]*tmpObjSEndTerm[38] + tmpFx[64]*tmpObjSEndTerm[51] + tmpFx[82]*tmpObjSEndTerm[64] + tmpFx[100]*tmpObjSEndTerm[77] + tmpFx[118]*tmpObjSEndTerm[90] + tmpFx[136]*tmpObjSEndTerm[103] + tmpFx[154]*tmpObjSEndTerm[116] + tmpFx[172]*tmpObjSEndTerm[129] + tmpFx[190]*tmpObjSEndTerm[142] + tmpFx[208]*tmpObjSEndTerm[155] + tmpFx[226]*tmpObjSEndTerm[168];
tmpQN2[143] = + tmpFx[11]*tmpObjSEndTerm[0] + tmpFx[29]*tmpObjSEndTerm[13] + tmpFx[47]*tmpObjSEndTerm[26] + tmpFx[65]*tmpObjSEndTerm[39] + tmpFx[83]*tmpObjSEndTerm[52] + tmpFx[101]*tmpObjSEndTerm[65] + tmpFx[119]*tmpObjSEndTerm[78] + tmpFx[137]*tmpObjSEndTerm[91] + tmpFx[155]*tmpObjSEndTerm[104] + tmpFx[173]*tmpObjSEndTerm[117] + tmpFx[191]*tmpObjSEndTerm[130] + tmpFx[209]*tmpObjSEndTerm[143] + tmpFx[227]*tmpObjSEndTerm[156];
tmpQN2[144] = + tmpFx[11]*tmpObjSEndTerm[1] + tmpFx[29]*tmpObjSEndTerm[14] + tmpFx[47]*tmpObjSEndTerm[27] + tmpFx[65]*tmpObjSEndTerm[40] + tmpFx[83]*tmpObjSEndTerm[53] + tmpFx[101]*tmpObjSEndTerm[66] + tmpFx[119]*tmpObjSEndTerm[79] + tmpFx[137]*tmpObjSEndTerm[92] + tmpFx[155]*tmpObjSEndTerm[105] + tmpFx[173]*tmpObjSEndTerm[118] + tmpFx[191]*tmpObjSEndTerm[131] + tmpFx[209]*tmpObjSEndTerm[144] + tmpFx[227]*tmpObjSEndTerm[157];
tmpQN2[145] = + tmpFx[11]*tmpObjSEndTerm[2] + tmpFx[29]*tmpObjSEndTerm[15] + tmpFx[47]*tmpObjSEndTerm[28] + tmpFx[65]*tmpObjSEndTerm[41] + tmpFx[83]*tmpObjSEndTerm[54] + tmpFx[101]*tmpObjSEndTerm[67] + tmpFx[119]*tmpObjSEndTerm[80] + tmpFx[137]*tmpObjSEndTerm[93] + tmpFx[155]*tmpObjSEndTerm[106] + tmpFx[173]*tmpObjSEndTerm[119] + tmpFx[191]*tmpObjSEndTerm[132] + tmpFx[209]*tmpObjSEndTerm[145] + tmpFx[227]*tmpObjSEndTerm[158];
tmpQN2[146] = + tmpFx[11]*tmpObjSEndTerm[3] + tmpFx[29]*tmpObjSEndTerm[16] + tmpFx[47]*tmpObjSEndTerm[29] + tmpFx[65]*tmpObjSEndTerm[42] + tmpFx[83]*tmpObjSEndTerm[55] + tmpFx[101]*tmpObjSEndTerm[68] + tmpFx[119]*tmpObjSEndTerm[81] + tmpFx[137]*tmpObjSEndTerm[94] + tmpFx[155]*tmpObjSEndTerm[107] + tmpFx[173]*tmpObjSEndTerm[120] + tmpFx[191]*tmpObjSEndTerm[133] + tmpFx[209]*tmpObjSEndTerm[146] + tmpFx[227]*tmpObjSEndTerm[159];
tmpQN2[147] = + tmpFx[11]*tmpObjSEndTerm[4] + tmpFx[29]*tmpObjSEndTerm[17] + tmpFx[47]*tmpObjSEndTerm[30] + tmpFx[65]*tmpObjSEndTerm[43] + tmpFx[83]*tmpObjSEndTerm[56] + tmpFx[101]*tmpObjSEndTerm[69] + tmpFx[119]*tmpObjSEndTerm[82] + tmpFx[137]*tmpObjSEndTerm[95] + tmpFx[155]*tmpObjSEndTerm[108] + tmpFx[173]*tmpObjSEndTerm[121] + tmpFx[191]*tmpObjSEndTerm[134] + tmpFx[209]*tmpObjSEndTerm[147] + tmpFx[227]*tmpObjSEndTerm[160];
tmpQN2[148] = + tmpFx[11]*tmpObjSEndTerm[5] + tmpFx[29]*tmpObjSEndTerm[18] + tmpFx[47]*tmpObjSEndTerm[31] + tmpFx[65]*tmpObjSEndTerm[44] + tmpFx[83]*tmpObjSEndTerm[57] + tmpFx[101]*tmpObjSEndTerm[70] + tmpFx[119]*tmpObjSEndTerm[83] + tmpFx[137]*tmpObjSEndTerm[96] + tmpFx[155]*tmpObjSEndTerm[109] + tmpFx[173]*tmpObjSEndTerm[122] + tmpFx[191]*tmpObjSEndTerm[135] + tmpFx[209]*tmpObjSEndTerm[148] + tmpFx[227]*tmpObjSEndTerm[161];
tmpQN2[149] = + tmpFx[11]*tmpObjSEndTerm[6] + tmpFx[29]*tmpObjSEndTerm[19] + tmpFx[47]*tmpObjSEndTerm[32] + tmpFx[65]*tmpObjSEndTerm[45] + tmpFx[83]*tmpObjSEndTerm[58] + tmpFx[101]*tmpObjSEndTerm[71] + tmpFx[119]*tmpObjSEndTerm[84] + tmpFx[137]*tmpObjSEndTerm[97] + tmpFx[155]*tmpObjSEndTerm[110] + tmpFx[173]*tmpObjSEndTerm[123] + tmpFx[191]*tmpObjSEndTerm[136] + tmpFx[209]*tmpObjSEndTerm[149] + tmpFx[227]*tmpObjSEndTerm[162];
tmpQN2[150] = + tmpFx[11]*tmpObjSEndTerm[7] + tmpFx[29]*tmpObjSEndTerm[20] + tmpFx[47]*tmpObjSEndTerm[33] + tmpFx[65]*tmpObjSEndTerm[46] + tmpFx[83]*tmpObjSEndTerm[59] + tmpFx[101]*tmpObjSEndTerm[72] + tmpFx[119]*tmpObjSEndTerm[85] + tmpFx[137]*tmpObjSEndTerm[98] + tmpFx[155]*tmpObjSEndTerm[111] + tmpFx[173]*tmpObjSEndTerm[124] + tmpFx[191]*tmpObjSEndTerm[137] + tmpFx[209]*tmpObjSEndTerm[150] + tmpFx[227]*tmpObjSEndTerm[163];
tmpQN2[151] = + tmpFx[11]*tmpObjSEndTerm[8] + tmpFx[29]*tmpObjSEndTerm[21] + tmpFx[47]*tmpObjSEndTerm[34] + tmpFx[65]*tmpObjSEndTerm[47] + tmpFx[83]*tmpObjSEndTerm[60] + tmpFx[101]*tmpObjSEndTerm[73] + tmpFx[119]*tmpObjSEndTerm[86] + tmpFx[137]*tmpObjSEndTerm[99] + tmpFx[155]*tmpObjSEndTerm[112] + tmpFx[173]*tmpObjSEndTerm[125] + tmpFx[191]*tmpObjSEndTerm[138] + tmpFx[209]*tmpObjSEndTerm[151] + tmpFx[227]*tmpObjSEndTerm[164];
tmpQN2[152] = + tmpFx[11]*tmpObjSEndTerm[9] + tmpFx[29]*tmpObjSEndTerm[22] + tmpFx[47]*tmpObjSEndTerm[35] + tmpFx[65]*tmpObjSEndTerm[48] + tmpFx[83]*tmpObjSEndTerm[61] + tmpFx[101]*tmpObjSEndTerm[74] + tmpFx[119]*tmpObjSEndTerm[87] + tmpFx[137]*tmpObjSEndTerm[100] + tmpFx[155]*tmpObjSEndTerm[113] + tmpFx[173]*tmpObjSEndTerm[126] + tmpFx[191]*tmpObjSEndTerm[139] + tmpFx[209]*tmpObjSEndTerm[152] + tmpFx[227]*tmpObjSEndTerm[165];
tmpQN2[153] = + tmpFx[11]*tmpObjSEndTerm[10] + tmpFx[29]*tmpObjSEndTerm[23] + tmpFx[47]*tmpObjSEndTerm[36] + tmpFx[65]*tmpObjSEndTerm[49] + tmpFx[83]*tmpObjSEndTerm[62] + tmpFx[101]*tmpObjSEndTerm[75] + tmpFx[119]*tmpObjSEndTerm[88] + tmpFx[137]*tmpObjSEndTerm[101] + tmpFx[155]*tmpObjSEndTerm[114] + tmpFx[173]*tmpObjSEndTerm[127] + tmpFx[191]*tmpObjSEndTerm[140] + tmpFx[209]*tmpObjSEndTerm[153] + tmpFx[227]*tmpObjSEndTerm[166];
tmpQN2[154] = + tmpFx[11]*tmpObjSEndTerm[11] + tmpFx[29]*tmpObjSEndTerm[24] + tmpFx[47]*tmpObjSEndTerm[37] + tmpFx[65]*tmpObjSEndTerm[50] + tmpFx[83]*tmpObjSEndTerm[63] + tmpFx[101]*tmpObjSEndTerm[76] + tmpFx[119]*tmpObjSEndTerm[89] + tmpFx[137]*tmpObjSEndTerm[102] + tmpFx[155]*tmpObjSEndTerm[115] + tmpFx[173]*tmpObjSEndTerm[128] + tmpFx[191]*tmpObjSEndTerm[141] + tmpFx[209]*tmpObjSEndTerm[154] + tmpFx[227]*tmpObjSEndTerm[167];
tmpQN2[155] = + tmpFx[11]*tmpObjSEndTerm[12] + tmpFx[29]*tmpObjSEndTerm[25] + tmpFx[47]*tmpObjSEndTerm[38] + tmpFx[65]*tmpObjSEndTerm[51] + tmpFx[83]*tmpObjSEndTerm[64] + tmpFx[101]*tmpObjSEndTerm[77] + tmpFx[119]*tmpObjSEndTerm[90] + tmpFx[137]*tmpObjSEndTerm[103] + tmpFx[155]*tmpObjSEndTerm[116] + tmpFx[173]*tmpObjSEndTerm[129] + tmpFx[191]*tmpObjSEndTerm[142] + tmpFx[209]*tmpObjSEndTerm[155] + tmpFx[227]*tmpObjSEndTerm[168];
tmpQN2[156] = + tmpFx[12]*tmpObjSEndTerm[0] + tmpFx[30]*tmpObjSEndTerm[13] + tmpFx[48]*tmpObjSEndTerm[26] + tmpFx[66]*tmpObjSEndTerm[39] + tmpFx[84]*tmpObjSEndTerm[52] + tmpFx[102]*tmpObjSEndTerm[65] + tmpFx[120]*tmpObjSEndTerm[78] + tmpFx[138]*tmpObjSEndTerm[91] + tmpFx[156]*tmpObjSEndTerm[104] + tmpFx[174]*tmpObjSEndTerm[117] + tmpFx[192]*tmpObjSEndTerm[130] + tmpFx[210]*tmpObjSEndTerm[143] + tmpFx[228]*tmpObjSEndTerm[156];
tmpQN2[157] = + tmpFx[12]*tmpObjSEndTerm[1] + tmpFx[30]*tmpObjSEndTerm[14] + tmpFx[48]*tmpObjSEndTerm[27] + tmpFx[66]*tmpObjSEndTerm[40] + tmpFx[84]*tmpObjSEndTerm[53] + tmpFx[102]*tmpObjSEndTerm[66] + tmpFx[120]*tmpObjSEndTerm[79] + tmpFx[138]*tmpObjSEndTerm[92] + tmpFx[156]*tmpObjSEndTerm[105] + tmpFx[174]*tmpObjSEndTerm[118] + tmpFx[192]*tmpObjSEndTerm[131] + tmpFx[210]*tmpObjSEndTerm[144] + tmpFx[228]*tmpObjSEndTerm[157];
tmpQN2[158] = + tmpFx[12]*tmpObjSEndTerm[2] + tmpFx[30]*tmpObjSEndTerm[15] + tmpFx[48]*tmpObjSEndTerm[28] + tmpFx[66]*tmpObjSEndTerm[41] + tmpFx[84]*tmpObjSEndTerm[54] + tmpFx[102]*tmpObjSEndTerm[67] + tmpFx[120]*tmpObjSEndTerm[80] + tmpFx[138]*tmpObjSEndTerm[93] + tmpFx[156]*tmpObjSEndTerm[106] + tmpFx[174]*tmpObjSEndTerm[119] + tmpFx[192]*tmpObjSEndTerm[132] + tmpFx[210]*tmpObjSEndTerm[145] + tmpFx[228]*tmpObjSEndTerm[158];
tmpQN2[159] = + tmpFx[12]*tmpObjSEndTerm[3] + tmpFx[30]*tmpObjSEndTerm[16] + tmpFx[48]*tmpObjSEndTerm[29] + tmpFx[66]*tmpObjSEndTerm[42] + tmpFx[84]*tmpObjSEndTerm[55] + tmpFx[102]*tmpObjSEndTerm[68] + tmpFx[120]*tmpObjSEndTerm[81] + tmpFx[138]*tmpObjSEndTerm[94] + tmpFx[156]*tmpObjSEndTerm[107] + tmpFx[174]*tmpObjSEndTerm[120] + tmpFx[192]*tmpObjSEndTerm[133] + tmpFx[210]*tmpObjSEndTerm[146] + tmpFx[228]*tmpObjSEndTerm[159];
tmpQN2[160] = + tmpFx[12]*tmpObjSEndTerm[4] + tmpFx[30]*tmpObjSEndTerm[17] + tmpFx[48]*tmpObjSEndTerm[30] + tmpFx[66]*tmpObjSEndTerm[43] + tmpFx[84]*tmpObjSEndTerm[56] + tmpFx[102]*tmpObjSEndTerm[69] + tmpFx[120]*tmpObjSEndTerm[82] + tmpFx[138]*tmpObjSEndTerm[95] + tmpFx[156]*tmpObjSEndTerm[108] + tmpFx[174]*tmpObjSEndTerm[121] + tmpFx[192]*tmpObjSEndTerm[134] + tmpFx[210]*tmpObjSEndTerm[147] + tmpFx[228]*tmpObjSEndTerm[160];
tmpQN2[161] = + tmpFx[12]*tmpObjSEndTerm[5] + tmpFx[30]*tmpObjSEndTerm[18] + tmpFx[48]*tmpObjSEndTerm[31] + tmpFx[66]*tmpObjSEndTerm[44] + tmpFx[84]*tmpObjSEndTerm[57] + tmpFx[102]*tmpObjSEndTerm[70] + tmpFx[120]*tmpObjSEndTerm[83] + tmpFx[138]*tmpObjSEndTerm[96] + tmpFx[156]*tmpObjSEndTerm[109] + tmpFx[174]*tmpObjSEndTerm[122] + tmpFx[192]*tmpObjSEndTerm[135] + tmpFx[210]*tmpObjSEndTerm[148] + tmpFx[228]*tmpObjSEndTerm[161];
tmpQN2[162] = + tmpFx[12]*tmpObjSEndTerm[6] + tmpFx[30]*tmpObjSEndTerm[19] + tmpFx[48]*tmpObjSEndTerm[32] + tmpFx[66]*tmpObjSEndTerm[45] + tmpFx[84]*tmpObjSEndTerm[58] + tmpFx[102]*tmpObjSEndTerm[71] + tmpFx[120]*tmpObjSEndTerm[84] + tmpFx[138]*tmpObjSEndTerm[97] + tmpFx[156]*tmpObjSEndTerm[110] + tmpFx[174]*tmpObjSEndTerm[123] + tmpFx[192]*tmpObjSEndTerm[136] + tmpFx[210]*tmpObjSEndTerm[149] + tmpFx[228]*tmpObjSEndTerm[162];
tmpQN2[163] = + tmpFx[12]*tmpObjSEndTerm[7] + tmpFx[30]*tmpObjSEndTerm[20] + tmpFx[48]*tmpObjSEndTerm[33] + tmpFx[66]*tmpObjSEndTerm[46] + tmpFx[84]*tmpObjSEndTerm[59] + tmpFx[102]*tmpObjSEndTerm[72] + tmpFx[120]*tmpObjSEndTerm[85] + tmpFx[138]*tmpObjSEndTerm[98] + tmpFx[156]*tmpObjSEndTerm[111] + tmpFx[174]*tmpObjSEndTerm[124] + tmpFx[192]*tmpObjSEndTerm[137] + tmpFx[210]*tmpObjSEndTerm[150] + tmpFx[228]*tmpObjSEndTerm[163];
tmpQN2[164] = + tmpFx[12]*tmpObjSEndTerm[8] + tmpFx[30]*tmpObjSEndTerm[21] + tmpFx[48]*tmpObjSEndTerm[34] + tmpFx[66]*tmpObjSEndTerm[47] + tmpFx[84]*tmpObjSEndTerm[60] + tmpFx[102]*tmpObjSEndTerm[73] + tmpFx[120]*tmpObjSEndTerm[86] + tmpFx[138]*tmpObjSEndTerm[99] + tmpFx[156]*tmpObjSEndTerm[112] + tmpFx[174]*tmpObjSEndTerm[125] + tmpFx[192]*tmpObjSEndTerm[138] + tmpFx[210]*tmpObjSEndTerm[151] + tmpFx[228]*tmpObjSEndTerm[164];
tmpQN2[165] = + tmpFx[12]*tmpObjSEndTerm[9] + tmpFx[30]*tmpObjSEndTerm[22] + tmpFx[48]*tmpObjSEndTerm[35] + tmpFx[66]*tmpObjSEndTerm[48] + tmpFx[84]*tmpObjSEndTerm[61] + tmpFx[102]*tmpObjSEndTerm[74] + tmpFx[120]*tmpObjSEndTerm[87] + tmpFx[138]*tmpObjSEndTerm[100] + tmpFx[156]*tmpObjSEndTerm[113] + tmpFx[174]*tmpObjSEndTerm[126] + tmpFx[192]*tmpObjSEndTerm[139] + tmpFx[210]*tmpObjSEndTerm[152] + tmpFx[228]*tmpObjSEndTerm[165];
tmpQN2[166] = + tmpFx[12]*tmpObjSEndTerm[10] + tmpFx[30]*tmpObjSEndTerm[23] + tmpFx[48]*tmpObjSEndTerm[36] + tmpFx[66]*tmpObjSEndTerm[49] + tmpFx[84]*tmpObjSEndTerm[62] + tmpFx[102]*tmpObjSEndTerm[75] + tmpFx[120]*tmpObjSEndTerm[88] + tmpFx[138]*tmpObjSEndTerm[101] + tmpFx[156]*tmpObjSEndTerm[114] + tmpFx[174]*tmpObjSEndTerm[127] + tmpFx[192]*tmpObjSEndTerm[140] + tmpFx[210]*tmpObjSEndTerm[153] + tmpFx[228]*tmpObjSEndTerm[166];
tmpQN2[167] = + tmpFx[12]*tmpObjSEndTerm[11] + tmpFx[30]*tmpObjSEndTerm[24] + tmpFx[48]*tmpObjSEndTerm[37] + tmpFx[66]*tmpObjSEndTerm[50] + tmpFx[84]*tmpObjSEndTerm[63] + tmpFx[102]*tmpObjSEndTerm[76] + tmpFx[120]*tmpObjSEndTerm[89] + tmpFx[138]*tmpObjSEndTerm[102] + tmpFx[156]*tmpObjSEndTerm[115] + tmpFx[174]*tmpObjSEndTerm[128] + tmpFx[192]*tmpObjSEndTerm[141] + tmpFx[210]*tmpObjSEndTerm[154] + tmpFx[228]*tmpObjSEndTerm[167];
tmpQN2[168] = + tmpFx[12]*tmpObjSEndTerm[12] + tmpFx[30]*tmpObjSEndTerm[25] + tmpFx[48]*tmpObjSEndTerm[38] + tmpFx[66]*tmpObjSEndTerm[51] + tmpFx[84]*tmpObjSEndTerm[64] + tmpFx[102]*tmpObjSEndTerm[77] + tmpFx[120]*tmpObjSEndTerm[90] + tmpFx[138]*tmpObjSEndTerm[103] + tmpFx[156]*tmpObjSEndTerm[116] + tmpFx[174]*tmpObjSEndTerm[129] + tmpFx[192]*tmpObjSEndTerm[142] + tmpFx[210]*tmpObjSEndTerm[155] + tmpFx[228]*tmpObjSEndTerm[168];
tmpQN2[169] = + tmpFx[13]*tmpObjSEndTerm[0] + tmpFx[31]*tmpObjSEndTerm[13] + tmpFx[49]*tmpObjSEndTerm[26] + tmpFx[67]*tmpObjSEndTerm[39] + tmpFx[85]*tmpObjSEndTerm[52] + tmpFx[103]*tmpObjSEndTerm[65] + tmpFx[121]*tmpObjSEndTerm[78] + tmpFx[139]*tmpObjSEndTerm[91] + tmpFx[157]*tmpObjSEndTerm[104] + tmpFx[175]*tmpObjSEndTerm[117] + tmpFx[193]*tmpObjSEndTerm[130] + tmpFx[211]*tmpObjSEndTerm[143] + tmpFx[229]*tmpObjSEndTerm[156];
tmpQN2[170] = + tmpFx[13]*tmpObjSEndTerm[1] + tmpFx[31]*tmpObjSEndTerm[14] + tmpFx[49]*tmpObjSEndTerm[27] + tmpFx[67]*tmpObjSEndTerm[40] + tmpFx[85]*tmpObjSEndTerm[53] + tmpFx[103]*tmpObjSEndTerm[66] + tmpFx[121]*tmpObjSEndTerm[79] + tmpFx[139]*tmpObjSEndTerm[92] + tmpFx[157]*tmpObjSEndTerm[105] + tmpFx[175]*tmpObjSEndTerm[118] + tmpFx[193]*tmpObjSEndTerm[131] + tmpFx[211]*tmpObjSEndTerm[144] + tmpFx[229]*tmpObjSEndTerm[157];
tmpQN2[171] = + tmpFx[13]*tmpObjSEndTerm[2] + tmpFx[31]*tmpObjSEndTerm[15] + tmpFx[49]*tmpObjSEndTerm[28] + tmpFx[67]*tmpObjSEndTerm[41] + tmpFx[85]*tmpObjSEndTerm[54] + tmpFx[103]*tmpObjSEndTerm[67] + tmpFx[121]*tmpObjSEndTerm[80] + tmpFx[139]*tmpObjSEndTerm[93] + tmpFx[157]*tmpObjSEndTerm[106] + tmpFx[175]*tmpObjSEndTerm[119] + tmpFx[193]*tmpObjSEndTerm[132] + tmpFx[211]*tmpObjSEndTerm[145] + tmpFx[229]*tmpObjSEndTerm[158];
tmpQN2[172] = + tmpFx[13]*tmpObjSEndTerm[3] + tmpFx[31]*tmpObjSEndTerm[16] + tmpFx[49]*tmpObjSEndTerm[29] + tmpFx[67]*tmpObjSEndTerm[42] + tmpFx[85]*tmpObjSEndTerm[55] + tmpFx[103]*tmpObjSEndTerm[68] + tmpFx[121]*tmpObjSEndTerm[81] + tmpFx[139]*tmpObjSEndTerm[94] + tmpFx[157]*tmpObjSEndTerm[107] + tmpFx[175]*tmpObjSEndTerm[120] + tmpFx[193]*tmpObjSEndTerm[133] + tmpFx[211]*tmpObjSEndTerm[146] + tmpFx[229]*tmpObjSEndTerm[159];
tmpQN2[173] = + tmpFx[13]*tmpObjSEndTerm[4] + tmpFx[31]*tmpObjSEndTerm[17] + tmpFx[49]*tmpObjSEndTerm[30] + tmpFx[67]*tmpObjSEndTerm[43] + tmpFx[85]*tmpObjSEndTerm[56] + tmpFx[103]*tmpObjSEndTerm[69] + tmpFx[121]*tmpObjSEndTerm[82] + tmpFx[139]*tmpObjSEndTerm[95] + tmpFx[157]*tmpObjSEndTerm[108] + tmpFx[175]*tmpObjSEndTerm[121] + tmpFx[193]*tmpObjSEndTerm[134] + tmpFx[211]*tmpObjSEndTerm[147] + tmpFx[229]*tmpObjSEndTerm[160];
tmpQN2[174] = + tmpFx[13]*tmpObjSEndTerm[5] + tmpFx[31]*tmpObjSEndTerm[18] + tmpFx[49]*tmpObjSEndTerm[31] + tmpFx[67]*tmpObjSEndTerm[44] + tmpFx[85]*tmpObjSEndTerm[57] + tmpFx[103]*tmpObjSEndTerm[70] + tmpFx[121]*tmpObjSEndTerm[83] + tmpFx[139]*tmpObjSEndTerm[96] + tmpFx[157]*tmpObjSEndTerm[109] + tmpFx[175]*tmpObjSEndTerm[122] + tmpFx[193]*tmpObjSEndTerm[135] + tmpFx[211]*tmpObjSEndTerm[148] + tmpFx[229]*tmpObjSEndTerm[161];
tmpQN2[175] = + tmpFx[13]*tmpObjSEndTerm[6] + tmpFx[31]*tmpObjSEndTerm[19] + tmpFx[49]*tmpObjSEndTerm[32] + tmpFx[67]*tmpObjSEndTerm[45] + tmpFx[85]*tmpObjSEndTerm[58] + tmpFx[103]*tmpObjSEndTerm[71] + tmpFx[121]*tmpObjSEndTerm[84] + tmpFx[139]*tmpObjSEndTerm[97] + tmpFx[157]*tmpObjSEndTerm[110] + tmpFx[175]*tmpObjSEndTerm[123] + tmpFx[193]*tmpObjSEndTerm[136] + tmpFx[211]*tmpObjSEndTerm[149] + tmpFx[229]*tmpObjSEndTerm[162];
tmpQN2[176] = + tmpFx[13]*tmpObjSEndTerm[7] + tmpFx[31]*tmpObjSEndTerm[20] + tmpFx[49]*tmpObjSEndTerm[33] + tmpFx[67]*tmpObjSEndTerm[46] + tmpFx[85]*tmpObjSEndTerm[59] + tmpFx[103]*tmpObjSEndTerm[72] + tmpFx[121]*tmpObjSEndTerm[85] + tmpFx[139]*tmpObjSEndTerm[98] + tmpFx[157]*tmpObjSEndTerm[111] + tmpFx[175]*tmpObjSEndTerm[124] + tmpFx[193]*tmpObjSEndTerm[137] + tmpFx[211]*tmpObjSEndTerm[150] + tmpFx[229]*tmpObjSEndTerm[163];
tmpQN2[177] = + tmpFx[13]*tmpObjSEndTerm[8] + tmpFx[31]*tmpObjSEndTerm[21] + tmpFx[49]*tmpObjSEndTerm[34] + tmpFx[67]*tmpObjSEndTerm[47] + tmpFx[85]*tmpObjSEndTerm[60] + tmpFx[103]*tmpObjSEndTerm[73] + tmpFx[121]*tmpObjSEndTerm[86] + tmpFx[139]*tmpObjSEndTerm[99] + tmpFx[157]*tmpObjSEndTerm[112] + tmpFx[175]*tmpObjSEndTerm[125] + tmpFx[193]*tmpObjSEndTerm[138] + tmpFx[211]*tmpObjSEndTerm[151] + tmpFx[229]*tmpObjSEndTerm[164];
tmpQN2[178] = + tmpFx[13]*tmpObjSEndTerm[9] + tmpFx[31]*tmpObjSEndTerm[22] + tmpFx[49]*tmpObjSEndTerm[35] + tmpFx[67]*tmpObjSEndTerm[48] + tmpFx[85]*tmpObjSEndTerm[61] + tmpFx[103]*tmpObjSEndTerm[74] + tmpFx[121]*tmpObjSEndTerm[87] + tmpFx[139]*tmpObjSEndTerm[100] + tmpFx[157]*tmpObjSEndTerm[113] + tmpFx[175]*tmpObjSEndTerm[126] + tmpFx[193]*tmpObjSEndTerm[139] + tmpFx[211]*tmpObjSEndTerm[152] + tmpFx[229]*tmpObjSEndTerm[165];
tmpQN2[179] = + tmpFx[13]*tmpObjSEndTerm[10] + tmpFx[31]*tmpObjSEndTerm[23] + tmpFx[49]*tmpObjSEndTerm[36] + tmpFx[67]*tmpObjSEndTerm[49] + tmpFx[85]*tmpObjSEndTerm[62] + tmpFx[103]*tmpObjSEndTerm[75] + tmpFx[121]*tmpObjSEndTerm[88] + tmpFx[139]*tmpObjSEndTerm[101] + tmpFx[157]*tmpObjSEndTerm[114] + tmpFx[175]*tmpObjSEndTerm[127] + tmpFx[193]*tmpObjSEndTerm[140] + tmpFx[211]*tmpObjSEndTerm[153] + tmpFx[229]*tmpObjSEndTerm[166];
tmpQN2[180] = + tmpFx[13]*tmpObjSEndTerm[11] + tmpFx[31]*tmpObjSEndTerm[24] + tmpFx[49]*tmpObjSEndTerm[37] + tmpFx[67]*tmpObjSEndTerm[50] + tmpFx[85]*tmpObjSEndTerm[63] + tmpFx[103]*tmpObjSEndTerm[76] + tmpFx[121]*tmpObjSEndTerm[89] + tmpFx[139]*tmpObjSEndTerm[102] + tmpFx[157]*tmpObjSEndTerm[115] + tmpFx[175]*tmpObjSEndTerm[128] + tmpFx[193]*tmpObjSEndTerm[141] + tmpFx[211]*tmpObjSEndTerm[154] + tmpFx[229]*tmpObjSEndTerm[167];
tmpQN2[181] = + tmpFx[13]*tmpObjSEndTerm[12] + tmpFx[31]*tmpObjSEndTerm[25] + tmpFx[49]*tmpObjSEndTerm[38] + tmpFx[67]*tmpObjSEndTerm[51] + tmpFx[85]*tmpObjSEndTerm[64] + tmpFx[103]*tmpObjSEndTerm[77] + tmpFx[121]*tmpObjSEndTerm[90] + tmpFx[139]*tmpObjSEndTerm[103] + tmpFx[157]*tmpObjSEndTerm[116] + tmpFx[175]*tmpObjSEndTerm[129] + tmpFx[193]*tmpObjSEndTerm[142] + tmpFx[211]*tmpObjSEndTerm[155] + tmpFx[229]*tmpObjSEndTerm[168];
tmpQN2[182] = + tmpFx[14]*tmpObjSEndTerm[0] + tmpFx[32]*tmpObjSEndTerm[13] + tmpFx[50]*tmpObjSEndTerm[26] + tmpFx[68]*tmpObjSEndTerm[39] + tmpFx[86]*tmpObjSEndTerm[52] + tmpFx[104]*tmpObjSEndTerm[65] + tmpFx[122]*tmpObjSEndTerm[78] + tmpFx[140]*tmpObjSEndTerm[91] + tmpFx[158]*tmpObjSEndTerm[104] + tmpFx[176]*tmpObjSEndTerm[117] + tmpFx[194]*tmpObjSEndTerm[130] + tmpFx[212]*tmpObjSEndTerm[143] + tmpFx[230]*tmpObjSEndTerm[156];
tmpQN2[183] = + tmpFx[14]*tmpObjSEndTerm[1] + tmpFx[32]*tmpObjSEndTerm[14] + tmpFx[50]*tmpObjSEndTerm[27] + tmpFx[68]*tmpObjSEndTerm[40] + tmpFx[86]*tmpObjSEndTerm[53] + tmpFx[104]*tmpObjSEndTerm[66] + tmpFx[122]*tmpObjSEndTerm[79] + tmpFx[140]*tmpObjSEndTerm[92] + tmpFx[158]*tmpObjSEndTerm[105] + tmpFx[176]*tmpObjSEndTerm[118] + tmpFx[194]*tmpObjSEndTerm[131] + tmpFx[212]*tmpObjSEndTerm[144] + tmpFx[230]*tmpObjSEndTerm[157];
tmpQN2[184] = + tmpFx[14]*tmpObjSEndTerm[2] + tmpFx[32]*tmpObjSEndTerm[15] + tmpFx[50]*tmpObjSEndTerm[28] + tmpFx[68]*tmpObjSEndTerm[41] + tmpFx[86]*tmpObjSEndTerm[54] + tmpFx[104]*tmpObjSEndTerm[67] + tmpFx[122]*tmpObjSEndTerm[80] + tmpFx[140]*tmpObjSEndTerm[93] + tmpFx[158]*tmpObjSEndTerm[106] + tmpFx[176]*tmpObjSEndTerm[119] + tmpFx[194]*tmpObjSEndTerm[132] + tmpFx[212]*tmpObjSEndTerm[145] + tmpFx[230]*tmpObjSEndTerm[158];
tmpQN2[185] = + tmpFx[14]*tmpObjSEndTerm[3] + tmpFx[32]*tmpObjSEndTerm[16] + tmpFx[50]*tmpObjSEndTerm[29] + tmpFx[68]*tmpObjSEndTerm[42] + tmpFx[86]*tmpObjSEndTerm[55] + tmpFx[104]*tmpObjSEndTerm[68] + tmpFx[122]*tmpObjSEndTerm[81] + tmpFx[140]*tmpObjSEndTerm[94] + tmpFx[158]*tmpObjSEndTerm[107] + tmpFx[176]*tmpObjSEndTerm[120] + tmpFx[194]*tmpObjSEndTerm[133] + tmpFx[212]*tmpObjSEndTerm[146] + tmpFx[230]*tmpObjSEndTerm[159];
tmpQN2[186] = + tmpFx[14]*tmpObjSEndTerm[4] + tmpFx[32]*tmpObjSEndTerm[17] + tmpFx[50]*tmpObjSEndTerm[30] + tmpFx[68]*tmpObjSEndTerm[43] + tmpFx[86]*tmpObjSEndTerm[56] + tmpFx[104]*tmpObjSEndTerm[69] + tmpFx[122]*tmpObjSEndTerm[82] + tmpFx[140]*tmpObjSEndTerm[95] + tmpFx[158]*tmpObjSEndTerm[108] + tmpFx[176]*tmpObjSEndTerm[121] + tmpFx[194]*tmpObjSEndTerm[134] + tmpFx[212]*tmpObjSEndTerm[147] + tmpFx[230]*tmpObjSEndTerm[160];
tmpQN2[187] = + tmpFx[14]*tmpObjSEndTerm[5] + tmpFx[32]*tmpObjSEndTerm[18] + tmpFx[50]*tmpObjSEndTerm[31] + tmpFx[68]*tmpObjSEndTerm[44] + tmpFx[86]*tmpObjSEndTerm[57] + tmpFx[104]*tmpObjSEndTerm[70] + tmpFx[122]*tmpObjSEndTerm[83] + tmpFx[140]*tmpObjSEndTerm[96] + tmpFx[158]*tmpObjSEndTerm[109] + tmpFx[176]*tmpObjSEndTerm[122] + tmpFx[194]*tmpObjSEndTerm[135] + tmpFx[212]*tmpObjSEndTerm[148] + tmpFx[230]*tmpObjSEndTerm[161];
tmpQN2[188] = + tmpFx[14]*tmpObjSEndTerm[6] + tmpFx[32]*tmpObjSEndTerm[19] + tmpFx[50]*tmpObjSEndTerm[32] + tmpFx[68]*tmpObjSEndTerm[45] + tmpFx[86]*tmpObjSEndTerm[58] + tmpFx[104]*tmpObjSEndTerm[71] + tmpFx[122]*tmpObjSEndTerm[84] + tmpFx[140]*tmpObjSEndTerm[97] + tmpFx[158]*tmpObjSEndTerm[110] + tmpFx[176]*tmpObjSEndTerm[123] + tmpFx[194]*tmpObjSEndTerm[136] + tmpFx[212]*tmpObjSEndTerm[149] + tmpFx[230]*tmpObjSEndTerm[162];
tmpQN2[189] = + tmpFx[14]*tmpObjSEndTerm[7] + tmpFx[32]*tmpObjSEndTerm[20] + tmpFx[50]*tmpObjSEndTerm[33] + tmpFx[68]*tmpObjSEndTerm[46] + tmpFx[86]*tmpObjSEndTerm[59] + tmpFx[104]*tmpObjSEndTerm[72] + tmpFx[122]*tmpObjSEndTerm[85] + tmpFx[140]*tmpObjSEndTerm[98] + tmpFx[158]*tmpObjSEndTerm[111] + tmpFx[176]*tmpObjSEndTerm[124] + tmpFx[194]*tmpObjSEndTerm[137] + tmpFx[212]*tmpObjSEndTerm[150] + tmpFx[230]*tmpObjSEndTerm[163];
tmpQN2[190] = + tmpFx[14]*tmpObjSEndTerm[8] + tmpFx[32]*tmpObjSEndTerm[21] + tmpFx[50]*tmpObjSEndTerm[34] + tmpFx[68]*tmpObjSEndTerm[47] + tmpFx[86]*tmpObjSEndTerm[60] + tmpFx[104]*tmpObjSEndTerm[73] + tmpFx[122]*tmpObjSEndTerm[86] + tmpFx[140]*tmpObjSEndTerm[99] + tmpFx[158]*tmpObjSEndTerm[112] + tmpFx[176]*tmpObjSEndTerm[125] + tmpFx[194]*tmpObjSEndTerm[138] + tmpFx[212]*tmpObjSEndTerm[151] + tmpFx[230]*tmpObjSEndTerm[164];
tmpQN2[191] = + tmpFx[14]*tmpObjSEndTerm[9] + tmpFx[32]*tmpObjSEndTerm[22] + tmpFx[50]*tmpObjSEndTerm[35] + tmpFx[68]*tmpObjSEndTerm[48] + tmpFx[86]*tmpObjSEndTerm[61] + tmpFx[104]*tmpObjSEndTerm[74] + tmpFx[122]*tmpObjSEndTerm[87] + tmpFx[140]*tmpObjSEndTerm[100] + tmpFx[158]*tmpObjSEndTerm[113] + tmpFx[176]*tmpObjSEndTerm[126] + tmpFx[194]*tmpObjSEndTerm[139] + tmpFx[212]*tmpObjSEndTerm[152] + tmpFx[230]*tmpObjSEndTerm[165];
tmpQN2[192] = + tmpFx[14]*tmpObjSEndTerm[10] + tmpFx[32]*tmpObjSEndTerm[23] + tmpFx[50]*tmpObjSEndTerm[36] + tmpFx[68]*tmpObjSEndTerm[49] + tmpFx[86]*tmpObjSEndTerm[62] + tmpFx[104]*tmpObjSEndTerm[75] + tmpFx[122]*tmpObjSEndTerm[88] + tmpFx[140]*tmpObjSEndTerm[101] + tmpFx[158]*tmpObjSEndTerm[114] + tmpFx[176]*tmpObjSEndTerm[127] + tmpFx[194]*tmpObjSEndTerm[140] + tmpFx[212]*tmpObjSEndTerm[153] + tmpFx[230]*tmpObjSEndTerm[166];
tmpQN2[193] = + tmpFx[14]*tmpObjSEndTerm[11] + tmpFx[32]*tmpObjSEndTerm[24] + tmpFx[50]*tmpObjSEndTerm[37] + tmpFx[68]*tmpObjSEndTerm[50] + tmpFx[86]*tmpObjSEndTerm[63] + tmpFx[104]*tmpObjSEndTerm[76] + tmpFx[122]*tmpObjSEndTerm[89] + tmpFx[140]*tmpObjSEndTerm[102] + tmpFx[158]*tmpObjSEndTerm[115] + tmpFx[176]*tmpObjSEndTerm[128] + tmpFx[194]*tmpObjSEndTerm[141] + tmpFx[212]*tmpObjSEndTerm[154] + tmpFx[230]*tmpObjSEndTerm[167];
tmpQN2[194] = + tmpFx[14]*tmpObjSEndTerm[12] + tmpFx[32]*tmpObjSEndTerm[25] + tmpFx[50]*tmpObjSEndTerm[38] + tmpFx[68]*tmpObjSEndTerm[51] + tmpFx[86]*tmpObjSEndTerm[64] + tmpFx[104]*tmpObjSEndTerm[77] + tmpFx[122]*tmpObjSEndTerm[90] + tmpFx[140]*tmpObjSEndTerm[103] + tmpFx[158]*tmpObjSEndTerm[116] + tmpFx[176]*tmpObjSEndTerm[129] + tmpFx[194]*tmpObjSEndTerm[142] + tmpFx[212]*tmpObjSEndTerm[155] + tmpFx[230]*tmpObjSEndTerm[168];
tmpQN2[195] = + tmpFx[15]*tmpObjSEndTerm[0] + tmpFx[33]*tmpObjSEndTerm[13] + tmpFx[51]*tmpObjSEndTerm[26] + tmpFx[69]*tmpObjSEndTerm[39] + tmpFx[87]*tmpObjSEndTerm[52] + tmpFx[105]*tmpObjSEndTerm[65] + tmpFx[123]*tmpObjSEndTerm[78] + tmpFx[141]*tmpObjSEndTerm[91] + tmpFx[159]*tmpObjSEndTerm[104] + tmpFx[177]*tmpObjSEndTerm[117] + tmpFx[195]*tmpObjSEndTerm[130] + tmpFx[213]*tmpObjSEndTerm[143] + tmpFx[231]*tmpObjSEndTerm[156];
tmpQN2[196] = + tmpFx[15]*tmpObjSEndTerm[1] + tmpFx[33]*tmpObjSEndTerm[14] + tmpFx[51]*tmpObjSEndTerm[27] + tmpFx[69]*tmpObjSEndTerm[40] + tmpFx[87]*tmpObjSEndTerm[53] + tmpFx[105]*tmpObjSEndTerm[66] + tmpFx[123]*tmpObjSEndTerm[79] + tmpFx[141]*tmpObjSEndTerm[92] + tmpFx[159]*tmpObjSEndTerm[105] + tmpFx[177]*tmpObjSEndTerm[118] + tmpFx[195]*tmpObjSEndTerm[131] + tmpFx[213]*tmpObjSEndTerm[144] + tmpFx[231]*tmpObjSEndTerm[157];
tmpQN2[197] = + tmpFx[15]*tmpObjSEndTerm[2] + tmpFx[33]*tmpObjSEndTerm[15] + tmpFx[51]*tmpObjSEndTerm[28] + tmpFx[69]*tmpObjSEndTerm[41] + tmpFx[87]*tmpObjSEndTerm[54] + tmpFx[105]*tmpObjSEndTerm[67] + tmpFx[123]*tmpObjSEndTerm[80] + tmpFx[141]*tmpObjSEndTerm[93] + tmpFx[159]*tmpObjSEndTerm[106] + tmpFx[177]*tmpObjSEndTerm[119] + tmpFx[195]*tmpObjSEndTerm[132] + tmpFx[213]*tmpObjSEndTerm[145] + tmpFx[231]*tmpObjSEndTerm[158];
tmpQN2[198] = + tmpFx[15]*tmpObjSEndTerm[3] + tmpFx[33]*tmpObjSEndTerm[16] + tmpFx[51]*tmpObjSEndTerm[29] + tmpFx[69]*tmpObjSEndTerm[42] + tmpFx[87]*tmpObjSEndTerm[55] + tmpFx[105]*tmpObjSEndTerm[68] + tmpFx[123]*tmpObjSEndTerm[81] + tmpFx[141]*tmpObjSEndTerm[94] + tmpFx[159]*tmpObjSEndTerm[107] + tmpFx[177]*tmpObjSEndTerm[120] + tmpFx[195]*tmpObjSEndTerm[133] + tmpFx[213]*tmpObjSEndTerm[146] + tmpFx[231]*tmpObjSEndTerm[159];
tmpQN2[199] = + tmpFx[15]*tmpObjSEndTerm[4] + tmpFx[33]*tmpObjSEndTerm[17] + tmpFx[51]*tmpObjSEndTerm[30] + tmpFx[69]*tmpObjSEndTerm[43] + tmpFx[87]*tmpObjSEndTerm[56] + tmpFx[105]*tmpObjSEndTerm[69] + tmpFx[123]*tmpObjSEndTerm[82] + tmpFx[141]*tmpObjSEndTerm[95] + tmpFx[159]*tmpObjSEndTerm[108] + tmpFx[177]*tmpObjSEndTerm[121] + tmpFx[195]*tmpObjSEndTerm[134] + tmpFx[213]*tmpObjSEndTerm[147] + tmpFx[231]*tmpObjSEndTerm[160];
tmpQN2[200] = + tmpFx[15]*tmpObjSEndTerm[5] + tmpFx[33]*tmpObjSEndTerm[18] + tmpFx[51]*tmpObjSEndTerm[31] + tmpFx[69]*tmpObjSEndTerm[44] + tmpFx[87]*tmpObjSEndTerm[57] + tmpFx[105]*tmpObjSEndTerm[70] + tmpFx[123]*tmpObjSEndTerm[83] + tmpFx[141]*tmpObjSEndTerm[96] + tmpFx[159]*tmpObjSEndTerm[109] + tmpFx[177]*tmpObjSEndTerm[122] + tmpFx[195]*tmpObjSEndTerm[135] + tmpFx[213]*tmpObjSEndTerm[148] + tmpFx[231]*tmpObjSEndTerm[161];
tmpQN2[201] = + tmpFx[15]*tmpObjSEndTerm[6] + tmpFx[33]*tmpObjSEndTerm[19] + tmpFx[51]*tmpObjSEndTerm[32] + tmpFx[69]*tmpObjSEndTerm[45] + tmpFx[87]*tmpObjSEndTerm[58] + tmpFx[105]*tmpObjSEndTerm[71] + tmpFx[123]*tmpObjSEndTerm[84] + tmpFx[141]*tmpObjSEndTerm[97] + tmpFx[159]*tmpObjSEndTerm[110] + tmpFx[177]*tmpObjSEndTerm[123] + tmpFx[195]*tmpObjSEndTerm[136] + tmpFx[213]*tmpObjSEndTerm[149] + tmpFx[231]*tmpObjSEndTerm[162];
tmpQN2[202] = + tmpFx[15]*tmpObjSEndTerm[7] + tmpFx[33]*tmpObjSEndTerm[20] + tmpFx[51]*tmpObjSEndTerm[33] + tmpFx[69]*tmpObjSEndTerm[46] + tmpFx[87]*tmpObjSEndTerm[59] + tmpFx[105]*tmpObjSEndTerm[72] + tmpFx[123]*tmpObjSEndTerm[85] + tmpFx[141]*tmpObjSEndTerm[98] + tmpFx[159]*tmpObjSEndTerm[111] + tmpFx[177]*tmpObjSEndTerm[124] + tmpFx[195]*tmpObjSEndTerm[137] + tmpFx[213]*tmpObjSEndTerm[150] + tmpFx[231]*tmpObjSEndTerm[163];
tmpQN2[203] = + tmpFx[15]*tmpObjSEndTerm[8] + tmpFx[33]*tmpObjSEndTerm[21] + tmpFx[51]*tmpObjSEndTerm[34] + tmpFx[69]*tmpObjSEndTerm[47] + tmpFx[87]*tmpObjSEndTerm[60] + tmpFx[105]*tmpObjSEndTerm[73] + tmpFx[123]*tmpObjSEndTerm[86] + tmpFx[141]*tmpObjSEndTerm[99] + tmpFx[159]*tmpObjSEndTerm[112] + tmpFx[177]*tmpObjSEndTerm[125] + tmpFx[195]*tmpObjSEndTerm[138] + tmpFx[213]*tmpObjSEndTerm[151] + tmpFx[231]*tmpObjSEndTerm[164];
tmpQN2[204] = + tmpFx[15]*tmpObjSEndTerm[9] + tmpFx[33]*tmpObjSEndTerm[22] + tmpFx[51]*tmpObjSEndTerm[35] + tmpFx[69]*tmpObjSEndTerm[48] + tmpFx[87]*tmpObjSEndTerm[61] + tmpFx[105]*tmpObjSEndTerm[74] + tmpFx[123]*tmpObjSEndTerm[87] + tmpFx[141]*tmpObjSEndTerm[100] + tmpFx[159]*tmpObjSEndTerm[113] + tmpFx[177]*tmpObjSEndTerm[126] + tmpFx[195]*tmpObjSEndTerm[139] + tmpFx[213]*tmpObjSEndTerm[152] + tmpFx[231]*tmpObjSEndTerm[165];
tmpQN2[205] = + tmpFx[15]*tmpObjSEndTerm[10] + tmpFx[33]*tmpObjSEndTerm[23] + tmpFx[51]*tmpObjSEndTerm[36] + tmpFx[69]*tmpObjSEndTerm[49] + tmpFx[87]*tmpObjSEndTerm[62] + tmpFx[105]*tmpObjSEndTerm[75] + tmpFx[123]*tmpObjSEndTerm[88] + tmpFx[141]*tmpObjSEndTerm[101] + tmpFx[159]*tmpObjSEndTerm[114] + tmpFx[177]*tmpObjSEndTerm[127] + tmpFx[195]*tmpObjSEndTerm[140] + tmpFx[213]*tmpObjSEndTerm[153] + tmpFx[231]*tmpObjSEndTerm[166];
tmpQN2[206] = + tmpFx[15]*tmpObjSEndTerm[11] + tmpFx[33]*tmpObjSEndTerm[24] + tmpFx[51]*tmpObjSEndTerm[37] + tmpFx[69]*tmpObjSEndTerm[50] + tmpFx[87]*tmpObjSEndTerm[63] + tmpFx[105]*tmpObjSEndTerm[76] + tmpFx[123]*tmpObjSEndTerm[89] + tmpFx[141]*tmpObjSEndTerm[102] + tmpFx[159]*tmpObjSEndTerm[115] + tmpFx[177]*tmpObjSEndTerm[128] + tmpFx[195]*tmpObjSEndTerm[141] + tmpFx[213]*tmpObjSEndTerm[154] + tmpFx[231]*tmpObjSEndTerm[167];
tmpQN2[207] = + tmpFx[15]*tmpObjSEndTerm[12] + tmpFx[33]*tmpObjSEndTerm[25] + tmpFx[51]*tmpObjSEndTerm[38] + tmpFx[69]*tmpObjSEndTerm[51] + tmpFx[87]*tmpObjSEndTerm[64] + tmpFx[105]*tmpObjSEndTerm[77] + tmpFx[123]*tmpObjSEndTerm[90] + tmpFx[141]*tmpObjSEndTerm[103] + tmpFx[159]*tmpObjSEndTerm[116] + tmpFx[177]*tmpObjSEndTerm[129] + tmpFx[195]*tmpObjSEndTerm[142] + tmpFx[213]*tmpObjSEndTerm[155] + tmpFx[231]*tmpObjSEndTerm[168];
tmpQN2[208] = + tmpFx[16]*tmpObjSEndTerm[0] + tmpFx[34]*tmpObjSEndTerm[13] + tmpFx[52]*tmpObjSEndTerm[26] + tmpFx[70]*tmpObjSEndTerm[39] + tmpFx[88]*tmpObjSEndTerm[52] + tmpFx[106]*tmpObjSEndTerm[65] + tmpFx[124]*tmpObjSEndTerm[78] + tmpFx[142]*tmpObjSEndTerm[91] + tmpFx[160]*tmpObjSEndTerm[104] + tmpFx[178]*tmpObjSEndTerm[117] + tmpFx[196]*tmpObjSEndTerm[130] + tmpFx[214]*tmpObjSEndTerm[143] + tmpFx[232]*tmpObjSEndTerm[156];
tmpQN2[209] = + tmpFx[16]*tmpObjSEndTerm[1] + tmpFx[34]*tmpObjSEndTerm[14] + tmpFx[52]*tmpObjSEndTerm[27] + tmpFx[70]*tmpObjSEndTerm[40] + tmpFx[88]*tmpObjSEndTerm[53] + tmpFx[106]*tmpObjSEndTerm[66] + tmpFx[124]*tmpObjSEndTerm[79] + tmpFx[142]*tmpObjSEndTerm[92] + tmpFx[160]*tmpObjSEndTerm[105] + tmpFx[178]*tmpObjSEndTerm[118] + tmpFx[196]*tmpObjSEndTerm[131] + tmpFx[214]*tmpObjSEndTerm[144] + tmpFx[232]*tmpObjSEndTerm[157];
tmpQN2[210] = + tmpFx[16]*tmpObjSEndTerm[2] + tmpFx[34]*tmpObjSEndTerm[15] + tmpFx[52]*tmpObjSEndTerm[28] + tmpFx[70]*tmpObjSEndTerm[41] + tmpFx[88]*tmpObjSEndTerm[54] + tmpFx[106]*tmpObjSEndTerm[67] + tmpFx[124]*tmpObjSEndTerm[80] + tmpFx[142]*tmpObjSEndTerm[93] + tmpFx[160]*tmpObjSEndTerm[106] + tmpFx[178]*tmpObjSEndTerm[119] + tmpFx[196]*tmpObjSEndTerm[132] + tmpFx[214]*tmpObjSEndTerm[145] + tmpFx[232]*tmpObjSEndTerm[158];
tmpQN2[211] = + tmpFx[16]*tmpObjSEndTerm[3] + tmpFx[34]*tmpObjSEndTerm[16] + tmpFx[52]*tmpObjSEndTerm[29] + tmpFx[70]*tmpObjSEndTerm[42] + tmpFx[88]*tmpObjSEndTerm[55] + tmpFx[106]*tmpObjSEndTerm[68] + tmpFx[124]*tmpObjSEndTerm[81] + tmpFx[142]*tmpObjSEndTerm[94] + tmpFx[160]*tmpObjSEndTerm[107] + tmpFx[178]*tmpObjSEndTerm[120] + tmpFx[196]*tmpObjSEndTerm[133] + tmpFx[214]*tmpObjSEndTerm[146] + tmpFx[232]*tmpObjSEndTerm[159];
tmpQN2[212] = + tmpFx[16]*tmpObjSEndTerm[4] + tmpFx[34]*tmpObjSEndTerm[17] + tmpFx[52]*tmpObjSEndTerm[30] + tmpFx[70]*tmpObjSEndTerm[43] + tmpFx[88]*tmpObjSEndTerm[56] + tmpFx[106]*tmpObjSEndTerm[69] + tmpFx[124]*tmpObjSEndTerm[82] + tmpFx[142]*tmpObjSEndTerm[95] + tmpFx[160]*tmpObjSEndTerm[108] + tmpFx[178]*tmpObjSEndTerm[121] + tmpFx[196]*tmpObjSEndTerm[134] + tmpFx[214]*tmpObjSEndTerm[147] + tmpFx[232]*tmpObjSEndTerm[160];
tmpQN2[213] = + tmpFx[16]*tmpObjSEndTerm[5] + tmpFx[34]*tmpObjSEndTerm[18] + tmpFx[52]*tmpObjSEndTerm[31] + tmpFx[70]*tmpObjSEndTerm[44] + tmpFx[88]*tmpObjSEndTerm[57] + tmpFx[106]*tmpObjSEndTerm[70] + tmpFx[124]*tmpObjSEndTerm[83] + tmpFx[142]*tmpObjSEndTerm[96] + tmpFx[160]*tmpObjSEndTerm[109] + tmpFx[178]*tmpObjSEndTerm[122] + tmpFx[196]*tmpObjSEndTerm[135] + tmpFx[214]*tmpObjSEndTerm[148] + tmpFx[232]*tmpObjSEndTerm[161];
tmpQN2[214] = + tmpFx[16]*tmpObjSEndTerm[6] + tmpFx[34]*tmpObjSEndTerm[19] + tmpFx[52]*tmpObjSEndTerm[32] + tmpFx[70]*tmpObjSEndTerm[45] + tmpFx[88]*tmpObjSEndTerm[58] + tmpFx[106]*tmpObjSEndTerm[71] + tmpFx[124]*tmpObjSEndTerm[84] + tmpFx[142]*tmpObjSEndTerm[97] + tmpFx[160]*tmpObjSEndTerm[110] + tmpFx[178]*tmpObjSEndTerm[123] + tmpFx[196]*tmpObjSEndTerm[136] + tmpFx[214]*tmpObjSEndTerm[149] + tmpFx[232]*tmpObjSEndTerm[162];
tmpQN2[215] = + tmpFx[16]*tmpObjSEndTerm[7] + tmpFx[34]*tmpObjSEndTerm[20] + tmpFx[52]*tmpObjSEndTerm[33] + tmpFx[70]*tmpObjSEndTerm[46] + tmpFx[88]*tmpObjSEndTerm[59] + tmpFx[106]*tmpObjSEndTerm[72] + tmpFx[124]*tmpObjSEndTerm[85] + tmpFx[142]*tmpObjSEndTerm[98] + tmpFx[160]*tmpObjSEndTerm[111] + tmpFx[178]*tmpObjSEndTerm[124] + tmpFx[196]*tmpObjSEndTerm[137] + tmpFx[214]*tmpObjSEndTerm[150] + tmpFx[232]*tmpObjSEndTerm[163];
tmpQN2[216] = + tmpFx[16]*tmpObjSEndTerm[8] + tmpFx[34]*tmpObjSEndTerm[21] + tmpFx[52]*tmpObjSEndTerm[34] + tmpFx[70]*tmpObjSEndTerm[47] + tmpFx[88]*tmpObjSEndTerm[60] + tmpFx[106]*tmpObjSEndTerm[73] + tmpFx[124]*tmpObjSEndTerm[86] + tmpFx[142]*tmpObjSEndTerm[99] + tmpFx[160]*tmpObjSEndTerm[112] + tmpFx[178]*tmpObjSEndTerm[125] + tmpFx[196]*tmpObjSEndTerm[138] + tmpFx[214]*tmpObjSEndTerm[151] + tmpFx[232]*tmpObjSEndTerm[164];
tmpQN2[217] = + tmpFx[16]*tmpObjSEndTerm[9] + tmpFx[34]*tmpObjSEndTerm[22] + tmpFx[52]*tmpObjSEndTerm[35] + tmpFx[70]*tmpObjSEndTerm[48] + tmpFx[88]*tmpObjSEndTerm[61] + tmpFx[106]*tmpObjSEndTerm[74] + tmpFx[124]*tmpObjSEndTerm[87] + tmpFx[142]*tmpObjSEndTerm[100] + tmpFx[160]*tmpObjSEndTerm[113] + tmpFx[178]*tmpObjSEndTerm[126] + tmpFx[196]*tmpObjSEndTerm[139] + tmpFx[214]*tmpObjSEndTerm[152] + tmpFx[232]*tmpObjSEndTerm[165];
tmpQN2[218] = + tmpFx[16]*tmpObjSEndTerm[10] + tmpFx[34]*tmpObjSEndTerm[23] + tmpFx[52]*tmpObjSEndTerm[36] + tmpFx[70]*tmpObjSEndTerm[49] + tmpFx[88]*tmpObjSEndTerm[62] + tmpFx[106]*tmpObjSEndTerm[75] + tmpFx[124]*tmpObjSEndTerm[88] + tmpFx[142]*tmpObjSEndTerm[101] + tmpFx[160]*tmpObjSEndTerm[114] + tmpFx[178]*tmpObjSEndTerm[127] + tmpFx[196]*tmpObjSEndTerm[140] + tmpFx[214]*tmpObjSEndTerm[153] + tmpFx[232]*tmpObjSEndTerm[166];
tmpQN2[219] = + tmpFx[16]*tmpObjSEndTerm[11] + tmpFx[34]*tmpObjSEndTerm[24] + tmpFx[52]*tmpObjSEndTerm[37] + tmpFx[70]*tmpObjSEndTerm[50] + tmpFx[88]*tmpObjSEndTerm[63] + tmpFx[106]*tmpObjSEndTerm[76] + tmpFx[124]*tmpObjSEndTerm[89] + tmpFx[142]*tmpObjSEndTerm[102] + tmpFx[160]*tmpObjSEndTerm[115] + tmpFx[178]*tmpObjSEndTerm[128] + tmpFx[196]*tmpObjSEndTerm[141] + tmpFx[214]*tmpObjSEndTerm[154] + tmpFx[232]*tmpObjSEndTerm[167];
tmpQN2[220] = + tmpFx[16]*tmpObjSEndTerm[12] + tmpFx[34]*tmpObjSEndTerm[25] + tmpFx[52]*tmpObjSEndTerm[38] + tmpFx[70]*tmpObjSEndTerm[51] + tmpFx[88]*tmpObjSEndTerm[64] + tmpFx[106]*tmpObjSEndTerm[77] + tmpFx[124]*tmpObjSEndTerm[90] + tmpFx[142]*tmpObjSEndTerm[103] + tmpFx[160]*tmpObjSEndTerm[116] + tmpFx[178]*tmpObjSEndTerm[129] + tmpFx[196]*tmpObjSEndTerm[142] + tmpFx[214]*tmpObjSEndTerm[155] + tmpFx[232]*tmpObjSEndTerm[168];
tmpQN2[221] = + tmpFx[17]*tmpObjSEndTerm[0] + tmpFx[35]*tmpObjSEndTerm[13] + tmpFx[53]*tmpObjSEndTerm[26] + tmpFx[71]*tmpObjSEndTerm[39] + tmpFx[89]*tmpObjSEndTerm[52] + tmpFx[107]*tmpObjSEndTerm[65] + tmpFx[125]*tmpObjSEndTerm[78] + tmpFx[143]*tmpObjSEndTerm[91] + tmpFx[161]*tmpObjSEndTerm[104] + tmpFx[179]*tmpObjSEndTerm[117] + tmpFx[197]*tmpObjSEndTerm[130] + tmpFx[215]*tmpObjSEndTerm[143] + tmpFx[233]*tmpObjSEndTerm[156];
tmpQN2[222] = + tmpFx[17]*tmpObjSEndTerm[1] + tmpFx[35]*tmpObjSEndTerm[14] + tmpFx[53]*tmpObjSEndTerm[27] + tmpFx[71]*tmpObjSEndTerm[40] + tmpFx[89]*tmpObjSEndTerm[53] + tmpFx[107]*tmpObjSEndTerm[66] + tmpFx[125]*tmpObjSEndTerm[79] + tmpFx[143]*tmpObjSEndTerm[92] + tmpFx[161]*tmpObjSEndTerm[105] + tmpFx[179]*tmpObjSEndTerm[118] + tmpFx[197]*tmpObjSEndTerm[131] + tmpFx[215]*tmpObjSEndTerm[144] + tmpFx[233]*tmpObjSEndTerm[157];
tmpQN2[223] = + tmpFx[17]*tmpObjSEndTerm[2] + tmpFx[35]*tmpObjSEndTerm[15] + tmpFx[53]*tmpObjSEndTerm[28] + tmpFx[71]*tmpObjSEndTerm[41] + tmpFx[89]*tmpObjSEndTerm[54] + tmpFx[107]*tmpObjSEndTerm[67] + tmpFx[125]*tmpObjSEndTerm[80] + tmpFx[143]*tmpObjSEndTerm[93] + tmpFx[161]*tmpObjSEndTerm[106] + tmpFx[179]*tmpObjSEndTerm[119] + tmpFx[197]*tmpObjSEndTerm[132] + tmpFx[215]*tmpObjSEndTerm[145] + tmpFx[233]*tmpObjSEndTerm[158];
tmpQN2[224] = + tmpFx[17]*tmpObjSEndTerm[3] + tmpFx[35]*tmpObjSEndTerm[16] + tmpFx[53]*tmpObjSEndTerm[29] + tmpFx[71]*tmpObjSEndTerm[42] + tmpFx[89]*tmpObjSEndTerm[55] + tmpFx[107]*tmpObjSEndTerm[68] + tmpFx[125]*tmpObjSEndTerm[81] + tmpFx[143]*tmpObjSEndTerm[94] + tmpFx[161]*tmpObjSEndTerm[107] + tmpFx[179]*tmpObjSEndTerm[120] + tmpFx[197]*tmpObjSEndTerm[133] + tmpFx[215]*tmpObjSEndTerm[146] + tmpFx[233]*tmpObjSEndTerm[159];
tmpQN2[225] = + tmpFx[17]*tmpObjSEndTerm[4] + tmpFx[35]*tmpObjSEndTerm[17] + tmpFx[53]*tmpObjSEndTerm[30] + tmpFx[71]*tmpObjSEndTerm[43] + tmpFx[89]*tmpObjSEndTerm[56] + tmpFx[107]*tmpObjSEndTerm[69] + tmpFx[125]*tmpObjSEndTerm[82] + tmpFx[143]*tmpObjSEndTerm[95] + tmpFx[161]*tmpObjSEndTerm[108] + tmpFx[179]*tmpObjSEndTerm[121] + tmpFx[197]*tmpObjSEndTerm[134] + tmpFx[215]*tmpObjSEndTerm[147] + tmpFx[233]*tmpObjSEndTerm[160];
tmpQN2[226] = + tmpFx[17]*tmpObjSEndTerm[5] + tmpFx[35]*tmpObjSEndTerm[18] + tmpFx[53]*tmpObjSEndTerm[31] + tmpFx[71]*tmpObjSEndTerm[44] + tmpFx[89]*tmpObjSEndTerm[57] + tmpFx[107]*tmpObjSEndTerm[70] + tmpFx[125]*tmpObjSEndTerm[83] + tmpFx[143]*tmpObjSEndTerm[96] + tmpFx[161]*tmpObjSEndTerm[109] + tmpFx[179]*tmpObjSEndTerm[122] + tmpFx[197]*tmpObjSEndTerm[135] + tmpFx[215]*tmpObjSEndTerm[148] + tmpFx[233]*tmpObjSEndTerm[161];
tmpQN2[227] = + tmpFx[17]*tmpObjSEndTerm[6] + tmpFx[35]*tmpObjSEndTerm[19] + tmpFx[53]*tmpObjSEndTerm[32] + tmpFx[71]*tmpObjSEndTerm[45] + tmpFx[89]*tmpObjSEndTerm[58] + tmpFx[107]*tmpObjSEndTerm[71] + tmpFx[125]*tmpObjSEndTerm[84] + tmpFx[143]*tmpObjSEndTerm[97] + tmpFx[161]*tmpObjSEndTerm[110] + tmpFx[179]*tmpObjSEndTerm[123] + tmpFx[197]*tmpObjSEndTerm[136] + tmpFx[215]*tmpObjSEndTerm[149] + tmpFx[233]*tmpObjSEndTerm[162];
tmpQN2[228] = + tmpFx[17]*tmpObjSEndTerm[7] + tmpFx[35]*tmpObjSEndTerm[20] + tmpFx[53]*tmpObjSEndTerm[33] + tmpFx[71]*tmpObjSEndTerm[46] + tmpFx[89]*tmpObjSEndTerm[59] + tmpFx[107]*tmpObjSEndTerm[72] + tmpFx[125]*tmpObjSEndTerm[85] + tmpFx[143]*tmpObjSEndTerm[98] + tmpFx[161]*tmpObjSEndTerm[111] + tmpFx[179]*tmpObjSEndTerm[124] + tmpFx[197]*tmpObjSEndTerm[137] + tmpFx[215]*tmpObjSEndTerm[150] + tmpFx[233]*tmpObjSEndTerm[163];
tmpQN2[229] = + tmpFx[17]*tmpObjSEndTerm[8] + tmpFx[35]*tmpObjSEndTerm[21] + tmpFx[53]*tmpObjSEndTerm[34] + tmpFx[71]*tmpObjSEndTerm[47] + tmpFx[89]*tmpObjSEndTerm[60] + tmpFx[107]*tmpObjSEndTerm[73] + tmpFx[125]*tmpObjSEndTerm[86] + tmpFx[143]*tmpObjSEndTerm[99] + tmpFx[161]*tmpObjSEndTerm[112] + tmpFx[179]*tmpObjSEndTerm[125] + tmpFx[197]*tmpObjSEndTerm[138] + tmpFx[215]*tmpObjSEndTerm[151] + tmpFx[233]*tmpObjSEndTerm[164];
tmpQN2[230] = + tmpFx[17]*tmpObjSEndTerm[9] + tmpFx[35]*tmpObjSEndTerm[22] + tmpFx[53]*tmpObjSEndTerm[35] + tmpFx[71]*tmpObjSEndTerm[48] + tmpFx[89]*tmpObjSEndTerm[61] + tmpFx[107]*tmpObjSEndTerm[74] + tmpFx[125]*tmpObjSEndTerm[87] + tmpFx[143]*tmpObjSEndTerm[100] + tmpFx[161]*tmpObjSEndTerm[113] + tmpFx[179]*tmpObjSEndTerm[126] + tmpFx[197]*tmpObjSEndTerm[139] + tmpFx[215]*tmpObjSEndTerm[152] + tmpFx[233]*tmpObjSEndTerm[165];
tmpQN2[231] = + tmpFx[17]*tmpObjSEndTerm[10] + tmpFx[35]*tmpObjSEndTerm[23] + tmpFx[53]*tmpObjSEndTerm[36] + tmpFx[71]*tmpObjSEndTerm[49] + tmpFx[89]*tmpObjSEndTerm[62] + tmpFx[107]*tmpObjSEndTerm[75] + tmpFx[125]*tmpObjSEndTerm[88] + tmpFx[143]*tmpObjSEndTerm[101] + tmpFx[161]*tmpObjSEndTerm[114] + tmpFx[179]*tmpObjSEndTerm[127] + tmpFx[197]*tmpObjSEndTerm[140] + tmpFx[215]*tmpObjSEndTerm[153] + tmpFx[233]*tmpObjSEndTerm[166];
tmpQN2[232] = + tmpFx[17]*tmpObjSEndTerm[11] + tmpFx[35]*tmpObjSEndTerm[24] + tmpFx[53]*tmpObjSEndTerm[37] + tmpFx[71]*tmpObjSEndTerm[50] + tmpFx[89]*tmpObjSEndTerm[63] + tmpFx[107]*tmpObjSEndTerm[76] + tmpFx[125]*tmpObjSEndTerm[89] + tmpFx[143]*tmpObjSEndTerm[102] + tmpFx[161]*tmpObjSEndTerm[115] + tmpFx[179]*tmpObjSEndTerm[128] + tmpFx[197]*tmpObjSEndTerm[141] + tmpFx[215]*tmpObjSEndTerm[154] + tmpFx[233]*tmpObjSEndTerm[167];
tmpQN2[233] = + tmpFx[17]*tmpObjSEndTerm[12] + tmpFx[35]*tmpObjSEndTerm[25] + tmpFx[53]*tmpObjSEndTerm[38] + tmpFx[71]*tmpObjSEndTerm[51] + tmpFx[89]*tmpObjSEndTerm[64] + tmpFx[107]*tmpObjSEndTerm[77] + tmpFx[125]*tmpObjSEndTerm[90] + tmpFx[143]*tmpObjSEndTerm[103] + tmpFx[161]*tmpObjSEndTerm[116] + tmpFx[179]*tmpObjSEndTerm[129] + tmpFx[197]*tmpObjSEndTerm[142] + tmpFx[215]*tmpObjSEndTerm[155] + tmpFx[233]*tmpObjSEndTerm[168];
for (lRun1 = 0; lRun1 < 18; ++lRun1)
{
for (lRun2 = 0; lRun2 < 18; ++lRun2)
{
real_t t = 0.0;
for (lRun3 = 0; lRun3 < 13; ++lRun3)
{
t += + tmpQN2[(lRun1 * 13) + (lRun3)]*tmpFx[(lRun3 * 18) + (lRun2)];
}
tmpQN1[(lRun1 * 18) + (lRun2)] = t;
}
}
}

void acado_evaluateObjective(  )
{
int runObj;
for (runObj = 0; runObj < 10; ++runObj)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[runObj * 18];
acadoWorkspace.objValueIn[1] = acadoVariables.x[runObj * 18 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[runObj * 18 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[runObj * 18 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[runObj * 18 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[runObj * 18 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[runObj * 18 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[runObj * 18 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[runObj * 18 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.x[runObj * 18 + 9];
acadoWorkspace.objValueIn[10] = acadoVariables.x[runObj * 18 + 10];
acadoWorkspace.objValueIn[11] = acadoVariables.x[runObj * 18 + 11];
acadoWorkspace.objValueIn[12] = acadoVariables.x[runObj * 18 + 12];
acadoWorkspace.objValueIn[13] = acadoVariables.x[runObj * 18 + 13];
acadoWorkspace.objValueIn[14] = acadoVariables.x[runObj * 18 + 14];
acadoWorkspace.objValueIn[15] = acadoVariables.x[runObj * 18 + 15];
acadoWorkspace.objValueIn[16] = acadoVariables.x[runObj * 18 + 16];
acadoWorkspace.objValueIn[17] = acadoVariables.x[runObj * 18 + 17];
acadoWorkspace.objValueIn[18] = acadoVariables.u[runObj * 6];
acadoWorkspace.objValueIn[19] = acadoVariables.u[runObj * 6 + 1];
acadoWorkspace.objValueIn[20] = acadoVariables.u[runObj * 6 + 2];
acadoWorkspace.objValueIn[21] = acadoVariables.u[runObj * 6 + 3];
acadoWorkspace.objValueIn[22] = acadoVariables.u[runObj * 6 + 4];
acadoWorkspace.objValueIn[23] = acadoVariables.u[runObj * 6 + 5];
acadoWorkspace.objValueIn[24] = acadoVariables.od[runObj * 9];
acadoWorkspace.objValueIn[25] = acadoVariables.od[runObj * 9 + 1];
acadoWorkspace.objValueIn[26] = acadoVariables.od[runObj * 9 + 2];
acadoWorkspace.objValueIn[27] = acadoVariables.od[runObj * 9 + 3];
acadoWorkspace.objValueIn[28] = acadoVariables.od[runObj * 9 + 4];
acadoWorkspace.objValueIn[29] = acadoVariables.od[runObj * 9 + 5];
acadoWorkspace.objValueIn[30] = acadoVariables.od[runObj * 9 + 6];
acadoWorkspace.objValueIn[31] = acadoVariables.od[runObj * 9 + 7];
acadoWorkspace.objValueIn[32] = acadoVariables.od[runObj * 9 + 8];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[runObj * 19] = acadoWorkspace.objValueOut[0];
acadoWorkspace.Dy[runObj * 19 + 1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.Dy[runObj * 19 + 2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.Dy[runObj * 19 + 3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.Dy[runObj * 19 + 4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.Dy[runObj * 19 + 5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.Dy[runObj * 19 + 6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.Dy[runObj * 19 + 7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.Dy[runObj * 19 + 8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.Dy[runObj * 19 + 9] = acadoWorkspace.objValueOut[9];
acadoWorkspace.Dy[runObj * 19 + 10] = acadoWorkspace.objValueOut[10];
acadoWorkspace.Dy[runObj * 19 + 11] = acadoWorkspace.objValueOut[11];
acadoWorkspace.Dy[runObj * 19 + 12] = acadoWorkspace.objValueOut[12];
acadoWorkspace.Dy[runObj * 19 + 13] = acadoWorkspace.objValueOut[13];
acadoWorkspace.Dy[runObj * 19 + 14] = acadoWorkspace.objValueOut[14];
acadoWorkspace.Dy[runObj * 19 + 15] = acadoWorkspace.objValueOut[15];
acadoWorkspace.Dy[runObj * 19 + 16] = acadoWorkspace.objValueOut[16];
acadoWorkspace.Dy[runObj * 19 + 17] = acadoWorkspace.objValueOut[17];
acadoWorkspace.Dy[runObj * 19 + 18] = acadoWorkspace.objValueOut[18];

acado_setObjQ1Q2( &(acadoWorkspace.objValueOut[ 19 ]), acadoVariables.W, &(acadoWorkspace.Q1[ runObj * 324 ]), &(acadoWorkspace.Q2[ runObj * 342 ]) );

acado_setObjR1R2( acadoVariables.W, &(acadoWorkspace.R1[ runObj * 36 ]), &(acadoWorkspace.R2[ runObj * 114 ]) );

}
acadoWorkspace.objValueIn[0] = acadoVariables.x[180];
acadoWorkspace.objValueIn[1] = acadoVariables.x[181];
acadoWorkspace.objValueIn[2] = acadoVariables.x[182];
acadoWorkspace.objValueIn[3] = acadoVariables.x[183];
acadoWorkspace.objValueIn[4] = acadoVariables.x[184];
acadoWorkspace.objValueIn[5] = acadoVariables.x[185];
acadoWorkspace.objValueIn[6] = acadoVariables.x[186];
acadoWorkspace.objValueIn[7] = acadoVariables.x[187];
acadoWorkspace.objValueIn[8] = acadoVariables.x[188];
acadoWorkspace.objValueIn[9] = acadoVariables.x[189];
acadoWorkspace.objValueIn[10] = acadoVariables.x[190];
acadoWorkspace.objValueIn[11] = acadoVariables.x[191];
acadoWorkspace.objValueIn[12] = acadoVariables.x[192];
acadoWorkspace.objValueIn[13] = acadoVariables.x[193];
acadoWorkspace.objValueIn[14] = acadoVariables.x[194];
acadoWorkspace.objValueIn[15] = acadoVariables.x[195];
acadoWorkspace.objValueIn[16] = acadoVariables.x[196];
acadoWorkspace.objValueIn[17] = acadoVariables.x[197];
acadoWorkspace.objValueIn[18] = acadoVariables.od[90];
acadoWorkspace.objValueIn[19] = acadoVariables.od[91];
acadoWorkspace.objValueIn[20] = acadoVariables.od[92];
acadoWorkspace.objValueIn[21] = acadoVariables.od[93];
acadoWorkspace.objValueIn[22] = acadoVariables.od[94];
acadoWorkspace.objValueIn[23] = acadoVariables.od[95];
acadoWorkspace.objValueIn[24] = acadoVariables.od[96];
acadoWorkspace.objValueIn[25] = acadoVariables.od[97];
acadoWorkspace.objValueIn[26] = acadoVariables.od[98];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );

acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6];
acadoWorkspace.DyN[7] = acadoWorkspace.objValueOut[7];
acadoWorkspace.DyN[8] = acadoWorkspace.objValueOut[8];
acadoWorkspace.DyN[9] = acadoWorkspace.objValueOut[9];
acadoWorkspace.DyN[10] = acadoWorkspace.objValueOut[10];
acadoWorkspace.DyN[11] = acadoWorkspace.objValueOut[11];
acadoWorkspace.DyN[12] = acadoWorkspace.objValueOut[12];

acado_setObjQN1QN2( &(acadoWorkspace.objValueOut[ 13 ]), acadoVariables.WN, acadoWorkspace.QN1, acadoWorkspace.QN2 );

}

void acado_multGxGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[1]*Gu1[6] + Gx1[2]*Gu1[12] + Gx1[3]*Gu1[18] + Gx1[4]*Gu1[24] + Gx1[5]*Gu1[30] + Gx1[6]*Gu1[36] + Gx1[7]*Gu1[42] + Gx1[8]*Gu1[48] + Gx1[9]*Gu1[54] + Gx1[10]*Gu1[60] + Gx1[11]*Gu1[66] + Gx1[12]*Gu1[72] + Gx1[13]*Gu1[78] + Gx1[14]*Gu1[84] + Gx1[15]*Gu1[90] + Gx1[16]*Gu1[96] + Gx1[17]*Gu1[102];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[1]*Gu1[7] + Gx1[2]*Gu1[13] + Gx1[3]*Gu1[19] + Gx1[4]*Gu1[25] + Gx1[5]*Gu1[31] + Gx1[6]*Gu1[37] + Gx1[7]*Gu1[43] + Gx1[8]*Gu1[49] + Gx1[9]*Gu1[55] + Gx1[10]*Gu1[61] + Gx1[11]*Gu1[67] + Gx1[12]*Gu1[73] + Gx1[13]*Gu1[79] + Gx1[14]*Gu1[85] + Gx1[15]*Gu1[91] + Gx1[16]*Gu1[97] + Gx1[17]*Gu1[103];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[1]*Gu1[8] + Gx1[2]*Gu1[14] + Gx1[3]*Gu1[20] + Gx1[4]*Gu1[26] + Gx1[5]*Gu1[32] + Gx1[6]*Gu1[38] + Gx1[7]*Gu1[44] + Gx1[8]*Gu1[50] + Gx1[9]*Gu1[56] + Gx1[10]*Gu1[62] + Gx1[11]*Gu1[68] + Gx1[12]*Gu1[74] + Gx1[13]*Gu1[80] + Gx1[14]*Gu1[86] + Gx1[15]*Gu1[92] + Gx1[16]*Gu1[98] + Gx1[17]*Gu1[104];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[1]*Gu1[9] + Gx1[2]*Gu1[15] + Gx1[3]*Gu1[21] + Gx1[4]*Gu1[27] + Gx1[5]*Gu1[33] + Gx1[6]*Gu1[39] + Gx1[7]*Gu1[45] + Gx1[8]*Gu1[51] + Gx1[9]*Gu1[57] + Gx1[10]*Gu1[63] + Gx1[11]*Gu1[69] + Gx1[12]*Gu1[75] + Gx1[13]*Gu1[81] + Gx1[14]*Gu1[87] + Gx1[15]*Gu1[93] + Gx1[16]*Gu1[99] + Gx1[17]*Gu1[105];
Gu2[4] = + Gx1[0]*Gu1[4] + Gx1[1]*Gu1[10] + Gx1[2]*Gu1[16] + Gx1[3]*Gu1[22] + Gx1[4]*Gu1[28] + Gx1[5]*Gu1[34] + Gx1[6]*Gu1[40] + Gx1[7]*Gu1[46] + Gx1[8]*Gu1[52] + Gx1[9]*Gu1[58] + Gx1[10]*Gu1[64] + Gx1[11]*Gu1[70] + Gx1[12]*Gu1[76] + Gx1[13]*Gu1[82] + Gx1[14]*Gu1[88] + Gx1[15]*Gu1[94] + Gx1[16]*Gu1[100] + Gx1[17]*Gu1[106];
Gu2[5] = + Gx1[0]*Gu1[5] + Gx1[1]*Gu1[11] + Gx1[2]*Gu1[17] + Gx1[3]*Gu1[23] + Gx1[4]*Gu1[29] + Gx1[5]*Gu1[35] + Gx1[6]*Gu1[41] + Gx1[7]*Gu1[47] + Gx1[8]*Gu1[53] + Gx1[9]*Gu1[59] + Gx1[10]*Gu1[65] + Gx1[11]*Gu1[71] + Gx1[12]*Gu1[77] + Gx1[13]*Gu1[83] + Gx1[14]*Gu1[89] + Gx1[15]*Gu1[95] + Gx1[16]*Gu1[101] + Gx1[17]*Gu1[107];
Gu2[6] = + Gx1[18]*Gu1[0] + Gx1[19]*Gu1[6] + Gx1[20]*Gu1[12] + Gx1[21]*Gu1[18] + Gx1[22]*Gu1[24] + Gx1[23]*Gu1[30] + Gx1[24]*Gu1[36] + Gx1[25]*Gu1[42] + Gx1[26]*Gu1[48] + Gx1[27]*Gu1[54] + Gx1[28]*Gu1[60] + Gx1[29]*Gu1[66] + Gx1[30]*Gu1[72] + Gx1[31]*Gu1[78] + Gx1[32]*Gu1[84] + Gx1[33]*Gu1[90] + Gx1[34]*Gu1[96] + Gx1[35]*Gu1[102];
Gu2[7] = + Gx1[18]*Gu1[1] + Gx1[19]*Gu1[7] + Gx1[20]*Gu1[13] + Gx1[21]*Gu1[19] + Gx1[22]*Gu1[25] + Gx1[23]*Gu1[31] + Gx1[24]*Gu1[37] + Gx1[25]*Gu1[43] + Gx1[26]*Gu1[49] + Gx1[27]*Gu1[55] + Gx1[28]*Gu1[61] + Gx1[29]*Gu1[67] + Gx1[30]*Gu1[73] + Gx1[31]*Gu1[79] + Gx1[32]*Gu1[85] + Gx1[33]*Gu1[91] + Gx1[34]*Gu1[97] + Gx1[35]*Gu1[103];
Gu2[8] = + Gx1[18]*Gu1[2] + Gx1[19]*Gu1[8] + Gx1[20]*Gu1[14] + Gx1[21]*Gu1[20] + Gx1[22]*Gu1[26] + Gx1[23]*Gu1[32] + Gx1[24]*Gu1[38] + Gx1[25]*Gu1[44] + Gx1[26]*Gu1[50] + Gx1[27]*Gu1[56] + Gx1[28]*Gu1[62] + Gx1[29]*Gu1[68] + Gx1[30]*Gu1[74] + Gx1[31]*Gu1[80] + Gx1[32]*Gu1[86] + Gx1[33]*Gu1[92] + Gx1[34]*Gu1[98] + Gx1[35]*Gu1[104];
Gu2[9] = + Gx1[18]*Gu1[3] + Gx1[19]*Gu1[9] + Gx1[20]*Gu1[15] + Gx1[21]*Gu1[21] + Gx1[22]*Gu1[27] + Gx1[23]*Gu1[33] + Gx1[24]*Gu1[39] + Gx1[25]*Gu1[45] + Gx1[26]*Gu1[51] + Gx1[27]*Gu1[57] + Gx1[28]*Gu1[63] + Gx1[29]*Gu1[69] + Gx1[30]*Gu1[75] + Gx1[31]*Gu1[81] + Gx1[32]*Gu1[87] + Gx1[33]*Gu1[93] + Gx1[34]*Gu1[99] + Gx1[35]*Gu1[105];
Gu2[10] = + Gx1[18]*Gu1[4] + Gx1[19]*Gu1[10] + Gx1[20]*Gu1[16] + Gx1[21]*Gu1[22] + Gx1[22]*Gu1[28] + Gx1[23]*Gu1[34] + Gx1[24]*Gu1[40] + Gx1[25]*Gu1[46] + Gx1[26]*Gu1[52] + Gx1[27]*Gu1[58] + Gx1[28]*Gu1[64] + Gx1[29]*Gu1[70] + Gx1[30]*Gu1[76] + Gx1[31]*Gu1[82] + Gx1[32]*Gu1[88] + Gx1[33]*Gu1[94] + Gx1[34]*Gu1[100] + Gx1[35]*Gu1[106];
Gu2[11] = + Gx1[18]*Gu1[5] + Gx1[19]*Gu1[11] + Gx1[20]*Gu1[17] + Gx1[21]*Gu1[23] + Gx1[22]*Gu1[29] + Gx1[23]*Gu1[35] + Gx1[24]*Gu1[41] + Gx1[25]*Gu1[47] + Gx1[26]*Gu1[53] + Gx1[27]*Gu1[59] + Gx1[28]*Gu1[65] + Gx1[29]*Gu1[71] + Gx1[30]*Gu1[77] + Gx1[31]*Gu1[83] + Gx1[32]*Gu1[89] + Gx1[33]*Gu1[95] + Gx1[34]*Gu1[101] + Gx1[35]*Gu1[107];
Gu2[12] = + Gx1[36]*Gu1[0] + Gx1[37]*Gu1[6] + Gx1[38]*Gu1[12] + Gx1[39]*Gu1[18] + Gx1[40]*Gu1[24] + Gx1[41]*Gu1[30] + Gx1[42]*Gu1[36] + Gx1[43]*Gu1[42] + Gx1[44]*Gu1[48] + Gx1[45]*Gu1[54] + Gx1[46]*Gu1[60] + Gx1[47]*Gu1[66] + Gx1[48]*Gu1[72] + Gx1[49]*Gu1[78] + Gx1[50]*Gu1[84] + Gx1[51]*Gu1[90] + Gx1[52]*Gu1[96] + Gx1[53]*Gu1[102];
Gu2[13] = + Gx1[36]*Gu1[1] + Gx1[37]*Gu1[7] + Gx1[38]*Gu1[13] + Gx1[39]*Gu1[19] + Gx1[40]*Gu1[25] + Gx1[41]*Gu1[31] + Gx1[42]*Gu1[37] + Gx1[43]*Gu1[43] + Gx1[44]*Gu1[49] + Gx1[45]*Gu1[55] + Gx1[46]*Gu1[61] + Gx1[47]*Gu1[67] + Gx1[48]*Gu1[73] + Gx1[49]*Gu1[79] + Gx1[50]*Gu1[85] + Gx1[51]*Gu1[91] + Gx1[52]*Gu1[97] + Gx1[53]*Gu1[103];
Gu2[14] = + Gx1[36]*Gu1[2] + Gx1[37]*Gu1[8] + Gx1[38]*Gu1[14] + Gx1[39]*Gu1[20] + Gx1[40]*Gu1[26] + Gx1[41]*Gu1[32] + Gx1[42]*Gu1[38] + Gx1[43]*Gu1[44] + Gx1[44]*Gu1[50] + Gx1[45]*Gu1[56] + Gx1[46]*Gu1[62] + Gx1[47]*Gu1[68] + Gx1[48]*Gu1[74] + Gx1[49]*Gu1[80] + Gx1[50]*Gu1[86] + Gx1[51]*Gu1[92] + Gx1[52]*Gu1[98] + Gx1[53]*Gu1[104];
Gu2[15] = + Gx1[36]*Gu1[3] + Gx1[37]*Gu1[9] + Gx1[38]*Gu1[15] + Gx1[39]*Gu1[21] + Gx1[40]*Gu1[27] + Gx1[41]*Gu1[33] + Gx1[42]*Gu1[39] + Gx1[43]*Gu1[45] + Gx1[44]*Gu1[51] + Gx1[45]*Gu1[57] + Gx1[46]*Gu1[63] + Gx1[47]*Gu1[69] + Gx1[48]*Gu1[75] + Gx1[49]*Gu1[81] + Gx1[50]*Gu1[87] + Gx1[51]*Gu1[93] + Gx1[52]*Gu1[99] + Gx1[53]*Gu1[105];
Gu2[16] = + Gx1[36]*Gu1[4] + Gx1[37]*Gu1[10] + Gx1[38]*Gu1[16] + Gx1[39]*Gu1[22] + Gx1[40]*Gu1[28] + Gx1[41]*Gu1[34] + Gx1[42]*Gu1[40] + Gx1[43]*Gu1[46] + Gx1[44]*Gu1[52] + Gx1[45]*Gu1[58] + Gx1[46]*Gu1[64] + Gx1[47]*Gu1[70] + Gx1[48]*Gu1[76] + Gx1[49]*Gu1[82] + Gx1[50]*Gu1[88] + Gx1[51]*Gu1[94] + Gx1[52]*Gu1[100] + Gx1[53]*Gu1[106];
Gu2[17] = + Gx1[36]*Gu1[5] + Gx1[37]*Gu1[11] + Gx1[38]*Gu1[17] + Gx1[39]*Gu1[23] + Gx1[40]*Gu1[29] + Gx1[41]*Gu1[35] + Gx1[42]*Gu1[41] + Gx1[43]*Gu1[47] + Gx1[44]*Gu1[53] + Gx1[45]*Gu1[59] + Gx1[46]*Gu1[65] + Gx1[47]*Gu1[71] + Gx1[48]*Gu1[77] + Gx1[49]*Gu1[83] + Gx1[50]*Gu1[89] + Gx1[51]*Gu1[95] + Gx1[52]*Gu1[101] + Gx1[53]*Gu1[107];
Gu2[18] = + Gx1[54]*Gu1[0] + Gx1[55]*Gu1[6] + Gx1[56]*Gu1[12] + Gx1[57]*Gu1[18] + Gx1[58]*Gu1[24] + Gx1[59]*Gu1[30] + Gx1[60]*Gu1[36] + Gx1[61]*Gu1[42] + Gx1[62]*Gu1[48] + Gx1[63]*Gu1[54] + Gx1[64]*Gu1[60] + Gx1[65]*Gu1[66] + Gx1[66]*Gu1[72] + Gx1[67]*Gu1[78] + Gx1[68]*Gu1[84] + Gx1[69]*Gu1[90] + Gx1[70]*Gu1[96] + Gx1[71]*Gu1[102];
Gu2[19] = + Gx1[54]*Gu1[1] + Gx1[55]*Gu1[7] + Gx1[56]*Gu1[13] + Gx1[57]*Gu1[19] + Gx1[58]*Gu1[25] + Gx1[59]*Gu1[31] + Gx1[60]*Gu1[37] + Gx1[61]*Gu1[43] + Gx1[62]*Gu1[49] + Gx1[63]*Gu1[55] + Gx1[64]*Gu1[61] + Gx1[65]*Gu1[67] + Gx1[66]*Gu1[73] + Gx1[67]*Gu1[79] + Gx1[68]*Gu1[85] + Gx1[69]*Gu1[91] + Gx1[70]*Gu1[97] + Gx1[71]*Gu1[103];
Gu2[20] = + Gx1[54]*Gu1[2] + Gx1[55]*Gu1[8] + Gx1[56]*Gu1[14] + Gx1[57]*Gu1[20] + Gx1[58]*Gu1[26] + Gx1[59]*Gu1[32] + Gx1[60]*Gu1[38] + Gx1[61]*Gu1[44] + Gx1[62]*Gu1[50] + Gx1[63]*Gu1[56] + Gx1[64]*Gu1[62] + Gx1[65]*Gu1[68] + Gx1[66]*Gu1[74] + Gx1[67]*Gu1[80] + Gx1[68]*Gu1[86] + Gx1[69]*Gu1[92] + Gx1[70]*Gu1[98] + Gx1[71]*Gu1[104];
Gu2[21] = + Gx1[54]*Gu1[3] + Gx1[55]*Gu1[9] + Gx1[56]*Gu1[15] + Gx1[57]*Gu1[21] + Gx1[58]*Gu1[27] + Gx1[59]*Gu1[33] + Gx1[60]*Gu1[39] + Gx1[61]*Gu1[45] + Gx1[62]*Gu1[51] + Gx1[63]*Gu1[57] + Gx1[64]*Gu1[63] + Gx1[65]*Gu1[69] + Gx1[66]*Gu1[75] + Gx1[67]*Gu1[81] + Gx1[68]*Gu1[87] + Gx1[69]*Gu1[93] + Gx1[70]*Gu1[99] + Gx1[71]*Gu1[105];
Gu2[22] = + Gx1[54]*Gu1[4] + Gx1[55]*Gu1[10] + Gx1[56]*Gu1[16] + Gx1[57]*Gu1[22] + Gx1[58]*Gu1[28] + Gx1[59]*Gu1[34] + Gx1[60]*Gu1[40] + Gx1[61]*Gu1[46] + Gx1[62]*Gu1[52] + Gx1[63]*Gu1[58] + Gx1[64]*Gu1[64] + Gx1[65]*Gu1[70] + Gx1[66]*Gu1[76] + Gx1[67]*Gu1[82] + Gx1[68]*Gu1[88] + Gx1[69]*Gu1[94] + Gx1[70]*Gu1[100] + Gx1[71]*Gu1[106];
Gu2[23] = + Gx1[54]*Gu1[5] + Gx1[55]*Gu1[11] + Gx1[56]*Gu1[17] + Gx1[57]*Gu1[23] + Gx1[58]*Gu1[29] + Gx1[59]*Gu1[35] + Gx1[60]*Gu1[41] + Gx1[61]*Gu1[47] + Gx1[62]*Gu1[53] + Gx1[63]*Gu1[59] + Gx1[64]*Gu1[65] + Gx1[65]*Gu1[71] + Gx1[66]*Gu1[77] + Gx1[67]*Gu1[83] + Gx1[68]*Gu1[89] + Gx1[69]*Gu1[95] + Gx1[70]*Gu1[101] + Gx1[71]*Gu1[107];
Gu2[24] = + Gx1[72]*Gu1[0] + Gx1[73]*Gu1[6] + Gx1[74]*Gu1[12] + Gx1[75]*Gu1[18] + Gx1[76]*Gu1[24] + Gx1[77]*Gu1[30] + Gx1[78]*Gu1[36] + Gx1[79]*Gu1[42] + Gx1[80]*Gu1[48] + Gx1[81]*Gu1[54] + Gx1[82]*Gu1[60] + Gx1[83]*Gu1[66] + Gx1[84]*Gu1[72] + Gx1[85]*Gu1[78] + Gx1[86]*Gu1[84] + Gx1[87]*Gu1[90] + Gx1[88]*Gu1[96] + Gx1[89]*Gu1[102];
Gu2[25] = + Gx1[72]*Gu1[1] + Gx1[73]*Gu1[7] + Gx1[74]*Gu1[13] + Gx1[75]*Gu1[19] + Gx1[76]*Gu1[25] + Gx1[77]*Gu1[31] + Gx1[78]*Gu1[37] + Gx1[79]*Gu1[43] + Gx1[80]*Gu1[49] + Gx1[81]*Gu1[55] + Gx1[82]*Gu1[61] + Gx1[83]*Gu1[67] + Gx1[84]*Gu1[73] + Gx1[85]*Gu1[79] + Gx1[86]*Gu1[85] + Gx1[87]*Gu1[91] + Gx1[88]*Gu1[97] + Gx1[89]*Gu1[103];
Gu2[26] = + Gx1[72]*Gu1[2] + Gx1[73]*Gu1[8] + Gx1[74]*Gu1[14] + Gx1[75]*Gu1[20] + Gx1[76]*Gu1[26] + Gx1[77]*Gu1[32] + Gx1[78]*Gu1[38] + Gx1[79]*Gu1[44] + Gx1[80]*Gu1[50] + Gx1[81]*Gu1[56] + Gx1[82]*Gu1[62] + Gx1[83]*Gu1[68] + Gx1[84]*Gu1[74] + Gx1[85]*Gu1[80] + Gx1[86]*Gu1[86] + Gx1[87]*Gu1[92] + Gx1[88]*Gu1[98] + Gx1[89]*Gu1[104];
Gu2[27] = + Gx1[72]*Gu1[3] + Gx1[73]*Gu1[9] + Gx1[74]*Gu1[15] + Gx1[75]*Gu1[21] + Gx1[76]*Gu1[27] + Gx1[77]*Gu1[33] + Gx1[78]*Gu1[39] + Gx1[79]*Gu1[45] + Gx1[80]*Gu1[51] + Gx1[81]*Gu1[57] + Gx1[82]*Gu1[63] + Gx1[83]*Gu1[69] + Gx1[84]*Gu1[75] + Gx1[85]*Gu1[81] + Gx1[86]*Gu1[87] + Gx1[87]*Gu1[93] + Gx1[88]*Gu1[99] + Gx1[89]*Gu1[105];
Gu2[28] = + Gx1[72]*Gu1[4] + Gx1[73]*Gu1[10] + Gx1[74]*Gu1[16] + Gx1[75]*Gu1[22] + Gx1[76]*Gu1[28] + Gx1[77]*Gu1[34] + Gx1[78]*Gu1[40] + Gx1[79]*Gu1[46] + Gx1[80]*Gu1[52] + Gx1[81]*Gu1[58] + Gx1[82]*Gu1[64] + Gx1[83]*Gu1[70] + Gx1[84]*Gu1[76] + Gx1[85]*Gu1[82] + Gx1[86]*Gu1[88] + Gx1[87]*Gu1[94] + Gx1[88]*Gu1[100] + Gx1[89]*Gu1[106];
Gu2[29] = + Gx1[72]*Gu1[5] + Gx1[73]*Gu1[11] + Gx1[74]*Gu1[17] + Gx1[75]*Gu1[23] + Gx1[76]*Gu1[29] + Gx1[77]*Gu1[35] + Gx1[78]*Gu1[41] + Gx1[79]*Gu1[47] + Gx1[80]*Gu1[53] + Gx1[81]*Gu1[59] + Gx1[82]*Gu1[65] + Gx1[83]*Gu1[71] + Gx1[84]*Gu1[77] + Gx1[85]*Gu1[83] + Gx1[86]*Gu1[89] + Gx1[87]*Gu1[95] + Gx1[88]*Gu1[101] + Gx1[89]*Gu1[107];
Gu2[30] = + Gx1[90]*Gu1[0] + Gx1[91]*Gu1[6] + Gx1[92]*Gu1[12] + Gx1[93]*Gu1[18] + Gx1[94]*Gu1[24] + Gx1[95]*Gu1[30] + Gx1[96]*Gu1[36] + Gx1[97]*Gu1[42] + Gx1[98]*Gu1[48] + Gx1[99]*Gu1[54] + Gx1[100]*Gu1[60] + Gx1[101]*Gu1[66] + Gx1[102]*Gu1[72] + Gx1[103]*Gu1[78] + Gx1[104]*Gu1[84] + Gx1[105]*Gu1[90] + Gx1[106]*Gu1[96] + Gx1[107]*Gu1[102];
Gu2[31] = + Gx1[90]*Gu1[1] + Gx1[91]*Gu1[7] + Gx1[92]*Gu1[13] + Gx1[93]*Gu1[19] + Gx1[94]*Gu1[25] + Gx1[95]*Gu1[31] + Gx1[96]*Gu1[37] + Gx1[97]*Gu1[43] + Gx1[98]*Gu1[49] + Gx1[99]*Gu1[55] + Gx1[100]*Gu1[61] + Gx1[101]*Gu1[67] + Gx1[102]*Gu1[73] + Gx1[103]*Gu1[79] + Gx1[104]*Gu1[85] + Gx1[105]*Gu1[91] + Gx1[106]*Gu1[97] + Gx1[107]*Gu1[103];
Gu2[32] = + Gx1[90]*Gu1[2] + Gx1[91]*Gu1[8] + Gx1[92]*Gu1[14] + Gx1[93]*Gu1[20] + Gx1[94]*Gu1[26] + Gx1[95]*Gu1[32] + Gx1[96]*Gu1[38] + Gx1[97]*Gu1[44] + Gx1[98]*Gu1[50] + Gx1[99]*Gu1[56] + Gx1[100]*Gu1[62] + Gx1[101]*Gu1[68] + Gx1[102]*Gu1[74] + Gx1[103]*Gu1[80] + Gx1[104]*Gu1[86] + Gx1[105]*Gu1[92] + Gx1[106]*Gu1[98] + Gx1[107]*Gu1[104];
Gu2[33] = + Gx1[90]*Gu1[3] + Gx1[91]*Gu1[9] + Gx1[92]*Gu1[15] + Gx1[93]*Gu1[21] + Gx1[94]*Gu1[27] + Gx1[95]*Gu1[33] + Gx1[96]*Gu1[39] + Gx1[97]*Gu1[45] + Gx1[98]*Gu1[51] + Gx1[99]*Gu1[57] + Gx1[100]*Gu1[63] + Gx1[101]*Gu1[69] + Gx1[102]*Gu1[75] + Gx1[103]*Gu1[81] + Gx1[104]*Gu1[87] + Gx1[105]*Gu1[93] + Gx1[106]*Gu1[99] + Gx1[107]*Gu1[105];
Gu2[34] = + Gx1[90]*Gu1[4] + Gx1[91]*Gu1[10] + Gx1[92]*Gu1[16] + Gx1[93]*Gu1[22] + Gx1[94]*Gu1[28] + Gx1[95]*Gu1[34] + Gx1[96]*Gu1[40] + Gx1[97]*Gu1[46] + Gx1[98]*Gu1[52] + Gx1[99]*Gu1[58] + Gx1[100]*Gu1[64] + Gx1[101]*Gu1[70] + Gx1[102]*Gu1[76] + Gx1[103]*Gu1[82] + Gx1[104]*Gu1[88] + Gx1[105]*Gu1[94] + Gx1[106]*Gu1[100] + Gx1[107]*Gu1[106];
Gu2[35] = + Gx1[90]*Gu1[5] + Gx1[91]*Gu1[11] + Gx1[92]*Gu1[17] + Gx1[93]*Gu1[23] + Gx1[94]*Gu1[29] + Gx1[95]*Gu1[35] + Gx1[96]*Gu1[41] + Gx1[97]*Gu1[47] + Gx1[98]*Gu1[53] + Gx1[99]*Gu1[59] + Gx1[100]*Gu1[65] + Gx1[101]*Gu1[71] + Gx1[102]*Gu1[77] + Gx1[103]*Gu1[83] + Gx1[104]*Gu1[89] + Gx1[105]*Gu1[95] + Gx1[106]*Gu1[101] + Gx1[107]*Gu1[107];
Gu2[36] = + Gx1[108]*Gu1[0] + Gx1[109]*Gu1[6] + Gx1[110]*Gu1[12] + Gx1[111]*Gu1[18] + Gx1[112]*Gu1[24] + Gx1[113]*Gu1[30] + Gx1[114]*Gu1[36] + Gx1[115]*Gu1[42] + Gx1[116]*Gu1[48] + Gx1[117]*Gu1[54] + Gx1[118]*Gu1[60] + Gx1[119]*Gu1[66] + Gx1[120]*Gu1[72] + Gx1[121]*Gu1[78] + Gx1[122]*Gu1[84] + Gx1[123]*Gu1[90] + Gx1[124]*Gu1[96] + Gx1[125]*Gu1[102];
Gu2[37] = + Gx1[108]*Gu1[1] + Gx1[109]*Gu1[7] + Gx1[110]*Gu1[13] + Gx1[111]*Gu1[19] + Gx1[112]*Gu1[25] + Gx1[113]*Gu1[31] + Gx1[114]*Gu1[37] + Gx1[115]*Gu1[43] + Gx1[116]*Gu1[49] + Gx1[117]*Gu1[55] + Gx1[118]*Gu1[61] + Gx1[119]*Gu1[67] + Gx1[120]*Gu1[73] + Gx1[121]*Gu1[79] + Gx1[122]*Gu1[85] + Gx1[123]*Gu1[91] + Gx1[124]*Gu1[97] + Gx1[125]*Gu1[103];
Gu2[38] = + Gx1[108]*Gu1[2] + Gx1[109]*Gu1[8] + Gx1[110]*Gu1[14] + Gx1[111]*Gu1[20] + Gx1[112]*Gu1[26] + Gx1[113]*Gu1[32] + Gx1[114]*Gu1[38] + Gx1[115]*Gu1[44] + Gx1[116]*Gu1[50] + Gx1[117]*Gu1[56] + Gx1[118]*Gu1[62] + Gx1[119]*Gu1[68] + Gx1[120]*Gu1[74] + Gx1[121]*Gu1[80] + Gx1[122]*Gu1[86] + Gx1[123]*Gu1[92] + Gx1[124]*Gu1[98] + Gx1[125]*Gu1[104];
Gu2[39] = + Gx1[108]*Gu1[3] + Gx1[109]*Gu1[9] + Gx1[110]*Gu1[15] + Gx1[111]*Gu1[21] + Gx1[112]*Gu1[27] + Gx1[113]*Gu1[33] + Gx1[114]*Gu1[39] + Gx1[115]*Gu1[45] + Gx1[116]*Gu1[51] + Gx1[117]*Gu1[57] + Gx1[118]*Gu1[63] + Gx1[119]*Gu1[69] + Gx1[120]*Gu1[75] + Gx1[121]*Gu1[81] + Gx1[122]*Gu1[87] + Gx1[123]*Gu1[93] + Gx1[124]*Gu1[99] + Gx1[125]*Gu1[105];
Gu2[40] = + Gx1[108]*Gu1[4] + Gx1[109]*Gu1[10] + Gx1[110]*Gu1[16] + Gx1[111]*Gu1[22] + Gx1[112]*Gu1[28] + Gx1[113]*Gu1[34] + Gx1[114]*Gu1[40] + Gx1[115]*Gu1[46] + Gx1[116]*Gu1[52] + Gx1[117]*Gu1[58] + Gx1[118]*Gu1[64] + Gx1[119]*Gu1[70] + Gx1[120]*Gu1[76] + Gx1[121]*Gu1[82] + Gx1[122]*Gu1[88] + Gx1[123]*Gu1[94] + Gx1[124]*Gu1[100] + Gx1[125]*Gu1[106];
Gu2[41] = + Gx1[108]*Gu1[5] + Gx1[109]*Gu1[11] + Gx1[110]*Gu1[17] + Gx1[111]*Gu1[23] + Gx1[112]*Gu1[29] + Gx1[113]*Gu1[35] + Gx1[114]*Gu1[41] + Gx1[115]*Gu1[47] + Gx1[116]*Gu1[53] + Gx1[117]*Gu1[59] + Gx1[118]*Gu1[65] + Gx1[119]*Gu1[71] + Gx1[120]*Gu1[77] + Gx1[121]*Gu1[83] + Gx1[122]*Gu1[89] + Gx1[123]*Gu1[95] + Gx1[124]*Gu1[101] + Gx1[125]*Gu1[107];
Gu2[42] = + Gx1[126]*Gu1[0] + Gx1[127]*Gu1[6] + Gx1[128]*Gu1[12] + Gx1[129]*Gu1[18] + Gx1[130]*Gu1[24] + Gx1[131]*Gu1[30] + Gx1[132]*Gu1[36] + Gx1[133]*Gu1[42] + Gx1[134]*Gu1[48] + Gx1[135]*Gu1[54] + Gx1[136]*Gu1[60] + Gx1[137]*Gu1[66] + Gx1[138]*Gu1[72] + Gx1[139]*Gu1[78] + Gx1[140]*Gu1[84] + Gx1[141]*Gu1[90] + Gx1[142]*Gu1[96] + Gx1[143]*Gu1[102];
Gu2[43] = + Gx1[126]*Gu1[1] + Gx1[127]*Gu1[7] + Gx1[128]*Gu1[13] + Gx1[129]*Gu1[19] + Gx1[130]*Gu1[25] + Gx1[131]*Gu1[31] + Gx1[132]*Gu1[37] + Gx1[133]*Gu1[43] + Gx1[134]*Gu1[49] + Gx1[135]*Gu1[55] + Gx1[136]*Gu1[61] + Gx1[137]*Gu1[67] + Gx1[138]*Gu1[73] + Gx1[139]*Gu1[79] + Gx1[140]*Gu1[85] + Gx1[141]*Gu1[91] + Gx1[142]*Gu1[97] + Gx1[143]*Gu1[103];
Gu2[44] = + Gx1[126]*Gu1[2] + Gx1[127]*Gu1[8] + Gx1[128]*Gu1[14] + Gx1[129]*Gu1[20] + Gx1[130]*Gu1[26] + Gx1[131]*Gu1[32] + Gx1[132]*Gu1[38] + Gx1[133]*Gu1[44] + Gx1[134]*Gu1[50] + Gx1[135]*Gu1[56] + Gx1[136]*Gu1[62] + Gx1[137]*Gu1[68] + Gx1[138]*Gu1[74] + Gx1[139]*Gu1[80] + Gx1[140]*Gu1[86] + Gx1[141]*Gu1[92] + Gx1[142]*Gu1[98] + Gx1[143]*Gu1[104];
Gu2[45] = + Gx1[126]*Gu1[3] + Gx1[127]*Gu1[9] + Gx1[128]*Gu1[15] + Gx1[129]*Gu1[21] + Gx1[130]*Gu1[27] + Gx1[131]*Gu1[33] + Gx1[132]*Gu1[39] + Gx1[133]*Gu1[45] + Gx1[134]*Gu1[51] + Gx1[135]*Gu1[57] + Gx1[136]*Gu1[63] + Gx1[137]*Gu1[69] + Gx1[138]*Gu1[75] + Gx1[139]*Gu1[81] + Gx1[140]*Gu1[87] + Gx1[141]*Gu1[93] + Gx1[142]*Gu1[99] + Gx1[143]*Gu1[105];
Gu2[46] = + Gx1[126]*Gu1[4] + Gx1[127]*Gu1[10] + Gx1[128]*Gu1[16] + Gx1[129]*Gu1[22] + Gx1[130]*Gu1[28] + Gx1[131]*Gu1[34] + Gx1[132]*Gu1[40] + Gx1[133]*Gu1[46] + Gx1[134]*Gu1[52] + Gx1[135]*Gu1[58] + Gx1[136]*Gu1[64] + Gx1[137]*Gu1[70] + Gx1[138]*Gu1[76] + Gx1[139]*Gu1[82] + Gx1[140]*Gu1[88] + Gx1[141]*Gu1[94] + Gx1[142]*Gu1[100] + Gx1[143]*Gu1[106];
Gu2[47] = + Gx1[126]*Gu1[5] + Gx1[127]*Gu1[11] + Gx1[128]*Gu1[17] + Gx1[129]*Gu1[23] + Gx1[130]*Gu1[29] + Gx1[131]*Gu1[35] + Gx1[132]*Gu1[41] + Gx1[133]*Gu1[47] + Gx1[134]*Gu1[53] + Gx1[135]*Gu1[59] + Gx1[136]*Gu1[65] + Gx1[137]*Gu1[71] + Gx1[138]*Gu1[77] + Gx1[139]*Gu1[83] + Gx1[140]*Gu1[89] + Gx1[141]*Gu1[95] + Gx1[142]*Gu1[101] + Gx1[143]*Gu1[107];
Gu2[48] = + Gx1[144]*Gu1[0] + Gx1[145]*Gu1[6] + Gx1[146]*Gu1[12] + Gx1[147]*Gu1[18] + Gx1[148]*Gu1[24] + Gx1[149]*Gu1[30] + Gx1[150]*Gu1[36] + Gx1[151]*Gu1[42] + Gx1[152]*Gu1[48] + Gx1[153]*Gu1[54] + Gx1[154]*Gu1[60] + Gx1[155]*Gu1[66] + Gx1[156]*Gu1[72] + Gx1[157]*Gu1[78] + Gx1[158]*Gu1[84] + Gx1[159]*Gu1[90] + Gx1[160]*Gu1[96] + Gx1[161]*Gu1[102];
Gu2[49] = + Gx1[144]*Gu1[1] + Gx1[145]*Gu1[7] + Gx1[146]*Gu1[13] + Gx1[147]*Gu1[19] + Gx1[148]*Gu1[25] + Gx1[149]*Gu1[31] + Gx1[150]*Gu1[37] + Gx1[151]*Gu1[43] + Gx1[152]*Gu1[49] + Gx1[153]*Gu1[55] + Gx1[154]*Gu1[61] + Gx1[155]*Gu1[67] + Gx1[156]*Gu1[73] + Gx1[157]*Gu1[79] + Gx1[158]*Gu1[85] + Gx1[159]*Gu1[91] + Gx1[160]*Gu1[97] + Gx1[161]*Gu1[103];
Gu2[50] = + Gx1[144]*Gu1[2] + Gx1[145]*Gu1[8] + Gx1[146]*Gu1[14] + Gx1[147]*Gu1[20] + Gx1[148]*Gu1[26] + Gx1[149]*Gu1[32] + Gx1[150]*Gu1[38] + Gx1[151]*Gu1[44] + Gx1[152]*Gu1[50] + Gx1[153]*Gu1[56] + Gx1[154]*Gu1[62] + Gx1[155]*Gu1[68] + Gx1[156]*Gu1[74] + Gx1[157]*Gu1[80] + Gx1[158]*Gu1[86] + Gx1[159]*Gu1[92] + Gx1[160]*Gu1[98] + Gx1[161]*Gu1[104];
Gu2[51] = + Gx1[144]*Gu1[3] + Gx1[145]*Gu1[9] + Gx1[146]*Gu1[15] + Gx1[147]*Gu1[21] + Gx1[148]*Gu1[27] + Gx1[149]*Gu1[33] + Gx1[150]*Gu1[39] + Gx1[151]*Gu1[45] + Gx1[152]*Gu1[51] + Gx1[153]*Gu1[57] + Gx1[154]*Gu1[63] + Gx1[155]*Gu1[69] + Gx1[156]*Gu1[75] + Gx1[157]*Gu1[81] + Gx1[158]*Gu1[87] + Gx1[159]*Gu1[93] + Gx1[160]*Gu1[99] + Gx1[161]*Gu1[105];
Gu2[52] = + Gx1[144]*Gu1[4] + Gx1[145]*Gu1[10] + Gx1[146]*Gu1[16] + Gx1[147]*Gu1[22] + Gx1[148]*Gu1[28] + Gx1[149]*Gu1[34] + Gx1[150]*Gu1[40] + Gx1[151]*Gu1[46] + Gx1[152]*Gu1[52] + Gx1[153]*Gu1[58] + Gx1[154]*Gu1[64] + Gx1[155]*Gu1[70] + Gx1[156]*Gu1[76] + Gx1[157]*Gu1[82] + Gx1[158]*Gu1[88] + Gx1[159]*Gu1[94] + Gx1[160]*Gu1[100] + Gx1[161]*Gu1[106];
Gu2[53] = + Gx1[144]*Gu1[5] + Gx1[145]*Gu1[11] + Gx1[146]*Gu1[17] + Gx1[147]*Gu1[23] + Gx1[148]*Gu1[29] + Gx1[149]*Gu1[35] + Gx1[150]*Gu1[41] + Gx1[151]*Gu1[47] + Gx1[152]*Gu1[53] + Gx1[153]*Gu1[59] + Gx1[154]*Gu1[65] + Gx1[155]*Gu1[71] + Gx1[156]*Gu1[77] + Gx1[157]*Gu1[83] + Gx1[158]*Gu1[89] + Gx1[159]*Gu1[95] + Gx1[160]*Gu1[101] + Gx1[161]*Gu1[107];
Gu2[54] = + Gx1[162]*Gu1[0] + Gx1[163]*Gu1[6] + Gx1[164]*Gu1[12] + Gx1[165]*Gu1[18] + Gx1[166]*Gu1[24] + Gx1[167]*Gu1[30] + Gx1[168]*Gu1[36] + Gx1[169]*Gu1[42] + Gx1[170]*Gu1[48] + Gx1[171]*Gu1[54] + Gx1[172]*Gu1[60] + Gx1[173]*Gu1[66] + Gx1[174]*Gu1[72] + Gx1[175]*Gu1[78] + Gx1[176]*Gu1[84] + Gx1[177]*Gu1[90] + Gx1[178]*Gu1[96] + Gx1[179]*Gu1[102];
Gu2[55] = + Gx1[162]*Gu1[1] + Gx1[163]*Gu1[7] + Gx1[164]*Gu1[13] + Gx1[165]*Gu1[19] + Gx1[166]*Gu1[25] + Gx1[167]*Gu1[31] + Gx1[168]*Gu1[37] + Gx1[169]*Gu1[43] + Gx1[170]*Gu1[49] + Gx1[171]*Gu1[55] + Gx1[172]*Gu1[61] + Gx1[173]*Gu1[67] + Gx1[174]*Gu1[73] + Gx1[175]*Gu1[79] + Gx1[176]*Gu1[85] + Gx1[177]*Gu1[91] + Gx1[178]*Gu1[97] + Gx1[179]*Gu1[103];
Gu2[56] = + Gx1[162]*Gu1[2] + Gx1[163]*Gu1[8] + Gx1[164]*Gu1[14] + Gx1[165]*Gu1[20] + Gx1[166]*Gu1[26] + Gx1[167]*Gu1[32] + Gx1[168]*Gu1[38] + Gx1[169]*Gu1[44] + Gx1[170]*Gu1[50] + Gx1[171]*Gu1[56] + Gx1[172]*Gu1[62] + Gx1[173]*Gu1[68] + Gx1[174]*Gu1[74] + Gx1[175]*Gu1[80] + Gx1[176]*Gu1[86] + Gx1[177]*Gu1[92] + Gx1[178]*Gu1[98] + Gx1[179]*Gu1[104];
Gu2[57] = + Gx1[162]*Gu1[3] + Gx1[163]*Gu1[9] + Gx1[164]*Gu1[15] + Gx1[165]*Gu1[21] + Gx1[166]*Gu1[27] + Gx1[167]*Gu1[33] + Gx1[168]*Gu1[39] + Gx1[169]*Gu1[45] + Gx1[170]*Gu1[51] + Gx1[171]*Gu1[57] + Gx1[172]*Gu1[63] + Gx1[173]*Gu1[69] + Gx1[174]*Gu1[75] + Gx1[175]*Gu1[81] + Gx1[176]*Gu1[87] + Gx1[177]*Gu1[93] + Gx1[178]*Gu1[99] + Gx1[179]*Gu1[105];
Gu2[58] = + Gx1[162]*Gu1[4] + Gx1[163]*Gu1[10] + Gx1[164]*Gu1[16] + Gx1[165]*Gu1[22] + Gx1[166]*Gu1[28] + Gx1[167]*Gu1[34] + Gx1[168]*Gu1[40] + Gx1[169]*Gu1[46] + Gx1[170]*Gu1[52] + Gx1[171]*Gu1[58] + Gx1[172]*Gu1[64] + Gx1[173]*Gu1[70] + Gx1[174]*Gu1[76] + Gx1[175]*Gu1[82] + Gx1[176]*Gu1[88] + Gx1[177]*Gu1[94] + Gx1[178]*Gu1[100] + Gx1[179]*Gu1[106];
Gu2[59] = + Gx1[162]*Gu1[5] + Gx1[163]*Gu1[11] + Gx1[164]*Gu1[17] + Gx1[165]*Gu1[23] + Gx1[166]*Gu1[29] + Gx1[167]*Gu1[35] + Gx1[168]*Gu1[41] + Gx1[169]*Gu1[47] + Gx1[170]*Gu1[53] + Gx1[171]*Gu1[59] + Gx1[172]*Gu1[65] + Gx1[173]*Gu1[71] + Gx1[174]*Gu1[77] + Gx1[175]*Gu1[83] + Gx1[176]*Gu1[89] + Gx1[177]*Gu1[95] + Gx1[178]*Gu1[101] + Gx1[179]*Gu1[107];
Gu2[60] = + Gx1[180]*Gu1[0] + Gx1[181]*Gu1[6] + Gx1[182]*Gu1[12] + Gx1[183]*Gu1[18] + Gx1[184]*Gu1[24] + Gx1[185]*Gu1[30] + Gx1[186]*Gu1[36] + Gx1[187]*Gu1[42] + Gx1[188]*Gu1[48] + Gx1[189]*Gu1[54] + Gx1[190]*Gu1[60] + Gx1[191]*Gu1[66] + Gx1[192]*Gu1[72] + Gx1[193]*Gu1[78] + Gx1[194]*Gu1[84] + Gx1[195]*Gu1[90] + Gx1[196]*Gu1[96] + Gx1[197]*Gu1[102];
Gu2[61] = + Gx1[180]*Gu1[1] + Gx1[181]*Gu1[7] + Gx1[182]*Gu1[13] + Gx1[183]*Gu1[19] + Gx1[184]*Gu1[25] + Gx1[185]*Gu1[31] + Gx1[186]*Gu1[37] + Gx1[187]*Gu1[43] + Gx1[188]*Gu1[49] + Gx1[189]*Gu1[55] + Gx1[190]*Gu1[61] + Gx1[191]*Gu1[67] + Gx1[192]*Gu1[73] + Gx1[193]*Gu1[79] + Gx1[194]*Gu1[85] + Gx1[195]*Gu1[91] + Gx1[196]*Gu1[97] + Gx1[197]*Gu1[103];
Gu2[62] = + Gx1[180]*Gu1[2] + Gx1[181]*Gu1[8] + Gx1[182]*Gu1[14] + Gx1[183]*Gu1[20] + Gx1[184]*Gu1[26] + Gx1[185]*Gu1[32] + Gx1[186]*Gu1[38] + Gx1[187]*Gu1[44] + Gx1[188]*Gu1[50] + Gx1[189]*Gu1[56] + Gx1[190]*Gu1[62] + Gx1[191]*Gu1[68] + Gx1[192]*Gu1[74] + Gx1[193]*Gu1[80] + Gx1[194]*Gu1[86] + Gx1[195]*Gu1[92] + Gx1[196]*Gu1[98] + Gx1[197]*Gu1[104];
Gu2[63] = + Gx1[180]*Gu1[3] + Gx1[181]*Gu1[9] + Gx1[182]*Gu1[15] + Gx1[183]*Gu1[21] + Gx1[184]*Gu1[27] + Gx1[185]*Gu1[33] + Gx1[186]*Gu1[39] + Gx1[187]*Gu1[45] + Gx1[188]*Gu1[51] + Gx1[189]*Gu1[57] + Gx1[190]*Gu1[63] + Gx1[191]*Gu1[69] + Gx1[192]*Gu1[75] + Gx1[193]*Gu1[81] + Gx1[194]*Gu1[87] + Gx1[195]*Gu1[93] + Gx1[196]*Gu1[99] + Gx1[197]*Gu1[105];
Gu2[64] = + Gx1[180]*Gu1[4] + Gx1[181]*Gu1[10] + Gx1[182]*Gu1[16] + Gx1[183]*Gu1[22] + Gx1[184]*Gu1[28] + Gx1[185]*Gu1[34] + Gx1[186]*Gu1[40] + Gx1[187]*Gu1[46] + Gx1[188]*Gu1[52] + Gx1[189]*Gu1[58] + Gx1[190]*Gu1[64] + Gx1[191]*Gu1[70] + Gx1[192]*Gu1[76] + Gx1[193]*Gu1[82] + Gx1[194]*Gu1[88] + Gx1[195]*Gu1[94] + Gx1[196]*Gu1[100] + Gx1[197]*Gu1[106];
Gu2[65] = + Gx1[180]*Gu1[5] + Gx1[181]*Gu1[11] + Gx1[182]*Gu1[17] + Gx1[183]*Gu1[23] + Gx1[184]*Gu1[29] + Gx1[185]*Gu1[35] + Gx1[186]*Gu1[41] + Gx1[187]*Gu1[47] + Gx1[188]*Gu1[53] + Gx1[189]*Gu1[59] + Gx1[190]*Gu1[65] + Gx1[191]*Gu1[71] + Gx1[192]*Gu1[77] + Gx1[193]*Gu1[83] + Gx1[194]*Gu1[89] + Gx1[195]*Gu1[95] + Gx1[196]*Gu1[101] + Gx1[197]*Gu1[107];
Gu2[66] = + Gx1[198]*Gu1[0] + Gx1[199]*Gu1[6] + Gx1[200]*Gu1[12] + Gx1[201]*Gu1[18] + Gx1[202]*Gu1[24] + Gx1[203]*Gu1[30] + Gx1[204]*Gu1[36] + Gx1[205]*Gu1[42] + Gx1[206]*Gu1[48] + Gx1[207]*Gu1[54] + Gx1[208]*Gu1[60] + Gx1[209]*Gu1[66] + Gx1[210]*Gu1[72] + Gx1[211]*Gu1[78] + Gx1[212]*Gu1[84] + Gx1[213]*Gu1[90] + Gx1[214]*Gu1[96] + Gx1[215]*Gu1[102];
Gu2[67] = + Gx1[198]*Gu1[1] + Gx1[199]*Gu1[7] + Gx1[200]*Gu1[13] + Gx1[201]*Gu1[19] + Gx1[202]*Gu1[25] + Gx1[203]*Gu1[31] + Gx1[204]*Gu1[37] + Gx1[205]*Gu1[43] + Gx1[206]*Gu1[49] + Gx1[207]*Gu1[55] + Gx1[208]*Gu1[61] + Gx1[209]*Gu1[67] + Gx1[210]*Gu1[73] + Gx1[211]*Gu1[79] + Gx1[212]*Gu1[85] + Gx1[213]*Gu1[91] + Gx1[214]*Gu1[97] + Gx1[215]*Gu1[103];
Gu2[68] = + Gx1[198]*Gu1[2] + Gx1[199]*Gu1[8] + Gx1[200]*Gu1[14] + Gx1[201]*Gu1[20] + Gx1[202]*Gu1[26] + Gx1[203]*Gu1[32] + Gx1[204]*Gu1[38] + Gx1[205]*Gu1[44] + Gx1[206]*Gu1[50] + Gx1[207]*Gu1[56] + Gx1[208]*Gu1[62] + Gx1[209]*Gu1[68] + Gx1[210]*Gu1[74] + Gx1[211]*Gu1[80] + Gx1[212]*Gu1[86] + Gx1[213]*Gu1[92] + Gx1[214]*Gu1[98] + Gx1[215]*Gu1[104];
Gu2[69] = + Gx1[198]*Gu1[3] + Gx1[199]*Gu1[9] + Gx1[200]*Gu1[15] + Gx1[201]*Gu1[21] + Gx1[202]*Gu1[27] + Gx1[203]*Gu1[33] + Gx1[204]*Gu1[39] + Gx1[205]*Gu1[45] + Gx1[206]*Gu1[51] + Gx1[207]*Gu1[57] + Gx1[208]*Gu1[63] + Gx1[209]*Gu1[69] + Gx1[210]*Gu1[75] + Gx1[211]*Gu1[81] + Gx1[212]*Gu1[87] + Gx1[213]*Gu1[93] + Gx1[214]*Gu1[99] + Gx1[215]*Gu1[105];
Gu2[70] = + Gx1[198]*Gu1[4] + Gx1[199]*Gu1[10] + Gx1[200]*Gu1[16] + Gx1[201]*Gu1[22] + Gx1[202]*Gu1[28] + Gx1[203]*Gu1[34] + Gx1[204]*Gu1[40] + Gx1[205]*Gu1[46] + Gx1[206]*Gu1[52] + Gx1[207]*Gu1[58] + Gx1[208]*Gu1[64] + Gx1[209]*Gu1[70] + Gx1[210]*Gu1[76] + Gx1[211]*Gu1[82] + Gx1[212]*Gu1[88] + Gx1[213]*Gu1[94] + Gx1[214]*Gu1[100] + Gx1[215]*Gu1[106];
Gu2[71] = + Gx1[198]*Gu1[5] + Gx1[199]*Gu1[11] + Gx1[200]*Gu1[17] + Gx1[201]*Gu1[23] + Gx1[202]*Gu1[29] + Gx1[203]*Gu1[35] + Gx1[204]*Gu1[41] + Gx1[205]*Gu1[47] + Gx1[206]*Gu1[53] + Gx1[207]*Gu1[59] + Gx1[208]*Gu1[65] + Gx1[209]*Gu1[71] + Gx1[210]*Gu1[77] + Gx1[211]*Gu1[83] + Gx1[212]*Gu1[89] + Gx1[213]*Gu1[95] + Gx1[214]*Gu1[101] + Gx1[215]*Gu1[107];
Gu2[72] = + Gx1[216]*Gu1[0] + Gx1[217]*Gu1[6] + Gx1[218]*Gu1[12] + Gx1[219]*Gu1[18] + Gx1[220]*Gu1[24] + Gx1[221]*Gu1[30] + Gx1[222]*Gu1[36] + Gx1[223]*Gu1[42] + Gx1[224]*Gu1[48] + Gx1[225]*Gu1[54] + Gx1[226]*Gu1[60] + Gx1[227]*Gu1[66] + Gx1[228]*Gu1[72] + Gx1[229]*Gu1[78] + Gx1[230]*Gu1[84] + Gx1[231]*Gu1[90] + Gx1[232]*Gu1[96] + Gx1[233]*Gu1[102];
Gu2[73] = + Gx1[216]*Gu1[1] + Gx1[217]*Gu1[7] + Gx1[218]*Gu1[13] + Gx1[219]*Gu1[19] + Gx1[220]*Gu1[25] + Gx1[221]*Gu1[31] + Gx1[222]*Gu1[37] + Gx1[223]*Gu1[43] + Gx1[224]*Gu1[49] + Gx1[225]*Gu1[55] + Gx1[226]*Gu1[61] + Gx1[227]*Gu1[67] + Gx1[228]*Gu1[73] + Gx1[229]*Gu1[79] + Gx1[230]*Gu1[85] + Gx1[231]*Gu1[91] + Gx1[232]*Gu1[97] + Gx1[233]*Gu1[103];
Gu2[74] = + Gx1[216]*Gu1[2] + Gx1[217]*Gu1[8] + Gx1[218]*Gu1[14] + Gx1[219]*Gu1[20] + Gx1[220]*Gu1[26] + Gx1[221]*Gu1[32] + Gx1[222]*Gu1[38] + Gx1[223]*Gu1[44] + Gx1[224]*Gu1[50] + Gx1[225]*Gu1[56] + Gx1[226]*Gu1[62] + Gx1[227]*Gu1[68] + Gx1[228]*Gu1[74] + Gx1[229]*Gu1[80] + Gx1[230]*Gu1[86] + Gx1[231]*Gu1[92] + Gx1[232]*Gu1[98] + Gx1[233]*Gu1[104];
Gu2[75] = + Gx1[216]*Gu1[3] + Gx1[217]*Gu1[9] + Gx1[218]*Gu1[15] + Gx1[219]*Gu1[21] + Gx1[220]*Gu1[27] + Gx1[221]*Gu1[33] + Gx1[222]*Gu1[39] + Gx1[223]*Gu1[45] + Gx1[224]*Gu1[51] + Gx1[225]*Gu1[57] + Gx1[226]*Gu1[63] + Gx1[227]*Gu1[69] + Gx1[228]*Gu1[75] + Gx1[229]*Gu1[81] + Gx1[230]*Gu1[87] + Gx1[231]*Gu1[93] + Gx1[232]*Gu1[99] + Gx1[233]*Gu1[105];
Gu2[76] = + Gx1[216]*Gu1[4] + Gx1[217]*Gu1[10] + Gx1[218]*Gu1[16] + Gx1[219]*Gu1[22] + Gx1[220]*Gu1[28] + Gx1[221]*Gu1[34] + Gx1[222]*Gu1[40] + Gx1[223]*Gu1[46] + Gx1[224]*Gu1[52] + Gx1[225]*Gu1[58] + Gx1[226]*Gu1[64] + Gx1[227]*Gu1[70] + Gx1[228]*Gu1[76] + Gx1[229]*Gu1[82] + Gx1[230]*Gu1[88] + Gx1[231]*Gu1[94] + Gx1[232]*Gu1[100] + Gx1[233]*Gu1[106];
Gu2[77] = + Gx1[216]*Gu1[5] + Gx1[217]*Gu1[11] + Gx1[218]*Gu1[17] + Gx1[219]*Gu1[23] + Gx1[220]*Gu1[29] + Gx1[221]*Gu1[35] + Gx1[222]*Gu1[41] + Gx1[223]*Gu1[47] + Gx1[224]*Gu1[53] + Gx1[225]*Gu1[59] + Gx1[226]*Gu1[65] + Gx1[227]*Gu1[71] + Gx1[228]*Gu1[77] + Gx1[229]*Gu1[83] + Gx1[230]*Gu1[89] + Gx1[231]*Gu1[95] + Gx1[232]*Gu1[101] + Gx1[233]*Gu1[107];
Gu2[78] = + Gx1[234]*Gu1[0] + Gx1[235]*Gu1[6] + Gx1[236]*Gu1[12] + Gx1[237]*Gu1[18] + Gx1[238]*Gu1[24] + Gx1[239]*Gu1[30] + Gx1[240]*Gu1[36] + Gx1[241]*Gu1[42] + Gx1[242]*Gu1[48] + Gx1[243]*Gu1[54] + Gx1[244]*Gu1[60] + Gx1[245]*Gu1[66] + Gx1[246]*Gu1[72] + Gx1[247]*Gu1[78] + Gx1[248]*Gu1[84] + Gx1[249]*Gu1[90] + Gx1[250]*Gu1[96] + Gx1[251]*Gu1[102];
Gu2[79] = + Gx1[234]*Gu1[1] + Gx1[235]*Gu1[7] + Gx1[236]*Gu1[13] + Gx1[237]*Gu1[19] + Gx1[238]*Gu1[25] + Gx1[239]*Gu1[31] + Gx1[240]*Gu1[37] + Gx1[241]*Gu1[43] + Gx1[242]*Gu1[49] + Gx1[243]*Gu1[55] + Gx1[244]*Gu1[61] + Gx1[245]*Gu1[67] + Gx1[246]*Gu1[73] + Gx1[247]*Gu1[79] + Gx1[248]*Gu1[85] + Gx1[249]*Gu1[91] + Gx1[250]*Gu1[97] + Gx1[251]*Gu1[103];
Gu2[80] = + Gx1[234]*Gu1[2] + Gx1[235]*Gu1[8] + Gx1[236]*Gu1[14] + Gx1[237]*Gu1[20] + Gx1[238]*Gu1[26] + Gx1[239]*Gu1[32] + Gx1[240]*Gu1[38] + Gx1[241]*Gu1[44] + Gx1[242]*Gu1[50] + Gx1[243]*Gu1[56] + Gx1[244]*Gu1[62] + Gx1[245]*Gu1[68] + Gx1[246]*Gu1[74] + Gx1[247]*Gu1[80] + Gx1[248]*Gu1[86] + Gx1[249]*Gu1[92] + Gx1[250]*Gu1[98] + Gx1[251]*Gu1[104];
Gu2[81] = + Gx1[234]*Gu1[3] + Gx1[235]*Gu1[9] + Gx1[236]*Gu1[15] + Gx1[237]*Gu1[21] + Gx1[238]*Gu1[27] + Gx1[239]*Gu1[33] + Gx1[240]*Gu1[39] + Gx1[241]*Gu1[45] + Gx1[242]*Gu1[51] + Gx1[243]*Gu1[57] + Gx1[244]*Gu1[63] + Gx1[245]*Gu1[69] + Gx1[246]*Gu1[75] + Gx1[247]*Gu1[81] + Gx1[248]*Gu1[87] + Gx1[249]*Gu1[93] + Gx1[250]*Gu1[99] + Gx1[251]*Gu1[105];
Gu2[82] = + Gx1[234]*Gu1[4] + Gx1[235]*Gu1[10] + Gx1[236]*Gu1[16] + Gx1[237]*Gu1[22] + Gx1[238]*Gu1[28] + Gx1[239]*Gu1[34] + Gx1[240]*Gu1[40] + Gx1[241]*Gu1[46] + Gx1[242]*Gu1[52] + Gx1[243]*Gu1[58] + Gx1[244]*Gu1[64] + Gx1[245]*Gu1[70] + Gx1[246]*Gu1[76] + Gx1[247]*Gu1[82] + Gx1[248]*Gu1[88] + Gx1[249]*Gu1[94] + Gx1[250]*Gu1[100] + Gx1[251]*Gu1[106];
Gu2[83] = + Gx1[234]*Gu1[5] + Gx1[235]*Gu1[11] + Gx1[236]*Gu1[17] + Gx1[237]*Gu1[23] + Gx1[238]*Gu1[29] + Gx1[239]*Gu1[35] + Gx1[240]*Gu1[41] + Gx1[241]*Gu1[47] + Gx1[242]*Gu1[53] + Gx1[243]*Gu1[59] + Gx1[244]*Gu1[65] + Gx1[245]*Gu1[71] + Gx1[246]*Gu1[77] + Gx1[247]*Gu1[83] + Gx1[248]*Gu1[89] + Gx1[249]*Gu1[95] + Gx1[250]*Gu1[101] + Gx1[251]*Gu1[107];
Gu2[84] = + Gx1[252]*Gu1[0] + Gx1[253]*Gu1[6] + Gx1[254]*Gu1[12] + Gx1[255]*Gu1[18] + Gx1[256]*Gu1[24] + Gx1[257]*Gu1[30] + Gx1[258]*Gu1[36] + Gx1[259]*Gu1[42] + Gx1[260]*Gu1[48] + Gx1[261]*Gu1[54] + Gx1[262]*Gu1[60] + Gx1[263]*Gu1[66] + Gx1[264]*Gu1[72] + Gx1[265]*Gu1[78] + Gx1[266]*Gu1[84] + Gx1[267]*Gu1[90] + Gx1[268]*Gu1[96] + Gx1[269]*Gu1[102];
Gu2[85] = + Gx1[252]*Gu1[1] + Gx1[253]*Gu1[7] + Gx1[254]*Gu1[13] + Gx1[255]*Gu1[19] + Gx1[256]*Gu1[25] + Gx1[257]*Gu1[31] + Gx1[258]*Gu1[37] + Gx1[259]*Gu1[43] + Gx1[260]*Gu1[49] + Gx1[261]*Gu1[55] + Gx1[262]*Gu1[61] + Gx1[263]*Gu1[67] + Gx1[264]*Gu1[73] + Gx1[265]*Gu1[79] + Gx1[266]*Gu1[85] + Gx1[267]*Gu1[91] + Gx1[268]*Gu1[97] + Gx1[269]*Gu1[103];
Gu2[86] = + Gx1[252]*Gu1[2] + Gx1[253]*Gu1[8] + Gx1[254]*Gu1[14] + Gx1[255]*Gu1[20] + Gx1[256]*Gu1[26] + Gx1[257]*Gu1[32] + Gx1[258]*Gu1[38] + Gx1[259]*Gu1[44] + Gx1[260]*Gu1[50] + Gx1[261]*Gu1[56] + Gx1[262]*Gu1[62] + Gx1[263]*Gu1[68] + Gx1[264]*Gu1[74] + Gx1[265]*Gu1[80] + Gx1[266]*Gu1[86] + Gx1[267]*Gu1[92] + Gx1[268]*Gu1[98] + Gx1[269]*Gu1[104];
Gu2[87] = + Gx1[252]*Gu1[3] + Gx1[253]*Gu1[9] + Gx1[254]*Gu1[15] + Gx1[255]*Gu1[21] + Gx1[256]*Gu1[27] + Gx1[257]*Gu1[33] + Gx1[258]*Gu1[39] + Gx1[259]*Gu1[45] + Gx1[260]*Gu1[51] + Gx1[261]*Gu1[57] + Gx1[262]*Gu1[63] + Gx1[263]*Gu1[69] + Gx1[264]*Gu1[75] + Gx1[265]*Gu1[81] + Gx1[266]*Gu1[87] + Gx1[267]*Gu1[93] + Gx1[268]*Gu1[99] + Gx1[269]*Gu1[105];
Gu2[88] = + Gx1[252]*Gu1[4] + Gx1[253]*Gu1[10] + Gx1[254]*Gu1[16] + Gx1[255]*Gu1[22] + Gx1[256]*Gu1[28] + Gx1[257]*Gu1[34] + Gx1[258]*Gu1[40] + Gx1[259]*Gu1[46] + Gx1[260]*Gu1[52] + Gx1[261]*Gu1[58] + Gx1[262]*Gu1[64] + Gx1[263]*Gu1[70] + Gx1[264]*Gu1[76] + Gx1[265]*Gu1[82] + Gx1[266]*Gu1[88] + Gx1[267]*Gu1[94] + Gx1[268]*Gu1[100] + Gx1[269]*Gu1[106];
Gu2[89] = + Gx1[252]*Gu1[5] + Gx1[253]*Gu1[11] + Gx1[254]*Gu1[17] + Gx1[255]*Gu1[23] + Gx1[256]*Gu1[29] + Gx1[257]*Gu1[35] + Gx1[258]*Gu1[41] + Gx1[259]*Gu1[47] + Gx1[260]*Gu1[53] + Gx1[261]*Gu1[59] + Gx1[262]*Gu1[65] + Gx1[263]*Gu1[71] + Gx1[264]*Gu1[77] + Gx1[265]*Gu1[83] + Gx1[266]*Gu1[89] + Gx1[267]*Gu1[95] + Gx1[268]*Gu1[101] + Gx1[269]*Gu1[107];
Gu2[90] = + Gx1[270]*Gu1[0] + Gx1[271]*Gu1[6] + Gx1[272]*Gu1[12] + Gx1[273]*Gu1[18] + Gx1[274]*Gu1[24] + Gx1[275]*Gu1[30] + Gx1[276]*Gu1[36] + Gx1[277]*Gu1[42] + Gx1[278]*Gu1[48] + Gx1[279]*Gu1[54] + Gx1[280]*Gu1[60] + Gx1[281]*Gu1[66] + Gx1[282]*Gu1[72] + Gx1[283]*Gu1[78] + Gx1[284]*Gu1[84] + Gx1[285]*Gu1[90] + Gx1[286]*Gu1[96] + Gx1[287]*Gu1[102];
Gu2[91] = + Gx1[270]*Gu1[1] + Gx1[271]*Gu1[7] + Gx1[272]*Gu1[13] + Gx1[273]*Gu1[19] + Gx1[274]*Gu1[25] + Gx1[275]*Gu1[31] + Gx1[276]*Gu1[37] + Gx1[277]*Gu1[43] + Gx1[278]*Gu1[49] + Gx1[279]*Gu1[55] + Gx1[280]*Gu1[61] + Gx1[281]*Gu1[67] + Gx1[282]*Gu1[73] + Gx1[283]*Gu1[79] + Gx1[284]*Gu1[85] + Gx1[285]*Gu1[91] + Gx1[286]*Gu1[97] + Gx1[287]*Gu1[103];
Gu2[92] = + Gx1[270]*Gu1[2] + Gx1[271]*Gu1[8] + Gx1[272]*Gu1[14] + Gx1[273]*Gu1[20] + Gx1[274]*Gu1[26] + Gx1[275]*Gu1[32] + Gx1[276]*Gu1[38] + Gx1[277]*Gu1[44] + Gx1[278]*Gu1[50] + Gx1[279]*Gu1[56] + Gx1[280]*Gu1[62] + Gx1[281]*Gu1[68] + Gx1[282]*Gu1[74] + Gx1[283]*Gu1[80] + Gx1[284]*Gu1[86] + Gx1[285]*Gu1[92] + Gx1[286]*Gu1[98] + Gx1[287]*Gu1[104];
Gu2[93] = + Gx1[270]*Gu1[3] + Gx1[271]*Gu1[9] + Gx1[272]*Gu1[15] + Gx1[273]*Gu1[21] + Gx1[274]*Gu1[27] + Gx1[275]*Gu1[33] + Gx1[276]*Gu1[39] + Gx1[277]*Gu1[45] + Gx1[278]*Gu1[51] + Gx1[279]*Gu1[57] + Gx1[280]*Gu1[63] + Gx1[281]*Gu1[69] + Gx1[282]*Gu1[75] + Gx1[283]*Gu1[81] + Gx1[284]*Gu1[87] + Gx1[285]*Gu1[93] + Gx1[286]*Gu1[99] + Gx1[287]*Gu1[105];
Gu2[94] = + Gx1[270]*Gu1[4] + Gx1[271]*Gu1[10] + Gx1[272]*Gu1[16] + Gx1[273]*Gu1[22] + Gx1[274]*Gu1[28] + Gx1[275]*Gu1[34] + Gx1[276]*Gu1[40] + Gx1[277]*Gu1[46] + Gx1[278]*Gu1[52] + Gx1[279]*Gu1[58] + Gx1[280]*Gu1[64] + Gx1[281]*Gu1[70] + Gx1[282]*Gu1[76] + Gx1[283]*Gu1[82] + Gx1[284]*Gu1[88] + Gx1[285]*Gu1[94] + Gx1[286]*Gu1[100] + Gx1[287]*Gu1[106];
Gu2[95] = + Gx1[270]*Gu1[5] + Gx1[271]*Gu1[11] + Gx1[272]*Gu1[17] + Gx1[273]*Gu1[23] + Gx1[274]*Gu1[29] + Gx1[275]*Gu1[35] + Gx1[276]*Gu1[41] + Gx1[277]*Gu1[47] + Gx1[278]*Gu1[53] + Gx1[279]*Gu1[59] + Gx1[280]*Gu1[65] + Gx1[281]*Gu1[71] + Gx1[282]*Gu1[77] + Gx1[283]*Gu1[83] + Gx1[284]*Gu1[89] + Gx1[285]*Gu1[95] + Gx1[286]*Gu1[101] + Gx1[287]*Gu1[107];
Gu2[96] = + Gx1[288]*Gu1[0] + Gx1[289]*Gu1[6] + Gx1[290]*Gu1[12] + Gx1[291]*Gu1[18] + Gx1[292]*Gu1[24] + Gx1[293]*Gu1[30] + Gx1[294]*Gu1[36] + Gx1[295]*Gu1[42] + Gx1[296]*Gu1[48] + Gx1[297]*Gu1[54] + Gx1[298]*Gu1[60] + Gx1[299]*Gu1[66] + Gx1[300]*Gu1[72] + Gx1[301]*Gu1[78] + Gx1[302]*Gu1[84] + Gx1[303]*Gu1[90] + Gx1[304]*Gu1[96] + Gx1[305]*Gu1[102];
Gu2[97] = + Gx1[288]*Gu1[1] + Gx1[289]*Gu1[7] + Gx1[290]*Gu1[13] + Gx1[291]*Gu1[19] + Gx1[292]*Gu1[25] + Gx1[293]*Gu1[31] + Gx1[294]*Gu1[37] + Gx1[295]*Gu1[43] + Gx1[296]*Gu1[49] + Gx1[297]*Gu1[55] + Gx1[298]*Gu1[61] + Gx1[299]*Gu1[67] + Gx1[300]*Gu1[73] + Gx1[301]*Gu1[79] + Gx1[302]*Gu1[85] + Gx1[303]*Gu1[91] + Gx1[304]*Gu1[97] + Gx1[305]*Gu1[103];
Gu2[98] = + Gx1[288]*Gu1[2] + Gx1[289]*Gu1[8] + Gx1[290]*Gu1[14] + Gx1[291]*Gu1[20] + Gx1[292]*Gu1[26] + Gx1[293]*Gu1[32] + Gx1[294]*Gu1[38] + Gx1[295]*Gu1[44] + Gx1[296]*Gu1[50] + Gx1[297]*Gu1[56] + Gx1[298]*Gu1[62] + Gx1[299]*Gu1[68] + Gx1[300]*Gu1[74] + Gx1[301]*Gu1[80] + Gx1[302]*Gu1[86] + Gx1[303]*Gu1[92] + Gx1[304]*Gu1[98] + Gx1[305]*Gu1[104];
Gu2[99] = + Gx1[288]*Gu1[3] + Gx1[289]*Gu1[9] + Gx1[290]*Gu1[15] + Gx1[291]*Gu1[21] + Gx1[292]*Gu1[27] + Gx1[293]*Gu1[33] + Gx1[294]*Gu1[39] + Gx1[295]*Gu1[45] + Gx1[296]*Gu1[51] + Gx1[297]*Gu1[57] + Gx1[298]*Gu1[63] + Gx1[299]*Gu1[69] + Gx1[300]*Gu1[75] + Gx1[301]*Gu1[81] + Gx1[302]*Gu1[87] + Gx1[303]*Gu1[93] + Gx1[304]*Gu1[99] + Gx1[305]*Gu1[105];
Gu2[100] = + Gx1[288]*Gu1[4] + Gx1[289]*Gu1[10] + Gx1[290]*Gu1[16] + Gx1[291]*Gu1[22] + Gx1[292]*Gu1[28] + Gx1[293]*Gu1[34] + Gx1[294]*Gu1[40] + Gx1[295]*Gu1[46] + Gx1[296]*Gu1[52] + Gx1[297]*Gu1[58] + Gx1[298]*Gu1[64] + Gx1[299]*Gu1[70] + Gx1[300]*Gu1[76] + Gx1[301]*Gu1[82] + Gx1[302]*Gu1[88] + Gx1[303]*Gu1[94] + Gx1[304]*Gu1[100] + Gx1[305]*Gu1[106];
Gu2[101] = + Gx1[288]*Gu1[5] + Gx1[289]*Gu1[11] + Gx1[290]*Gu1[17] + Gx1[291]*Gu1[23] + Gx1[292]*Gu1[29] + Gx1[293]*Gu1[35] + Gx1[294]*Gu1[41] + Gx1[295]*Gu1[47] + Gx1[296]*Gu1[53] + Gx1[297]*Gu1[59] + Gx1[298]*Gu1[65] + Gx1[299]*Gu1[71] + Gx1[300]*Gu1[77] + Gx1[301]*Gu1[83] + Gx1[302]*Gu1[89] + Gx1[303]*Gu1[95] + Gx1[304]*Gu1[101] + Gx1[305]*Gu1[107];
Gu2[102] = + Gx1[306]*Gu1[0] + Gx1[307]*Gu1[6] + Gx1[308]*Gu1[12] + Gx1[309]*Gu1[18] + Gx1[310]*Gu1[24] + Gx1[311]*Gu1[30] + Gx1[312]*Gu1[36] + Gx1[313]*Gu1[42] + Gx1[314]*Gu1[48] + Gx1[315]*Gu1[54] + Gx1[316]*Gu1[60] + Gx1[317]*Gu1[66] + Gx1[318]*Gu1[72] + Gx1[319]*Gu1[78] + Gx1[320]*Gu1[84] + Gx1[321]*Gu1[90] + Gx1[322]*Gu1[96] + Gx1[323]*Gu1[102];
Gu2[103] = + Gx1[306]*Gu1[1] + Gx1[307]*Gu1[7] + Gx1[308]*Gu1[13] + Gx1[309]*Gu1[19] + Gx1[310]*Gu1[25] + Gx1[311]*Gu1[31] + Gx1[312]*Gu1[37] + Gx1[313]*Gu1[43] + Gx1[314]*Gu1[49] + Gx1[315]*Gu1[55] + Gx1[316]*Gu1[61] + Gx1[317]*Gu1[67] + Gx1[318]*Gu1[73] + Gx1[319]*Gu1[79] + Gx1[320]*Gu1[85] + Gx1[321]*Gu1[91] + Gx1[322]*Gu1[97] + Gx1[323]*Gu1[103];
Gu2[104] = + Gx1[306]*Gu1[2] + Gx1[307]*Gu1[8] + Gx1[308]*Gu1[14] + Gx1[309]*Gu1[20] + Gx1[310]*Gu1[26] + Gx1[311]*Gu1[32] + Gx1[312]*Gu1[38] + Gx1[313]*Gu1[44] + Gx1[314]*Gu1[50] + Gx1[315]*Gu1[56] + Gx1[316]*Gu1[62] + Gx1[317]*Gu1[68] + Gx1[318]*Gu1[74] + Gx1[319]*Gu1[80] + Gx1[320]*Gu1[86] + Gx1[321]*Gu1[92] + Gx1[322]*Gu1[98] + Gx1[323]*Gu1[104];
Gu2[105] = + Gx1[306]*Gu1[3] + Gx1[307]*Gu1[9] + Gx1[308]*Gu1[15] + Gx1[309]*Gu1[21] + Gx1[310]*Gu1[27] + Gx1[311]*Gu1[33] + Gx1[312]*Gu1[39] + Gx1[313]*Gu1[45] + Gx1[314]*Gu1[51] + Gx1[315]*Gu1[57] + Gx1[316]*Gu1[63] + Gx1[317]*Gu1[69] + Gx1[318]*Gu1[75] + Gx1[319]*Gu1[81] + Gx1[320]*Gu1[87] + Gx1[321]*Gu1[93] + Gx1[322]*Gu1[99] + Gx1[323]*Gu1[105];
Gu2[106] = + Gx1[306]*Gu1[4] + Gx1[307]*Gu1[10] + Gx1[308]*Gu1[16] + Gx1[309]*Gu1[22] + Gx1[310]*Gu1[28] + Gx1[311]*Gu1[34] + Gx1[312]*Gu1[40] + Gx1[313]*Gu1[46] + Gx1[314]*Gu1[52] + Gx1[315]*Gu1[58] + Gx1[316]*Gu1[64] + Gx1[317]*Gu1[70] + Gx1[318]*Gu1[76] + Gx1[319]*Gu1[82] + Gx1[320]*Gu1[88] + Gx1[321]*Gu1[94] + Gx1[322]*Gu1[100] + Gx1[323]*Gu1[106];
Gu2[107] = + Gx1[306]*Gu1[5] + Gx1[307]*Gu1[11] + Gx1[308]*Gu1[17] + Gx1[309]*Gu1[23] + Gx1[310]*Gu1[29] + Gx1[311]*Gu1[35] + Gx1[312]*Gu1[41] + Gx1[313]*Gu1[47] + Gx1[314]*Gu1[53] + Gx1[315]*Gu1[59] + Gx1[316]*Gu1[65] + Gx1[317]*Gu1[71] + Gx1[318]*Gu1[77] + Gx1[319]*Gu1[83] + Gx1[320]*Gu1[89] + Gx1[321]*Gu1[95] + Gx1[322]*Gu1[101] + Gx1[323]*Gu1[107];
}

void acado_moveGuE( real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = Gu1[0];
Gu2[1] = Gu1[1];
Gu2[2] = Gu1[2];
Gu2[3] = Gu1[3];
Gu2[4] = Gu1[4];
Gu2[5] = Gu1[5];
Gu2[6] = Gu1[6];
Gu2[7] = Gu1[7];
Gu2[8] = Gu1[8];
Gu2[9] = Gu1[9];
Gu2[10] = Gu1[10];
Gu2[11] = Gu1[11];
Gu2[12] = Gu1[12];
Gu2[13] = Gu1[13];
Gu2[14] = Gu1[14];
Gu2[15] = Gu1[15];
Gu2[16] = Gu1[16];
Gu2[17] = Gu1[17];
Gu2[18] = Gu1[18];
Gu2[19] = Gu1[19];
Gu2[20] = Gu1[20];
Gu2[21] = Gu1[21];
Gu2[22] = Gu1[22];
Gu2[23] = Gu1[23];
Gu2[24] = Gu1[24];
Gu2[25] = Gu1[25];
Gu2[26] = Gu1[26];
Gu2[27] = Gu1[27];
Gu2[28] = Gu1[28];
Gu2[29] = Gu1[29];
Gu2[30] = Gu1[30];
Gu2[31] = Gu1[31];
Gu2[32] = Gu1[32];
Gu2[33] = Gu1[33];
Gu2[34] = Gu1[34];
Gu2[35] = Gu1[35];
Gu2[36] = Gu1[36];
Gu2[37] = Gu1[37];
Gu2[38] = Gu1[38];
Gu2[39] = Gu1[39];
Gu2[40] = Gu1[40];
Gu2[41] = Gu1[41];
Gu2[42] = Gu1[42];
Gu2[43] = Gu1[43];
Gu2[44] = Gu1[44];
Gu2[45] = Gu1[45];
Gu2[46] = Gu1[46];
Gu2[47] = Gu1[47];
Gu2[48] = Gu1[48];
Gu2[49] = Gu1[49];
Gu2[50] = Gu1[50];
Gu2[51] = Gu1[51];
Gu2[52] = Gu1[52];
Gu2[53] = Gu1[53];
Gu2[54] = Gu1[54];
Gu2[55] = Gu1[55];
Gu2[56] = Gu1[56];
Gu2[57] = Gu1[57];
Gu2[58] = Gu1[58];
Gu2[59] = Gu1[59];
Gu2[60] = Gu1[60];
Gu2[61] = Gu1[61];
Gu2[62] = Gu1[62];
Gu2[63] = Gu1[63];
Gu2[64] = Gu1[64];
Gu2[65] = Gu1[65];
Gu2[66] = Gu1[66];
Gu2[67] = Gu1[67];
Gu2[68] = Gu1[68];
Gu2[69] = Gu1[69];
Gu2[70] = Gu1[70];
Gu2[71] = Gu1[71];
Gu2[72] = Gu1[72];
Gu2[73] = Gu1[73];
Gu2[74] = Gu1[74];
Gu2[75] = Gu1[75];
Gu2[76] = Gu1[76];
Gu2[77] = Gu1[77];
Gu2[78] = Gu1[78];
Gu2[79] = Gu1[79];
Gu2[80] = Gu1[80];
Gu2[81] = Gu1[81];
Gu2[82] = Gu1[82];
Gu2[83] = Gu1[83];
Gu2[84] = Gu1[84];
Gu2[85] = Gu1[85];
Gu2[86] = Gu1[86];
Gu2[87] = Gu1[87];
Gu2[88] = Gu1[88];
Gu2[89] = Gu1[89];
Gu2[90] = Gu1[90];
Gu2[91] = Gu1[91];
Gu2[92] = Gu1[92];
Gu2[93] = Gu1[93];
Gu2[94] = Gu1[94];
Gu2[95] = Gu1[95];
Gu2[96] = Gu1[96];
Gu2[97] = Gu1[97];
Gu2[98] = Gu1[98];
Gu2[99] = Gu1[99];
Gu2[100] = Gu1[100];
Gu2[101] = Gu1[101];
Gu2[102] = Gu1[102];
Gu2[103] = Gu1[103];
Gu2[104] = Gu1[104];
Gu2[105] = Gu1[105];
Gu2[106] = Gu1[106];
Gu2[107] = Gu1[107];
}

void acado_multBTW1( real_t* const Gu1, real_t* const Gu2, int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 360) + (iCol * 6)] = + Gu1[0]*Gu2[0] + Gu1[6]*Gu2[6] + Gu1[12]*Gu2[12] + Gu1[18]*Gu2[18] + Gu1[24]*Gu2[24] + Gu1[30]*Gu2[30] + Gu1[36]*Gu2[36] + Gu1[42]*Gu2[42] + Gu1[48]*Gu2[48] + Gu1[54]*Gu2[54] + Gu1[60]*Gu2[60] + Gu1[66]*Gu2[66] + Gu1[72]*Gu2[72] + Gu1[78]*Gu2[78] + Gu1[84]*Gu2[84] + Gu1[90]*Gu2[90] + Gu1[96]*Gu2[96] + Gu1[102]*Gu2[102];
acadoWorkspace.H[(iRow * 360) + (iCol * 6 + 1)] = + Gu1[0]*Gu2[1] + Gu1[6]*Gu2[7] + Gu1[12]*Gu2[13] + Gu1[18]*Gu2[19] + Gu1[24]*Gu2[25] + Gu1[30]*Gu2[31] + Gu1[36]*Gu2[37] + Gu1[42]*Gu2[43] + Gu1[48]*Gu2[49] + Gu1[54]*Gu2[55] + Gu1[60]*Gu2[61] + Gu1[66]*Gu2[67] + Gu1[72]*Gu2[73] + Gu1[78]*Gu2[79] + Gu1[84]*Gu2[85] + Gu1[90]*Gu2[91] + Gu1[96]*Gu2[97] + Gu1[102]*Gu2[103];
acadoWorkspace.H[(iRow * 360) + (iCol * 6 + 2)] = + Gu1[0]*Gu2[2] + Gu1[6]*Gu2[8] + Gu1[12]*Gu2[14] + Gu1[18]*Gu2[20] + Gu1[24]*Gu2[26] + Gu1[30]*Gu2[32] + Gu1[36]*Gu2[38] + Gu1[42]*Gu2[44] + Gu1[48]*Gu2[50] + Gu1[54]*Gu2[56] + Gu1[60]*Gu2[62] + Gu1[66]*Gu2[68] + Gu1[72]*Gu2[74] + Gu1[78]*Gu2[80] + Gu1[84]*Gu2[86] + Gu1[90]*Gu2[92] + Gu1[96]*Gu2[98] + Gu1[102]*Gu2[104];
acadoWorkspace.H[(iRow * 360) + (iCol * 6 + 3)] = + Gu1[0]*Gu2[3] + Gu1[6]*Gu2[9] + Gu1[12]*Gu2[15] + Gu1[18]*Gu2[21] + Gu1[24]*Gu2[27] + Gu1[30]*Gu2[33] + Gu1[36]*Gu2[39] + Gu1[42]*Gu2[45] + Gu1[48]*Gu2[51] + Gu1[54]*Gu2[57] + Gu1[60]*Gu2[63] + Gu1[66]*Gu2[69] + Gu1[72]*Gu2[75] + Gu1[78]*Gu2[81] + Gu1[84]*Gu2[87] + Gu1[90]*Gu2[93] + Gu1[96]*Gu2[99] + Gu1[102]*Gu2[105];
acadoWorkspace.H[(iRow * 360) + (iCol * 6 + 4)] = + Gu1[0]*Gu2[4] + Gu1[6]*Gu2[10] + Gu1[12]*Gu2[16] + Gu1[18]*Gu2[22] + Gu1[24]*Gu2[28] + Gu1[30]*Gu2[34] + Gu1[36]*Gu2[40] + Gu1[42]*Gu2[46] + Gu1[48]*Gu2[52] + Gu1[54]*Gu2[58] + Gu1[60]*Gu2[64] + Gu1[66]*Gu2[70] + Gu1[72]*Gu2[76] + Gu1[78]*Gu2[82] + Gu1[84]*Gu2[88] + Gu1[90]*Gu2[94] + Gu1[96]*Gu2[100] + Gu1[102]*Gu2[106];
acadoWorkspace.H[(iRow * 360) + (iCol * 6 + 5)] = + Gu1[0]*Gu2[5] + Gu1[6]*Gu2[11] + Gu1[12]*Gu2[17] + Gu1[18]*Gu2[23] + Gu1[24]*Gu2[29] + Gu1[30]*Gu2[35] + Gu1[36]*Gu2[41] + Gu1[42]*Gu2[47] + Gu1[48]*Gu2[53] + Gu1[54]*Gu2[59] + Gu1[60]*Gu2[65] + Gu1[66]*Gu2[71] + Gu1[72]*Gu2[77] + Gu1[78]*Gu2[83] + Gu1[84]*Gu2[89] + Gu1[90]*Gu2[95] + Gu1[96]*Gu2[101] + Gu1[102]*Gu2[107];
acadoWorkspace.H[(iRow * 360 + 60) + (iCol * 6)] = + Gu1[1]*Gu2[0] + Gu1[7]*Gu2[6] + Gu1[13]*Gu2[12] + Gu1[19]*Gu2[18] + Gu1[25]*Gu2[24] + Gu1[31]*Gu2[30] + Gu1[37]*Gu2[36] + Gu1[43]*Gu2[42] + Gu1[49]*Gu2[48] + Gu1[55]*Gu2[54] + Gu1[61]*Gu2[60] + Gu1[67]*Gu2[66] + Gu1[73]*Gu2[72] + Gu1[79]*Gu2[78] + Gu1[85]*Gu2[84] + Gu1[91]*Gu2[90] + Gu1[97]*Gu2[96] + Gu1[103]*Gu2[102];
acadoWorkspace.H[(iRow * 360 + 60) + (iCol * 6 + 1)] = + Gu1[1]*Gu2[1] + Gu1[7]*Gu2[7] + Gu1[13]*Gu2[13] + Gu1[19]*Gu2[19] + Gu1[25]*Gu2[25] + Gu1[31]*Gu2[31] + Gu1[37]*Gu2[37] + Gu1[43]*Gu2[43] + Gu1[49]*Gu2[49] + Gu1[55]*Gu2[55] + Gu1[61]*Gu2[61] + Gu1[67]*Gu2[67] + Gu1[73]*Gu2[73] + Gu1[79]*Gu2[79] + Gu1[85]*Gu2[85] + Gu1[91]*Gu2[91] + Gu1[97]*Gu2[97] + Gu1[103]*Gu2[103];
acadoWorkspace.H[(iRow * 360 + 60) + (iCol * 6 + 2)] = + Gu1[1]*Gu2[2] + Gu1[7]*Gu2[8] + Gu1[13]*Gu2[14] + Gu1[19]*Gu2[20] + Gu1[25]*Gu2[26] + Gu1[31]*Gu2[32] + Gu1[37]*Gu2[38] + Gu1[43]*Gu2[44] + Gu1[49]*Gu2[50] + Gu1[55]*Gu2[56] + Gu1[61]*Gu2[62] + Gu1[67]*Gu2[68] + Gu1[73]*Gu2[74] + Gu1[79]*Gu2[80] + Gu1[85]*Gu2[86] + Gu1[91]*Gu2[92] + Gu1[97]*Gu2[98] + Gu1[103]*Gu2[104];
acadoWorkspace.H[(iRow * 360 + 60) + (iCol * 6 + 3)] = + Gu1[1]*Gu2[3] + Gu1[7]*Gu2[9] + Gu1[13]*Gu2[15] + Gu1[19]*Gu2[21] + Gu1[25]*Gu2[27] + Gu1[31]*Gu2[33] + Gu1[37]*Gu2[39] + Gu1[43]*Gu2[45] + Gu1[49]*Gu2[51] + Gu1[55]*Gu2[57] + Gu1[61]*Gu2[63] + Gu1[67]*Gu2[69] + Gu1[73]*Gu2[75] + Gu1[79]*Gu2[81] + Gu1[85]*Gu2[87] + Gu1[91]*Gu2[93] + Gu1[97]*Gu2[99] + Gu1[103]*Gu2[105];
acadoWorkspace.H[(iRow * 360 + 60) + (iCol * 6 + 4)] = + Gu1[1]*Gu2[4] + Gu1[7]*Gu2[10] + Gu1[13]*Gu2[16] + Gu1[19]*Gu2[22] + Gu1[25]*Gu2[28] + Gu1[31]*Gu2[34] + Gu1[37]*Gu2[40] + Gu1[43]*Gu2[46] + Gu1[49]*Gu2[52] + Gu1[55]*Gu2[58] + Gu1[61]*Gu2[64] + Gu1[67]*Gu2[70] + Gu1[73]*Gu2[76] + Gu1[79]*Gu2[82] + Gu1[85]*Gu2[88] + Gu1[91]*Gu2[94] + Gu1[97]*Gu2[100] + Gu1[103]*Gu2[106];
acadoWorkspace.H[(iRow * 360 + 60) + (iCol * 6 + 5)] = + Gu1[1]*Gu2[5] + Gu1[7]*Gu2[11] + Gu1[13]*Gu2[17] + Gu1[19]*Gu2[23] + Gu1[25]*Gu2[29] + Gu1[31]*Gu2[35] + Gu1[37]*Gu2[41] + Gu1[43]*Gu2[47] + Gu1[49]*Gu2[53] + Gu1[55]*Gu2[59] + Gu1[61]*Gu2[65] + Gu1[67]*Gu2[71] + Gu1[73]*Gu2[77] + Gu1[79]*Gu2[83] + Gu1[85]*Gu2[89] + Gu1[91]*Gu2[95] + Gu1[97]*Gu2[101] + Gu1[103]*Gu2[107];
acadoWorkspace.H[(iRow * 360 + 120) + (iCol * 6)] = + Gu1[2]*Gu2[0] + Gu1[8]*Gu2[6] + Gu1[14]*Gu2[12] + Gu1[20]*Gu2[18] + Gu1[26]*Gu2[24] + Gu1[32]*Gu2[30] + Gu1[38]*Gu2[36] + Gu1[44]*Gu2[42] + Gu1[50]*Gu2[48] + Gu1[56]*Gu2[54] + Gu1[62]*Gu2[60] + Gu1[68]*Gu2[66] + Gu1[74]*Gu2[72] + Gu1[80]*Gu2[78] + Gu1[86]*Gu2[84] + Gu1[92]*Gu2[90] + Gu1[98]*Gu2[96] + Gu1[104]*Gu2[102];
acadoWorkspace.H[(iRow * 360 + 120) + (iCol * 6 + 1)] = + Gu1[2]*Gu2[1] + Gu1[8]*Gu2[7] + Gu1[14]*Gu2[13] + Gu1[20]*Gu2[19] + Gu1[26]*Gu2[25] + Gu1[32]*Gu2[31] + Gu1[38]*Gu2[37] + Gu1[44]*Gu2[43] + Gu1[50]*Gu2[49] + Gu1[56]*Gu2[55] + Gu1[62]*Gu2[61] + Gu1[68]*Gu2[67] + Gu1[74]*Gu2[73] + Gu1[80]*Gu2[79] + Gu1[86]*Gu2[85] + Gu1[92]*Gu2[91] + Gu1[98]*Gu2[97] + Gu1[104]*Gu2[103];
acadoWorkspace.H[(iRow * 360 + 120) + (iCol * 6 + 2)] = + Gu1[2]*Gu2[2] + Gu1[8]*Gu2[8] + Gu1[14]*Gu2[14] + Gu1[20]*Gu2[20] + Gu1[26]*Gu2[26] + Gu1[32]*Gu2[32] + Gu1[38]*Gu2[38] + Gu1[44]*Gu2[44] + Gu1[50]*Gu2[50] + Gu1[56]*Gu2[56] + Gu1[62]*Gu2[62] + Gu1[68]*Gu2[68] + Gu1[74]*Gu2[74] + Gu1[80]*Gu2[80] + Gu1[86]*Gu2[86] + Gu1[92]*Gu2[92] + Gu1[98]*Gu2[98] + Gu1[104]*Gu2[104];
acadoWorkspace.H[(iRow * 360 + 120) + (iCol * 6 + 3)] = + Gu1[2]*Gu2[3] + Gu1[8]*Gu2[9] + Gu1[14]*Gu2[15] + Gu1[20]*Gu2[21] + Gu1[26]*Gu2[27] + Gu1[32]*Gu2[33] + Gu1[38]*Gu2[39] + Gu1[44]*Gu2[45] + Gu1[50]*Gu2[51] + Gu1[56]*Gu2[57] + Gu1[62]*Gu2[63] + Gu1[68]*Gu2[69] + Gu1[74]*Gu2[75] + Gu1[80]*Gu2[81] + Gu1[86]*Gu2[87] + Gu1[92]*Gu2[93] + Gu1[98]*Gu2[99] + Gu1[104]*Gu2[105];
acadoWorkspace.H[(iRow * 360 + 120) + (iCol * 6 + 4)] = + Gu1[2]*Gu2[4] + Gu1[8]*Gu2[10] + Gu1[14]*Gu2[16] + Gu1[20]*Gu2[22] + Gu1[26]*Gu2[28] + Gu1[32]*Gu2[34] + Gu1[38]*Gu2[40] + Gu1[44]*Gu2[46] + Gu1[50]*Gu2[52] + Gu1[56]*Gu2[58] + Gu1[62]*Gu2[64] + Gu1[68]*Gu2[70] + Gu1[74]*Gu2[76] + Gu1[80]*Gu2[82] + Gu1[86]*Gu2[88] + Gu1[92]*Gu2[94] + Gu1[98]*Gu2[100] + Gu1[104]*Gu2[106];
acadoWorkspace.H[(iRow * 360 + 120) + (iCol * 6 + 5)] = + Gu1[2]*Gu2[5] + Gu1[8]*Gu2[11] + Gu1[14]*Gu2[17] + Gu1[20]*Gu2[23] + Gu1[26]*Gu2[29] + Gu1[32]*Gu2[35] + Gu1[38]*Gu2[41] + Gu1[44]*Gu2[47] + Gu1[50]*Gu2[53] + Gu1[56]*Gu2[59] + Gu1[62]*Gu2[65] + Gu1[68]*Gu2[71] + Gu1[74]*Gu2[77] + Gu1[80]*Gu2[83] + Gu1[86]*Gu2[89] + Gu1[92]*Gu2[95] + Gu1[98]*Gu2[101] + Gu1[104]*Gu2[107];
acadoWorkspace.H[(iRow * 360 + 180) + (iCol * 6)] = + Gu1[3]*Gu2[0] + Gu1[9]*Gu2[6] + Gu1[15]*Gu2[12] + Gu1[21]*Gu2[18] + Gu1[27]*Gu2[24] + Gu1[33]*Gu2[30] + Gu1[39]*Gu2[36] + Gu1[45]*Gu2[42] + Gu1[51]*Gu2[48] + Gu1[57]*Gu2[54] + Gu1[63]*Gu2[60] + Gu1[69]*Gu2[66] + Gu1[75]*Gu2[72] + Gu1[81]*Gu2[78] + Gu1[87]*Gu2[84] + Gu1[93]*Gu2[90] + Gu1[99]*Gu2[96] + Gu1[105]*Gu2[102];
acadoWorkspace.H[(iRow * 360 + 180) + (iCol * 6 + 1)] = + Gu1[3]*Gu2[1] + Gu1[9]*Gu2[7] + Gu1[15]*Gu2[13] + Gu1[21]*Gu2[19] + Gu1[27]*Gu2[25] + Gu1[33]*Gu2[31] + Gu1[39]*Gu2[37] + Gu1[45]*Gu2[43] + Gu1[51]*Gu2[49] + Gu1[57]*Gu2[55] + Gu1[63]*Gu2[61] + Gu1[69]*Gu2[67] + Gu1[75]*Gu2[73] + Gu1[81]*Gu2[79] + Gu1[87]*Gu2[85] + Gu1[93]*Gu2[91] + Gu1[99]*Gu2[97] + Gu1[105]*Gu2[103];
acadoWorkspace.H[(iRow * 360 + 180) + (iCol * 6 + 2)] = + Gu1[3]*Gu2[2] + Gu1[9]*Gu2[8] + Gu1[15]*Gu2[14] + Gu1[21]*Gu2[20] + Gu1[27]*Gu2[26] + Gu1[33]*Gu2[32] + Gu1[39]*Gu2[38] + Gu1[45]*Gu2[44] + Gu1[51]*Gu2[50] + Gu1[57]*Gu2[56] + Gu1[63]*Gu2[62] + Gu1[69]*Gu2[68] + Gu1[75]*Gu2[74] + Gu1[81]*Gu2[80] + Gu1[87]*Gu2[86] + Gu1[93]*Gu2[92] + Gu1[99]*Gu2[98] + Gu1[105]*Gu2[104];
acadoWorkspace.H[(iRow * 360 + 180) + (iCol * 6 + 3)] = + Gu1[3]*Gu2[3] + Gu1[9]*Gu2[9] + Gu1[15]*Gu2[15] + Gu1[21]*Gu2[21] + Gu1[27]*Gu2[27] + Gu1[33]*Gu2[33] + Gu1[39]*Gu2[39] + Gu1[45]*Gu2[45] + Gu1[51]*Gu2[51] + Gu1[57]*Gu2[57] + Gu1[63]*Gu2[63] + Gu1[69]*Gu2[69] + Gu1[75]*Gu2[75] + Gu1[81]*Gu2[81] + Gu1[87]*Gu2[87] + Gu1[93]*Gu2[93] + Gu1[99]*Gu2[99] + Gu1[105]*Gu2[105];
acadoWorkspace.H[(iRow * 360 + 180) + (iCol * 6 + 4)] = + Gu1[3]*Gu2[4] + Gu1[9]*Gu2[10] + Gu1[15]*Gu2[16] + Gu1[21]*Gu2[22] + Gu1[27]*Gu2[28] + Gu1[33]*Gu2[34] + Gu1[39]*Gu2[40] + Gu1[45]*Gu2[46] + Gu1[51]*Gu2[52] + Gu1[57]*Gu2[58] + Gu1[63]*Gu2[64] + Gu1[69]*Gu2[70] + Gu1[75]*Gu2[76] + Gu1[81]*Gu2[82] + Gu1[87]*Gu2[88] + Gu1[93]*Gu2[94] + Gu1[99]*Gu2[100] + Gu1[105]*Gu2[106];
acadoWorkspace.H[(iRow * 360 + 180) + (iCol * 6 + 5)] = + Gu1[3]*Gu2[5] + Gu1[9]*Gu2[11] + Gu1[15]*Gu2[17] + Gu1[21]*Gu2[23] + Gu1[27]*Gu2[29] + Gu1[33]*Gu2[35] + Gu1[39]*Gu2[41] + Gu1[45]*Gu2[47] + Gu1[51]*Gu2[53] + Gu1[57]*Gu2[59] + Gu1[63]*Gu2[65] + Gu1[69]*Gu2[71] + Gu1[75]*Gu2[77] + Gu1[81]*Gu2[83] + Gu1[87]*Gu2[89] + Gu1[93]*Gu2[95] + Gu1[99]*Gu2[101] + Gu1[105]*Gu2[107];
acadoWorkspace.H[(iRow * 360 + 240) + (iCol * 6)] = + Gu1[4]*Gu2[0] + Gu1[10]*Gu2[6] + Gu1[16]*Gu2[12] + Gu1[22]*Gu2[18] + Gu1[28]*Gu2[24] + Gu1[34]*Gu2[30] + Gu1[40]*Gu2[36] + Gu1[46]*Gu2[42] + Gu1[52]*Gu2[48] + Gu1[58]*Gu2[54] + Gu1[64]*Gu2[60] + Gu1[70]*Gu2[66] + Gu1[76]*Gu2[72] + Gu1[82]*Gu2[78] + Gu1[88]*Gu2[84] + Gu1[94]*Gu2[90] + Gu1[100]*Gu2[96] + Gu1[106]*Gu2[102];
acadoWorkspace.H[(iRow * 360 + 240) + (iCol * 6 + 1)] = + Gu1[4]*Gu2[1] + Gu1[10]*Gu2[7] + Gu1[16]*Gu2[13] + Gu1[22]*Gu2[19] + Gu1[28]*Gu2[25] + Gu1[34]*Gu2[31] + Gu1[40]*Gu2[37] + Gu1[46]*Gu2[43] + Gu1[52]*Gu2[49] + Gu1[58]*Gu2[55] + Gu1[64]*Gu2[61] + Gu1[70]*Gu2[67] + Gu1[76]*Gu2[73] + Gu1[82]*Gu2[79] + Gu1[88]*Gu2[85] + Gu1[94]*Gu2[91] + Gu1[100]*Gu2[97] + Gu1[106]*Gu2[103];
acadoWorkspace.H[(iRow * 360 + 240) + (iCol * 6 + 2)] = + Gu1[4]*Gu2[2] + Gu1[10]*Gu2[8] + Gu1[16]*Gu2[14] + Gu1[22]*Gu2[20] + Gu1[28]*Gu2[26] + Gu1[34]*Gu2[32] + Gu1[40]*Gu2[38] + Gu1[46]*Gu2[44] + Gu1[52]*Gu2[50] + Gu1[58]*Gu2[56] + Gu1[64]*Gu2[62] + Gu1[70]*Gu2[68] + Gu1[76]*Gu2[74] + Gu1[82]*Gu2[80] + Gu1[88]*Gu2[86] + Gu1[94]*Gu2[92] + Gu1[100]*Gu2[98] + Gu1[106]*Gu2[104];
acadoWorkspace.H[(iRow * 360 + 240) + (iCol * 6 + 3)] = + Gu1[4]*Gu2[3] + Gu1[10]*Gu2[9] + Gu1[16]*Gu2[15] + Gu1[22]*Gu2[21] + Gu1[28]*Gu2[27] + Gu1[34]*Gu2[33] + Gu1[40]*Gu2[39] + Gu1[46]*Gu2[45] + Gu1[52]*Gu2[51] + Gu1[58]*Gu2[57] + Gu1[64]*Gu2[63] + Gu1[70]*Gu2[69] + Gu1[76]*Gu2[75] + Gu1[82]*Gu2[81] + Gu1[88]*Gu2[87] + Gu1[94]*Gu2[93] + Gu1[100]*Gu2[99] + Gu1[106]*Gu2[105];
acadoWorkspace.H[(iRow * 360 + 240) + (iCol * 6 + 4)] = + Gu1[4]*Gu2[4] + Gu1[10]*Gu2[10] + Gu1[16]*Gu2[16] + Gu1[22]*Gu2[22] + Gu1[28]*Gu2[28] + Gu1[34]*Gu2[34] + Gu1[40]*Gu2[40] + Gu1[46]*Gu2[46] + Gu1[52]*Gu2[52] + Gu1[58]*Gu2[58] + Gu1[64]*Gu2[64] + Gu1[70]*Gu2[70] + Gu1[76]*Gu2[76] + Gu1[82]*Gu2[82] + Gu1[88]*Gu2[88] + Gu1[94]*Gu2[94] + Gu1[100]*Gu2[100] + Gu1[106]*Gu2[106];
acadoWorkspace.H[(iRow * 360 + 240) + (iCol * 6 + 5)] = + Gu1[4]*Gu2[5] + Gu1[10]*Gu2[11] + Gu1[16]*Gu2[17] + Gu1[22]*Gu2[23] + Gu1[28]*Gu2[29] + Gu1[34]*Gu2[35] + Gu1[40]*Gu2[41] + Gu1[46]*Gu2[47] + Gu1[52]*Gu2[53] + Gu1[58]*Gu2[59] + Gu1[64]*Gu2[65] + Gu1[70]*Gu2[71] + Gu1[76]*Gu2[77] + Gu1[82]*Gu2[83] + Gu1[88]*Gu2[89] + Gu1[94]*Gu2[95] + Gu1[100]*Gu2[101] + Gu1[106]*Gu2[107];
acadoWorkspace.H[(iRow * 360 + 300) + (iCol * 6)] = + Gu1[5]*Gu2[0] + Gu1[11]*Gu2[6] + Gu1[17]*Gu2[12] + Gu1[23]*Gu2[18] + Gu1[29]*Gu2[24] + Gu1[35]*Gu2[30] + Gu1[41]*Gu2[36] + Gu1[47]*Gu2[42] + Gu1[53]*Gu2[48] + Gu1[59]*Gu2[54] + Gu1[65]*Gu2[60] + Gu1[71]*Gu2[66] + Gu1[77]*Gu2[72] + Gu1[83]*Gu2[78] + Gu1[89]*Gu2[84] + Gu1[95]*Gu2[90] + Gu1[101]*Gu2[96] + Gu1[107]*Gu2[102];
acadoWorkspace.H[(iRow * 360 + 300) + (iCol * 6 + 1)] = + Gu1[5]*Gu2[1] + Gu1[11]*Gu2[7] + Gu1[17]*Gu2[13] + Gu1[23]*Gu2[19] + Gu1[29]*Gu2[25] + Gu1[35]*Gu2[31] + Gu1[41]*Gu2[37] + Gu1[47]*Gu2[43] + Gu1[53]*Gu2[49] + Gu1[59]*Gu2[55] + Gu1[65]*Gu2[61] + Gu1[71]*Gu2[67] + Gu1[77]*Gu2[73] + Gu1[83]*Gu2[79] + Gu1[89]*Gu2[85] + Gu1[95]*Gu2[91] + Gu1[101]*Gu2[97] + Gu1[107]*Gu2[103];
acadoWorkspace.H[(iRow * 360 + 300) + (iCol * 6 + 2)] = + Gu1[5]*Gu2[2] + Gu1[11]*Gu2[8] + Gu1[17]*Gu2[14] + Gu1[23]*Gu2[20] + Gu1[29]*Gu2[26] + Gu1[35]*Gu2[32] + Gu1[41]*Gu2[38] + Gu1[47]*Gu2[44] + Gu1[53]*Gu2[50] + Gu1[59]*Gu2[56] + Gu1[65]*Gu2[62] + Gu1[71]*Gu2[68] + Gu1[77]*Gu2[74] + Gu1[83]*Gu2[80] + Gu1[89]*Gu2[86] + Gu1[95]*Gu2[92] + Gu1[101]*Gu2[98] + Gu1[107]*Gu2[104];
acadoWorkspace.H[(iRow * 360 + 300) + (iCol * 6 + 3)] = + Gu1[5]*Gu2[3] + Gu1[11]*Gu2[9] + Gu1[17]*Gu2[15] + Gu1[23]*Gu2[21] + Gu1[29]*Gu2[27] + Gu1[35]*Gu2[33] + Gu1[41]*Gu2[39] + Gu1[47]*Gu2[45] + Gu1[53]*Gu2[51] + Gu1[59]*Gu2[57] + Gu1[65]*Gu2[63] + Gu1[71]*Gu2[69] + Gu1[77]*Gu2[75] + Gu1[83]*Gu2[81] + Gu1[89]*Gu2[87] + Gu1[95]*Gu2[93] + Gu1[101]*Gu2[99] + Gu1[107]*Gu2[105];
acadoWorkspace.H[(iRow * 360 + 300) + (iCol * 6 + 4)] = + Gu1[5]*Gu2[4] + Gu1[11]*Gu2[10] + Gu1[17]*Gu2[16] + Gu1[23]*Gu2[22] + Gu1[29]*Gu2[28] + Gu1[35]*Gu2[34] + Gu1[41]*Gu2[40] + Gu1[47]*Gu2[46] + Gu1[53]*Gu2[52] + Gu1[59]*Gu2[58] + Gu1[65]*Gu2[64] + Gu1[71]*Gu2[70] + Gu1[77]*Gu2[76] + Gu1[83]*Gu2[82] + Gu1[89]*Gu2[88] + Gu1[95]*Gu2[94] + Gu1[101]*Gu2[100] + Gu1[107]*Gu2[106];
acadoWorkspace.H[(iRow * 360 + 300) + (iCol * 6 + 5)] = + Gu1[5]*Gu2[5] + Gu1[11]*Gu2[11] + Gu1[17]*Gu2[17] + Gu1[23]*Gu2[23] + Gu1[29]*Gu2[29] + Gu1[35]*Gu2[35] + Gu1[41]*Gu2[41] + Gu1[47]*Gu2[47] + Gu1[53]*Gu2[53] + Gu1[59]*Gu2[59] + Gu1[65]*Gu2[65] + Gu1[71]*Gu2[71] + Gu1[77]*Gu2[77] + Gu1[83]*Gu2[83] + Gu1[89]*Gu2[89] + Gu1[95]*Gu2[95] + Gu1[101]*Gu2[101] + Gu1[107]*Gu2[107];
}

void acado_multBTW1_R1( real_t* const R11, real_t* const Gu1, real_t* const Gu2, int iRow )
{
acadoWorkspace.H[iRow * 366] = + Gu1[0]*Gu2[0] + Gu1[6]*Gu2[6] + Gu1[12]*Gu2[12] + Gu1[18]*Gu2[18] + Gu1[24]*Gu2[24] + Gu1[30]*Gu2[30] + Gu1[36]*Gu2[36] + Gu1[42]*Gu2[42] + Gu1[48]*Gu2[48] + Gu1[54]*Gu2[54] + Gu1[60]*Gu2[60] + Gu1[66]*Gu2[66] + Gu1[72]*Gu2[72] + Gu1[78]*Gu2[78] + Gu1[84]*Gu2[84] + Gu1[90]*Gu2[90] + Gu1[96]*Gu2[96] + Gu1[102]*Gu2[102] + R11[0];
acadoWorkspace.H[iRow * 366 + 1] = + Gu1[0]*Gu2[1] + Gu1[6]*Gu2[7] + Gu1[12]*Gu2[13] + Gu1[18]*Gu2[19] + Gu1[24]*Gu2[25] + Gu1[30]*Gu2[31] + Gu1[36]*Gu2[37] + Gu1[42]*Gu2[43] + Gu1[48]*Gu2[49] + Gu1[54]*Gu2[55] + Gu1[60]*Gu2[61] + Gu1[66]*Gu2[67] + Gu1[72]*Gu2[73] + Gu1[78]*Gu2[79] + Gu1[84]*Gu2[85] + Gu1[90]*Gu2[91] + Gu1[96]*Gu2[97] + Gu1[102]*Gu2[103] + R11[1];
acadoWorkspace.H[iRow * 366 + 2] = + Gu1[0]*Gu2[2] + Gu1[6]*Gu2[8] + Gu1[12]*Gu2[14] + Gu1[18]*Gu2[20] + Gu1[24]*Gu2[26] + Gu1[30]*Gu2[32] + Gu1[36]*Gu2[38] + Gu1[42]*Gu2[44] + Gu1[48]*Gu2[50] + Gu1[54]*Gu2[56] + Gu1[60]*Gu2[62] + Gu1[66]*Gu2[68] + Gu1[72]*Gu2[74] + Gu1[78]*Gu2[80] + Gu1[84]*Gu2[86] + Gu1[90]*Gu2[92] + Gu1[96]*Gu2[98] + Gu1[102]*Gu2[104] + R11[2];
acadoWorkspace.H[iRow * 366 + 3] = + Gu1[0]*Gu2[3] + Gu1[6]*Gu2[9] + Gu1[12]*Gu2[15] + Gu1[18]*Gu2[21] + Gu1[24]*Gu2[27] + Gu1[30]*Gu2[33] + Gu1[36]*Gu2[39] + Gu1[42]*Gu2[45] + Gu1[48]*Gu2[51] + Gu1[54]*Gu2[57] + Gu1[60]*Gu2[63] + Gu1[66]*Gu2[69] + Gu1[72]*Gu2[75] + Gu1[78]*Gu2[81] + Gu1[84]*Gu2[87] + Gu1[90]*Gu2[93] + Gu1[96]*Gu2[99] + Gu1[102]*Gu2[105] + R11[3];
acadoWorkspace.H[iRow * 366 + 4] = + Gu1[0]*Gu2[4] + Gu1[6]*Gu2[10] + Gu1[12]*Gu2[16] + Gu1[18]*Gu2[22] + Gu1[24]*Gu2[28] + Gu1[30]*Gu2[34] + Gu1[36]*Gu2[40] + Gu1[42]*Gu2[46] + Gu1[48]*Gu2[52] + Gu1[54]*Gu2[58] + Gu1[60]*Gu2[64] + Gu1[66]*Gu2[70] + Gu1[72]*Gu2[76] + Gu1[78]*Gu2[82] + Gu1[84]*Gu2[88] + Gu1[90]*Gu2[94] + Gu1[96]*Gu2[100] + Gu1[102]*Gu2[106] + R11[4];
acadoWorkspace.H[iRow * 366 + 5] = + Gu1[0]*Gu2[5] + Gu1[6]*Gu2[11] + Gu1[12]*Gu2[17] + Gu1[18]*Gu2[23] + Gu1[24]*Gu2[29] + Gu1[30]*Gu2[35] + Gu1[36]*Gu2[41] + Gu1[42]*Gu2[47] + Gu1[48]*Gu2[53] + Gu1[54]*Gu2[59] + Gu1[60]*Gu2[65] + Gu1[66]*Gu2[71] + Gu1[72]*Gu2[77] + Gu1[78]*Gu2[83] + Gu1[84]*Gu2[89] + Gu1[90]*Gu2[95] + Gu1[96]*Gu2[101] + Gu1[102]*Gu2[107] + R11[5];
acadoWorkspace.H[iRow * 366 + 60] = + Gu1[1]*Gu2[0] + Gu1[7]*Gu2[6] + Gu1[13]*Gu2[12] + Gu1[19]*Gu2[18] + Gu1[25]*Gu2[24] + Gu1[31]*Gu2[30] + Gu1[37]*Gu2[36] + Gu1[43]*Gu2[42] + Gu1[49]*Gu2[48] + Gu1[55]*Gu2[54] + Gu1[61]*Gu2[60] + Gu1[67]*Gu2[66] + Gu1[73]*Gu2[72] + Gu1[79]*Gu2[78] + Gu1[85]*Gu2[84] + Gu1[91]*Gu2[90] + Gu1[97]*Gu2[96] + Gu1[103]*Gu2[102] + R11[6];
acadoWorkspace.H[iRow * 366 + 61] = + Gu1[1]*Gu2[1] + Gu1[7]*Gu2[7] + Gu1[13]*Gu2[13] + Gu1[19]*Gu2[19] + Gu1[25]*Gu2[25] + Gu1[31]*Gu2[31] + Gu1[37]*Gu2[37] + Gu1[43]*Gu2[43] + Gu1[49]*Gu2[49] + Gu1[55]*Gu2[55] + Gu1[61]*Gu2[61] + Gu1[67]*Gu2[67] + Gu1[73]*Gu2[73] + Gu1[79]*Gu2[79] + Gu1[85]*Gu2[85] + Gu1[91]*Gu2[91] + Gu1[97]*Gu2[97] + Gu1[103]*Gu2[103] + R11[7];
acadoWorkspace.H[iRow * 366 + 62] = + Gu1[1]*Gu2[2] + Gu1[7]*Gu2[8] + Gu1[13]*Gu2[14] + Gu1[19]*Gu2[20] + Gu1[25]*Gu2[26] + Gu1[31]*Gu2[32] + Gu1[37]*Gu2[38] + Gu1[43]*Gu2[44] + Gu1[49]*Gu2[50] + Gu1[55]*Gu2[56] + Gu1[61]*Gu2[62] + Gu1[67]*Gu2[68] + Gu1[73]*Gu2[74] + Gu1[79]*Gu2[80] + Gu1[85]*Gu2[86] + Gu1[91]*Gu2[92] + Gu1[97]*Gu2[98] + Gu1[103]*Gu2[104] + R11[8];
acadoWorkspace.H[iRow * 366 + 63] = + Gu1[1]*Gu2[3] + Gu1[7]*Gu2[9] + Gu1[13]*Gu2[15] + Gu1[19]*Gu2[21] + Gu1[25]*Gu2[27] + Gu1[31]*Gu2[33] + Gu1[37]*Gu2[39] + Gu1[43]*Gu2[45] + Gu1[49]*Gu2[51] + Gu1[55]*Gu2[57] + Gu1[61]*Gu2[63] + Gu1[67]*Gu2[69] + Gu1[73]*Gu2[75] + Gu1[79]*Gu2[81] + Gu1[85]*Gu2[87] + Gu1[91]*Gu2[93] + Gu1[97]*Gu2[99] + Gu1[103]*Gu2[105] + R11[9];
acadoWorkspace.H[iRow * 366 + 64] = + Gu1[1]*Gu2[4] + Gu1[7]*Gu2[10] + Gu1[13]*Gu2[16] + Gu1[19]*Gu2[22] + Gu1[25]*Gu2[28] + Gu1[31]*Gu2[34] + Gu1[37]*Gu2[40] + Gu1[43]*Gu2[46] + Gu1[49]*Gu2[52] + Gu1[55]*Gu2[58] + Gu1[61]*Gu2[64] + Gu1[67]*Gu2[70] + Gu1[73]*Gu2[76] + Gu1[79]*Gu2[82] + Gu1[85]*Gu2[88] + Gu1[91]*Gu2[94] + Gu1[97]*Gu2[100] + Gu1[103]*Gu2[106] + R11[10];
acadoWorkspace.H[iRow * 366 + 65] = + Gu1[1]*Gu2[5] + Gu1[7]*Gu2[11] + Gu1[13]*Gu2[17] + Gu1[19]*Gu2[23] + Gu1[25]*Gu2[29] + Gu1[31]*Gu2[35] + Gu1[37]*Gu2[41] + Gu1[43]*Gu2[47] + Gu1[49]*Gu2[53] + Gu1[55]*Gu2[59] + Gu1[61]*Gu2[65] + Gu1[67]*Gu2[71] + Gu1[73]*Gu2[77] + Gu1[79]*Gu2[83] + Gu1[85]*Gu2[89] + Gu1[91]*Gu2[95] + Gu1[97]*Gu2[101] + Gu1[103]*Gu2[107] + R11[11];
acadoWorkspace.H[iRow * 366 + 120] = + Gu1[2]*Gu2[0] + Gu1[8]*Gu2[6] + Gu1[14]*Gu2[12] + Gu1[20]*Gu2[18] + Gu1[26]*Gu2[24] + Gu1[32]*Gu2[30] + Gu1[38]*Gu2[36] + Gu1[44]*Gu2[42] + Gu1[50]*Gu2[48] + Gu1[56]*Gu2[54] + Gu1[62]*Gu2[60] + Gu1[68]*Gu2[66] + Gu1[74]*Gu2[72] + Gu1[80]*Gu2[78] + Gu1[86]*Gu2[84] + Gu1[92]*Gu2[90] + Gu1[98]*Gu2[96] + Gu1[104]*Gu2[102] + R11[12];
acadoWorkspace.H[iRow * 366 + 121] = + Gu1[2]*Gu2[1] + Gu1[8]*Gu2[7] + Gu1[14]*Gu2[13] + Gu1[20]*Gu2[19] + Gu1[26]*Gu2[25] + Gu1[32]*Gu2[31] + Gu1[38]*Gu2[37] + Gu1[44]*Gu2[43] + Gu1[50]*Gu2[49] + Gu1[56]*Gu2[55] + Gu1[62]*Gu2[61] + Gu1[68]*Gu2[67] + Gu1[74]*Gu2[73] + Gu1[80]*Gu2[79] + Gu1[86]*Gu2[85] + Gu1[92]*Gu2[91] + Gu1[98]*Gu2[97] + Gu1[104]*Gu2[103] + R11[13];
acadoWorkspace.H[iRow * 366 + 122] = + Gu1[2]*Gu2[2] + Gu1[8]*Gu2[8] + Gu1[14]*Gu2[14] + Gu1[20]*Gu2[20] + Gu1[26]*Gu2[26] + Gu1[32]*Gu2[32] + Gu1[38]*Gu2[38] + Gu1[44]*Gu2[44] + Gu1[50]*Gu2[50] + Gu1[56]*Gu2[56] + Gu1[62]*Gu2[62] + Gu1[68]*Gu2[68] + Gu1[74]*Gu2[74] + Gu1[80]*Gu2[80] + Gu1[86]*Gu2[86] + Gu1[92]*Gu2[92] + Gu1[98]*Gu2[98] + Gu1[104]*Gu2[104] + R11[14];
acadoWorkspace.H[iRow * 366 + 123] = + Gu1[2]*Gu2[3] + Gu1[8]*Gu2[9] + Gu1[14]*Gu2[15] + Gu1[20]*Gu2[21] + Gu1[26]*Gu2[27] + Gu1[32]*Gu2[33] + Gu1[38]*Gu2[39] + Gu1[44]*Gu2[45] + Gu1[50]*Gu2[51] + Gu1[56]*Gu2[57] + Gu1[62]*Gu2[63] + Gu1[68]*Gu2[69] + Gu1[74]*Gu2[75] + Gu1[80]*Gu2[81] + Gu1[86]*Gu2[87] + Gu1[92]*Gu2[93] + Gu1[98]*Gu2[99] + Gu1[104]*Gu2[105] + R11[15];
acadoWorkspace.H[iRow * 366 + 124] = + Gu1[2]*Gu2[4] + Gu1[8]*Gu2[10] + Gu1[14]*Gu2[16] + Gu1[20]*Gu2[22] + Gu1[26]*Gu2[28] + Gu1[32]*Gu2[34] + Gu1[38]*Gu2[40] + Gu1[44]*Gu2[46] + Gu1[50]*Gu2[52] + Gu1[56]*Gu2[58] + Gu1[62]*Gu2[64] + Gu1[68]*Gu2[70] + Gu1[74]*Gu2[76] + Gu1[80]*Gu2[82] + Gu1[86]*Gu2[88] + Gu1[92]*Gu2[94] + Gu1[98]*Gu2[100] + Gu1[104]*Gu2[106] + R11[16];
acadoWorkspace.H[iRow * 366 + 125] = + Gu1[2]*Gu2[5] + Gu1[8]*Gu2[11] + Gu1[14]*Gu2[17] + Gu1[20]*Gu2[23] + Gu1[26]*Gu2[29] + Gu1[32]*Gu2[35] + Gu1[38]*Gu2[41] + Gu1[44]*Gu2[47] + Gu1[50]*Gu2[53] + Gu1[56]*Gu2[59] + Gu1[62]*Gu2[65] + Gu1[68]*Gu2[71] + Gu1[74]*Gu2[77] + Gu1[80]*Gu2[83] + Gu1[86]*Gu2[89] + Gu1[92]*Gu2[95] + Gu1[98]*Gu2[101] + Gu1[104]*Gu2[107] + R11[17];
acadoWorkspace.H[iRow * 366 + 180] = + Gu1[3]*Gu2[0] + Gu1[9]*Gu2[6] + Gu1[15]*Gu2[12] + Gu1[21]*Gu2[18] + Gu1[27]*Gu2[24] + Gu1[33]*Gu2[30] + Gu1[39]*Gu2[36] + Gu1[45]*Gu2[42] + Gu1[51]*Gu2[48] + Gu1[57]*Gu2[54] + Gu1[63]*Gu2[60] + Gu1[69]*Gu2[66] + Gu1[75]*Gu2[72] + Gu1[81]*Gu2[78] + Gu1[87]*Gu2[84] + Gu1[93]*Gu2[90] + Gu1[99]*Gu2[96] + Gu1[105]*Gu2[102] + R11[18];
acadoWorkspace.H[iRow * 366 + 181] = + Gu1[3]*Gu2[1] + Gu1[9]*Gu2[7] + Gu1[15]*Gu2[13] + Gu1[21]*Gu2[19] + Gu1[27]*Gu2[25] + Gu1[33]*Gu2[31] + Gu1[39]*Gu2[37] + Gu1[45]*Gu2[43] + Gu1[51]*Gu2[49] + Gu1[57]*Gu2[55] + Gu1[63]*Gu2[61] + Gu1[69]*Gu2[67] + Gu1[75]*Gu2[73] + Gu1[81]*Gu2[79] + Gu1[87]*Gu2[85] + Gu1[93]*Gu2[91] + Gu1[99]*Gu2[97] + Gu1[105]*Gu2[103] + R11[19];
acadoWorkspace.H[iRow * 366 + 182] = + Gu1[3]*Gu2[2] + Gu1[9]*Gu2[8] + Gu1[15]*Gu2[14] + Gu1[21]*Gu2[20] + Gu1[27]*Gu2[26] + Gu1[33]*Gu2[32] + Gu1[39]*Gu2[38] + Gu1[45]*Gu2[44] + Gu1[51]*Gu2[50] + Gu1[57]*Gu2[56] + Gu1[63]*Gu2[62] + Gu1[69]*Gu2[68] + Gu1[75]*Gu2[74] + Gu1[81]*Gu2[80] + Gu1[87]*Gu2[86] + Gu1[93]*Gu2[92] + Gu1[99]*Gu2[98] + Gu1[105]*Gu2[104] + R11[20];
acadoWorkspace.H[iRow * 366 + 183] = + Gu1[3]*Gu2[3] + Gu1[9]*Gu2[9] + Gu1[15]*Gu2[15] + Gu1[21]*Gu2[21] + Gu1[27]*Gu2[27] + Gu1[33]*Gu2[33] + Gu1[39]*Gu2[39] + Gu1[45]*Gu2[45] + Gu1[51]*Gu2[51] + Gu1[57]*Gu2[57] + Gu1[63]*Gu2[63] + Gu1[69]*Gu2[69] + Gu1[75]*Gu2[75] + Gu1[81]*Gu2[81] + Gu1[87]*Gu2[87] + Gu1[93]*Gu2[93] + Gu1[99]*Gu2[99] + Gu1[105]*Gu2[105] + R11[21];
acadoWorkspace.H[iRow * 366 + 184] = + Gu1[3]*Gu2[4] + Gu1[9]*Gu2[10] + Gu1[15]*Gu2[16] + Gu1[21]*Gu2[22] + Gu1[27]*Gu2[28] + Gu1[33]*Gu2[34] + Gu1[39]*Gu2[40] + Gu1[45]*Gu2[46] + Gu1[51]*Gu2[52] + Gu1[57]*Gu2[58] + Gu1[63]*Gu2[64] + Gu1[69]*Gu2[70] + Gu1[75]*Gu2[76] + Gu1[81]*Gu2[82] + Gu1[87]*Gu2[88] + Gu1[93]*Gu2[94] + Gu1[99]*Gu2[100] + Gu1[105]*Gu2[106] + R11[22];
acadoWorkspace.H[iRow * 366 + 185] = + Gu1[3]*Gu2[5] + Gu1[9]*Gu2[11] + Gu1[15]*Gu2[17] + Gu1[21]*Gu2[23] + Gu1[27]*Gu2[29] + Gu1[33]*Gu2[35] + Gu1[39]*Gu2[41] + Gu1[45]*Gu2[47] + Gu1[51]*Gu2[53] + Gu1[57]*Gu2[59] + Gu1[63]*Gu2[65] + Gu1[69]*Gu2[71] + Gu1[75]*Gu2[77] + Gu1[81]*Gu2[83] + Gu1[87]*Gu2[89] + Gu1[93]*Gu2[95] + Gu1[99]*Gu2[101] + Gu1[105]*Gu2[107] + R11[23];
acadoWorkspace.H[iRow * 366 + 240] = + Gu1[4]*Gu2[0] + Gu1[10]*Gu2[6] + Gu1[16]*Gu2[12] + Gu1[22]*Gu2[18] + Gu1[28]*Gu2[24] + Gu1[34]*Gu2[30] + Gu1[40]*Gu2[36] + Gu1[46]*Gu2[42] + Gu1[52]*Gu2[48] + Gu1[58]*Gu2[54] + Gu1[64]*Gu2[60] + Gu1[70]*Gu2[66] + Gu1[76]*Gu2[72] + Gu1[82]*Gu2[78] + Gu1[88]*Gu2[84] + Gu1[94]*Gu2[90] + Gu1[100]*Gu2[96] + Gu1[106]*Gu2[102] + R11[24];
acadoWorkspace.H[iRow * 366 + 241] = + Gu1[4]*Gu2[1] + Gu1[10]*Gu2[7] + Gu1[16]*Gu2[13] + Gu1[22]*Gu2[19] + Gu1[28]*Gu2[25] + Gu1[34]*Gu2[31] + Gu1[40]*Gu2[37] + Gu1[46]*Gu2[43] + Gu1[52]*Gu2[49] + Gu1[58]*Gu2[55] + Gu1[64]*Gu2[61] + Gu1[70]*Gu2[67] + Gu1[76]*Gu2[73] + Gu1[82]*Gu2[79] + Gu1[88]*Gu2[85] + Gu1[94]*Gu2[91] + Gu1[100]*Gu2[97] + Gu1[106]*Gu2[103] + R11[25];
acadoWorkspace.H[iRow * 366 + 242] = + Gu1[4]*Gu2[2] + Gu1[10]*Gu2[8] + Gu1[16]*Gu2[14] + Gu1[22]*Gu2[20] + Gu1[28]*Gu2[26] + Gu1[34]*Gu2[32] + Gu1[40]*Gu2[38] + Gu1[46]*Gu2[44] + Gu1[52]*Gu2[50] + Gu1[58]*Gu2[56] + Gu1[64]*Gu2[62] + Gu1[70]*Gu2[68] + Gu1[76]*Gu2[74] + Gu1[82]*Gu2[80] + Gu1[88]*Gu2[86] + Gu1[94]*Gu2[92] + Gu1[100]*Gu2[98] + Gu1[106]*Gu2[104] + R11[26];
acadoWorkspace.H[iRow * 366 + 243] = + Gu1[4]*Gu2[3] + Gu1[10]*Gu2[9] + Gu1[16]*Gu2[15] + Gu1[22]*Gu2[21] + Gu1[28]*Gu2[27] + Gu1[34]*Gu2[33] + Gu1[40]*Gu2[39] + Gu1[46]*Gu2[45] + Gu1[52]*Gu2[51] + Gu1[58]*Gu2[57] + Gu1[64]*Gu2[63] + Gu1[70]*Gu2[69] + Gu1[76]*Gu2[75] + Gu1[82]*Gu2[81] + Gu1[88]*Gu2[87] + Gu1[94]*Gu2[93] + Gu1[100]*Gu2[99] + Gu1[106]*Gu2[105] + R11[27];
acadoWorkspace.H[iRow * 366 + 244] = + Gu1[4]*Gu2[4] + Gu1[10]*Gu2[10] + Gu1[16]*Gu2[16] + Gu1[22]*Gu2[22] + Gu1[28]*Gu2[28] + Gu1[34]*Gu2[34] + Gu1[40]*Gu2[40] + Gu1[46]*Gu2[46] + Gu1[52]*Gu2[52] + Gu1[58]*Gu2[58] + Gu1[64]*Gu2[64] + Gu1[70]*Gu2[70] + Gu1[76]*Gu2[76] + Gu1[82]*Gu2[82] + Gu1[88]*Gu2[88] + Gu1[94]*Gu2[94] + Gu1[100]*Gu2[100] + Gu1[106]*Gu2[106] + R11[28];
acadoWorkspace.H[iRow * 366 + 245] = + Gu1[4]*Gu2[5] + Gu1[10]*Gu2[11] + Gu1[16]*Gu2[17] + Gu1[22]*Gu2[23] + Gu1[28]*Gu2[29] + Gu1[34]*Gu2[35] + Gu1[40]*Gu2[41] + Gu1[46]*Gu2[47] + Gu1[52]*Gu2[53] + Gu1[58]*Gu2[59] + Gu1[64]*Gu2[65] + Gu1[70]*Gu2[71] + Gu1[76]*Gu2[77] + Gu1[82]*Gu2[83] + Gu1[88]*Gu2[89] + Gu1[94]*Gu2[95] + Gu1[100]*Gu2[101] + Gu1[106]*Gu2[107] + R11[29];
acadoWorkspace.H[iRow * 366 + 300] = + Gu1[5]*Gu2[0] + Gu1[11]*Gu2[6] + Gu1[17]*Gu2[12] + Gu1[23]*Gu2[18] + Gu1[29]*Gu2[24] + Gu1[35]*Gu2[30] + Gu1[41]*Gu2[36] + Gu1[47]*Gu2[42] + Gu1[53]*Gu2[48] + Gu1[59]*Gu2[54] + Gu1[65]*Gu2[60] + Gu1[71]*Gu2[66] + Gu1[77]*Gu2[72] + Gu1[83]*Gu2[78] + Gu1[89]*Gu2[84] + Gu1[95]*Gu2[90] + Gu1[101]*Gu2[96] + Gu1[107]*Gu2[102] + R11[30];
acadoWorkspace.H[iRow * 366 + 301] = + Gu1[5]*Gu2[1] + Gu1[11]*Gu2[7] + Gu1[17]*Gu2[13] + Gu1[23]*Gu2[19] + Gu1[29]*Gu2[25] + Gu1[35]*Gu2[31] + Gu1[41]*Gu2[37] + Gu1[47]*Gu2[43] + Gu1[53]*Gu2[49] + Gu1[59]*Gu2[55] + Gu1[65]*Gu2[61] + Gu1[71]*Gu2[67] + Gu1[77]*Gu2[73] + Gu1[83]*Gu2[79] + Gu1[89]*Gu2[85] + Gu1[95]*Gu2[91] + Gu1[101]*Gu2[97] + Gu1[107]*Gu2[103] + R11[31];
acadoWorkspace.H[iRow * 366 + 302] = + Gu1[5]*Gu2[2] + Gu1[11]*Gu2[8] + Gu1[17]*Gu2[14] + Gu1[23]*Gu2[20] + Gu1[29]*Gu2[26] + Gu1[35]*Gu2[32] + Gu1[41]*Gu2[38] + Gu1[47]*Gu2[44] + Gu1[53]*Gu2[50] + Gu1[59]*Gu2[56] + Gu1[65]*Gu2[62] + Gu1[71]*Gu2[68] + Gu1[77]*Gu2[74] + Gu1[83]*Gu2[80] + Gu1[89]*Gu2[86] + Gu1[95]*Gu2[92] + Gu1[101]*Gu2[98] + Gu1[107]*Gu2[104] + R11[32];
acadoWorkspace.H[iRow * 366 + 303] = + Gu1[5]*Gu2[3] + Gu1[11]*Gu2[9] + Gu1[17]*Gu2[15] + Gu1[23]*Gu2[21] + Gu1[29]*Gu2[27] + Gu1[35]*Gu2[33] + Gu1[41]*Gu2[39] + Gu1[47]*Gu2[45] + Gu1[53]*Gu2[51] + Gu1[59]*Gu2[57] + Gu1[65]*Gu2[63] + Gu1[71]*Gu2[69] + Gu1[77]*Gu2[75] + Gu1[83]*Gu2[81] + Gu1[89]*Gu2[87] + Gu1[95]*Gu2[93] + Gu1[101]*Gu2[99] + Gu1[107]*Gu2[105] + R11[33];
acadoWorkspace.H[iRow * 366 + 304] = + Gu1[5]*Gu2[4] + Gu1[11]*Gu2[10] + Gu1[17]*Gu2[16] + Gu1[23]*Gu2[22] + Gu1[29]*Gu2[28] + Gu1[35]*Gu2[34] + Gu1[41]*Gu2[40] + Gu1[47]*Gu2[46] + Gu1[53]*Gu2[52] + Gu1[59]*Gu2[58] + Gu1[65]*Gu2[64] + Gu1[71]*Gu2[70] + Gu1[77]*Gu2[76] + Gu1[83]*Gu2[82] + Gu1[89]*Gu2[88] + Gu1[95]*Gu2[94] + Gu1[101]*Gu2[100] + Gu1[107]*Gu2[106] + R11[34];
acadoWorkspace.H[iRow * 366 + 305] = + Gu1[5]*Gu2[5] + Gu1[11]*Gu2[11] + Gu1[17]*Gu2[17] + Gu1[23]*Gu2[23] + Gu1[29]*Gu2[29] + Gu1[35]*Gu2[35] + Gu1[41]*Gu2[41] + Gu1[47]*Gu2[47] + Gu1[53]*Gu2[53] + Gu1[59]*Gu2[59] + Gu1[65]*Gu2[65] + Gu1[71]*Gu2[71] + Gu1[77]*Gu2[77] + Gu1[83]*Gu2[83] + Gu1[89]*Gu2[89] + Gu1[95]*Gu2[95] + Gu1[101]*Gu2[101] + Gu1[107]*Gu2[107] + R11[35];
acadoWorkspace.H[iRow * 366] += 1.0000000000000000e-04;
acadoWorkspace.H[iRow * 366 + 61] += 1.0000000000000000e-04;
acadoWorkspace.H[iRow * 366 + 122] += 1.0000000000000000e-04;
acadoWorkspace.H[iRow * 366 + 183] += 1.0000000000000000e-04;
acadoWorkspace.H[iRow * 366 + 244] += 1.0000000000000000e-04;
acadoWorkspace.H[iRow * 366 + 305] += 1.0000000000000000e-04;
}

void acado_multGxTGu( real_t* const Gx1, real_t* const Gu1, real_t* const Gu2 )
{
Gu2[0] = + Gx1[0]*Gu1[0] + Gx1[18]*Gu1[6] + Gx1[36]*Gu1[12] + Gx1[54]*Gu1[18] + Gx1[72]*Gu1[24] + Gx1[90]*Gu1[30] + Gx1[108]*Gu1[36] + Gx1[126]*Gu1[42] + Gx1[144]*Gu1[48] + Gx1[162]*Gu1[54] + Gx1[180]*Gu1[60] + Gx1[198]*Gu1[66] + Gx1[216]*Gu1[72] + Gx1[234]*Gu1[78] + Gx1[252]*Gu1[84] + Gx1[270]*Gu1[90] + Gx1[288]*Gu1[96] + Gx1[306]*Gu1[102];
Gu2[1] = + Gx1[0]*Gu1[1] + Gx1[18]*Gu1[7] + Gx1[36]*Gu1[13] + Gx1[54]*Gu1[19] + Gx1[72]*Gu1[25] + Gx1[90]*Gu1[31] + Gx1[108]*Gu1[37] + Gx1[126]*Gu1[43] + Gx1[144]*Gu1[49] + Gx1[162]*Gu1[55] + Gx1[180]*Gu1[61] + Gx1[198]*Gu1[67] + Gx1[216]*Gu1[73] + Gx1[234]*Gu1[79] + Gx1[252]*Gu1[85] + Gx1[270]*Gu1[91] + Gx1[288]*Gu1[97] + Gx1[306]*Gu1[103];
Gu2[2] = + Gx1[0]*Gu1[2] + Gx1[18]*Gu1[8] + Gx1[36]*Gu1[14] + Gx1[54]*Gu1[20] + Gx1[72]*Gu1[26] + Gx1[90]*Gu1[32] + Gx1[108]*Gu1[38] + Gx1[126]*Gu1[44] + Gx1[144]*Gu1[50] + Gx1[162]*Gu1[56] + Gx1[180]*Gu1[62] + Gx1[198]*Gu1[68] + Gx1[216]*Gu1[74] + Gx1[234]*Gu1[80] + Gx1[252]*Gu1[86] + Gx1[270]*Gu1[92] + Gx1[288]*Gu1[98] + Gx1[306]*Gu1[104];
Gu2[3] = + Gx1[0]*Gu1[3] + Gx1[18]*Gu1[9] + Gx1[36]*Gu1[15] + Gx1[54]*Gu1[21] + Gx1[72]*Gu1[27] + Gx1[90]*Gu1[33] + Gx1[108]*Gu1[39] + Gx1[126]*Gu1[45] + Gx1[144]*Gu1[51] + Gx1[162]*Gu1[57] + Gx1[180]*Gu1[63] + Gx1[198]*Gu1[69] + Gx1[216]*Gu1[75] + Gx1[234]*Gu1[81] + Gx1[252]*Gu1[87] + Gx1[270]*Gu1[93] + Gx1[288]*Gu1[99] + Gx1[306]*Gu1[105];
Gu2[4] = + Gx1[0]*Gu1[4] + Gx1[18]*Gu1[10] + Gx1[36]*Gu1[16] + Gx1[54]*Gu1[22] + Gx1[72]*Gu1[28] + Gx1[90]*Gu1[34] + Gx1[108]*Gu1[40] + Gx1[126]*Gu1[46] + Gx1[144]*Gu1[52] + Gx1[162]*Gu1[58] + Gx1[180]*Gu1[64] + Gx1[198]*Gu1[70] + Gx1[216]*Gu1[76] + Gx1[234]*Gu1[82] + Gx1[252]*Gu1[88] + Gx1[270]*Gu1[94] + Gx1[288]*Gu1[100] + Gx1[306]*Gu1[106];
Gu2[5] = + Gx1[0]*Gu1[5] + Gx1[18]*Gu1[11] + Gx1[36]*Gu1[17] + Gx1[54]*Gu1[23] + Gx1[72]*Gu1[29] + Gx1[90]*Gu1[35] + Gx1[108]*Gu1[41] + Gx1[126]*Gu1[47] + Gx1[144]*Gu1[53] + Gx1[162]*Gu1[59] + Gx1[180]*Gu1[65] + Gx1[198]*Gu1[71] + Gx1[216]*Gu1[77] + Gx1[234]*Gu1[83] + Gx1[252]*Gu1[89] + Gx1[270]*Gu1[95] + Gx1[288]*Gu1[101] + Gx1[306]*Gu1[107];
Gu2[6] = + Gx1[1]*Gu1[0] + Gx1[19]*Gu1[6] + Gx1[37]*Gu1[12] + Gx1[55]*Gu1[18] + Gx1[73]*Gu1[24] + Gx1[91]*Gu1[30] + Gx1[109]*Gu1[36] + Gx1[127]*Gu1[42] + Gx1[145]*Gu1[48] + Gx1[163]*Gu1[54] + Gx1[181]*Gu1[60] + Gx1[199]*Gu1[66] + Gx1[217]*Gu1[72] + Gx1[235]*Gu1[78] + Gx1[253]*Gu1[84] + Gx1[271]*Gu1[90] + Gx1[289]*Gu1[96] + Gx1[307]*Gu1[102];
Gu2[7] = + Gx1[1]*Gu1[1] + Gx1[19]*Gu1[7] + Gx1[37]*Gu1[13] + Gx1[55]*Gu1[19] + Gx1[73]*Gu1[25] + Gx1[91]*Gu1[31] + Gx1[109]*Gu1[37] + Gx1[127]*Gu1[43] + Gx1[145]*Gu1[49] + Gx1[163]*Gu1[55] + Gx1[181]*Gu1[61] + Gx1[199]*Gu1[67] + Gx1[217]*Gu1[73] + Gx1[235]*Gu1[79] + Gx1[253]*Gu1[85] + Gx1[271]*Gu1[91] + Gx1[289]*Gu1[97] + Gx1[307]*Gu1[103];
Gu2[8] = + Gx1[1]*Gu1[2] + Gx1[19]*Gu1[8] + Gx1[37]*Gu1[14] + Gx1[55]*Gu1[20] + Gx1[73]*Gu1[26] + Gx1[91]*Gu1[32] + Gx1[109]*Gu1[38] + Gx1[127]*Gu1[44] + Gx1[145]*Gu1[50] + Gx1[163]*Gu1[56] + Gx1[181]*Gu1[62] + Gx1[199]*Gu1[68] + Gx1[217]*Gu1[74] + Gx1[235]*Gu1[80] + Gx1[253]*Gu1[86] + Gx1[271]*Gu1[92] + Gx1[289]*Gu1[98] + Gx1[307]*Gu1[104];
Gu2[9] = + Gx1[1]*Gu1[3] + Gx1[19]*Gu1[9] + Gx1[37]*Gu1[15] + Gx1[55]*Gu1[21] + Gx1[73]*Gu1[27] + Gx1[91]*Gu1[33] + Gx1[109]*Gu1[39] + Gx1[127]*Gu1[45] + Gx1[145]*Gu1[51] + Gx1[163]*Gu1[57] + Gx1[181]*Gu1[63] + Gx1[199]*Gu1[69] + Gx1[217]*Gu1[75] + Gx1[235]*Gu1[81] + Gx1[253]*Gu1[87] + Gx1[271]*Gu1[93] + Gx1[289]*Gu1[99] + Gx1[307]*Gu1[105];
Gu2[10] = + Gx1[1]*Gu1[4] + Gx1[19]*Gu1[10] + Gx1[37]*Gu1[16] + Gx1[55]*Gu1[22] + Gx1[73]*Gu1[28] + Gx1[91]*Gu1[34] + Gx1[109]*Gu1[40] + Gx1[127]*Gu1[46] + Gx1[145]*Gu1[52] + Gx1[163]*Gu1[58] + Gx1[181]*Gu1[64] + Gx1[199]*Gu1[70] + Gx1[217]*Gu1[76] + Gx1[235]*Gu1[82] + Gx1[253]*Gu1[88] + Gx1[271]*Gu1[94] + Gx1[289]*Gu1[100] + Gx1[307]*Gu1[106];
Gu2[11] = + Gx1[1]*Gu1[5] + Gx1[19]*Gu1[11] + Gx1[37]*Gu1[17] + Gx1[55]*Gu1[23] + Gx1[73]*Gu1[29] + Gx1[91]*Gu1[35] + Gx1[109]*Gu1[41] + Gx1[127]*Gu1[47] + Gx1[145]*Gu1[53] + Gx1[163]*Gu1[59] + Gx1[181]*Gu1[65] + Gx1[199]*Gu1[71] + Gx1[217]*Gu1[77] + Gx1[235]*Gu1[83] + Gx1[253]*Gu1[89] + Gx1[271]*Gu1[95] + Gx1[289]*Gu1[101] + Gx1[307]*Gu1[107];
Gu2[12] = + Gx1[2]*Gu1[0] + Gx1[20]*Gu1[6] + Gx1[38]*Gu1[12] + Gx1[56]*Gu1[18] + Gx1[74]*Gu1[24] + Gx1[92]*Gu1[30] + Gx1[110]*Gu1[36] + Gx1[128]*Gu1[42] + Gx1[146]*Gu1[48] + Gx1[164]*Gu1[54] + Gx1[182]*Gu1[60] + Gx1[200]*Gu1[66] + Gx1[218]*Gu1[72] + Gx1[236]*Gu1[78] + Gx1[254]*Gu1[84] + Gx1[272]*Gu1[90] + Gx1[290]*Gu1[96] + Gx1[308]*Gu1[102];
Gu2[13] = + Gx1[2]*Gu1[1] + Gx1[20]*Gu1[7] + Gx1[38]*Gu1[13] + Gx1[56]*Gu1[19] + Gx1[74]*Gu1[25] + Gx1[92]*Gu1[31] + Gx1[110]*Gu1[37] + Gx1[128]*Gu1[43] + Gx1[146]*Gu1[49] + Gx1[164]*Gu1[55] + Gx1[182]*Gu1[61] + Gx1[200]*Gu1[67] + Gx1[218]*Gu1[73] + Gx1[236]*Gu1[79] + Gx1[254]*Gu1[85] + Gx1[272]*Gu1[91] + Gx1[290]*Gu1[97] + Gx1[308]*Gu1[103];
Gu2[14] = + Gx1[2]*Gu1[2] + Gx1[20]*Gu1[8] + Gx1[38]*Gu1[14] + Gx1[56]*Gu1[20] + Gx1[74]*Gu1[26] + Gx1[92]*Gu1[32] + Gx1[110]*Gu1[38] + Gx1[128]*Gu1[44] + Gx1[146]*Gu1[50] + Gx1[164]*Gu1[56] + Gx1[182]*Gu1[62] + Gx1[200]*Gu1[68] + Gx1[218]*Gu1[74] + Gx1[236]*Gu1[80] + Gx1[254]*Gu1[86] + Gx1[272]*Gu1[92] + Gx1[290]*Gu1[98] + Gx1[308]*Gu1[104];
Gu2[15] = + Gx1[2]*Gu1[3] + Gx1[20]*Gu1[9] + Gx1[38]*Gu1[15] + Gx1[56]*Gu1[21] + Gx1[74]*Gu1[27] + Gx1[92]*Gu1[33] + Gx1[110]*Gu1[39] + Gx1[128]*Gu1[45] + Gx1[146]*Gu1[51] + Gx1[164]*Gu1[57] + Gx1[182]*Gu1[63] + Gx1[200]*Gu1[69] + Gx1[218]*Gu1[75] + Gx1[236]*Gu1[81] + Gx1[254]*Gu1[87] + Gx1[272]*Gu1[93] + Gx1[290]*Gu1[99] + Gx1[308]*Gu1[105];
Gu2[16] = + Gx1[2]*Gu1[4] + Gx1[20]*Gu1[10] + Gx1[38]*Gu1[16] + Gx1[56]*Gu1[22] + Gx1[74]*Gu1[28] + Gx1[92]*Gu1[34] + Gx1[110]*Gu1[40] + Gx1[128]*Gu1[46] + Gx1[146]*Gu1[52] + Gx1[164]*Gu1[58] + Gx1[182]*Gu1[64] + Gx1[200]*Gu1[70] + Gx1[218]*Gu1[76] + Gx1[236]*Gu1[82] + Gx1[254]*Gu1[88] + Gx1[272]*Gu1[94] + Gx1[290]*Gu1[100] + Gx1[308]*Gu1[106];
Gu2[17] = + Gx1[2]*Gu1[5] + Gx1[20]*Gu1[11] + Gx1[38]*Gu1[17] + Gx1[56]*Gu1[23] + Gx1[74]*Gu1[29] + Gx1[92]*Gu1[35] + Gx1[110]*Gu1[41] + Gx1[128]*Gu1[47] + Gx1[146]*Gu1[53] + Gx1[164]*Gu1[59] + Gx1[182]*Gu1[65] + Gx1[200]*Gu1[71] + Gx1[218]*Gu1[77] + Gx1[236]*Gu1[83] + Gx1[254]*Gu1[89] + Gx1[272]*Gu1[95] + Gx1[290]*Gu1[101] + Gx1[308]*Gu1[107];
Gu2[18] = + Gx1[3]*Gu1[0] + Gx1[21]*Gu1[6] + Gx1[39]*Gu1[12] + Gx1[57]*Gu1[18] + Gx1[75]*Gu1[24] + Gx1[93]*Gu1[30] + Gx1[111]*Gu1[36] + Gx1[129]*Gu1[42] + Gx1[147]*Gu1[48] + Gx1[165]*Gu1[54] + Gx1[183]*Gu1[60] + Gx1[201]*Gu1[66] + Gx1[219]*Gu1[72] + Gx1[237]*Gu1[78] + Gx1[255]*Gu1[84] + Gx1[273]*Gu1[90] + Gx1[291]*Gu1[96] + Gx1[309]*Gu1[102];
Gu2[19] = + Gx1[3]*Gu1[1] + Gx1[21]*Gu1[7] + Gx1[39]*Gu1[13] + Gx1[57]*Gu1[19] + Gx1[75]*Gu1[25] + Gx1[93]*Gu1[31] + Gx1[111]*Gu1[37] + Gx1[129]*Gu1[43] + Gx1[147]*Gu1[49] + Gx1[165]*Gu1[55] + Gx1[183]*Gu1[61] + Gx1[201]*Gu1[67] + Gx1[219]*Gu1[73] + Gx1[237]*Gu1[79] + Gx1[255]*Gu1[85] + Gx1[273]*Gu1[91] + Gx1[291]*Gu1[97] + Gx1[309]*Gu1[103];
Gu2[20] = + Gx1[3]*Gu1[2] + Gx1[21]*Gu1[8] + Gx1[39]*Gu1[14] + Gx1[57]*Gu1[20] + Gx1[75]*Gu1[26] + Gx1[93]*Gu1[32] + Gx1[111]*Gu1[38] + Gx1[129]*Gu1[44] + Gx1[147]*Gu1[50] + Gx1[165]*Gu1[56] + Gx1[183]*Gu1[62] + Gx1[201]*Gu1[68] + Gx1[219]*Gu1[74] + Gx1[237]*Gu1[80] + Gx1[255]*Gu1[86] + Gx1[273]*Gu1[92] + Gx1[291]*Gu1[98] + Gx1[309]*Gu1[104];
Gu2[21] = + Gx1[3]*Gu1[3] + Gx1[21]*Gu1[9] + Gx1[39]*Gu1[15] + Gx1[57]*Gu1[21] + Gx1[75]*Gu1[27] + Gx1[93]*Gu1[33] + Gx1[111]*Gu1[39] + Gx1[129]*Gu1[45] + Gx1[147]*Gu1[51] + Gx1[165]*Gu1[57] + Gx1[183]*Gu1[63] + Gx1[201]*Gu1[69] + Gx1[219]*Gu1[75] + Gx1[237]*Gu1[81] + Gx1[255]*Gu1[87] + Gx1[273]*Gu1[93] + Gx1[291]*Gu1[99] + Gx1[309]*Gu1[105];
Gu2[22] = + Gx1[3]*Gu1[4] + Gx1[21]*Gu1[10] + Gx1[39]*Gu1[16] + Gx1[57]*Gu1[22] + Gx1[75]*Gu1[28] + Gx1[93]*Gu1[34] + Gx1[111]*Gu1[40] + Gx1[129]*Gu1[46] + Gx1[147]*Gu1[52] + Gx1[165]*Gu1[58] + Gx1[183]*Gu1[64] + Gx1[201]*Gu1[70] + Gx1[219]*Gu1[76] + Gx1[237]*Gu1[82] + Gx1[255]*Gu1[88] + Gx1[273]*Gu1[94] + Gx1[291]*Gu1[100] + Gx1[309]*Gu1[106];
Gu2[23] = + Gx1[3]*Gu1[5] + Gx1[21]*Gu1[11] + Gx1[39]*Gu1[17] + Gx1[57]*Gu1[23] + Gx1[75]*Gu1[29] + Gx1[93]*Gu1[35] + Gx1[111]*Gu1[41] + Gx1[129]*Gu1[47] + Gx1[147]*Gu1[53] + Gx1[165]*Gu1[59] + Gx1[183]*Gu1[65] + Gx1[201]*Gu1[71] + Gx1[219]*Gu1[77] + Gx1[237]*Gu1[83] + Gx1[255]*Gu1[89] + Gx1[273]*Gu1[95] + Gx1[291]*Gu1[101] + Gx1[309]*Gu1[107];
Gu2[24] = + Gx1[4]*Gu1[0] + Gx1[22]*Gu1[6] + Gx1[40]*Gu1[12] + Gx1[58]*Gu1[18] + Gx1[76]*Gu1[24] + Gx1[94]*Gu1[30] + Gx1[112]*Gu1[36] + Gx1[130]*Gu1[42] + Gx1[148]*Gu1[48] + Gx1[166]*Gu1[54] + Gx1[184]*Gu1[60] + Gx1[202]*Gu1[66] + Gx1[220]*Gu1[72] + Gx1[238]*Gu1[78] + Gx1[256]*Gu1[84] + Gx1[274]*Gu1[90] + Gx1[292]*Gu1[96] + Gx1[310]*Gu1[102];
Gu2[25] = + Gx1[4]*Gu1[1] + Gx1[22]*Gu1[7] + Gx1[40]*Gu1[13] + Gx1[58]*Gu1[19] + Gx1[76]*Gu1[25] + Gx1[94]*Gu1[31] + Gx1[112]*Gu1[37] + Gx1[130]*Gu1[43] + Gx1[148]*Gu1[49] + Gx1[166]*Gu1[55] + Gx1[184]*Gu1[61] + Gx1[202]*Gu1[67] + Gx1[220]*Gu1[73] + Gx1[238]*Gu1[79] + Gx1[256]*Gu1[85] + Gx1[274]*Gu1[91] + Gx1[292]*Gu1[97] + Gx1[310]*Gu1[103];
Gu2[26] = + Gx1[4]*Gu1[2] + Gx1[22]*Gu1[8] + Gx1[40]*Gu1[14] + Gx1[58]*Gu1[20] + Gx1[76]*Gu1[26] + Gx1[94]*Gu1[32] + Gx1[112]*Gu1[38] + Gx1[130]*Gu1[44] + Gx1[148]*Gu1[50] + Gx1[166]*Gu1[56] + Gx1[184]*Gu1[62] + Gx1[202]*Gu1[68] + Gx1[220]*Gu1[74] + Gx1[238]*Gu1[80] + Gx1[256]*Gu1[86] + Gx1[274]*Gu1[92] + Gx1[292]*Gu1[98] + Gx1[310]*Gu1[104];
Gu2[27] = + Gx1[4]*Gu1[3] + Gx1[22]*Gu1[9] + Gx1[40]*Gu1[15] + Gx1[58]*Gu1[21] + Gx1[76]*Gu1[27] + Gx1[94]*Gu1[33] + Gx1[112]*Gu1[39] + Gx1[130]*Gu1[45] + Gx1[148]*Gu1[51] + Gx1[166]*Gu1[57] + Gx1[184]*Gu1[63] + Gx1[202]*Gu1[69] + Gx1[220]*Gu1[75] + Gx1[238]*Gu1[81] + Gx1[256]*Gu1[87] + Gx1[274]*Gu1[93] + Gx1[292]*Gu1[99] + Gx1[310]*Gu1[105];
Gu2[28] = + Gx1[4]*Gu1[4] + Gx1[22]*Gu1[10] + Gx1[40]*Gu1[16] + Gx1[58]*Gu1[22] + Gx1[76]*Gu1[28] + Gx1[94]*Gu1[34] + Gx1[112]*Gu1[40] + Gx1[130]*Gu1[46] + Gx1[148]*Gu1[52] + Gx1[166]*Gu1[58] + Gx1[184]*Gu1[64] + Gx1[202]*Gu1[70] + Gx1[220]*Gu1[76] + Gx1[238]*Gu1[82] + Gx1[256]*Gu1[88] + Gx1[274]*Gu1[94] + Gx1[292]*Gu1[100] + Gx1[310]*Gu1[106];
Gu2[29] = + Gx1[4]*Gu1[5] + Gx1[22]*Gu1[11] + Gx1[40]*Gu1[17] + Gx1[58]*Gu1[23] + Gx1[76]*Gu1[29] + Gx1[94]*Gu1[35] + Gx1[112]*Gu1[41] + Gx1[130]*Gu1[47] + Gx1[148]*Gu1[53] + Gx1[166]*Gu1[59] + Gx1[184]*Gu1[65] + Gx1[202]*Gu1[71] + Gx1[220]*Gu1[77] + Gx1[238]*Gu1[83] + Gx1[256]*Gu1[89] + Gx1[274]*Gu1[95] + Gx1[292]*Gu1[101] + Gx1[310]*Gu1[107];
Gu2[30] = + Gx1[5]*Gu1[0] + Gx1[23]*Gu1[6] + Gx1[41]*Gu1[12] + Gx1[59]*Gu1[18] + Gx1[77]*Gu1[24] + Gx1[95]*Gu1[30] + Gx1[113]*Gu1[36] + Gx1[131]*Gu1[42] + Gx1[149]*Gu1[48] + Gx1[167]*Gu1[54] + Gx1[185]*Gu1[60] + Gx1[203]*Gu1[66] + Gx1[221]*Gu1[72] + Gx1[239]*Gu1[78] + Gx1[257]*Gu1[84] + Gx1[275]*Gu1[90] + Gx1[293]*Gu1[96] + Gx1[311]*Gu1[102];
Gu2[31] = + Gx1[5]*Gu1[1] + Gx1[23]*Gu1[7] + Gx1[41]*Gu1[13] + Gx1[59]*Gu1[19] + Gx1[77]*Gu1[25] + Gx1[95]*Gu1[31] + Gx1[113]*Gu1[37] + Gx1[131]*Gu1[43] + Gx1[149]*Gu1[49] + Gx1[167]*Gu1[55] + Gx1[185]*Gu1[61] + Gx1[203]*Gu1[67] + Gx1[221]*Gu1[73] + Gx1[239]*Gu1[79] + Gx1[257]*Gu1[85] + Gx1[275]*Gu1[91] + Gx1[293]*Gu1[97] + Gx1[311]*Gu1[103];
Gu2[32] = + Gx1[5]*Gu1[2] + Gx1[23]*Gu1[8] + Gx1[41]*Gu1[14] + Gx1[59]*Gu1[20] + Gx1[77]*Gu1[26] + Gx1[95]*Gu1[32] + Gx1[113]*Gu1[38] + Gx1[131]*Gu1[44] + Gx1[149]*Gu1[50] + Gx1[167]*Gu1[56] + Gx1[185]*Gu1[62] + Gx1[203]*Gu1[68] + Gx1[221]*Gu1[74] + Gx1[239]*Gu1[80] + Gx1[257]*Gu1[86] + Gx1[275]*Gu1[92] + Gx1[293]*Gu1[98] + Gx1[311]*Gu1[104];
Gu2[33] = + Gx1[5]*Gu1[3] + Gx1[23]*Gu1[9] + Gx1[41]*Gu1[15] + Gx1[59]*Gu1[21] + Gx1[77]*Gu1[27] + Gx1[95]*Gu1[33] + Gx1[113]*Gu1[39] + Gx1[131]*Gu1[45] + Gx1[149]*Gu1[51] + Gx1[167]*Gu1[57] + Gx1[185]*Gu1[63] + Gx1[203]*Gu1[69] + Gx1[221]*Gu1[75] + Gx1[239]*Gu1[81] + Gx1[257]*Gu1[87] + Gx1[275]*Gu1[93] + Gx1[293]*Gu1[99] + Gx1[311]*Gu1[105];
Gu2[34] = + Gx1[5]*Gu1[4] + Gx1[23]*Gu1[10] + Gx1[41]*Gu1[16] + Gx1[59]*Gu1[22] + Gx1[77]*Gu1[28] + Gx1[95]*Gu1[34] + Gx1[113]*Gu1[40] + Gx1[131]*Gu1[46] + Gx1[149]*Gu1[52] + Gx1[167]*Gu1[58] + Gx1[185]*Gu1[64] + Gx1[203]*Gu1[70] + Gx1[221]*Gu1[76] + Gx1[239]*Gu1[82] + Gx1[257]*Gu1[88] + Gx1[275]*Gu1[94] + Gx1[293]*Gu1[100] + Gx1[311]*Gu1[106];
Gu2[35] = + Gx1[5]*Gu1[5] + Gx1[23]*Gu1[11] + Gx1[41]*Gu1[17] + Gx1[59]*Gu1[23] + Gx1[77]*Gu1[29] + Gx1[95]*Gu1[35] + Gx1[113]*Gu1[41] + Gx1[131]*Gu1[47] + Gx1[149]*Gu1[53] + Gx1[167]*Gu1[59] + Gx1[185]*Gu1[65] + Gx1[203]*Gu1[71] + Gx1[221]*Gu1[77] + Gx1[239]*Gu1[83] + Gx1[257]*Gu1[89] + Gx1[275]*Gu1[95] + Gx1[293]*Gu1[101] + Gx1[311]*Gu1[107];
Gu2[36] = + Gx1[6]*Gu1[0] + Gx1[24]*Gu1[6] + Gx1[42]*Gu1[12] + Gx1[60]*Gu1[18] + Gx1[78]*Gu1[24] + Gx1[96]*Gu1[30] + Gx1[114]*Gu1[36] + Gx1[132]*Gu1[42] + Gx1[150]*Gu1[48] + Gx1[168]*Gu1[54] + Gx1[186]*Gu1[60] + Gx1[204]*Gu1[66] + Gx1[222]*Gu1[72] + Gx1[240]*Gu1[78] + Gx1[258]*Gu1[84] + Gx1[276]*Gu1[90] + Gx1[294]*Gu1[96] + Gx1[312]*Gu1[102];
Gu2[37] = + Gx1[6]*Gu1[1] + Gx1[24]*Gu1[7] + Gx1[42]*Gu1[13] + Gx1[60]*Gu1[19] + Gx1[78]*Gu1[25] + Gx1[96]*Gu1[31] + Gx1[114]*Gu1[37] + Gx1[132]*Gu1[43] + Gx1[150]*Gu1[49] + Gx1[168]*Gu1[55] + Gx1[186]*Gu1[61] + Gx1[204]*Gu1[67] + Gx1[222]*Gu1[73] + Gx1[240]*Gu1[79] + Gx1[258]*Gu1[85] + Gx1[276]*Gu1[91] + Gx1[294]*Gu1[97] + Gx1[312]*Gu1[103];
Gu2[38] = + Gx1[6]*Gu1[2] + Gx1[24]*Gu1[8] + Gx1[42]*Gu1[14] + Gx1[60]*Gu1[20] + Gx1[78]*Gu1[26] + Gx1[96]*Gu1[32] + Gx1[114]*Gu1[38] + Gx1[132]*Gu1[44] + Gx1[150]*Gu1[50] + Gx1[168]*Gu1[56] + Gx1[186]*Gu1[62] + Gx1[204]*Gu1[68] + Gx1[222]*Gu1[74] + Gx1[240]*Gu1[80] + Gx1[258]*Gu1[86] + Gx1[276]*Gu1[92] + Gx1[294]*Gu1[98] + Gx1[312]*Gu1[104];
Gu2[39] = + Gx1[6]*Gu1[3] + Gx1[24]*Gu1[9] + Gx1[42]*Gu1[15] + Gx1[60]*Gu1[21] + Gx1[78]*Gu1[27] + Gx1[96]*Gu1[33] + Gx1[114]*Gu1[39] + Gx1[132]*Gu1[45] + Gx1[150]*Gu1[51] + Gx1[168]*Gu1[57] + Gx1[186]*Gu1[63] + Gx1[204]*Gu1[69] + Gx1[222]*Gu1[75] + Gx1[240]*Gu1[81] + Gx1[258]*Gu1[87] + Gx1[276]*Gu1[93] + Gx1[294]*Gu1[99] + Gx1[312]*Gu1[105];
Gu2[40] = + Gx1[6]*Gu1[4] + Gx1[24]*Gu1[10] + Gx1[42]*Gu1[16] + Gx1[60]*Gu1[22] + Gx1[78]*Gu1[28] + Gx1[96]*Gu1[34] + Gx1[114]*Gu1[40] + Gx1[132]*Gu1[46] + Gx1[150]*Gu1[52] + Gx1[168]*Gu1[58] + Gx1[186]*Gu1[64] + Gx1[204]*Gu1[70] + Gx1[222]*Gu1[76] + Gx1[240]*Gu1[82] + Gx1[258]*Gu1[88] + Gx1[276]*Gu1[94] + Gx1[294]*Gu1[100] + Gx1[312]*Gu1[106];
Gu2[41] = + Gx1[6]*Gu1[5] + Gx1[24]*Gu1[11] + Gx1[42]*Gu1[17] + Gx1[60]*Gu1[23] + Gx1[78]*Gu1[29] + Gx1[96]*Gu1[35] + Gx1[114]*Gu1[41] + Gx1[132]*Gu1[47] + Gx1[150]*Gu1[53] + Gx1[168]*Gu1[59] + Gx1[186]*Gu1[65] + Gx1[204]*Gu1[71] + Gx1[222]*Gu1[77] + Gx1[240]*Gu1[83] + Gx1[258]*Gu1[89] + Gx1[276]*Gu1[95] + Gx1[294]*Gu1[101] + Gx1[312]*Gu1[107];
Gu2[42] = + Gx1[7]*Gu1[0] + Gx1[25]*Gu1[6] + Gx1[43]*Gu1[12] + Gx1[61]*Gu1[18] + Gx1[79]*Gu1[24] + Gx1[97]*Gu1[30] + Gx1[115]*Gu1[36] + Gx1[133]*Gu1[42] + Gx1[151]*Gu1[48] + Gx1[169]*Gu1[54] + Gx1[187]*Gu1[60] + Gx1[205]*Gu1[66] + Gx1[223]*Gu1[72] + Gx1[241]*Gu1[78] + Gx1[259]*Gu1[84] + Gx1[277]*Gu1[90] + Gx1[295]*Gu1[96] + Gx1[313]*Gu1[102];
Gu2[43] = + Gx1[7]*Gu1[1] + Gx1[25]*Gu1[7] + Gx1[43]*Gu1[13] + Gx1[61]*Gu1[19] + Gx1[79]*Gu1[25] + Gx1[97]*Gu1[31] + Gx1[115]*Gu1[37] + Gx1[133]*Gu1[43] + Gx1[151]*Gu1[49] + Gx1[169]*Gu1[55] + Gx1[187]*Gu1[61] + Gx1[205]*Gu1[67] + Gx1[223]*Gu1[73] + Gx1[241]*Gu1[79] + Gx1[259]*Gu1[85] + Gx1[277]*Gu1[91] + Gx1[295]*Gu1[97] + Gx1[313]*Gu1[103];
Gu2[44] = + Gx1[7]*Gu1[2] + Gx1[25]*Gu1[8] + Gx1[43]*Gu1[14] + Gx1[61]*Gu1[20] + Gx1[79]*Gu1[26] + Gx1[97]*Gu1[32] + Gx1[115]*Gu1[38] + Gx1[133]*Gu1[44] + Gx1[151]*Gu1[50] + Gx1[169]*Gu1[56] + Gx1[187]*Gu1[62] + Gx1[205]*Gu1[68] + Gx1[223]*Gu1[74] + Gx1[241]*Gu1[80] + Gx1[259]*Gu1[86] + Gx1[277]*Gu1[92] + Gx1[295]*Gu1[98] + Gx1[313]*Gu1[104];
Gu2[45] = + Gx1[7]*Gu1[3] + Gx1[25]*Gu1[9] + Gx1[43]*Gu1[15] + Gx1[61]*Gu1[21] + Gx1[79]*Gu1[27] + Gx1[97]*Gu1[33] + Gx1[115]*Gu1[39] + Gx1[133]*Gu1[45] + Gx1[151]*Gu1[51] + Gx1[169]*Gu1[57] + Gx1[187]*Gu1[63] + Gx1[205]*Gu1[69] + Gx1[223]*Gu1[75] + Gx1[241]*Gu1[81] + Gx1[259]*Gu1[87] + Gx1[277]*Gu1[93] + Gx1[295]*Gu1[99] + Gx1[313]*Gu1[105];
Gu2[46] = + Gx1[7]*Gu1[4] + Gx1[25]*Gu1[10] + Gx1[43]*Gu1[16] + Gx1[61]*Gu1[22] + Gx1[79]*Gu1[28] + Gx1[97]*Gu1[34] + Gx1[115]*Gu1[40] + Gx1[133]*Gu1[46] + Gx1[151]*Gu1[52] + Gx1[169]*Gu1[58] + Gx1[187]*Gu1[64] + Gx1[205]*Gu1[70] + Gx1[223]*Gu1[76] + Gx1[241]*Gu1[82] + Gx1[259]*Gu1[88] + Gx1[277]*Gu1[94] + Gx1[295]*Gu1[100] + Gx1[313]*Gu1[106];
Gu2[47] = + Gx1[7]*Gu1[5] + Gx1[25]*Gu1[11] + Gx1[43]*Gu1[17] + Gx1[61]*Gu1[23] + Gx1[79]*Gu1[29] + Gx1[97]*Gu1[35] + Gx1[115]*Gu1[41] + Gx1[133]*Gu1[47] + Gx1[151]*Gu1[53] + Gx1[169]*Gu1[59] + Gx1[187]*Gu1[65] + Gx1[205]*Gu1[71] + Gx1[223]*Gu1[77] + Gx1[241]*Gu1[83] + Gx1[259]*Gu1[89] + Gx1[277]*Gu1[95] + Gx1[295]*Gu1[101] + Gx1[313]*Gu1[107];
Gu2[48] = + Gx1[8]*Gu1[0] + Gx1[26]*Gu1[6] + Gx1[44]*Gu1[12] + Gx1[62]*Gu1[18] + Gx1[80]*Gu1[24] + Gx1[98]*Gu1[30] + Gx1[116]*Gu1[36] + Gx1[134]*Gu1[42] + Gx1[152]*Gu1[48] + Gx1[170]*Gu1[54] + Gx1[188]*Gu1[60] + Gx1[206]*Gu1[66] + Gx1[224]*Gu1[72] + Gx1[242]*Gu1[78] + Gx1[260]*Gu1[84] + Gx1[278]*Gu1[90] + Gx1[296]*Gu1[96] + Gx1[314]*Gu1[102];
Gu2[49] = + Gx1[8]*Gu1[1] + Gx1[26]*Gu1[7] + Gx1[44]*Gu1[13] + Gx1[62]*Gu1[19] + Gx1[80]*Gu1[25] + Gx1[98]*Gu1[31] + Gx1[116]*Gu1[37] + Gx1[134]*Gu1[43] + Gx1[152]*Gu1[49] + Gx1[170]*Gu1[55] + Gx1[188]*Gu1[61] + Gx1[206]*Gu1[67] + Gx1[224]*Gu1[73] + Gx1[242]*Gu1[79] + Gx1[260]*Gu1[85] + Gx1[278]*Gu1[91] + Gx1[296]*Gu1[97] + Gx1[314]*Gu1[103];
Gu2[50] = + Gx1[8]*Gu1[2] + Gx1[26]*Gu1[8] + Gx1[44]*Gu1[14] + Gx1[62]*Gu1[20] + Gx1[80]*Gu1[26] + Gx1[98]*Gu1[32] + Gx1[116]*Gu1[38] + Gx1[134]*Gu1[44] + Gx1[152]*Gu1[50] + Gx1[170]*Gu1[56] + Gx1[188]*Gu1[62] + Gx1[206]*Gu1[68] + Gx1[224]*Gu1[74] + Gx1[242]*Gu1[80] + Gx1[260]*Gu1[86] + Gx1[278]*Gu1[92] + Gx1[296]*Gu1[98] + Gx1[314]*Gu1[104];
Gu2[51] = + Gx1[8]*Gu1[3] + Gx1[26]*Gu1[9] + Gx1[44]*Gu1[15] + Gx1[62]*Gu1[21] + Gx1[80]*Gu1[27] + Gx1[98]*Gu1[33] + Gx1[116]*Gu1[39] + Gx1[134]*Gu1[45] + Gx1[152]*Gu1[51] + Gx1[170]*Gu1[57] + Gx1[188]*Gu1[63] + Gx1[206]*Gu1[69] + Gx1[224]*Gu1[75] + Gx1[242]*Gu1[81] + Gx1[260]*Gu1[87] + Gx1[278]*Gu1[93] + Gx1[296]*Gu1[99] + Gx1[314]*Gu1[105];
Gu2[52] = + Gx1[8]*Gu1[4] + Gx1[26]*Gu1[10] + Gx1[44]*Gu1[16] + Gx1[62]*Gu1[22] + Gx1[80]*Gu1[28] + Gx1[98]*Gu1[34] + Gx1[116]*Gu1[40] + Gx1[134]*Gu1[46] + Gx1[152]*Gu1[52] + Gx1[170]*Gu1[58] + Gx1[188]*Gu1[64] + Gx1[206]*Gu1[70] + Gx1[224]*Gu1[76] + Gx1[242]*Gu1[82] + Gx1[260]*Gu1[88] + Gx1[278]*Gu1[94] + Gx1[296]*Gu1[100] + Gx1[314]*Gu1[106];
Gu2[53] = + Gx1[8]*Gu1[5] + Gx1[26]*Gu1[11] + Gx1[44]*Gu1[17] + Gx1[62]*Gu1[23] + Gx1[80]*Gu1[29] + Gx1[98]*Gu1[35] + Gx1[116]*Gu1[41] + Gx1[134]*Gu1[47] + Gx1[152]*Gu1[53] + Gx1[170]*Gu1[59] + Gx1[188]*Gu1[65] + Gx1[206]*Gu1[71] + Gx1[224]*Gu1[77] + Gx1[242]*Gu1[83] + Gx1[260]*Gu1[89] + Gx1[278]*Gu1[95] + Gx1[296]*Gu1[101] + Gx1[314]*Gu1[107];
Gu2[54] = + Gx1[9]*Gu1[0] + Gx1[27]*Gu1[6] + Gx1[45]*Gu1[12] + Gx1[63]*Gu1[18] + Gx1[81]*Gu1[24] + Gx1[99]*Gu1[30] + Gx1[117]*Gu1[36] + Gx1[135]*Gu1[42] + Gx1[153]*Gu1[48] + Gx1[171]*Gu1[54] + Gx1[189]*Gu1[60] + Gx1[207]*Gu1[66] + Gx1[225]*Gu1[72] + Gx1[243]*Gu1[78] + Gx1[261]*Gu1[84] + Gx1[279]*Gu1[90] + Gx1[297]*Gu1[96] + Gx1[315]*Gu1[102];
Gu2[55] = + Gx1[9]*Gu1[1] + Gx1[27]*Gu1[7] + Gx1[45]*Gu1[13] + Gx1[63]*Gu1[19] + Gx1[81]*Gu1[25] + Gx1[99]*Gu1[31] + Gx1[117]*Gu1[37] + Gx1[135]*Gu1[43] + Gx1[153]*Gu1[49] + Gx1[171]*Gu1[55] + Gx1[189]*Gu1[61] + Gx1[207]*Gu1[67] + Gx1[225]*Gu1[73] + Gx1[243]*Gu1[79] + Gx1[261]*Gu1[85] + Gx1[279]*Gu1[91] + Gx1[297]*Gu1[97] + Gx1[315]*Gu1[103];
Gu2[56] = + Gx1[9]*Gu1[2] + Gx1[27]*Gu1[8] + Gx1[45]*Gu1[14] + Gx1[63]*Gu1[20] + Gx1[81]*Gu1[26] + Gx1[99]*Gu1[32] + Gx1[117]*Gu1[38] + Gx1[135]*Gu1[44] + Gx1[153]*Gu1[50] + Gx1[171]*Gu1[56] + Gx1[189]*Gu1[62] + Gx1[207]*Gu1[68] + Gx1[225]*Gu1[74] + Gx1[243]*Gu1[80] + Gx1[261]*Gu1[86] + Gx1[279]*Gu1[92] + Gx1[297]*Gu1[98] + Gx1[315]*Gu1[104];
Gu2[57] = + Gx1[9]*Gu1[3] + Gx1[27]*Gu1[9] + Gx1[45]*Gu1[15] + Gx1[63]*Gu1[21] + Gx1[81]*Gu1[27] + Gx1[99]*Gu1[33] + Gx1[117]*Gu1[39] + Gx1[135]*Gu1[45] + Gx1[153]*Gu1[51] + Gx1[171]*Gu1[57] + Gx1[189]*Gu1[63] + Gx1[207]*Gu1[69] + Gx1[225]*Gu1[75] + Gx1[243]*Gu1[81] + Gx1[261]*Gu1[87] + Gx1[279]*Gu1[93] + Gx1[297]*Gu1[99] + Gx1[315]*Gu1[105];
Gu2[58] = + Gx1[9]*Gu1[4] + Gx1[27]*Gu1[10] + Gx1[45]*Gu1[16] + Gx1[63]*Gu1[22] + Gx1[81]*Gu1[28] + Gx1[99]*Gu1[34] + Gx1[117]*Gu1[40] + Gx1[135]*Gu1[46] + Gx1[153]*Gu1[52] + Gx1[171]*Gu1[58] + Gx1[189]*Gu1[64] + Gx1[207]*Gu1[70] + Gx1[225]*Gu1[76] + Gx1[243]*Gu1[82] + Gx1[261]*Gu1[88] + Gx1[279]*Gu1[94] + Gx1[297]*Gu1[100] + Gx1[315]*Gu1[106];
Gu2[59] = + Gx1[9]*Gu1[5] + Gx1[27]*Gu1[11] + Gx1[45]*Gu1[17] + Gx1[63]*Gu1[23] + Gx1[81]*Gu1[29] + Gx1[99]*Gu1[35] + Gx1[117]*Gu1[41] + Gx1[135]*Gu1[47] + Gx1[153]*Gu1[53] + Gx1[171]*Gu1[59] + Gx1[189]*Gu1[65] + Gx1[207]*Gu1[71] + Gx1[225]*Gu1[77] + Gx1[243]*Gu1[83] + Gx1[261]*Gu1[89] + Gx1[279]*Gu1[95] + Gx1[297]*Gu1[101] + Gx1[315]*Gu1[107];
Gu2[60] = + Gx1[10]*Gu1[0] + Gx1[28]*Gu1[6] + Gx1[46]*Gu1[12] + Gx1[64]*Gu1[18] + Gx1[82]*Gu1[24] + Gx1[100]*Gu1[30] + Gx1[118]*Gu1[36] + Gx1[136]*Gu1[42] + Gx1[154]*Gu1[48] + Gx1[172]*Gu1[54] + Gx1[190]*Gu1[60] + Gx1[208]*Gu1[66] + Gx1[226]*Gu1[72] + Gx1[244]*Gu1[78] + Gx1[262]*Gu1[84] + Gx1[280]*Gu1[90] + Gx1[298]*Gu1[96] + Gx1[316]*Gu1[102];
Gu2[61] = + Gx1[10]*Gu1[1] + Gx1[28]*Gu1[7] + Gx1[46]*Gu1[13] + Gx1[64]*Gu1[19] + Gx1[82]*Gu1[25] + Gx1[100]*Gu1[31] + Gx1[118]*Gu1[37] + Gx1[136]*Gu1[43] + Gx1[154]*Gu1[49] + Gx1[172]*Gu1[55] + Gx1[190]*Gu1[61] + Gx1[208]*Gu1[67] + Gx1[226]*Gu1[73] + Gx1[244]*Gu1[79] + Gx1[262]*Gu1[85] + Gx1[280]*Gu1[91] + Gx1[298]*Gu1[97] + Gx1[316]*Gu1[103];
Gu2[62] = + Gx1[10]*Gu1[2] + Gx1[28]*Gu1[8] + Gx1[46]*Gu1[14] + Gx1[64]*Gu1[20] + Gx1[82]*Gu1[26] + Gx1[100]*Gu1[32] + Gx1[118]*Gu1[38] + Gx1[136]*Gu1[44] + Gx1[154]*Gu1[50] + Gx1[172]*Gu1[56] + Gx1[190]*Gu1[62] + Gx1[208]*Gu1[68] + Gx1[226]*Gu1[74] + Gx1[244]*Gu1[80] + Gx1[262]*Gu1[86] + Gx1[280]*Gu1[92] + Gx1[298]*Gu1[98] + Gx1[316]*Gu1[104];
Gu2[63] = + Gx1[10]*Gu1[3] + Gx1[28]*Gu1[9] + Gx1[46]*Gu1[15] + Gx1[64]*Gu1[21] + Gx1[82]*Gu1[27] + Gx1[100]*Gu1[33] + Gx1[118]*Gu1[39] + Gx1[136]*Gu1[45] + Gx1[154]*Gu1[51] + Gx1[172]*Gu1[57] + Gx1[190]*Gu1[63] + Gx1[208]*Gu1[69] + Gx1[226]*Gu1[75] + Gx1[244]*Gu1[81] + Gx1[262]*Gu1[87] + Gx1[280]*Gu1[93] + Gx1[298]*Gu1[99] + Gx1[316]*Gu1[105];
Gu2[64] = + Gx1[10]*Gu1[4] + Gx1[28]*Gu1[10] + Gx1[46]*Gu1[16] + Gx1[64]*Gu1[22] + Gx1[82]*Gu1[28] + Gx1[100]*Gu1[34] + Gx1[118]*Gu1[40] + Gx1[136]*Gu1[46] + Gx1[154]*Gu1[52] + Gx1[172]*Gu1[58] + Gx1[190]*Gu1[64] + Gx1[208]*Gu1[70] + Gx1[226]*Gu1[76] + Gx1[244]*Gu1[82] + Gx1[262]*Gu1[88] + Gx1[280]*Gu1[94] + Gx1[298]*Gu1[100] + Gx1[316]*Gu1[106];
Gu2[65] = + Gx1[10]*Gu1[5] + Gx1[28]*Gu1[11] + Gx1[46]*Gu1[17] + Gx1[64]*Gu1[23] + Gx1[82]*Gu1[29] + Gx1[100]*Gu1[35] + Gx1[118]*Gu1[41] + Gx1[136]*Gu1[47] + Gx1[154]*Gu1[53] + Gx1[172]*Gu1[59] + Gx1[190]*Gu1[65] + Gx1[208]*Gu1[71] + Gx1[226]*Gu1[77] + Gx1[244]*Gu1[83] + Gx1[262]*Gu1[89] + Gx1[280]*Gu1[95] + Gx1[298]*Gu1[101] + Gx1[316]*Gu1[107];
Gu2[66] = + Gx1[11]*Gu1[0] + Gx1[29]*Gu1[6] + Gx1[47]*Gu1[12] + Gx1[65]*Gu1[18] + Gx1[83]*Gu1[24] + Gx1[101]*Gu1[30] + Gx1[119]*Gu1[36] + Gx1[137]*Gu1[42] + Gx1[155]*Gu1[48] + Gx1[173]*Gu1[54] + Gx1[191]*Gu1[60] + Gx1[209]*Gu1[66] + Gx1[227]*Gu1[72] + Gx1[245]*Gu1[78] + Gx1[263]*Gu1[84] + Gx1[281]*Gu1[90] + Gx1[299]*Gu1[96] + Gx1[317]*Gu1[102];
Gu2[67] = + Gx1[11]*Gu1[1] + Gx1[29]*Gu1[7] + Gx1[47]*Gu1[13] + Gx1[65]*Gu1[19] + Gx1[83]*Gu1[25] + Gx1[101]*Gu1[31] + Gx1[119]*Gu1[37] + Gx1[137]*Gu1[43] + Gx1[155]*Gu1[49] + Gx1[173]*Gu1[55] + Gx1[191]*Gu1[61] + Gx1[209]*Gu1[67] + Gx1[227]*Gu1[73] + Gx1[245]*Gu1[79] + Gx1[263]*Gu1[85] + Gx1[281]*Gu1[91] + Gx1[299]*Gu1[97] + Gx1[317]*Gu1[103];
Gu2[68] = + Gx1[11]*Gu1[2] + Gx1[29]*Gu1[8] + Gx1[47]*Gu1[14] + Gx1[65]*Gu1[20] + Gx1[83]*Gu1[26] + Gx1[101]*Gu1[32] + Gx1[119]*Gu1[38] + Gx1[137]*Gu1[44] + Gx1[155]*Gu1[50] + Gx1[173]*Gu1[56] + Gx1[191]*Gu1[62] + Gx1[209]*Gu1[68] + Gx1[227]*Gu1[74] + Gx1[245]*Gu1[80] + Gx1[263]*Gu1[86] + Gx1[281]*Gu1[92] + Gx1[299]*Gu1[98] + Gx1[317]*Gu1[104];
Gu2[69] = + Gx1[11]*Gu1[3] + Gx1[29]*Gu1[9] + Gx1[47]*Gu1[15] + Gx1[65]*Gu1[21] + Gx1[83]*Gu1[27] + Gx1[101]*Gu1[33] + Gx1[119]*Gu1[39] + Gx1[137]*Gu1[45] + Gx1[155]*Gu1[51] + Gx1[173]*Gu1[57] + Gx1[191]*Gu1[63] + Gx1[209]*Gu1[69] + Gx1[227]*Gu1[75] + Gx1[245]*Gu1[81] + Gx1[263]*Gu1[87] + Gx1[281]*Gu1[93] + Gx1[299]*Gu1[99] + Gx1[317]*Gu1[105];
Gu2[70] = + Gx1[11]*Gu1[4] + Gx1[29]*Gu1[10] + Gx1[47]*Gu1[16] + Gx1[65]*Gu1[22] + Gx1[83]*Gu1[28] + Gx1[101]*Gu1[34] + Gx1[119]*Gu1[40] + Gx1[137]*Gu1[46] + Gx1[155]*Gu1[52] + Gx1[173]*Gu1[58] + Gx1[191]*Gu1[64] + Gx1[209]*Gu1[70] + Gx1[227]*Gu1[76] + Gx1[245]*Gu1[82] + Gx1[263]*Gu1[88] + Gx1[281]*Gu1[94] + Gx1[299]*Gu1[100] + Gx1[317]*Gu1[106];
Gu2[71] = + Gx1[11]*Gu1[5] + Gx1[29]*Gu1[11] + Gx1[47]*Gu1[17] + Gx1[65]*Gu1[23] + Gx1[83]*Gu1[29] + Gx1[101]*Gu1[35] + Gx1[119]*Gu1[41] + Gx1[137]*Gu1[47] + Gx1[155]*Gu1[53] + Gx1[173]*Gu1[59] + Gx1[191]*Gu1[65] + Gx1[209]*Gu1[71] + Gx1[227]*Gu1[77] + Gx1[245]*Gu1[83] + Gx1[263]*Gu1[89] + Gx1[281]*Gu1[95] + Gx1[299]*Gu1[101] + Gx1[317]*Gu1[107];
Gu2[72] = + Gx1[12]*Gu1[0] + Gx1[30]*Gu1[6] + Gx1[48]*Gu1[12] + Gx1[66]*Gu1[18] + Gx1[84]*Gu1[24] + Gx1[102]*Gu1[30] + Gx1[120]*Gu1[36] + Gx1[138]*Gu1[42] + Gx1[156]*Gu1[48] + Gx1[174]*Gu1[54] + Gx1[192]*Gu1[60] + Gx1[210]*Gu1[66] + Gx1[228]*Gu1[72] + Gx1[246]*Gu1[78] + Gx1[264]*Gu1[84] + Gx1[282]*Gu1[90] + Gx1[300]*Gu1[96] + Gx1[318]*Gu1[102];
Gu2[73] = + Gx1[12]*Gu1[1] + Gx1[30]*Gu1[7] + Gx1[48]*Gu1[13] + Gx1[66]*Gu1[19] + Gx1[84]*Gu1[25] + Gx1[102]*Gu1[31] + Gx1[120]*Gu1[37] + Gx1[138]*Gu1[43] + Gx1[156]*Gu1[49] + Gx1[174]*Gu1[55] + Gx1[192]*Gu1[61] + Gx1[210]*Gu1[67] + Gx1[228]*Gu1[73] + Gx1[246]*Gu1[79] + Gx1[264]*Gu1[85] + Gx1[282]*Gu1[91] + Gx1[300]*Gu1[97] + Gx1[318]*Gu1[103];
Gu2[74] = + Gx1[12]*Gu1[2] + Gx1[30]*Gu1[8] + Gx1[48]*Gu1[14] + Gx1[66]*Gu1[20] + Gx1[84]*Gu1[26] + Gx1[102]*Gu1[32] + Gx1[120]*Gu1[38] + Gx1[138]*Gu1[44] + Gx1[156]*Gu1[50] + Gx1[174]*Gu1[56] + Gx1[192]*Gu1[62] + Gx1[210]*Gu1[68] + Gx1[228]*Gu1[74] + Gx1[246]*Gu1[80] + Gx1[264]*Gu1[86] + Gx1[282]*Gu1[92] + Gx1[300]*Gu1[98] + Gx1[318]*Gu1[104];
Gu2[75] = + Gx1[12]*Gu1[3] + Gx1[30]*Gu1[9] + Gx1[48]*Gu1[15] + Gx1[66]*Gu1[21] + Gx1[84]*Gu1[27] + Gx1[102]*Gu1[33] + Gx1[120]*Gu1[39] + Gx1[138]*Gu1[45] + Gx1[156]*Gu1[51] + Gx1[174]*Gu1[57] + Gx1[192]*Gu1[63] + Gx1[210]*Gu1[69] + Gx1[228]*Gu1[75] + Gx1[246]*Gu1[81] + Gx1[264]*Gu1[87] + Gx1[282]*Gu1[93] + Gx1[300]*Gu1[99] + Gx1[318]*Gu1[105];
Gu2[76] = + Gx1[12]*Gu1[4] + Gx1[30]*Gu1[10] + Gx1[48]*Gu1[16] + Gx1[66]*Gu1[22] + Gx1[84]*Gu1[28] + Gx1[102]*Gu1[34] + Gx1[120]*Gu1[40] + Gx1[138]*Gu1[46] + Gx1[156]*Gu1[52] + Gx1[174]*Gu1[58] + Gx1[192]*Gu1[64] + Gx1[210]*Gu1[70] + Gx1[228]*Gu1[76] + Gx1[246]*Gu1[82] + Gx1[264]*Gu1[88] + Gx1[282]*Gu1[94] + Gx1[300]*Gu1[100] + Gx1[318]*Gu1[106];
Gu2[77] = + Gx1[12]*Gu1[5] + Gx1[30]*Gu1[11] + Gx1[48]*Gu1[17] + Gx1[66]*Gu1[23] + Gx1[84]*Gu1[29] + Gx1[102]*Gu1[35] + Gx1[120]*Gu1[41] + Gx1[138]*Gu1[47] + Gx1[156]*Gu1[53] + Gx1[174]*Gu1[59] + Gx1[192]*Gu1[65] + Gx1[210]*Gu1[71] + Gx1[228]*Gu1[77] + Gx1[246]*Gu1[83] + Gx1[264]*Gu1[89] + Gx1[282]*Gu1[95] + Gx1[300]*Gu1[101] + Gx1[318]*Gu1[107];
Gu2[78] = + Gx1[13]*Gu1[0] + Gx1[31]*Gu1[6] + Gx1[49]*Gu1[12] + Gx1[67]*Gu1[18] + Gx1[85]*Gu1[24] + Gx1[103]*Gu1[30] + Gx1[121]*Gu1[36] + Gx1[139]*Gu1[42] + Gx1[157]*Gu1[48] + Gx1[175]*Gu1[54] + Gx1[193]*Gu1[60] + Gx1[211]*Gu1[66] + Gx1[229]*Gu1[72] + Gx1[247]*Gu1[78] + Gx1[265]*Gu1[84] + Gx1[283]*Gu1[90] + Gx1[301]*Gu1[96] + Gx1[319]*Gu1[102];
Gu2[79] = + Gx1[13]*Gu1[1] + Gx1[31]*Gu1[7] + Gx1[49]*Gu1[13] + Gx1[67]*Gu1[19] + Gx1[85]*Gu1[25] + Gx1[103]*Gu1[31] + Gx1[121]*Gu1[37] + Gx1[139]*Gu1[43] + Gx1[157]*Gu1[49] + Gx1[175]*Gu1[55] + Gx1[193]*Gu1[61] + Gx1[211]*Gu1[67] + Gx1[229]*Gu1[73] + Gx1[247]*Gu1[79] + Gx1[265]*Gu1[85] + Gx1[283]*Gu1[91] + Gx1[301]*Gu1[97] + Gx1[319]*Gu1[103];
Gu2[80] = + Gx1[13]*Gu1[2] + Gx1[31]*Gu1[8] + Gx1[49]*Gu1[14] + Gx1[67]*Gu1[20] + Gx1[85]*Gu1[26] + Gx1[103]*Gu1[32] + Gx1[121]*Gu1[38] + Gx1[139]*Gu1[44] + Gx1[157]*Gu1[50] + Gx1[175]*Gu1[56] + Gx1[193]*Gu1[62] + Gx1[211]*Gu1[68] + Gx1[229]*Gu1[74] + Gx1[247]*Gu1[80] + Gx1[265]*Gu1[86] + Gx1[283]*Gu1[92] + Gx1[301]*Gu1[98] + Gx1[319]*Gu1[104];
Gu2[81] = + Gx1[13]*Gu1[3] + Gx1[31]*Gu1[9] + Gx1[49]*Gu1[15] + Gx1[67]*Gu1[21] + Gx1[85]*Gu1[27] + Gx1[103]*Gu1[33] + Gx1[121]*Gu1[39] + Gx1[139]*Gu1[45] + Gx1[157]*Gu1[51] + Gx1[175]*Gu1[57] + Gx1[193]*Gu1[63] + Gx1[211]*Gu1[69] + Gx1[229]*Gu1[75] + Gx1[247]*Gu1[81] + Gx1[265]*Gu1[87] + Gx1[283]*Gu1[93] + Gx1[301]*Gu1[99] + Gx1[319]*Gu1[105];
Gu2[82] = + Gx1[13]*Gu1[4] + Gx1[31]*Gu1[10] + Gx1[49]*Gu1[16] + Gx1[67]*Gu1[22] + Gx1[85]*Gu1[28] + Gx1[103]*Gu1[34] + Gx1[121]*Gu1[40] + Gx1[139]*Gu1[46] + Gx1[157]*Gu1[52] + Gx1[175]*Gu1[58] + Gx1[193]*Gu1[64] + Gx1[211]*Gu1[70] + Gx1[229]*Gu1[76] + Gx1[247]*Gu1[82] + Gx1[265]*Gu1[88] + Gx1[283]*Gu1[94] + Gx1[301]*Gu1[100] + Gx1[319]*Gu1[106];
Gu2[83] = + Gx1[13]*Gu1[5] + Gx1[31]*Gu1[11] + Gx1[49]*Gu1[17] + Gx1[67]*Gu1[23] + Gx1[85]*Gu1[29] + Gx1[103]*Gu1[35] + Gx1[121]*Gu1[41] + Gx1[139]*Gu1[47] + Gx1[157]*Gu1[53] + Gx1[175]*Gu1[59] + Gx1[193]*Gu1[65] + Gx1[211]*Gu1[71] + Gx1[229]*Gu1[77] + Gx1[247]*Gu1[83] + Gx1[265]*Gu1[89] + Gx1[283]*Gu1[95] + Gx1[301]*Gu1[101] + Gx1[319]*Gu1[107];
Gu2[84] = + Gx1[14]*Gu1[0] + Gx1[32]*Gu1[6] + Gx1[50]*Gu1[12] + Gx1[68]*Gu1[18] + Gx1[86]*Gu1[24] + Gx1[104]*Gu1[30] + Gx1[122]*Gu1[36] + Gx1[140]*Gu1[42] + Gx1[158]*Gu1[48] + Gx1[176]*Gu1[54] + Gx1[194]*Gu1[60] + Gx1[212]*Gu1[66] + Gx1[230]*Gu1[72] + Gx1[248]*Gu1[78] + Gx1[266]*Gu1[84] + Gx1[284]*Gu1[90] + Gx1[302]*Gu1[96] + Gx1[320]*Gu1[102];
Gu2[85] = + Gx1[14]*Gu1[1] + Gx1[32]*Gu1[7] + Gx1[50]*Gu1[13] + Gx1[68]*Gu1[19] + Gx1[86]*Gu1[25] + Gx1[104]*Gu1[31] + Gx1[122]*Gu1[37] + Gx1[140]*Gu1[43] + Gx1[158]*Gu1[49] + Gx1[176]*Gu1[55] + Gx1[194]*Gu1[61] + Gx1[212]*Gu1[67] + Gx1[230]*Gu1[73] + Gx1[248]*Gu1[79] + Gx1[266]*Gu1[85] + Gx1[284]*Gu1[91] + Gx1[302]*Gu1[97] + Gx1[320]*Gu1[103];
Gu2[86] = + Gx1[14]*Gu1[2] + Gx1[32]*Gu1[8] + Gx1[50]*Gu1[14] + Gx1[68]*Gu1[20] + Gx1[86]*Gu1[26] + Gx1[104]*Gu1[32] + Gx1[122]*Gu1[38] + Gx1[140]*Gu1[44] + Gx1[158]*Gu1[50] + Gx1[176]*Gu1[56] + Gx1[194]*Gu1[62] + Gx1[212]*Gu1[68] + Gx1[230]*Gu1[74] + Gx1[248]*Gu1[80] + Gx1[266]*Gu1[86] + Gx1[284]*Gu1[92] + Gx1[302]*Gu1[98] + Gx1[320]*Gu1[104];
Gu2[87] = + Gx1[14]*Gu1[3] + Gx1[32]*Gu1[9] + Gx1[50]*Gu1[15] + Gx1[68]*Gu1[21] + Gx1[86]*Gu1[27] + Gx1[104]*Gu1[33] + Gx1[122]*Gu1[39] + Gx1[140]*Gu1[45] + Gx1[158]*Gu1[51] + Gx1[176]*Gu1[57] + Gx1[194]*Gu1[63] + Gx1[212]*Gu1[69] + Gx1[230]*Gu1[75] + Gx1[248]*Gu1[81] + Gx1[266]*Gu1[87] + Gx1[284]*Gu1[93] + Gx1[302]*Gu1[99] + Gx1[320]*Gu1[105];
Gu2[88] = + Gx1[14]*Gu1[4] + Gx1[32]*Gu1[10] + Gx1[50]*Gu1[16] + Gx1[68]*Gu1[22] + Gx1[86]*Gu1[28] + Gx1[104]*Gu1[34] + Gx1[122]*Gu1[40] + Gx1[140]*Gu1[46] + Gx1[158]*Gu1[52] + Gx1[176]*Gu1[58] + Gx1[194]*Gu1[64] + Gx1[212]*Gu1[70] + Gx1[230]*Gu1[76] + Gx1[248]*Gu1[82] + Gx1[266]*Gu1[88] + Gx1[284]*Gu1[94] + Gx1[302]*Gu1[100] + Gx1[320]*Gu1[106];
Gu2[89] = + Gx1[14]*Gu1[5] + Gx1[32]*Gu1[11] + Gx1[50]*Gu1[17] + Gx1[68]*Gu1[23] + Gx1[86]*Gu1[29] + Gx1[104]*Gu1[35] + Gx1[122]*Gu1[41] + Gx1[140]*Gu1[47] + Gx1[158]*Gu1[53] + Gx1[176]*Gu1[59] + Gx1[194]*Gu1[65] + Gx1[212]*Gu1[71] + Gx1[230]*Gu1[77] + Gx1[248]*Gu1[83] + Gx1[266]*Gu1[89] + Gx1[284]*Gu1[95] + Gx1[302]*Gu1[101] + Gx1[320]*Gu1[107];
Gu2[90] = + Gx1[15]*Gu1[0] + Gx1[33]*Gu1[6] + Gx1[51]*Gu1[12] + Gx1[69]*Gu1[18] + Gx1[87]*Gu1[24] + Gx1[105]*Gu1[30] + Gx1[123]*Gu1[36] + Gx1[141]*Gu1[42] + Gx1[159]*Gu1[48] + Gx1[177]*Gu1[54] + Gx1[195]*Gu1[60] + Gx1[213]*Gu1[66] + Gx1[231]*Gu1[72] + Gx1[249]*Gu1[78] + Gx1[267]*Gu1[84] + Gx1[285]*Gu1[90] + Gx1[303]*Gu1[96] + Gx1[321]*Gu1[102];
Gu2[91] = + Gx1[15]*Gu1[1] + Gx1[33]*Gu1[7] + Gx1[51]*Gu1[13] + Gx1[69]*Gu1[19] + Gx1[87]*Gu1[25] + Gx1[105]*Gu1[31] + Gx1[123]*Gu1[37] + Gx1[141]*Gu1[43] + Gx1[159]*Gu1[49] + Gx1[177]*Gu1[55] + Gx1[195]*Gu1[61] + Gx1[213]*Gu1[67] + Gx1[231]*Gu1[73] + Gx1[249]*Gu1[79] + Gx1[267]*Gu1[85] + Gx1[285]*Gu1[91] + Gx1[303]*Gu1[97] + Gx1[321]*Gu1[103];
Gu2[92] = + Gx1[15]*Gu1[2] + Gx1[33]*Gu1[8] + Gx1[51]*Gu1[14] + Gx1[69]*Gu1[20] + Gx1[87]*Gu1[26] + Gx1[105]*Gu1[32] + Gx1[123]*Gu1[38] + Gx1[141]*Gu1[44] + Gx1[159]*Gu1[50] + Gx1[177]*Gu1[56] + Gx1[195]*Gu1[62] + Gx1[213]*Gu1[68] + Gx1[231]*Gu1[74] + Gx1[249]*Gu1[80] + Gx1[267]*Gu1[86] + Gx1[285]*Gu1[92] + Gx1[303]*Gu1[98] + Gx1[321]*Gu1[104];
Gu2[93] = + Gx1[15]*Gu1[3] + Gx1[33]*Gu1[9] + Gx1[51]*Gu1[15] + Gx1[69]*Gu1[21] + Gx1[87]*Gu1[27] + Gx1[105]*Gu1[33] + Gx1[123]*Gu1[39] + Gx1[141]*Gu1[45] + Gx1[159]*Gu1[51] + Gx1[177]*Gu1[57] + Gx1[195]*Gu1[63] + Gx1[213]*Gu1[69] + Gx1[231]*Gu1[75] + Gx1[249]*Gu1[81] + Gx1[267]*Gu1[87] + Gx1[285]*Gu1[93] + Gx1[303]*Gu1[99] + Gx1[321]*Gu1[105];
Gu2[94] = + Gx1[15]*Gu1[4] + Gx1[33]*Gu1[10] + Gx1[51]*Gu1[16] + Gx1[69]*Gu1[22] + Gx1[87]*Gu1[28] + Gx1[105]*Gu1[34] + Gx1[123]*Gu1[40] + Gx1[141]*Gu1[46] + Gx1[159]*Gu1[52] + Gx1[177]*Gu1[58] + Gx1[195]*Gu1[64] + Gx1[213]*Gu1[70] + Gx1[231]*Gu1[76] + Gx1[249]*Gu1[82] + Gx1[267]*Gu1[88] + Gx1[285]*Gu1[94] + Gx1[303]*Gu1[100] + Gx1[321]*Gu1[106];
Gu2[95] = + Gx1[15]*Gu1[5] + Gx1[33]*Gu1[11] + Gx1[51]*Gu1[17] + Gx1[69]*Gu1[23] + Gx1[87]*Gu1[29] + Gx1[105]*Gu1[35] + Gx1[123]*Gu1[41] + Gx1[141]*Gu1[47] + Gx1[159]*Gu1[53] + Gx1[177]*Gu1[59] + Gx1[195]*Gu1[65] + Gx1[213]*Gu1[71] + Gx1[231]*Gu1[77] + Gx1[249]*Gu1[83] + Gx1[267]*Gu1[89] + Gx1[285]*Gu1[95] + Gx1[303]*Gu1[101] + Gx1[321]*Gu1[107];
Gu2[96] = + Gx1[16]*Gu1[0] + Gx1[34]*Gu1[6] + Gx1[52]*Gu1[12] + Gx1[70]*Gu1[18] + Gx1[88]*Gu1[24] + Gx1[106]*Gu1[30] + Gx1[124]*Gu1[36] + Gx1[142]*Gu1[42] + Gx1[160]*Gu1[48] + Gx1[178]*Gu1[54] + Gx1[196]*Gu1[60] + Gx1[214]*Gu1[66] + Gx1[232]*Gu1[72] + Gx1[250]*Gu1[78] + Gx1[268]*Gu1[84] + Gx1[286]*Gu1[90] + Gx1[304]*Gu1[96] + Gx1[322]*Gu1[102];
Gu2[97] = + Gx1[16]*Gu1[1] + Gx1[34]*Gu1[7] + Gx1[52]*Gu1[13] + Gx1[70]*Gu1[19] + Gx1[88]*Gu1[25] + Gx1[106]*Gu1[31] + Gx1[124]*Gu1[37] + Gx1[142]*Gu1[43] + Gx1[160]*Gu1[49] + Gx1[178]*Gu1[55] + Gx1[196]*Gu1[61] + Gx1[214]*Gu1[67] + Gx1[232]*Gu1[73] + Gx1[250]*Gu1[79] + Gx1[268]*Gu1[85] + Gx1[286]*Gu1[91] + Gx1[304]*Gu1[97] + Gx1[322]*Gu1[103];
Gu2[98] = + Gx1[16]*Gu1[2] + Gx1[34]*Gu1[8] + Gx1[52]*Gu1[14] + Gx1[70]*Gu1[20] + Gx1[88]*Gu1[26] + Gx1[106]*Gu1[32] + Gx1[124]*Gu1[38] + Gx1[142]*Gu1[44] + Gx1[160]*Gu1[50] + Gx1[178]*Gu1[56] + Gx1[196]*Gu1[62] + Gx1[214]*Gu1[68] + Gx1[232]*Gu1[74] + Gx1[250]*Gu1[80] + Gx1[268]*Gu1[86] + Gx1[286]*Gu1[92] + Gx1[304]*Gu1[98] + Gx1[322]*Gu1[104];
Gu2[99] = + Gx1[16]*Gu1[3] + Gx1[34]*Gu1[9] + Gx1[52]*Gu1[15] + Gx1[70]*Gu1[21] + Gx1[88]*Gu1[27] + Gx1[106]*Gu1[33] + Gx1[124]*Gu1[39] + Gx1[142]*Gu1[45] + Gx1[160]*Gu1[51] + Gx1[178]*Gu1[57] + Gx1[196]*Gu1[63] + Gx1[214]*Gu1[69] + Gx1[232]*Gu1[75] + Gx1[250]*Gu1[81] + Gx1[268]*Gu1[87] + Gx1[286]*Gu1[93] + Gx1[304]*Gu1[99] + Gx1[322]*Gu1[105];
Gu2[100] = + Gx1[16]*Gu1[4] + Gx1[34]*Gu1[10] + Gx1[52]*Gu1[16] + Gx1[70]*Gu1[22] + Gx1[88]*Gu1[28] + Gx1[106]*Gu1[34] + Gx1[124]*Gu1[40] + Gx1[142]*Gu1[46] + Gx1[160]*Gu1[52] + Gx1[178]*Gu1[58] + Gx1[196]*Gu1[64] + Gx1[214]*Gu1[70] + Gx1[232]*Gu1[76] + Gx1[250]*Gu1[82] + Gx1[268]*Gu1[88] + Gx1[286]*Gu1[94] + Gx1[304]*Gu1[100] + Gx1[322]*Gu1[106];
Gu2[101] = + Gx1[16]*Gu1[5] + Gx1[34]*Gu1[11] + Gx1[52]*Gu1[17] + Gx1[70]*Gu1[23] + Gx1[88]*Gu1[29] + Gx1[106]*Gu1[35] + Gx1[124]*Gu1[41] + Gx1[142]*Gu1[47] + Gx1[160]*Gu1[53] + Gx1[178]*Gu1[59] + Gx1[196]*Gu1[65] + Gx1[214]*Gu1[71] + Gx1[232]*Gu1[77] + Gx1[250]*Gu1[83] + Gx1[268]*Gu1[89] + Gx1[286]*Gu1[95] + Gx1[304]*Gu1[101] + Gx1[322]*Gu1[107];
Gu2[102] = + Gx1[17]*Gu1[0] + Gx1[35]*Gu1[6] + Gx1[53]*Gu1[12] + Gx1[71]*Gu1[18] + Gx1[89]*Gu1[24] + Gx1[107]*Gu1[30] + Gx1[125]*Gu1[36] + Gx1[143]*Gu1[42] + Gx1[161]*Gu1[48] + Gx1[179]*Gu1[54] + Gx1[197]*Gu1[60] + Gx1[215]*Gu1[66] + Gx1[233]*Gu1[72] + Gx1[251]*Gu1[78] + Gx1[269]*Gu1[84] + Gx1[287]*Gu1[90] + Gx1[305]*Gu1[96] + Gx1[323]*Gu1[102];
Gu2[103] = + Gx1[17]*Gu1[1] + Gx1[35]*Gu1[7] + Gx1[53]*Gu1[13] + Gx1[71]*Gu1[19] + Gx1[89]*Gu1[25] + Gx1[107]*Gu1[31] + Gx1[125]*Gu1[37] + Gx1[143]*Gu1[43] + Gx1[161]*Gu1[49] + Gx1[179]*Gu1[55] + Gx1[197]*Gu1[61] + Gx1[215]*Gu1[67] + Gx1[233]*Gu1[73] + Gx1[251]*Gu1[79] + Gx1[269]*Gu1[85] + Gx1[287]*Gu1[91] + Gx1[305]*Gu1[97] + Gx1[323]*Gu1[103];
Gu2[104] = + Gx1[17]*Gu1[2] + Gx1[35]*Gu1[8] + Gx1[53]*Gu1[14] + Gx1[71]*Gu1[20] + Gx1[89]*Gu1[26] + Gx1[107]*Gu1[32] + Gx1[125]*Gu1[38] + Gx1[143]*Gu1[44] + Gx1[161]*Gu1[50] + Gx1[179]*Gu1[56] + Gx1[197]*Gu1[62] + Gx1[215]*Gu1[68] + Gx1[233]*Gu1[74] + Gx1[251]*Gu1[80] + Gx1[269]*Gu1[86] + Gx1[287]*Gu1[92] + Gx1[305]*Gu1[98] + Gx1[323]*Gu1[104];
Gu2[105] = + Gx1[17]*Gu1[3] + Gx1[35]*Gu1[9] + Gx1[53]*Gu1[15] + Gx1[71]*Gu1[21] + Gx1[89]*Gu1[27] + Gx1[107]*Gu1[33] + Gx1[125]*Gu1[39] + Gx1[143]*Gu1[45] + Gx1[161]*Gu1[51] + Gx1[179]*Gu1[57] + Gx1[197]*Gu1[63] + Gx1[215]*Gu1[69] + Gx1[233]*Gu1[75] + Gx1[251]*Gu1[81] + Gx1[269]*Gu1[87] + Gx1[287]*Gu1[93] + Gx1[305]*Gu1[99] + Gx1[323]*Gu1[105];
Gu2[106] = + Gx1[17]*Gu1[4] + Gx1[35]*Gu1[10] + Gx1[53]*Gu1[16] + Gx1[71]*Gu1[22] + Gx1[89]*Gu1[28] + Gx1[107]*Gu1[34] + Gx1[125]*Gu1[40] + Gx1[143]*Gu1[46] + Gx1[161]*Gu1[52] + Gx1[179]*Gu1[58] + Gx1[197]*Gu1[64] + Gx1[215]*Gu1[70] + Gx1[233]*Gu1[76] + Gx1[251]*Gu1[82] + Gx1[269]*Gu1[88] + Gx1[287]*Gu1[94] + Gx1[305]*Gu1[100] + Gx1[323]*Gu1[106];
Gu2[107] = + Gx1[17]*Gu1[5] + Gx1[35]*Gu1[11] + Gx1[53]*Gu1[17] + Gx1[71]*Gu1[23] + Gx1[89]*Gu1[29] + Gx1[107]*Gu1[35] + Gx1[125]*Gu1[41] + Gx1[143]*Gu1[47] + Gx1[161]*Gu1[53] + Gx1[179]*Gu1[59] + Gx1[197]*Gu1[65] + Gx1[215]*Gu1[71] + Gx1[233]*Gu1[77] + Gx1[251]*Gu1[83] + Gx1[269]*Gu1[89] + Gx1[287]*Gu1[95] + Gx1[305]*Gu1[101] + Gx1[323]*Gu1[107];
}

void acado_multQEW2( real_t* const Q11, real_t* const Gu1, real_t* const Gu2, real_t* const Gu3 )
{
Gu3[0] = + Q11[0]*Gu1[0] + Q11[1]*Gu1[6] + Q11[2]*Gu1[12] + Q11[3]*Gu1[18] + Q11[4]*Gu1[24] + Q11[5]*Gu1[30] + Q11[6]*Gu1[36] + Q11[7]*Gu1[42] + Q11[8]*Gu1[48] + Q11[9]*Gu1[54] + Q11[10]*Gu1[60] + Q11[11]*Gu1[66] + Q11[12]*Gu1[72] + Q11[13]*Gu1[78] + Q11[14]*Gu1[84] + Q11[15]*Gu1[90] + Q11[16]*Gu1[96] + Q11[17]*Gu1[102] + Gu2[0];
Gu3[1] = + Q11[0]*Gu1[1] + Q11[1]*Gu1[7] + Q11[2]*Gu1[13] + Q11[3]*Gu1[19] + Q11[4]*Gu1[25] + Q11[5]*Gu1[31] + Q11[6]*Gu1[37] + Q11[7]*Gu1[43] + Q11[8]*Gu1[49] + Q11[9]*Gu1[55] + Q11[10]*Gu1[61] + Q11[11]*Gu1[67] + Q11[12]*Gu1[73] + Q11[13]*Gu1[79] + Q11[14]*Gu1[85] + Q11[15]*Gu1[91] + Q11[16]*Gu1[97] + Q11[17]*Gu1[103] + Gu2[1];
Gu3[2] = + Q11[0]*Gu1[2] + Q11[1]*Gu1[8] + Q11[2]*Gu1[14] + Q11[3]*Gu1[20] + Q11[4]*Gu1[26] + Q11[5]*Gu1[32] + Q11[6]*Gu1[38] + Q11[7]*Gu1[44] + Q11[8]*Gu1[50] + Q11[9]*Gu1[56] + Q11[10]*Gu1[62] + Q11[11]*Gu1[68] + Q11[12]*Gu1[74] + Q11[13]*Gu1[80] + Q11[14]*Gu1[86] + Q11[15]*Gu1[92] + Q11[16]*Gu1[98] + Q11[17]*Gu1[104] + Gu2[2];
Gu3[3] = + Q11[0]*Gu1[3] + Q11[1]*Gu1[9] + Q11[2]*Gu1[15] + Q11[3]*Gu1[21] + Q11[4]*Gu1[27] + Q11[5]*Gu1[33] + Q11[6]*Gu1[39] + Q11[7]*Gu1[45] + Q11[8]*Gu1[51] + Q11[9]*Gu1[57] + Q11[10]*Gu1[63] + Q11[11]*Gu1[69] + Q11[12]*Gu1[75] + Q11[13]*Gu1[81] + Q11[14]*Gu1[87] + Q11[15]*Gu1[93] + Q11[16]*Gu1[99] + Q11[17]*Gu1[105] + Gu2[3];
Gu3[4] = + Q11[0]*Gu1[4] + Q11[1]*Gu1[10] + Q11[2]*Gu1[16] + Q11[3]*Gu1[22] + Q11[4]*Gu1[28] + Q11[5]*Gu1[34] + Q11[6]*Gu1[40] + Q11[7]*Gu1[46] + Q11[8]*Gu1[52] + Q11[9]*Gu1[58] + Q11[10]*Gu1[64] + Q11[11]*Gu1[70] + Q11[12]*Gu1[76] + Q11[13]*Gu1[82] + Q11[14]*Gu1[88] + Q11[15]*Gu1[94] + Q11[16]*Gu1[100] + Q11[17]*Gu1[106] + Gu2[4];
Gu3[5] = + Q11[0]*Gu1[5] + Q11[1]*Gu1[11] + Q11[2]*Gu1[17] + Q11[3]*Gu1[23] + Q11[4]*Gu1[29] + Q11[5]*Gu1[35] + Q11[6]*Gu1[41] + Q11[7]*Gu1[47] + Q11[8]*Gu1[53] + Q11[9]*Gu1[59] + Q11[10]*Gu1[65] + Q11[11]*Gu1[71] + Q11[12]*Gu1[77] + Q11[13]*Gu1[83] + Q11[14]*Gu1[89] + Q11[15]*Gu1[95] + Q11[16]*Gu1[101] + Q11[17]*Gu1[107] + Gu2[5];
Gu3[6] = + Q11[18]*Gu1[0] + Q11[19]*Gu1[6] + Q11[20]*Gu1[12] + Q11[21]*Gu1[18] + Q11[22]*Gu1[24] + Q11[23]*Gu1[30] + Q11[24]*Gu1[36] + Q11[25]*Gu1[42] + Q11[26]*Gu1[48] + Q11[27]*Gu1[54] + Q11[28]*Gu1[60] + Q11[29]*Gu1[66] + Q11[30]*Gu1[72] + Q11[31]*Gu1[78] + Q11[32]*Gu1[84] + Q11[33]*Gu1[90] + Q11[34]*Gu1[96] + Q11[35]*Gu1[102] + Gu2[6];
Gu3[7] = + Q11[18]*Gu1[1] + Q11[19]*Gu1[7] + Q11[20]*Gu1[13] + Q11[21]*Gu1[19] + Q11[22]*Gu1[25] + Q11[23]*Gu1[31] + Q11[24]*Gu1[37] + Q11[25]*Gu1[43] + Q11[26]*Gu1[49] + Q11[27]*Gu1[55] + Q11[28]*Gu1[61] + Q11[29]*Gu1[67] + Q11[30]*Gu1[73] + Q11[31]*Gu1[79] + Q11[32]*Gu1[85] + Q11[33]*Gu1[91] + Q11[34]*Gu1[97] + Q11[35]*Gu1[103] + Gu2[7];
Gu3[8] = + Q11[18]*Gu1[2] + Q11[19]*Gu1[8] + Q11[20]*Gu1[14] + Q11[21]*Gu1[20] + Q11[22]*Gu1[26] + Q11[23]*Gu1[32] + Q11[24]*Gu1[38] + Q11[25]*Gu1[44] + Q11[26]*Gu1[50] + Q11[27]*Gu1[56] + Q11[28]*Gu1[62] + Q11[29]*Gu1[68] + Q11[30]*Gu1[74] + Q11[31]*Gu1[80] + Q11[32]*Gu1[86] + Q11[33]*Gu1[92] + Q11[34]*Gu1[98] + Q11[35]*Gu1[104] + Gu2[8];
Gu3[9] = + Q11[18]*Gu1[3] + Q11[19]*Gu1[9] + Q11[20]*Gu1[15] + Q11[21]*Gu1[21] + Q11[22]*Gu1[27] + Q11[23]*Gu1[33] + Q11[24]*Gu1[39] + Q11[25]*Gu1[45] + Q11[26]*Gu1[51] + Q11[27]*Gu1[57] + Q11[28]*Gu1[63] + Q11[29]*Gu1[69] + Q11[30]*Gu1[75] + Q11[31]*Gu1[81] + Q11[32]*Gu1[87] + Q11[33]*Gu1[93] + Q11[34]*Gu1[99] + Q11[35]*Gu1[105] + Gu2[9];
Gu3[10] = + Q11[18]*Gu1[4] + Q11[19]*Gu1[10] + Q11[20]*Gu1[16] + Q11[21]*Gu1[22] + Q11[22]*Gu1[28] + Q11[23]*Gu1[34] + Q11[24]*Gu1[40] + Q11[25]*Gu1[46] + Q11[26]*Gu1[52] + Q11[27]*Gu1[58] + Q11[28]*Gu1[64] + Q11[29]*Gu1[70] + Q11[30]*Gu1[76] + Q11[31]*Gu1[82] + Q11[32]*Gu1[88] + Q11[33]*Gu1[94] + Q11[34]*Gu1[100] + Q11[35]*Gu1[106] + Gu2[10];
Gu3[11] = + Q11[18]*Gu1[5] + Q11[19]*Gu1[11] + Q11[20]*Gu1[17] + Q11[21]*Gu1[23] + Q11[22]*Gu1[29] + Q11[23]*Gu1[35] + Q11[24]*Gu1[41] + Q11[25]*Gu1[47] + Q11[26]*Gu1[53] + Q11[27]*Gu1[59] + Q11[28]*Gu1[65] + Q11[29]*Gu1[71] + Q11[30]*Gu1[77] + Q11[31]*Gu1[83] + Q11[32]*Gu1[89] + Q11[33]*Gu1[95] + Q11[34]*Gu1[101] + Q11[35]*Gu1[107] + Gu2[11];
Gu3[12] = + Q11[36]*Gu1[0] + Q11[37]*Gu1[6] + Q11[38]*Gu1[12] + Q11[39]*Gu1[18] + Q11[40]*Gu1[24] + Q11[41]*Gu1[30] + Q11[42]*Gu1[36] + Q11[43]*Gu1[42] + Q11[44]*Gu1[48] + Q11[45]*Gu1[54] + Q11[46]*Gu1[60] + Q11[47]*Gu1[66] + Q11[48]*Gu1[72] + Q11[49]*Gu1[78] + Q11[50]*Gu1[84] + Q11[51]*Gu1[90] + Q11[52]*Gu1[96] + Q11[53]*Gu1[102] + Gu2[12];
Gu3[13] = + Q11[36]*Gu1[1] + Q11[37]*Gu1[7] + Q11[38]*Gu1[13] + Q11[39]*Gu1[19] + Q11[40]*Gu1[25] + Q11[41]*Gu1[31] + Q11[42]*Gu1[37] + Q11[43]*Gu1[43] + Q11[44]*Gu1[49] + Q11[45]*Gu1[55] + Q11[46]*Gu1[61] + Q11[47]*Gu1[67] + Q11[48]*Gu1[73] + Q11[49]*Gu1[79] + Q11[50]*Gu1[85] + Q11[51]*Gu1[91] + Q11[52]*Gu1[97] + Q11[53]*Gu1[103] + Gu2[13];
Gu3[14] = + Q11[36]*Gu1[2] + Q11[37]*Gu1[8] + Q11[38]*Gu1[14] + Q11[39]*Gu1[20] + Q11[40]*Gu1[26] + Q11[41]*Gu1[32] + Q11[42]*Gu1[38] + Q11[43]*Gu1[44] + Q11[44]*Gu1[50] + Q11[45]*Gu1[56] + Q11[46]*Gu1[62] + Q11[47]*Gu1[68] + Q11[48]*Gu1[74] + Q11[49]*Gu1[80] + Q11[50]*Gu1[86] + Q11[51]*Gu1[92] + Q11[52]*Gu1[98] + Q11[53]*Gu1[104] + Gu2[14];
Gu3[15] = + Q11[36]*Gu1[3] + Q11[37]*Gu1[9] + Q11[38]*Gu1[15] + Q11[39]*Gu1[21] + Q11[40]*Gu1[27] + Q11[41]*Gu1[33] + Q11[42]*Gu1[39] + Q11[43]*Gu1[45] + Q11[44]*Gu1[51] + Q11[45]*Gu1[57] + Q11[46]*Gu1[63] + Q11[47]*Gu1[69] + Q11[48]*Gu1[75] + Q11[49]*Gu1[81] + Q11[50]*Gu1[87] + Q11[51]*Gu1[93] + Q11[52]*Gu1[99] + Q11[53]*Gu1[105] + Gu2[15];
Gu3[16] = + Q11[36]*Gu1[4] + Q11[37]*Gu1[10] + Q11[38]*Gu1[16] + Q11[39]*Gu1[22] + Q11[40]*Gu1[28] + Q11[41]*Gu1[34] + Q11[42]*Gu1[40] + Q11[43]*Gu1[46] + Q11[44]*Gu1[52] + Q11[45]*Gu1[58] + Q11[46]*Gu1[64] + Q11[47]*Gu1[70] + Q11[48]*Gu1[76] + Q11[49]*Gu1[82] + Q11[50]*Gu1[88] + Q11[51]*Gu1[94] + Q11[52]*Gu1[100] + Q11[53]*Gu1[106] + Gu2[16];
Gu3[17] = + Q11[36]*Gu1[5] + Q11[37]*Gu1[11] + Q11[38]*Gu1[17] + Q11[39]*Gu1[23] + Q11[40]*Gu1[29] + Q11[41]*Gu1[35] + Q11[42]*Gu1[41] + Q11[43]*Gu1[47] + Q11[44]*Gu1[53] + Q11[45]*Gu1[59] + Q11[46]*Gu1[65] + Q11[47]*Gu1[71] + Q11[48]*Gu1[77] + Q11[49]*Gu1[83] + Q11[50]*Gu1[89] + Q11[51]*Gu1[95] + Q11[52]*Gu1[101] + Q11[53]*Gu1[107] + Gu2[17];
Gu3[18] = + Q11[54]*Gu1[0] + Q11[55]*Gu1[6] + Q11[56]*Gu1[12] + Q11[57]*Gu1[18] + Q11[58]*Gu1[24] + Q11[59]*Gu1[30] + Q11[60]*Gu1[36] + Q11[61]*Gu1[42] + Q11[62]*Gu1[48] + Q11[63]*Gu1[54] + Q11[64]*Gu1[60] + Q11[65]*Gu1[66] + Q11[66]*Gu1[72] + Q11[67]*Gu1[78] + Q11[68]*Gu1[84] + Q11[69]*Gu1[90] + Q11[70]*Gu1[96] + Q11[71]*Gu1[102] + Gu2[18];
Gu3[19] = + Q11[54]*Gu1[1] + Q11[55]*Gu1[7] + Q11[56]*Gu1[13] + Q11[57]*Gu1[19] + Q11[58]*Gu1[25] + Q11[59]*Gu1[31] + Q11[60]*Gu1[37] + Q11[61]*Gu1[43] + Q11[62]*Gu1[49] + Q11[63]*Gu1[55] + Q11[64]*Gu1[61] + Q11[65]*Gu1[67] + Q11[66]*Gu1[73] + Q11[67]*Gu1[79] + Q11[68]*Gu1[85] + Q11[69]*Gu1[91] + Q11[70]*Gu1[97] + Q11[71]*Gu1[103] + Gu2[19];
Gu3[20] = + Q11[54]*Gu1[2] + Q11[55]*Gu1[8] + Q11[56]*Gu1[14] + Q11[57]*Gu1[20] + Q11[58]*Gu1[26] + Q11[59]*Gu1[32] + Q11[60]*Gu1[38] + Q11[61]*Gu1[44] + Q11[62]*Gu1[50] + Q11[63]*Gu1[56] + Q11[64]*Gu1[62] + Q11[65]*Gu1[68] + Q11[66]*Gu1[74] + Q11[67]*Gu1[80] + Q11[68]*Gu1[86] + Q11[69]*Gu1[92] + Q11[70]*Gu1[98] + Q11[71]*Gu1[104] + Gu2[20];
Gu3[21] = + Q11[54]*Gu1[3] + Q11[55]*Gu1[9] + Q11[56]*Gu1[15] + Q11[57]*Gu1[21] + Q11[58]*Gu1[27] + Q11[59]*Gu1[33] + Q11[60]*Gu1[39] + Q11[61]*Gu1[45] + Q11[62]*Gu1[51] + Q11[63]*Gu1[57] + Q11[64]*Gu1[63] + Q11[65]*Gu1[69] + Q11[66]*Gu1[75] + Q11[67]*Gu1[81] + Q11[68]*Gu1[87] + Q11[69]*Gu1[93] + Q11[70]*Gu1[99] + Q11[71]*Gu1[105] + Gu2[21];
Gu3[22] = + Q11[54]*Gu1[4] + Q11[55]*Gu1[10] + Q11[56]*Gu1[16] + Q11[57]*Gu1[22] + Q11[58]*Gu1[28] + Q11[59]*Gu1[34] + Q11[60]*Gu1[40] + Q11[61]*Gu1[46] + Q11[62]*Gu1[52] + Q11[63]*Gu1[58] + Q11[64]*Gu1[64] + Q11[65]*Gu1[70] + Q11[66]*Gu1[76] + Q11[67]*Gu1[82] + Q11[68]*Gu1[88] + Q11[69]*Gu1[94] + Q11[70]*Gu1[100] + Q11[71]*Gu1[106] + Gu2[22];
Gu3[23] = + Q11[54]*Gu1[5] + Q11[55]*Gu1[11] + Q11[56]*Gu1[17] + Q11[57]*Gu1[23] + Q11[58]*Gu1[29] + Q11[59]*Gu1[35] + Q11[60]*Gu1[41] + Q11[61]*Gu1[47] + Q11[62]*Gu1[53] + Q11[63]*Gu1[59] + Q11[64]*Gu1[65] + Q11[65]*Gu1[71] + Q11[66]*Gu1[77] + Q11[67]*Gu1[83] + Q11[68]*Gu1[89] + Q11[69]*Gu1[95] + Q11[70]*Gu1[101] + Q11[71]*Gu1[107] + Gu2[23];
Gu3[24] = + Q11[72]*Gu1[0] + Q11[73]*Gu1[6] + Q11[74]*Gu1[12] + Q11[75]*Gu1[18] + Q11[76]*Gu1[24] + Q11[77]*Gu1[30] + Q11[78]*Gu1[36] + Q11[79]*Gu1[42] + Q11[80]*Gu1[48] + Q11[81]*Gu1[54] + Q11[82]*Gu1[60] + Q11[83]*Gu1[66] + Q11[84]*Gu1[72] + Q11[85]*Gu1[78] + Q11[86]*Gu1[84] + Q11[87]*Gu1[90] + Q11[88]*Gu1[96] + Q11[89]*Gu1[102] + Gu2[24];
Gu3[25] = + Q11[72]*Gu1[1] + Q11[73]*Gu1[7] + Q11[74]*Gu1[13] + Q11[75]*Gu1[19] + Q11[76]*Gu1[25] + Q11[77]*Gu1[31] + Q11[78]*Gu1[37] + Q11[79]*Gu1[43] + Q11[80]*Gu1[49] + Q11[81]*Gu1[55] + Q11[82]*Gu1[61] + Q11[83]*Gu1[67] + Q11[84]*Gu1[73] + Q11[85]*Gu1[79] + Q11[86]*Gu1[85] + Q11[87]*Gu1[91] + Q11[88]*Gu1[97] + Q11[89]*Gu1[103] + Gu2[25];
Gu3[26] = + Q11[72]*Gu1[2] + Q11[73]*Gu1[8] + Q11[74]*Gu1[14] + Q11[75]*Gu1[20] + Q11[76]*Gu1[26] + Q11[77]*Gu1[32] + Q11[78]*Gu1[38] + Q11[79]*Gu1[44] + Q11[80]*Gu1[50] + Q11[81]*Gu1[56] + Q11[82]*Gu1[62] + Q11[83]*Gu1[68] + Q11[84]*Gu1[74] + Q11[85]*Gu1[80] + Q11[86]*Gu1[86] + Q11[87]*Gu1[92] + Q11[88]*Gu1[98] + Q11[89]*Gu1[104] + Gu2[26];
Gu3[27] = + Q11[72]*Gu1[3] + Q11[73]*Gu1[9] + Q11[74]*Gu1[15] + Q11[75]*Gu1[21] + Q11[76]*Gu1[27] + Q11[77]*Gu1[33] + Q11[78]*Gu1[39] + Q11[79]*Gu1[45] + Q11[80]*Gu1[51] + Q11[81]*Gu1[57] + Q11[82]*Gu1[63] + Q11[83]*Gu1[69] + Q11[84]*Gu1[75] + Q11[85]*Gu1[81] + Q11[86]*Gu1[87] + Q11[87]*Gu1[93] + Q11[88]*Gu1[99] + Q11[89]*Gu1[105] + Gu2[27];
Gu3[28] = + Q11[72]*Gu1[4] + Q11[73]*Gu1[10] + Q11[74]*Gu1[16] + Q11[75]*Gu1[22] + Q11[76]*Gu1[28] + Q11[77]*Gu1[34] + Q11[78]*Gu1[40] + Q11[79]*Gu1[46] + Q11[80]*Gu1[52] + Q11[81]*Gu1[58] + Q11[82]*Gu1[64] + Q11[83]*Gu1[70] + Q11[84]*Gu1[76] + Q11[85]*Gu1[82] + Q11[86]*Gu1[88] + Q11[87]*Gu1[94] + Q11[88]*Gu1[100] + Q11[89]*Gu1[106] + Gu2[28];
Gu3[29] = + Q11[72]*Gu1[5] + Q11[73]*Gu1[11] + Q11[74]*Gu1[17] + Q11[75]*Gu1[23] + Q11[76]*Gu1[29] + Q11[77]*Gu1[35] + Q11[78]*Gu1[41] + Q11[79]*Gu1[47] + Q11[80]*Gu1[53] + Q11[81]*Gu1[59] + Q11[82]*Gu1[65] + Q11[83]*Gu1[71] + Q11[84]*Gu1[77] + Q11[85]*Gu1[83] + Q11[86]*Gu1[89] + Q11[87]*Gu1[95] + Q11[88]*Gu1[101] + Q11[89]*Gu1[107] + Gu2[29];
Gu3[30] = + Q11[90]*Gu1[0] + Q11[91]*Gu1[6] + Q11[92]*Gu1[12] + Q11[93]*Gu1[18] + Q11[94]*Gu1[24] + Q11[95]*Gu1[30] + Q11[96]*Gu1[36] + Q11[97]*Gu1[42] + Q11[98]*Gu1[48] + Q11[99]*Gu1[54] + Q11[100]*Gu1[60] + Q11[101]*Gu1[66] + Q11[102]*Gu1[72] + Q11[103]*Gu1[78] + Q11[104]*Gu1[84] + Q11[105]*Gu1[90] + Q11[106]*Gu1[96] + Q11[107]*Gu1[102] + Gu2[30];
Gu3[31] = + Q11[90]*Gu1[1] + Q11[91]*Gu1[7] + Q11[92]*Gu1[13] + Q11[93]*Gu1[19] + Q11[94]*Gu1[25] + Q11[95]*Gu1[31] + Q11[96]*Gu1[37] + Q11[97]*Gu1[43] + Q11[98]*Gu1[49] + Q11[99]*Gu1[55] + Q11[100]*Gu1[61] + Q11[101]*Gu1[67] + Q11[102]*Gu1[73] + Q11[103]*Gu1[79] + Q11[104]*Gu1[85] + Q11[105]*Gu1[91] + Q11[106]*Gu1[97] + Q11[107]*Gu1[103] + Gu2[31];
Gu3[32] = + Q11[90]*Gu1[2] + Q11[91]*Gu1[8] + Q11[92]*Gu1[14] + Q11[93]*Gu1[20] + Q11[94]*Gu1[26] + Q11[95]*Gu1[32] + Q11[96]*Gu1[38] + Q11[97]*Gu1[44] + Q11[98]*Gu1[50] + Q11[99]*Gu1[56] + Q11[100]*Gu1[62] + Q11[101]*Gu1[68] + Q11[102]*Gu1[74] + Q11[103]*Gu1[80] + Q11[104]*Gu1[86] + Q11[105]*Gu1[92] + Q11[106]*Gu1[98] + Q11[107]*Gu1[104] + Gu2[32];
Gu3[33] = + Q11[90]*Gu1[3] + Q11[91]*Gu1[9] + Q11[92]*Gu1[15] + Q11[93]*Gu1[21] + Q11[94]*Gu1[27] + Q11[95]*Gu1[33] + Q11[96]*Gu1[39] + Q11[97]*Gu1[45] + Q11[98]*Gu1[51] + Q11[99]*Gu1[57] + Q11[100]*Gu1[63] + Q11[101]*Gu1[69] + Q11[102]*Gu1[75] + Q11[103]*Gu1[81] + Q11[104]*Gu1[87] + Q11[105]*Gu1[93] + Q11[106]*Gu1[99] + Q11[107]*Gu1[105] + Gu2[33];
Gu3[34] = + Q11[90]*Gu1[4] + Q11[91]*Gu1[10] + Q11[92]*Gu1[16] + Q11[93]*Gu1[22] + Q11[94]*Gu1[28] + Q11[95]*Gu1[34] + Q11[96]*Gu1[40] + Q11[97]*Gu1[46] + Q11[98]*Gu1[52] + Q11[99]*Gu1[58] + Q11[100]*Gu1[64] + Q11[101]*Gu1[70] + Q11[102]*Gu1[76] + Q11[103]*Gu1[82] + Q11[104]*Gu1[88] + Q11[105]*Gu1[94] + Q11[106]*Gu1[100] + Q11[107]*Gu1[106] + Gu2[34];
Gu3[35] = + Q11[90]*Gu1[5] + Q11[91]*Gu1[11] + Q11[92]*Gu1[17] + Q11[93]*Gu1[23] + Q11[94]*Gu1[29] + Q11[95]*Gu1[35] + Q11[96]*Gu1[41] + Q11[97]*Gu1[47] + Q11[98]*Gu1[53] + Q11[99]*Gu1[59] + Q11[100]*Gu1[65] + Q11[101]*Gu1[71] + Q11[102]*Gu1[77] + Q11[103]*Gu1[83] + Q11[104]*Gu1[89] + Q11[105]*Gu1[95] + Q11[106]*Gu1[101] + Q11[107]*Gu1[107] + Gu2[35];
Gu3[36] = + Q11[108]*Gu1[0] + Q11[109]*Gu1[6] + Q11[110]*Gu1[12] + Q11[111]*Gu1[18] + Q11[112]*Gu1[24] + Q11[113]*Gu1[30] + Q11[114]*Gu1[36] + Q11[115]*Gu1[42] + Q11[116]*Gu1[48] + Q11[117]*Gu1[54] + Q11[118]*Gu1[60] + Q11[119]*Gu1[66] + Q11[120]*Gu1[72] + Q11[121]*Gu1[78] + Q11[122]*Gu1[84] + Q11[123]*Gu1[90] + Q11[124]*Gu1[96] + Q11[125]*Gu1[102] + Gu2[36];
Gu3[37] = + Q11[108]*Gu1[1] + Q11[109]*Gu1[7] + Q11[110]*Gu1[13] + Q11[111]*Gu1[19] + Q11[112]*Gu1[25] + Q11[113]*Gu1[31] + Q11[114]*Gu1[37] + Q11[115]*Gu1[43] + Q11[116]*Gu1[49] + Q11[117]*Gu1[55] + Q11[118]*Gu1[61] + Q11[119]*Gu1[67] + Q11[120]*Gu1[73] + Q11[121]*Gu1[79] + Q11[122]*Gu1[85] + Q11[123]*Gu1[91] + Q11[124]*Gu1[97] + Q11[125]*Gu1[103] + Gu2[37];
Gu3[38] = + Q11[108]*Gu1[2] + Q11[109]*Gu1[8] + Q11[110]*Gu1[14] + Q11[111]*Gu1[20] + Q11[112]*Gu1[26] + Q11[113]*Gu1[32] + Q11[114]*Gu1[38] + Q11[115]*Gu1[44] + Q11[116]*Gu1[50] + Q11[117]*Gu1[56] + Q11[118]*Gu1[62] + Q11[119]*Gu1[68] + Q11[120]*Gu1[74] + Q11[121]*Gu1[80] + Q11[122]*Gu1[86] + Q11[123]*Gu1[92] + Q11[124]*Gu1[98] + Q11[125]*Gu1[104] + Gu2[38];
Gu3[39] = + Q11[108]*Gu1[3] + Q11[109]*Gu1[9] + Q11[110]*Gu1[15] + Q11[111]*Gu1[21] + Q11[112]*Gu1[27] + Q11[113]*Gu1[33] + Q11[114]*Gu1[39] + Q11[115]*Gu1[45] + Q11[116]*Gu1[51] + Q11[117]*Gu1[57] + Q11[118]*Gu1[63] + Q11[119]*Gu1[69] + Q11[120]*Gu1[75] + Q11[121]*Gu1[81] + Q11[122]*Gu1[87] + Q11[123]*Gu1[93] + Q11[124]*Gu1[99] + Q11[125]*Gu1[105] + Gu2[39];
Gu3[40] = + Q11[108]*Gu1[4] + Q11[109]*Gu1[10] + Q11[110]*Gu1[16] + Q11[111]*Gu1[22] + Q11[112]*Gu1[28] + Q11[113]*Gu1[34] + Q11[114]*Gu1[40] + Q11[115]*Gu1[46] + Q11[116]*Gu1[52] + Q11[117]*Gu1[58] + Q11[118]*Gu1[64] + Q11[119]*Gu1[70] + Q11[120]*Gu1[76] + Q11[121]*Gu1[82] + Q11[122]*Gu1[88] + Q11[123]*Gu1[94] + Q11[124]*Gu1[100] + Q11[125]*Gu1[106] + Gu2[40];
Gu3[41] = + Q11[108]*Gu1[5] + Q11[109]*Gu1[11] + Q11[110]*Gu1[17] + Q11[111]*Gu1[23] + Q11[112]*Gu1[29] + Q11[113]*Gu1[35] + Q11[114]*Gu1[41] + Q11[115]*Gu1[47] + Q11[116]*Gu1[53] + Q11[117]*Gu1[59] + Q11[118]*Gu1[65] + Q11[119]*Gu1[71] + Q11[120]*Gu1[77] + Q11[121]*Gu1[83] + Q11[122]*Gu1[89] + Q11[123]*Gu1[95] + Q11[124]*Gu1[101] + Q11[125]*Gu1[107] + Gu2[41];
Gu3[42] = + Q11[126]*Gu1[0] + Q11[127]*Gu1[6] + Q11[128]*Gu1[12] + Q11[129]*Gu1[18] + Q11[130]*Gu1[24] + Q11[131]*Gu1[30] + Q11[132]*Gu1[36] + Q11[133]*Gu1[42] + Q11[134]*Gu1[48] + Q11[135]*Gu1[54] + Q11[136]*Gu1[60] + Q11[137]*Gu1[66] + Q11[138]*Gu1[72] + Q11[139]*Gu1[78] + Q11[140]*Gu1[84] + Q11[141]*Gu1[90] + Q11[142]*Gu1[96] + Q11[143]*Gu1[102] + Gu2[42];
Gu3[43] = + Q11[126]*Gu1[1] + Q11[127]*Gu1[7] + Q11[128]*Gu1[13] + Q11[129]*Gu1[19] + Q11[130]*Gu1[25] + Q11[131]*Gu1[31] + Q11[132]*Gu1[37] + Q11[133]*Gu1[43] + Q11[134]*Gu1[49] + Q11[135]*Gu1[55] + Q11[136]*Gu1[61] + Q11[137]*Gu1[67] + Q11[138]*Gu1[73] + Q11[139]*Gu1[79] + Q11[140]*Gu1[85] + Q11[141]*Gu1[91] + Q11[142]*Gu1[97] + Q11[143]*Gu1[103] + Gu2[43];
Gu3[44] = + Q11[126]*Gu1[2] + Q11[127]*Gu1[8] + Q11[128]*Gu1[14] + Q11[129]*Gu1[20] + Q11[130]*Gu1[26] + Q11[131]*Gu1[32] + Q11[132]*Gu1[38] + Q11[133]*Gu1[44] + Q11[134]*Gu1[50] + Q11[135]*Gu1[56] + Q11[136]*Gu1[62] + Q11[137]*Gu1[68] + Q11[138]*Gu1[74] + Q11[139]*Gu1[80] + Q11[140]*Gu1[86] + Q11[141]*Gu1[92] + Q11[142]*Gu1[98] + Q11[143]*Gu1[104] + Gu2[44];
Gu3[45] = + Q11[126]*Gu1[3] + Q11[127]*Gu1[9] + Q11[128]*Gu1[15] + Q11[129]*Gu1[21] + Q11[130]*Gu1[27] + Q11[131]*Gu1[33] + Q11[132]*Gu1[39] + Q11[133]*Gu1[45] + Q11[134]*Gu1[51] + Q11[135]*Gu1[57] + Q11[136]*Gu1[63] + Q11[137]*Gu1[69] + Q11[138]*Gu1[75] + Q11[139]*Gu1[81] + Q11[140]*Gu1[87] + Q11[141]*Gu1[93] + Q11[142]*Gu1[99] + Q11[143]*Gu1[105] + Gu2[45];
Gu3[46] = + Q11[126]*Gu1[4] + Q11[127]*Gu1[10] + Q11[128]*Gu1[16] + Q11[129]*Gu1[22] + Q11[130]*Gu1[28] + Q11[131]*Gu1[34] + Q11[132]*Gu1[40] + Q11[133]*Gu1[46] + Q11[134]*Gu1[52] + Q11[135]*Gu1[58] + Q11[136]*Gu1[64] + Q11[137]*Gu1[70] + Q11[138]*Gu1[76] + Q11[139]*Gu1[82] + Q11[140]*Gu1[88] + Q11[141]*Gu1[94] + Q11[142]*Gu1[100] + Q11[143]*Gu1[106] + Gu2[46];
Gu3[47] = + Q11[126]*Gu1[5] + Q11[127]*Gu1[11] + Q11[128]*Gu1[17] + Q11[129]*Gu1[23] + Q11[130]*Gu1[29] + Q11[131]*Gu1[35] + Q11[132]*Gu1[41] + Q11[133]*Gu1[47] + Q11[134]*Gu1[53] + Q11[135]*Gu1[59] + Q11[136]*Gu1[65] + Q11[137]*Gu1[71] + Q11[138]*Gu1[77] + Q11[139]*Gu1[83] + Q11[140]*Gu1[89] + Q11[141]*Gu1[95] + Q11[142]*Gu1[101] + Q11[143]*Gu1[107] + Gu2[47];
Gu3[48] = + Q11[144]*Gu1[0] + Q11[145]*Gu1[6] + Q11[146]*Gu1[12] + Q11[147]*Gu1[18] + Q11[148]*Gu1[24] + Q11[149]*Gu1[30] + Q11[150]*Gu1[36] + Q11[151]*Gu1[42] + Q11[152]*Gu1[48] + Q11[153]*Gu1[54] + Q11[154]*Gu1[60] + Q11[155]*Gu1[66] + Q11[156]*Gu1[72] + Q11[157]*Gu1[78] + Q11[158]*Gu1[84] + Q11[159]*Gu1[90] + Q11[160]*Gu1[96] + Q11[161]*Gu1[102] + Gu2[48];
Gu3[49] = + Q11[144]*Gu1[1] + Q11[145]*Gu1[7] + Q11[146]*Gu1[13] + Q11[147]*Gu1[19] + Q11[148]*Gu1[25] + Q11[149]*Gu1[31] + Q11[150]*Gu1[37] + Q11[151]*Gu1[43] + Q11[152]*Gu1[49] + Q11[153]*Gu1[55] + Q11[154]*Gu1[61] + Q11[155]*Gu1[67] + Q11[156]*Gu1[73] + Q11[157]*Gu1[79] + Q11[158]*Gu1[85] + Q11[159]*Gu1[91] + Q11[160]*Gu1[97] + Q11[161]*Gu1[103] + Gu2[49];
Gu3[50] = + Q11[144]*Gu1[2] + Q11[145]*Gu1[8] + Q11[146]*Gu1[14] + Q11[147]*Gu1[20] + Q11[148]*Gu1[26] + Q11[149]*Gu1[32] + Q11[150]*Gu1[38] + Q11[151]*Gu1[44] + Q11[152]*Gu1[50] + Q11[153]*Gu1[56] + Q11[154]*Gu1[62] + Q11[155]*Gu1[68] + Q11[156]*Gu1[74] + Q11[157]*Gu1[80] + Q11[158]*Gu1[86] + Q11[159]*Gu1[92] + Q11[160]*Gu1[98] + Q11[161]*Gu1[104] + Gu2[50];
Gu3[51] = + Q11[144]*Gu1[3] + Q11[145]*Gu1[9] + Q11[146]*Gu1[15] + Q11[147]*Gu1[21] + Q11[148]*Gu1[27] + Q11[149]*Gu1[33] + Q11[150]*Gu1[39] + Q11[151]*Gu1[45] + Q11[152]*Gu1[51] + Q11[153]*Gu1[57] + Q11[154]*Gu1[63] + Q11[155]*Gu1[69] + Q11[156]*Gu1[75] + Q11[157]*Gu1[81] + Q11[158]*Gu1[87] + Q11[159]*Gu1[93] + Q11[160]*Gu1[99] + Q11[161]*Gu1[105] + Gu2[51];
Gu3[52] = + Q11[144]*Gu1[4] + Q11[145]*Gu1[10] + Q11[146]*Gu1[16] + Q11[147]*Gu1[22] + Q11[148]*Gu1[28] + Q11[149]*Gu1[34] + Q11[150]*Gu1[40] + Q11[151]*Gu1[46] + Q11[152]*Gu1[52] + Q11[153]*Gu1[58] + Q11[154]*Gu1[64] + Q11[155]*Gu1[70] + Q11[156]*Gu1[76] + Q11[157]*Gu1[82] + Q11[158]*Gu1[88] + Q11[159]*Gu1[94] + Q11[160]*Gu1[100] + Q11[161]*Gu1[106] + Gu2[52];
Gu3[53] = + Q11[144]*Gu1[5] + Q11[145]*Gu1[11] + Q11[146]*Gu1[17] + Q11[147]*Gu1[23] + Q11[148]*Gu1[29] + Q11[149]*Gu1[35] + Q11[150]*Gu1[41] + Q11[151]*Gu1[47] + Q11[152]*Gu1[53] + Q11[153]*Gu1[59] + Q11[154]*Gu1[65] + Q11[155]*Gu1[71] + Q11[156]*Gu1[77] + Q11[157]*Gu1[83] + Q11[158]*Gu1[89] + Q11[159]*Gu1[95] + Q11[160]*Gu1[101] + Q11[161]*Gu1[107] + Gu2[53];
Gu3[54] = + Q11[162]*Gu1[0] + Q11[163]*Gu1[6] + Q11[164]*Gu1[12] + Q11[165]*Gu1[18] + Q11[166]*Gu1[24] + Q11[167]*Gu1[30] + Q11[168]*Gu1[36] + Q11[169]*Gu1[42] + Q11[170]*Gu1[48] + Q11[171]*Gu1[54] + Q11[172]*Gu1[60] + Q11[173]*Gu1[66] + Q11[174]*Gu1[72] + Q11[175]*Gu1[78] + Q11[176]*Gu1[84] + Q11[177]*Gu1[90] + Q11[178]*Gu1[96] + Q11[179]*Gu1[102] + Gu2[54];
Gu3[55] = + Q11[162]*Gu1[1] + Q11[163]*Gu1[7] + Q11[164]*Gu1[13] + Q11[165]*Gu1[19] + Q11[166]*Gu1[25] + Q11[167]*Gu1[31] + Q11[168]*Gu1[37] + Q11[169]*Gu1[43] + Q11[170]*Gu1[49] + Q11[171]*Gu1[55] + Q11[172]*Gu1[61] + Q11[173]*Gu1[67] + Q11[174]*Gu1[73] + Q11[175]*Gu1[79] + Q11[176]*Gu1[85] + Q11[177]*Gu1[91] + Q11[178]*Gu1[97] + Q11[179]*Gu1[103] + Gu2[55];
Gu3[56] = + Q11[162]*Gu1[2] + Q11[163]*Gu1[8] + Q11[164]*Gu1[14] + Q11[165]*Gu1[20] + Q11[166]*Gu1[26] + Q11[167]*Gu1[32] + Q11[168]*Gu1[38] + Q11[169]*Gu1[44] + Q11[170]*Gu1[50] + Q11[171]*Gu1[56] + Q11[172]*Gu1[62] + Q11[173]*Gu1[68] + Q11[174]*Gu1[74] + Q11[175]*Gu1[80] + Q11[176]*Gu1[86] + Q11[177]*Gu1[92] + Q11[178]*Gu1[98] + Q11[179]*Gu1[104] + Gu2[56];
Gu3[57] = + Q11[162]*Gu1[3] + Q11[163]*Gu1[9] + Q11[164]*Gu1[15] + Q11[165]*Gu1[21] + Q11[166]*Gu1[27] + Q11[167]*Gu1[33] + Q11[168]*Gu1[39] + Q11[169]*Gu1[45] + Q11[170]*Gu1[51] + Q11[171]*Gu1[57] + Q11[172]*Gu1[63] + Q11[173]*Gu1[69] + Q11[174]*Gu1[75] + Q11[175]*Gu1[81] + Q11[176]*Gu1[87] + Q11[177]*Gu1[93] + Q11[178]*Gu1[99] + Q11[179]*Gu1[105] + Gu2[57];
Gu3[58] = + Q11[162]*Gu1[4] + Q11[163]*Gu1[10] + Q11[164]*Gu1[16] + Q11[165]*Gu1[22] + Q11[166]*Gu1[28] + Q11[167]*Gu1[34] + Q11[168]*Gu1[40] + Q11[169]*Gu1[46] + Q11[170]*Gu1[52] + Q11[171]*Gu1[58] + Q11[172]*Gu1[64] + Q11[173]*Gu1[70] + Q11[174]*Gu1[76] + Q11[175]*Gu1[82] + Q11[176]*Gu1[88] + Q11[177]*Gu1[94] + Q11[178]*Gu1[100] + Q11[179]*Gu1[106] + Gu2[58];
Gu3[59] = + Q11[162]*Gu1[5] + Q11[163]*Gu1[11] + Q11[164]*Gu1[17] + Q11[165]*Gu1[23] + Q11[166]*Gu1[29] + Q11[167]*Gu1[35] + Q11[168]*Gu1[41] + Q11[169]*Gu1[47] + Q11[170]*Gu1[53] + Q11[171]*Gu1[59] + Q11[172]*Gu1[65] + Q11[173]*Gu1[71] + Q11[174]*Gu1[77] + Q11[175]*Gu1[83] + Q11[176]*Gu1[89] + Q11[177]*Gu1[95] + Q11[178]*Gu1[101] + Q11[179]*Gu1[107] + Gu2[59];
Gu3[60] = + Q11[180]*Gu1[0] + Q11[181]*Gu1[6] + Q11[182]*Gu1[12] + Q11[183]*Gu1[18] + Q11[184]*Gu1[24] + Q11[185]*Gu1[30] + Q11[186]*Gu1[36] + Q11[187]*Gu1[42] + Q11[188]*Gu1[48] + Q11[189]*Gu1[54] + Q11[190]*Gu1[60] + Q11[191]*Gu1[66] + Q11[192]*Gu1[72] + Q11[193]*Gu1[78] + Q11[194]*Gu1[84] + Q11[195]*Gu1[90] + Q11[196]*Gu1[96] + Q11[197]*Gu1[102] + Gu2[60];
Gu3[61] = + Q11[180]*Gu1[1] + Q11[181]*Gu1[7] + Q11[182]*Gu1[13] + Q11[183]*Gu1[19] + Q11[184]*Gu1[25] + Q11[185]*Gu1[31] + Q11[186]*Gu1[37] + Q11[187]*Gu1[43] + Q11[188]*Gu1[49] + Q11[189]*Gu1[55] + Q11[190]*Gu1[61] + Q11[191]*Gu1[67] + Q11[192]*Gu1[73] + Q11[193]*Gu1[79] + Q11[194]*Gu1[85] + Q11[195]*Gu1[91] + Q11[196]*Gu1[97] + Q11[197]*Gu1[103] + Gu2[61];
Gu3[62] = + Q11[180]*Gu1[2] + Q11[181]*Gu1[8] + Q11[182]*Gu1[14] + Q11[183]*Gu1[20] + Q11[184]*Gu1[26] + Q11[185]*Gu1[32] + Q11[186]*Gu1[38] + Q11[187]*Gu1[44] + Q11[188]*Gu1[50] + Q11[189]*Gu1[56] + Q11[190]*Gu1[62] + Q11[191]*Gu1[68] + Q11[192]*Gu1[74] + Q11[193]*Gu1[80] + Q11[194]*Gu1[86] + Q11[195]*Gu1[92] + Q11[196]*Gu1[98] + Q11[197]*Gu1[104] + Gu2[62];
Gu3[63] = + Q11[180]*Gu1[3] + Q11[181]*Gu1[9] + Q11[182]*Gu1[15] + Q11[183]*Gu1[21] + Q11[184]*Gu1[27] + Q11[185]*Gu1[33] + Q11[186]*Gu1[39] + Q11[187]*Gu1[45] + Q11[188]*Gu1[51] + Q11[189]*Gu1[57] + Q11[190]*Gu1[63] + Q11[191]*Gu1[69] + Q11[192]*Gu1[75] + Q11[193]*Gu1[81] + Q11[194]*Gu1[87] + Q11[195]*Gu1[93] + Q11[196]*Gu1[99] + Q11[197]*Gu1[105] + Gu2[63];
Gu3[64] = + Q11[180]*Gu1[4] + Q11[181]*Gu1[10] + Q11[182]*Gu1[16] + Q11[183]*Gu1[22] + Q11[184]*Gu1[28] + Q11[185]*Gu1[34] + Q11[186]*Gu1[40] + Q11[187]*Gu1[46] + Q11[188]*Gu1[52] + Q11[189]*Gu1[58] + Q11[190]*Gu1[64] + Q11[191]*Gu1[70] + Q11[192]*Gu1[76] + Q11[193]*Gu1[82] + Q11[194]*Gu1[88] + Q11[195]*Gu1[94] + Q11[196]*Gu1[100] + Q11[197]*Gu1[106] + Gu2[64];
Gu3[65] = + Q11[180]*Gu1[5] + Q11[181]*Gu1[11] + Q11[182]*Gu1[17] + Q11[183]*Gu1[23] + Q11[184]*Gu1[29] + Q11[185]*Gu1[35] + Q11[186]*Gu1[41] + Q11[187]*Gu1[47] + Q11[188]*Gu1[53] + Q11[189]*Gu1[59] + Q11[190]*Gu1[65] + Q11[191]*Gu1[71] + Q11[192]*Gu1[77] + Q11[193]*Gu1[83] + Q11[194]*Gu1[89] + Q11[195]*Gu1[95] + Q11[196]*Gu1[101] + Q11[197]*Gu1[107] + Gu2[65];
Gu3[66] = + Q11[198]*Gu1[0] + Q11[199]*Gu1[6] + Q11[200]*Gu1[12] + Q11[201]*Gu1[18] + Q11[202]*Gu1[24] + Q11[203]*Gu1[30] + Q11[204]*Gu1[36] + Q11[205]*Gu1[42] + Q11[206]*Gu1[48] + Q11[207]*Gu1[54] + Q11[208]*Gu1[60] + Q11[209]*Gu1[66] + Q11[210]*Gu1[72] + Q11[211]*Gu1[78] + Q11[212]*Gu1[84] + Q11[213]*Gu1[90] + Q11[214]*Gu1[96] + Q11[215]*Gu1[102] + Gu2[66];
Gu3[67] = + Q11[198]*Gu1[1] + Q11[199]*Gu1[7] + Q11[200]*Gu1[13] + Q11[201]*Gu1[19] + Q11[202]*Gu1[25] + Q11[203]*Gu1[31] + Q11[204]*Gu1[37] + Q11[205]*Gu1[43] + Q11[206]*Gu1[49] + Q11[207]*Gu1[55] + Q11[208]*Gu1[61] + Q11[209]*Gu1[67] + Q11[210]*Gu1[73] + Q11[211]*Gu1[79] + Q11[212]*Gu1[85] + Q11[213]*Gu1[91] + Q11[214]*Gu1[97] + Q11[215]*Gu1[103] + Gu2[67];
Gu3[68] = + Q11[198]*Gu1[2] + Q11[199]*Gu1[8] + Q11[200]*Gu1[14] + Q11[201]*Gu1[20] + Q11[202]*Gu1[26] + Q11[203]*Gu1[32] + Q11[204]*Gu1[38] + Q11[205]*Gu1[44] + Q11[206]*Gu1[50] + Q11[207]*Gu1[56] + Q11[208]*Gu1[62] + Q11[209]*Gu1[68] + Q11[210]*Gu1[74] + Q11[211]*Gu1[80] + Q11[212]*Gu1[86] + Q11[213]*Gu1[92] + Q11[214]*Gu1[98] + Q11[215]*Gu1[104] + Gu2[68];
Gu3[69] = + Q11[198]*Gu1[3] + Q11[199]*Gu1[9] + Q11[200]*Gu1[15] + Q11[201]*Gu1[21] + Q11[202]*Gu1[27] + Q11[203]*Gu1[33] + Q11[204]*Gu1[39] + Q11[205]*Gu1[45] + Q11[206]*Gu1[51] + Q11[207]*Gu1[57] + Q11[208]*Gu1[63] + Q11[209]*Gu1[69] + Q11[210]*Gu1[75] + Q11[211]*Gu1[81] + Q11[212]*Gu1[87] + Q11[213]*Gu1[93] + Q11[214]*Gu1[99] + Q11[215]*Gu1[105] + Gu2[69];
Gu3[70] = + Q11[198]*Gu1[4] + Q11[199]*Gu1[10] + Q11[200]*Gu1[16] + Q11[201]*Gu1[22] + Q11[202]*Gu1[28] + Q11[203]*Gu1[34] + Q11[204]*Gu1[40] + Q11[205]*Gu1[46] + Q11[206]*Gu1[52] + Q11[207]*Gu1[58] + Q11[208]*Gu1[64] + Q11[209]*Gu1[70] + Q11[210]*Gu1[76] + Q11[211]*Gu1[82] + Q11[212]*Gu1[88] + Q11[213]*Gu1[94] + Q11[214]*Gu1[100] + Q11[215]*Gu1[106] + Gu2[70];
Gu3[71] = + Q11[198]*Gu1[5] + Q11[199]*Gu1[11] + Q11[200]*Gu1[17] + Q11[201]*Gu1[23] + Q11[202]*Gu1[29] + Q11[203]*Gu1[35] + Q11[204]*Gu1[41] + Q11[205]*Gu1[47] + Q11[206]*Gu1[53] + Q11[207]*Gu1[59] + Q11[208]*Gu1[65] + Q11[209]*Gu1[71] + Q11[210]*Gu1[77] + Q11[211]*Gu1[83] + Q11[212]*Gu1[89] + Q11[213]*Gu1[95] + Q11[214]*Gu1[101] + Q11[215]*Gu1[107] + Gu2[71];
Gu3[72] = + Q11[216]*Gu1[0] + Q11[217]*Gu1[6] + Q11[218]*Gu1[12] + Q11[219]*Gu1[18] + Q11[220]*Gu1[24] + Q11[221]*Gu1[30] + Q11[222]*Gu1[36] + Q11[223]*Gu1[42] + Q11[224]*Gu1[48] + Q11[225]*Gu1[54] + Q11[226]*Gu1[60] + Q11[227]*Gu1[66] + Q11[228]*Gu1[72] + Q11[229]*Gu1[78] + Q11[230]*Gu1[84] + Q11[231]*Gu1[90] + Q11[232]*Gu1[96] + Q11[233]*Gu1[102] + Gu2[72];
Gu3[73] = + Q11[216]*Gu1[1] + Q11[217]*Gu1[7] + Q11[218]*Gu1[13] + Q11[219]*Gu1[19] + Q11[220]*Gu1[25] + Q11[221]*Gu1[31] + Q11[222]*Gu1[37] + Q11[223]*Gu1[43] + Q11[224]*Gu1[49] + Q11[225]*Gu1[55] + Q11[226]*Gu1[61] + Q11[227]*Gu1[67] + Q11[228]*Gu1[73] + Q11[229]*Gu1[79] + Q11[230]*Gu1[85] + Q11[231]*Gu1[91] + Q11[232]*Gu1[97] + Q11[233]*Gu1[103] + Gu2[73];
Gu3[74] = + Q11[216]*Gu1[2] + Q11[217]*Gu1[8] + Q11[218]*Gu1[14] + Q11[219]*Gu1[20] + Q11[220]*Gu1[26] + Q11[221]*Gu1[32] + Q11[222]*Gu1[38] + Q11[223]*Gu1[44] + Q11[224]*Gu1[50] + Q11[225]*Gu1[56] + Q11[226]*Gu1[62] + Q11[227]*Gu1[68] + Q11[228]*Gu1[74] + Q11[229]*Gu1[80] + Q11[230]*Gu1[86] + Q11[231]*Gu1[92] + Q11[232]*Gu1[98] + Q11[233]*Gu1[104] + Gu2[74];
Gu3[75] = + Q11[216]*Gu1[3] + Q11[217]*Gu1[9] + Q11[218]*Gu1[15] + Q11[219]*Gu1[21] + Q11[220]*Gu1[27] + Q11[221]*Gu1[33] + Q11[222]*Gu1[39] + Q11[223]*Gu1[45] + Q11[224]*Gu1[51] + Q11[225]*Gu1[57] + Q11[226]*Gu1[63] + Q11[227]*Gu1[69] + Q11[228]*Gu1[75] + Q11[229]*Gu1[81] + Q11[230]*Gu1[87] + Q11[231]*Gu1[93] + Q11[232]*Gu1[99] + Q11[233]*Gu1[105] + Gu2[75];
Gu3[76] = + Q11[216]*Gu1[4] + Q11[217]*Gu1[10] + Q11[218]*Gu1[16] + Q11[219]*Gu1[22] + Q11[220]*Gu1[28] + Q11[221]*Gu1[34] + Q11[222]*Gu1[40] + Q11[223]*Gu1[46] + Q11[224]*Gu1[52] + Q11[225]*Gu1[58] + Q11[226]*Gu1[64] + Q11[227]*Gu1[70] + Q11[228]*Gu1[76] + Q11[229]*Gu1[82] + Q11[230]*Gu1[88] + Q11[231]*Gu1[94] + Q11[232]*Gu1[100] + Q11[233]*Gu1[106] + Gu2[76];
Gu3[77] = + Q11[216]*Gu1[5] + Q11[217]*Gu1[11] + Q11[218]*Gu1[17] + Q11[219]*Gu1[23] + Q11[220]*Gu1[29] + Q11[221]*Gu1[35] + Q11[222]*Gu1[41] + Q11[223]*Gu1[47] + Q11[224]*Gu1[53] + Q11[225]*Gu1[59] + Q11[226]*Gu1[65] + Q11[227]*Gu1[71] + Q11[228]*Gu1[77] + Q11[229]*Gu1[83] + Q11[230]*Gu1[89] + Q11[231]*Gu1[95] + Q11[232]*Gu1[101] + Q11[233]*Gu1[107] + Gu2[77];
Gu3[78] = + Q11[234]*Gu1[0] + Q11[235]*Gu1[6] + Q11[236]*Gu1[12] + Q11[237]*Gu1[18] + Q11[238]*Gu1[24] + Q11[239]*Gu1[30] + Q11[240]*Gu1[36] + Q11[241]*Gu1[42] + Q11[242]*Gu1[48] + Q11[243]*Gu1[54] + Q11[244]*Gu1[60] + Q11[245]*Gu1[66] + Q11[246]*Gu1[72] + Q11[247]*Gu1[78] + Q11[248]*Gu1[84] + Q11[249]*Gu1[90] + Q11[250]*Gu1[96] + Q11[251]*Gu1[102] + Gu2[78];
Gu3[79] = + Q11[234]*Gu1[1] + Q11[235]*Gu1[7] + Q11[236]*Gu1[13] + Q11[237]*Gu1[19] + Q11[238]*Gu1[25] + Q11[239]*Gu1[31] + Q11[240]*Gu1[37] + Q11[241]*Gu1[43] + Q11[242]*Gu1[49] + Q11[243]*Gu1[55] + Q11[244]*Gu1[61] + Q11[245]*Gu1[67] + Q11[246]*Gu1[73] + Q11[247]*Gu1[79] + Q11[248]*Gu1[85] + Q11[249]*Gu1[91] + Q11[250]*Gu1[97] + Q11[251]*Gu1[103] + Gu2[79];
Gu3[80] = + Q11[234]*Gu1[2] + Q11[235]*Gu1[8] + Q11[236]*Gu1[14] + Q11[237]*Gu1[20] + Q11[238]*Gu1[26] + Q11[239]*Gu1[32] + Q11[240]*Gu1[38] + Q11[241]*Gu1[44] + Q11[242]*Gu1[50] + Q11[243]*Gu1[56] + Q11[244]*Gu1[62] + Q11[245]*Gu1[68] + Q11[246]*Gu1[74] + Q11[247]*Gu1[80] + Q11[248]*Gu1[86] + Q11[249]*Gu1[92] + Q11[250]*Gu1[98] + Q11[251]*Gu1[104] + Gu2[80];
Gu3[81] = + Q11[234]*Gu1[3] + Q11[235]*Gu1[9] + Q11[236]*Gu1[15] + Q11[237]*Gu1[21] + Q11[238]*Gu1[27] + Q11[239]*Gu1[33] + Q11[240]*Gu1[39] + Q11[241]*Gu1[45] + Q11[242]*Gu1[51] + Q11[243]*Gu1[57] + Q11[244]*Gu1[63] + Q11[245]*Gu1[69] + Q11[246]*Gu1[75] + Q11[247]*Gu1[81] + Q11[248]*Gu1[87] + Q11[249]*Gu1[93] + Q11[250]*Gu1[99] + Q11[251]*Gu1[105] + Gu2[81];
Gu3[82] = + Q11[234]*Gu1[4] + Q11[235]*Gu1[10] + Q11[236]*Gu1[16] + Q11[237]*Gu1[22] + Q11[238]*Gu1[28] + Q11[239]*Gu1[34] + Q11[240]*Gu1[40] + Q11[241]*Gu1[46] + Q11[242]*Gu1[52] + Q11[243]*Gu1[58] + Q11[244]*Gu1[64] + Q11[245]*Gu1[70] + Q11[246]*Gu1[76] + Q11[247]*Gu1[82] + Q11[248]*Gu1[88] + Q11[249]*Gu1[94] + Q11[250]*Gu1[100] + Q11[251]*Gu1[106] + Gu2[82];
Gu3[83] = + Q11[234]*Gu1[5] + Q11[235]*Gu1[11] + Q11[236]*Gu1[17] + Q11[237]*Gu1[23] + Q11[238]*Gu1[29] + Q11[239]*Gu1[35] + Q11[240]*Gu1[41] + Q11[241]*Gu1[47] + Q11[242]*Gu1[53] + Q11[243]*Gu1[59] + Q11[244]*Gu1[65] + Q11[245]*Gu1[71] + Q11[246]*Gu1[77] + Q11[247]*Gu1[83] + Q11[248]*Gu1[89] + Q11[249]*Gu1[95] + Q11[250]*Gu1[101] + Q11[251]*Gu1[107] + Gu2[83];
Gu3[84] = + Q11[252]*Gu1[0] + Q11[253]*Gu1[6] + Q11[254]*Gu1[12] + Q11[255]*Gu1[18] + Q11[256]*Gu1[24] + Q11[257]*Gu1[30] + Q11[258]*Gu1[36] + Q11[259]*Gu1[42] + Q11[260]*Gu1[48] + Q11[261]*Gu1[54] + Q11[262]*Gu1[60] + Q11[263]*Gu1[66] + Q11[264]*Gu1[72] + Q11[265]*Gu1[78] + Q11[266]*Gu1[84] + Q11[267]*Gu1[90] + Q11[268]*Gu1[96] + Q11[269]*Gu1[102] + Gu2[84];
Gu3[85] = + Q11[252]*Gu1[1] + Q11[253]*Gu1[7] + Q11[254]*Gu1[13] + Q11[255]*Gu1[19] + Q11[256]*Gu1[25] + Q11[257]*Gu1[31] + Q11[258]*Gu1[37] + Q11[259]*Gu1[43] + Q11[260]*Gu1[49] + Q11[261]*Gu1[55] + Q11[262]*Gu1[61] + Q11[263]*Gu1[67] + Q11[264]*Gu1[73] + Q11[265]*Gu1[79] + Q11[266]*Gu1[85] + Q11[267]*Gu1[91] + Q11[268]*Gu1[97] + Q11[269]*Gu1[103] + Gu2[85];
Gu3[86] = + Q11[252]*Gu1[2] + Q11[253]*Gu1[8] + Q11[254]*Gu1[14] + Q11[255]*Gu1[20] + Q11[256]*Gu1[26] + Q11[257]*Gu1[32] + Q11[258]*Gu1[38] + Q11[259]*Gu1[44] + Q11[260]*Gu1[50] + Q11[261]*Gu1[56] + Q11[262]*Gu1[62] + Q11[263]*Gu1[68] + Q11[264]*Gu1[74] + Q11[265]*Gu1[80] + Q11[266]*Gu1[86] + Q11[267]*Gu1[92] + Q11[268]*Gu1[98] + Q11[269]*Gu1[104] + Gu2[86];
Gu3[87] = + Q11[252]*Gu1[3] + Q11[253]*Gu1[9] + Q11[254]*Gu1[15] + Q11[255]*Gu1[21] + Q11[256]*Gu1[27] + Q11[257]*Gu1[33] + Q11[258]*Gu1[39] + Q11[259]*Gu1[45] + Q11[260]*Gu1[51] + Q11[261]*Gu1[57] + Q11[262]*Gu1[63] + Q11[263]*Gu1[69] + Q11[264]*Gu1[75] + Q11[265]*Gu1[81] + Q11[266]*Gu1[87] + Q11[267]*Gu1[93] + Q11[268]*Gu1[99] + Q11[269]*Gu1[105] + Gu2[87];
Gu3[88] = + Q11[252]*Gu1[4] + Q11[253]*Gu1[10] + Q11[254]*Gu1[16] + Q11[255]*Gu1[22] + Q11[256]*Gu1[28] + Q11[257]*Gu1[34] + Q11[258]*Gu1[40] + Q11[259]*Gu1[46] + Q11[260]*Gu1[52] + Q11[261]*Gu1[58] + Q11[262]*Gu1[64] + Q11[263]*Gu1[70] + Q11[264]*Gu1[76] + Q11[265]*Gu1[82] + Q11[266]*Gu1[88] + Q11[267]*Gu1[94] + Q11[268]*Gu1[100] + Q11[269]*Gu1[106] + Gu2[88];
Gu3[89] = + Q11[252]*Gu1[5] + Q11[253]*Gu1[11] + Q11[254]*Gu1[17] + Q11[255]*Gu1[23] + Q11[256]*Gu1[29] + Q11[257]*Gu1[35] + Q11[258]*Gu1[41] + Q11[259]*Gu1[47] + Q11[260]*Gu1[53] + Q11[261]*Gu1[59] + Q11[262]*Gu1[65] + Q11[263]*Gu1[71] + Q11[264]*Gu1[77] + Q11[265]*Gu1[83] + Q11[266]*Gu1[89] + Q11[267]*Gu1[95] + Q11[268]*Gu1[101] + Q11[269]*Gu1[107] + Gu2[89];
Gu3[90] = + Q11[270]*Gu1[0] + Q11[271]*Gu1[6] + Q11[272]*Gu1[12] + Q11[273]*Gu1[18] + Q11[274]*Gu1[24] + Q11[275]*Gu1[30] + Q11[276]*Gu1[36] + Q11[277]*Gu1[42] + Q11[278]*Gu1[48] + Q11[279]*Gu1[54] + Q11[280]*Gu1[60] + Q11[281]*Gu1[66] + Q11[282]*Gu1[72] + Q11[283]*Gu1[78] + Q11[284]*Gu1[84] + Q11[285]*Gu1[90] + Q11[286]*Gu1[96] + Q11[287]*Gu1[102] + Gu2[90];
Gu3[91] = + Q11[270]*Gu1[1] + Q11[271]*Gu1[7] + Q11[272]*Gu1[13] + Q11[273]*Gu1[19] + Q11[274]*Gu1[25] + Q11[275]*Gu1[31] + Q11[276]*Gu1[37] + Q11[277]*Gu1[43] + Q11[278]*Gu1[49] + Q11[279]*Gu1[55] + Q11[280]*Gu1[61] + Q11[281]*Gu1[67] + Q11[282]*Gu1[73] + Q11[283]*Gu1[79] + Q11[284]*Gu1[85] + Q11[285]*Gu1[91] + Q11[286]*Gu1[97] + Q11[287]*Gu1[103] + Gu2[91];
Gu3[92] = + Q11[270]*Gu1[2] + Q11[271]*Gu1[8] + Q11[272]*Gu1[14] + Q11[273]*Gu1[20] + Q11[274]*Gu1[26] + Q11[275]*Gu1[32] + Q11[276]*Gu1[38] + Q11[277]*Gu1[44] + Q11[278]*Gu1[50] + Q11[279]*Gu1[56] + Q11[280]*Gu1[62] + Q11[281]*Gu1[68] + Q11[282]*Gu1[74] + Q11[283]*Gu1[80] + Q11[284]*Gu1[86] + Q11[285]*Gu1[92] + Q11[286]*Gu1[98] + Q11[287]*Gu1[104] + Gu2[92];
Gu3[93] = + Q11[270]*Gu1[3] + Q11[271]*Gu1[9] + Q11[272]*Gu1[15] + Q11[273]*Gu1[21] + Q11[274]*Gu1[27] + Q11[275]*Gu1[33] + Q11[276]*Gu1[39] + Q11[277]*Gu1[45] + Q11[278]*Gu1[51] + Q11[279]*Gu1[57] + Q11[280]*Gu1[63] + Q11[281]*Gu1[69] + Q11[282]*Gu1[75] + Q11[283]*Gu1[81] + Q11[284]*Gu1[87] + Q11[285]*Gu1[93] + Q11[286]*Gu1[99] + Q11[287]*Gu1[105] + Gu2[93];
Gu3[94] = + Q11[270]*Gu1[4] + Q11[271]*Gu1[10] + Q11[272]*Gu1[16] + Q11[273]*Gu1[22] + Q11[274]*Gu1[28] + Q11[275]*Gu1[34] + Q11[276]*Gu1[40] + Q11[277]*Gu1[46] + Q11[278]*Gu1[52] + Q11[279]*Gu1[58] + Q11[280]*Gu1[64] + Q11[281]*Gu1[70] + Q11[282]*Gu1[76] + Q11[283]*Gu1[82] + Q11[284]*Gu1[88] + Q11[285]*Gu1[94] + Q11[286]*Gu1[100] + Q11[287]*Gu1[106] + Gu2[94];
Gu3[95] = + Q11[270]*Gu1[5] + Q11[271]*Gu1[11] + Q11[272]*Gu1[17] + Q11[273]*Gu1[23] + Q11[274]*Gu1[29] + Q11[275]*Gu1[35] + Q11[276]*Gu1[41] + Q11[277]*Gu1[47] + Q11[278]*Gu1[53] + Q11[279]*Gu1[59] + Q11[280]*Gu1[65] + Q11[281]*Gu1[71] + Q11[282]*Gu1[77] + Q11[283]*Gu1[83] + Q11[284]*Gu1[89] + Q11[285]*Gu1[95] + Q11[286]*Gu1[101] + Q11[287]*Gu1[107] + Gu2[95];
Gu3[96] = + Q11[288]*Gu1[0] + Q11[289]*Gu1[6] + Q11[290]*Gu1[12] + Q11[291]*Gu1[18] + Q11[292]*Gu1[24] + Q11[293]*Gu1[30] + Q11[294]*Gu1[36] + Q11[295]*Gu1[42] + Q11[296]*Gu1[48] + Q11[297]*Gu1[54] + Q11[298]*Gu1[60] + Q11[299]*Gu1[66] + Q11[300]*Gu1[72] + Q11[301]*Gu1[78] + Q11[302]*Gu1[84] + Q11[303]*Gu1[90] + Q11[304]*Gu1[96] + Q11[305]*Gu1[102] + Gu2[96];
Gu3[97] = + Q11[288]*Gu1[1] + Q11[289]*Gu1[7] + Q11[290]*Gu1[13] + Q11[291]*Gu1[19] + Q11[292]*Gu1[25] + Q11[293]*Gu1[31] + Q11[294]*Gu1[37] + Q11[295]*Gu1[43] + Q11[296]*Gu1[49] + Q11[297]*Gu1[55] + Q11[298]*Gu1[61] + Q11[299]*Gu1[67] + Q11[300]*Gu1[73] + Q11[301]*Gu1[79] + Q11[302]*Gu1[85] + Q11[303]*Gu1[91] + Q11[304]*Gu1[97] + Q11[305]*Gu1[103] + Gu2[97];
Gu3[98] = + Q11[288]*Gu1[2] + Q11[289]*Gu1[8] + Q11[290]*Gu1[14] + Q11[291]*Gu1[20] + Q11[292]*Gu1[26] + Q11[293]*Gu1[32] + Q11[294]*Gu1[38] + Q11[295]*Gu1[44] + Q11[296]*Gu1[50] + Q11[297]*Gu1[56] + Q11[298]*Gu1[62] + Q11[299]*Gu1[68] + Q11[300]*Gu1[74] + Q11[301]*Gu1[80] + Q11[302]*Gu1[86] + Q11[303]*Gu1[92] + Q11[304]*Gu1[98] + Q11[305]*Gu1[104] + Gu2[98];
Gu3[99] = + Q11[288]*Gu1[3] + Q11[289]*Gu1[9] + Q11[290]*Gu1[15] + Q11[291]*Gu1[21] + Q11[292]*Gu1[27] + Q11[293]*Gu1[33] + Q11[294]*Gu1[39] + Q11[295]*Gu1[45] + Q11[296]*Gu1[51] + Q11[297]*Gu1[57] + Q11[298]*Gu1[63] + Q11[299]*Gu1[69] + Q11[300]*Gu1[75] + Q11[301]*Gu1[81] + Q11[302]*Gu1[87] + Q11[303]*Gu1[93] + Q11[304]*Gu1[99] + Q11[305]*Gu1[105] + Gu2[99];
Gu3[100] = + Q11[288]*Gu1[4] + Q11[289]*Gu1[10] + Q11[290]*Gu1[16] + Q11[291]*Gu1[22] + Q11[292]*Gu1[28] + Q11[293]*Gu1[34] + Q11[294]*Gu1[40] + Q11[295]*Gu1[46] + Q11[296]*Gu1[52] + Q11[297]*Gu1[58] + Q11[298]*Gu1[64] + Q11[299]*Gu1[70] + Q11[300]*Gu1[76] + Q11[301]*Gu1[82] + Q11[302]*Gu1[88] + Q11[303]*Gu1[94] + Q11[304]*Gu1[100] + Q11[305]*Gu1[106] + Gu2[100];
Gu3[101] = + Q11[288]*Gu1[5] + Q11[289]*Gu1[11] + Q11[290]*Gu1[17] + Q11[291]*Gu1[23] + Q11[292]*Gu1[29] + Q11[293]*Gu1[35] + Q11[294]*Gu1[41] + Q11[295]*Gu1[47] + Q11[296]*Gu1[53] + Q11[297]*Gu1[59] + Q11[298]*Gu1[65] + Q11[299]*Gu1[71] + Q11[300]*Gu1[77] + Q11[301]*Gu1[83] + Q11[302]*Gu1[89] + Q11[303]*Gu1[95] + Q11[304]*Gu1[101] + Q11[305]*Gu1[107] + Gu2[101];
Gu3[102] = + Q11[306]*Gu1[0] + Q11[307]*Gu1[6] + Q11[308]*Gu1[12] + Q11[309]*Gu1[18] + Q11[310]*Gu1[24] + Q11[311]*Gu1[30] + Q11[312]*Gu1[36] + Q11[313]*Gu1[42] + Q11[314]*Gu1[48] + Q11[315]*Gu1[54] + Q11[316]*Gu1[60] + Q11[317]*Gu1[66] + Q11[318]*Gu1[72] + Q11[319]*Gu1[78] + Q11[320]*Gu1[84] + Q11[321]*Gu1[90] + Q11[322]*Gu1[96] + Q11[323]*Gu1[102] + Gu2[102];
Gu3[103] = + Q11[306]*Gu1[1] + Q11[307]*Gu1[7] + Q11[308]*Gu1[13] + Q11[309]*Gu1[19] + Q11[310]*Gu1[25] + Q11[311]*Gu1[31] + Q11[312]*Gu1[37] + Q11[313]*Gu1[43] + Q11[314]*Gu1[49] + Q11[315]*Gu1[55] + Q11[316]*Gu1[61] + Q11[317]*Gu1[67] + Q11[318]*Gu1[73] + Q11[319]*Gu1[79] + Q11[320]*Gu1[85] + Q11[321]*Gu1[91] + Q11[322]*Gu1[97] + Q11[323]*Gu1[103] + Gu2[103];
Gu3[104] = + Q11[306]*Gu1[2] + Q11[307]*Gu1[8] + Q11[308]*Gu1[14] + Q11[309]*Gu1[20] + Q11[310]*Gu1[26] + Q11[311]*Gu1[32] + Q11[312]*Gu1[38] + Q11[313]*Gu1[44] + Q11[314]*Gu1[50] + Q11[315]*Gu1[56] + Q11[316]*Gu1[62] + Q11[317]*Gu1[68] + Q11[318]*Gu1[74] + Q11[319]*Gu1[80] + Q11[320]*Gu1[86] + Q11[321]*Gu1[92] + Q11[322]*Gu1[98] + Q11[323]*Gu1[104] + Gu2[104];
Gu3[105] = + Q11[306]*Gu1[3] + Q11[307]*Gu1[9] + Q11[308]*Gu1[15] + Q11[309]*Gu1[21] + Q11[310]*Gu1[27] + Q11[311]*Gu1[33] + Q11[312]*Gu1[39] + Q11[313]*Gu1[45] + Q11[314]*Gu1[51] + Q11[315]*Gu1[57] + Q11[316]*Gu1[63] + Q11[317]*Gu1[69] + Q11[318]*Gu1[75] + Q11[319]*Gu1[81] + Q11[320]*Gu1[87] + Q11[321]*Gu1[93] + Q11[322]*Gu1[99] + Q11[323]*Gu1[105] + Gu2[105];
Gu3[106] = + Q11[306]*Gu1[4] + Q11[307]*Gu1[10] + Q11[308]*Gu1[16] + Q11[309]*Gu1[22] + Q11[310]*Gu1[28] + Q11[311]*Gu1[34] + Q11[312]*Gu1[40] + Q11[313]*Gu1[46] + Q11[314]*Gu1[52] + Q11[315]*Gu1[58] + Q11[316]*Gu1[64] + Q11[317]*Gu1[70] + Q11[318]*Gu1[76] + Q11[319]*Gu1[82] + Q11[320]*Gu1[88] + Q11[321]*Gu1[94] + Q11[322]*Gu1[100] + Q11[323]*Gu1[106] + Gu2[106];
Gu3[107] = + Q11[306]*Gu1[5] + Q11[307]*Gu1[11] + Q11[308]*Gu1[17] + Q11[309]*Gu1[23] + Q11[310]*Gu1[29] + Q11[311]*Gu1[35] + Q11[312]*Gu1[41] + Q11[313]*Gu1[47] + Q11[314]*Gu1[53] + Q11[315]*Gu1[59] + Q11[316]*Gu1[65] + Q11[317]*Gu1[71] + Q11[318]*Gu1[77] + Q11[319]*Gu1[83] + Q11[320]*Gu1[89] + Q11[321]*Gu1[95] + Q11[322]*Gu1[101] + Q11[323]*Gu1[107] + Gu2[107];
}

void acado_macATw1QDy( real_t* const Gx1, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Gx1[0]*w11[0] + Gx1[18]*w11[1] + Gx1[36]*w11[2] + Gx1[54]*w11[3] + Gx1[72]*w11[4] + Gx1[90]*w11[5] + Gx1[108]*w11[6] + Gx1[126]*w11[7] + Gx1[144]*w11[8] + Gx1[162]*w11[9] + Gx1[180]*w11[10] + Gx1[198]*w11[11] + Gx1[216]*w11[12] + Gx1[234]*w11[13] + Gx1[252]*w11[14] + Gx1[270]*w11[15] + Gx1[288]*w11[16] + Gx1[306]*w11[17] + w12[0];
w13[1] = + Gx1[1]*w11[0] + Gx1[19]*w11[1] + Gx1[37]*w11[2] + Gx1[55]*w11[3] + Gx1[73]*w11[4] + Gx1[91]*w11[5] + Gx1[109]*w11[6] + Gx1[127]*w11[7] + Gx1[145]*w11[8] + Gx1[163]*w11[9] + Gx1[181]*w11[10] + Gx1[199]*w11[11] + Gx1[217]*w11[12] + Gx1[235]*w11[13] + Gx1[253]*w11[14] + Gx1[271]*w11[15] + Gx1[289]*w11[16] + Gx1[307]*w11[17] + w12[1];
w13[2] = + Gx1[2]*w11[0] + Gx1[20]*w11[1] + Gx1[38]*w11[2] + Gx1[56]*w11[3] + Gx1[74]*w11[4] + Gx1[92]*w11[5] + Gx1[110]*w11[6] + Gx1[128]*w11[7] + Gx1[146]*w11[8] + Gx1[164]*w11[9] + Gx1[182]*w11[10] + Gx1[200]*w11[11] + Gx1[218]*w11[12] + Gx1[236]*w11[13] + Gx1[254]*w11[14] + Gx1[272]*w11[15] + Gx1[290]*w11[16] + Gx1[308]*w11[17] + w12[2];
w13[3] = + Gx1[3]*w11[0] + Gx1[21]*w11[1] + Gx1[39]*w11[2] + Gx1[57]*w11[3] + Gx1[75]*w11[4] + Gx1[93]*w11[5] + Gx1[111]*w11[6] + Gx1[129]*w11[7] + Gx1[147]*w11[8] + Gx1[165]*w11[9] + Gx1[183]*w11[10] + Gx1[201]*w11[11] + Gx1[219]*w11[12] + Gx1[237]*w11[13] + Gx1[255]*w11[14] + Gx1[273]*w11[15] + Gx1[291]*w11[16] + Gx1[309]*w11[17] + w12[3];
w13[4] = + Gx1[4]*w11[0] + Gx1[22]*w11[1] + Gx1[40]*w11[2] + Gx1[58]*w11[3] + Gx1[76]*w11[4] + Gx1[94]*w11[5] + Gx1[112]*w11[6] + Gx1[130]*w11[7] + Gx1[148]*w11[8] + Gx1[166]*w11[9] + Gx1[184]*w11[10] + Gx1[202]*w11[11] + Gx1[220]*w11[12] + Gx1[238]*w11[13] + Gx1[256]*w11[14] + Gx1[274]*w11[15] + Gx1[292]*w11[16] + Gx1[310]*w11[17] + w12[4];
w13[5] = + Gx1[5]*w11[0] + Gx1[23]*w11[1] + Gx1[41]*w11[2] + Gx1[59]*w11[3] + Gx1[77]*w11[4] + Gx1[95]*w11[5] + Gx1[113]*w11[6] + Gx1[131]*w11[7] + Gx1[149]*w11[8] + Gx1[167]*w11[9] + Gx1[185]*w11[10] + Gx1[203]*w11[11] + Gx1[221]*w11[12] + Gx1[239]*w11[13] + Gx1[257]*w11[14] + Gx1[275]*w11[15] + Gx1[293]*w11[16] + Gx1[311]*w11[17] + w12[5];
w13[6] = + Gx1[6]*w11[0] + Gx1[24]*w11[1] + Gx1[42]*w11[2] + Gx1[60]*w11[3] + Gx1[78]*w11[4] + Gx1[96]*w11[5] + Gx1[114]*w11[6] + Gx1[132]*w11[7] + Gx1[150]*w11[8] + Gx1[168]*w11[9] + Gx1[186]*w11[10] + Gx1[204]*w11[11] + Gx1[222]*w11[12] + Gx1[240]*w11[13] + Gx1[258]*w11[14] + Gx1[276]*w11[15] + Gx1[294]*w11[16] + Gx1[312]*w11[17] + w12[6];
w13[7] = + Gx1[7]*w11[0] + Gx1[25]*w11[1] + Gx1[43]*w11[2] + Gx1[61]*w11[3] + Gx1[79]*w11[4] + Gx1[97]*w11[5] + Gx1[115]*w11[6] + Gx1[133]*w11[7] + Gx1[151]*w11[8] + Gx1[169]*w11[9] + Gx1[187]*w11[10] + Gx1[205]*w11[11] + Gx1[223]*w11[12] + Gx1[241]*w11[13] + Gx1[259]*w11[14] + Gx1[277]*w11[15] + Gx1[295]*w11[16] + Gx1[313]*w11[17] + w12[7];
w13[8] = + Gx1[8]*w11[0] + Gx1[26]*w11[1] + Gx1[44]*w11[2] + Gx1[62]*w11[3] + Gx1[80]*w11[4] + Gx1[98]*w11[5] + Gx1[116]*w11[6] + Gx1[134]*w11[7] + Gx1[152]*w11[8] + Gx1[170]*w11[9] + Gx1[188]*w11[10] + Gx1[206]*w11[11] + Gx1[224]*w11[12] + Gx1[242]*w11[13] + Gx1[260]*w11[14] + Gx1[278]*w11[15] + Gx1[296]*w11[16] + Gx1[314]*w11[17] + w12[8];
w13[9] = + Gx1[9]*w11[0] + Gx1[27]*w11[1] + Gx1[45]*w11[2] + Gx1[63]*w11[3] + Gx1[81]*w11[4] + Gx1[99]*w11[5] + Gx1[117]*w11[6] + Gx1[135]*w11[7] + Gx1[153]*w11[8] + Gx1[171]*w11[9] + Gx1[189]*w11[10] + Gx1[207]*w11[11] + Gx1[225]*w11[12] + Gx1[243]*w11[13] + Gx1[261]*w11[14] + Gx1[279]*w11[15] + Gx1[297]*w11[16] + Gx1[315]*w11[17] + w12[9];
w13[10] = + Gx1[10]*w11[0] + Gx1[28]*w11[1] + Gx1[46]*w11[2] + Gx1[64]*w11[3] + Gx1[82]*w11[4] + Gx1[100]*w11[5] + Gx1[118]*w11[6] + Gx1[136]*w11[7] + Gx1[154]*w11[8] + Gx1[172]*w11[9] + Gx1[190]*w11[10] + Gx1[208]*w11[11] + Gx1[226]*w11[12] + Gx1[244]*w11[13] + Gx1[262]*w11[14] + Gx1[280]*w11[15] + Gx1[298]*w11[16] + Gx1[316]*w11[17] + w12[10];
w13[11] = + Gx1[11]*w11[0] + Gx1[29]*w11[1] + Gx1[47]*w11[2] + Gx1[65]*w11[3] + Gx1[83]*w11[4] + Gx1[101]*w11[5] + Gx1[119]*w11[6] + Gx1[137]*w11[7] + Gx1[155]*w11[8] + Gx1[173]*w11[9] + Gx1[191]*w11[10] + Gx1[209]*w11[11] + Gx1[227]*w11[12] + Gx1[245]*w11[13] + Gx1[263]*w11[14] + Gx1[281]*w11[15] + Gx1[299]*w11[16] + Gx1[317]*w11[17] + w12[11];
w13[12] = + Gx1[12]*w11[0] + Gx1[30]*w11[1] + Gx1[48]*w11[2] + Gx1[66]*w11[3] + Gx1[84]*w11[4] + Gx1[102]*w11[5] + Gx1[120]*w11[6] + Gx1[138]*w11[7] + Gx1[156]*w11[8] + Gx1[174]*w11[9] + Gx1[192]*w11[10] + Gx1[210]*w11[11] + Gx1[228]*w11[12] + Gx1[246]*w11[13] + Gx1[264]*w11[14] + Gx1[282]*w11[15] + Gx1[300]*w11[16] + Gx1[318]*w11[17] + w12[12];
w13[13] = + Gx1[13]*w11[0] + Gx1[31]*w11[1] + Gx1[49]*w11[2] + Gx1[67]*w11[3] + Gx1[85]*w11[4] + Gx1[103]*w11[5] + Gx1[121]*w11[6] + Gx1[139]*w11[7] + Gx1[157]*w11[8] + Gx1[175]*w11[9] + Gx1[193]*w11[10] + Gx1[211]*w11[11] + Gx1[229]*w11[12] + Gx1[247]*w11[13] + Gx1[265]*w11[14] + Gx1[283]*w11[15] + Gx1[301]*w11[16] + Gx1[319]*w11[17] + w12[13];
w13[14] = + Gx1[14]*w11[0] + Gx1[32]*w11[1] + Gx1[50]*w11[2] + Gx1[68]*w11[3] + Gx1[86]*w11[4] + Gx1[104]*w11[5] + Gx1[122]*w11[6] + Gx1[140]*w11[7] + Gx1[158]*w11[8] + Gx1[176]*w11[9] + Gx1[194]*w11[10] + Gx1[212]*w11[11] + Gx1[230]*w11[12] + Gx1[248]*w11[13] + Gx1[266]*w11[14] + Gx1[284]*w11[15] + Gx1[302]*w11[16] + Gx1[320]*w11[17] + w12[14];
w13[15] = + Gx1[15]*w11[0] + Gx1[33]*w11[1] + Gx1[51]*w11[2] + Gx1[69]*w11[3] + Gx1[87]*w11[4] + Gx1[105]*w11[5] + Gx1[123]*w11[6] + Gx1[141]*w11[7] + Gx1[159]*w11[8] + Gx1[177]*w11[9] + Gx1[195]*w11[10] + Gx1[213]*w11[11] + Gx1[231]*w11[12] + Gx1[249]*w11[13] + Gx1[267]*w11[14] + Gx1[285]*w11[15] + Gx1[303]*w11[16] + Gx1[321]*w11[17] + w12[15];
w13[16] = + Gx1[16]*w11[0] + Gx1[34]*w11[1] + Gx1[52]*w11[2] + Gx1[70]*w11[3] + Gx1[88]*w11[4] + Gx1[106]*w11[5] + Gx1[124]*w11[6] + Gx1[142]*w11[7] + Gx1[160]*w11[8] + Gx1[178]*w11[9] + Gx1[196]*w11[10] + Gx1[214]*w11[11] + Gx1[232]*w11[12] + Gx1[250]*w11[13] + Gx1[268]*w11[14] + Gx1[286]*w11[15] + Gx1[304]*w11[16] + Gx1[322]*w11[17] + w12[16];
w13[17] = + Gx1[17]*w11[0] + Gx1[35]*w11[1] + Gx1[53]*w11[2] + Gx1[71]*w11[3] + Gx1[89]*w11[4] + Gx1[107]*w11[5] + Gx1[125]*w11[6] + Gx1[143]*w11[7] + Gx1[161]*w11[8] + Gx1[179]*w11[9] + Gx1[197]*w11[10] + Gx1[215]*w11[11] + Gx1[233]*w11[12] + Gx1[251]*w11[13] + Gx1[269]*w11[14] + Gx1[287]*w11[15] + Gx1[305]*w11[16] + Gx1[323]*w11[17] + w12[17];
}

void acado_macBTw1( real_t* const Gu1, real_t* const w11, real_t* const U1 )
{
U1[0] += + Gu1[0]*w11[0] + Gu1[6]*w11[1] + Gu1[12]*w11[2] + Gu1[18]*w11[3] + Gu1[24]*w11[4] + Gu1[30]*w11[5] + Gu1[36]*w11[6] + Gu1[42]*w11[7] + Gu1[48]*w11[8] + Gu1[54]*w11[9] + Gu1[60]*w11[10] + Gu1[66]*w11[11] + Gu1[72]*w11[12] + Gu1[78]*w11[13] + Gu1[84]*w11[14] + Gu1[90]*w11[15] + Gu1[96]*w11[16] + Gu1[102]*w11[17];
U1[1] += + Gu1[1]*w11[0] + Gu1[7]*w11[1] + Gu1[13]*w11[2] + Gu1[19]*w11[3] + Gu1[25]*w11[4] + Gu1[31]*w11[5] + Gu1[37]*w11[6] + Gu1[43]*w11[7] + Gu1[49]*w11[8] + Gu1[55]*w11[9] + Gu1[61]*w11[10] + Gu1[67]*w11[11] + Gu1[73]*w11[12] + Gu1[79]*w11[13] + Gu1[85]*w11[14] + Gu1[91]*w11[15] + Gu1[97]*w11[16] + Gu1[103]*w11[17];
U1[2] += + Gu1[2]*w11[0] + Gu1[8]*w11[1] + Gu1[14]*w11[2] + Gu1[20]*w11[3] + Gu1[26]*w11[4] + Gu1[32]*w11[5] + Gu1[38]*w11[6] + Gu1[44]*w11[7] + Gu1[50]*w11[8] + Gu1[56]*w11[9] + Gu1[62]*w11[10] + Gu1[68]*w11[11] + Gu1[74]*w11[12] + Gu1[80]*w11[13] + Gu1[86]*w11[14] + Gu1[92]*w11[15] + Gu1[98]*w11[16] + Gu1[104]*w11[17];
U1[3] += + Gu1[3]*w11[0] + Gu1[9]*w11[1] + Gu1[15]*w11[2] + Gu1[21]*w11[3] + Gu1[27]*w11[4] + Gu1[33]*w11[5] + Gu1[39]*w11[6] + Gu1[45]*w11[7] + Gu1[51]*w11[8] + Gu1[57]*w11[9] + Gu1[63]*w11[10] + Gu1[69]*w11[11] + Gu1[75]*w11[12] + Gu1[81]*w11[13] + Gu1[87]*w11[14] + Gu1[93]*w11[15] + Gu1[99]*w11[16] + Gu1[105]*w11[17];
U1[4] += + Gu1[4]*w11[0] + Gu1[10]*w11[1] + Gu1[16]*w11[2] + Gu1[22]*w11[3] + Gu1[28]*w11[4] + Gu1[34]*w11[5] + Gu1[40]*w11[6] + Gu1[46]*w11[7] + Gu1[52]*w11[8] + Gu1[58]*w11[9] + Gu1[64]*w11[10] + Gu1[70]*w11[11] + Gu1[76]*w11[12] + Gu1[82]*w11[13] + Gu1[88]*w11[14] + Gu1[94]*w11[15] + Gu1[100]*w11[16] + Gu1[106]*w11[17];
U1[5] += + Gu1[5]*w11[0] + Gu1[11]*w11[1] + Gu1[17]*w11[2] + Gu1[23]*w11[3] + Gu1[29]*w11[4] + Gu1[35]*w11[5] + Gu1[41]*w11[6] + Gu1[47]*w11[7] + Gu1[53]*w11[8] + Gu1[59]*w11[9] + Gu1[65]*w11[10] + Gu1[71]*w11[11] + Gu1[77]*w11[12] + Gu1[83]*w11[13] + Gu1[89]*w11[14] + Gu1[95]*w11[15] + Gu1[101]*w11[16] + Gu1[107]*w11[17];
}

void acado_macQSbarW2( real_t* const Q11, real_t* const w11, real_t* const w12, real_t* const w13 )
{
w13[0] = + Q11[0]*w11[0] + Q11[1]*w11[1] + Q11[2]*w11[2] + Q11[3]*w11[3] + Q11[4]*w11[4] + Q11[5]*w11[5] + Q11[6]*w11[6] + Q11[7]*w11[7] + Q11[8]*w11[8] + Q11[9]*w11[9] + Q11[10]*w11[10] + Q11[11]*w11[11] + Q11[12]*w11[12] + Q11[13]*w11[13] + Q11[14]*w11[14] + Q11[15]*w11[15] + Q11[16]*w11[16] + Q11[17]*w11[17] + w12[0];
w13[1] = + Q11[18]*w11[0] + Q11[19]*w11[1] + Q11[20]*w11[2] + Q11[21]*w11[3] + Q11[22]*w11[4] + Q11[23]*w11[5] + Q11[24]*w11[6] + Q11[25]*w11[7] + Q11[26]*w11[8] + Q11[27]*w11[9] + Q11[28]*w11[10] + Q11[29]*w11[11] + Q11[30]*w11[12] + Q11[31]*w11[13] + Q11[32]*w11[14] + Q11[33]*w11[15] + Q11[34]*w11[16] + Q11[35]*w11[17] + w12[1];
w13[2] = + Q11[36]*w11[0] + Q11[37]*w11[1] + Q11[38]*w11[2] + Q11[39]*w11[3] + Q11[40]*w11[4] + Q11[41]*w11[5] + Q11[42]*w11[6] + Q11[43]*w11[7] + Q11[44]*w11[8] + Q11[45]*w11[9] + Q11[46]*w11[10] + Q11[47]*w11[11] + Q11[48]*w11[12] + Q11[49]*w11[13] + Q11[50]*w11[14] + Q11[51]*w11[15] + Q11[52]*w11[16] + Q11[53]*w11[17] + w12[2];
w13[3] = + Q11[54]*w11[0] + Q11[55]*w11[1] + Q11[56]*w11[2] + Q11[57]*w11[3] + Q11[58]*w11[4] + Q11[59]*w11[5] + Q11[60]*w11[6] + Q11[61]*w11[7] + Q11[62]*w11[8] + Q11[63]*w11[9] + Q11[64]*w11[10] + Q11[65]*w11[11] + Q11[66]*w11[12] + Q11[67]*w11[13] + Q11[68]*w11[14] + Q11[69]*w11[15] + Q11[70]*w11[16] + Q11[71]*w11[17] + w12[3];
w13[4] = + Q11[72]*w11[0] + Q11[73]*w11[1] + Q11[74]*w11[2] + Q11[75]*w11[3] + Q11[76]*w11[4] + Q11[77]*w11[5] + Q11[78]*w11[6] + Q11[79]*w11[7] + Q11[80]*w11[8] + Q11[81]*w11[9] + Q11[82]*w11[10] + Q11[83]*w11[11] + Q11[84]*w11[12] + Q11[85]*w11[13] + Q11[86]*w11[14] + Q11[87]*w11[15] + Q11[88]*w11[16] + Q11[89]*w11[17] + w12[4];
w13[5] = + Q11[90]*w11[0] + Q11[91]*w11[1] + Q11[92]*w11[2] + Q11[93]*w11[3] + Q11[94]*w11[4] + Q11[95]*w11[5] + Q11[96]*w11[6] + Q11[97]*w11[7] + Q11[98]*w11[8] + Q11[99]*w11[9] + Q11[100]*w11[10] + Q11[101]*w11[11] + Q11[102]*w11[12] + Q11[103]*w11[13] + Q11[104]*w11[14] + Q11[105]*w11[15] + Q11[106]*w11[16] + Q11[107]*w11[17] + w12[5];
w13[6] = + Q11[108]*w11[0] + Q11[109]*w11[1] + Q11[110]*w11[2] + Q11[111]*w11[3] + Q11[112]*w11[4] + Q11[113]*w11[5] + Q11[114]*w11[6] + Q11[115]*w11[7] + Q11[116]*w11[8] + Q11[117]*w11[9] + Q11[118]*w11[10] + Q11[119]*w11[11] + Q11[120]*w11[12] + Q11[121]*w11[13] + Q11[122]*w11[14] + Q11[123]*w11[15] + Q11[124]*w11[16] + Q11[125]*w11[17] + w12[6];
w13[7] = + Q11[126]*w11[0] + Q11[127]*w11[1] + Q11[128]*w11[2] + Q11[129]*w11[3] + Q11[130]*w11[4] + Q11[131]*w11[5] + Q11[132]*w11[6] + Q11[133]*w11[7] + Q11[134]*w11[8] + Q11[135]*w11[9] + Q11[136]*w11[10] + Q11[137]*w11[11] + Q11[138]*w11[12] + Q11[139]*w11[13] + Q11[140]*w11[14] + Q11[141]*w11[15] + Q11[142]*w11[16] + Q11[143]*w11[17] + w12[7];
w13[8] = + Q11[144]*w11[0] + Q11[145]*w11[1] + Q11[146]*w11[2] + Q11[147]*w11[3] + Q11[148]*w11[4] + Q11[149]*w11[5] + Q11[150]*w11[6] + Q11[151]*w11[7] + Q11[152]*w11[8] + Q11[153]*w11[9] + Q11[154]*w11[10] + Q11[155]*w11[11] + Q11[156]*w11[12] + Q11[157]*w11[13] + Q11[158]*w11[14] + Q11[159]*w11[15] + Q11[160]*w11[16] + Q11[161]*w11[17] + w12[8];
w13[9] = + Q11[162]*w11[0] + Q11[163]*w11[1] + Q11[164]*w11[2] + Q11[165]*w11[3] + Q11[166]*w11[4] + Q11[167]*w11[5] + Q11[168]*w11[6] + Q11[169]*w11[7] + Q11[170]*w11[8] + Q11[171]*w11[9] + Q11[172]*w11[10] + Q11[173]*w11[11] + Q11[174]*w11[12] + Q11[175]*w11[13] + Q11[176]*w11[14] + Q11[177]*w11[15] + Q11[178]*w11[16] + Q11[179]*w11[17] + w12[9];
w13[10] = + Q11[180]*w11[0] + Q11[181]*w11[1] + Q11[182]*w11[2] + Q11[183]*w11[3] + Q11[184]*w11[4] + Q11[185]*w11[5] + Q11[186]*w11[6] + Q11[187]*w11[7] + Q11[188]*w11[8] + Q11[189]*w11[9] + Q11[190]*w11[10] + Q11[191]*w11[11] + Q11[192]*w11[12] + Q11[193]*w11[13] + Q11[194]*w11[14] + Q11[195]*w11[15] + Q11[196]*w11[16] + Q11[197]*w11[17] + w12[10];
w13[11] = + Q11[198]*w11[0] + Q11[199]*w11[1] + Q11[200]*w11[2] + Q11[201]*w11[3] + Q11[202]*w11[4] + Q11[203]*w11[5] + Q11[204]*w11[6] + Q11[205]*w11[7] + Q11[206]*w11[8] + Q11[207]*w11[9] + Q11[208]*w11[10] + Q11[209]*w11[11] + Q11[210]*w11[12] + Q11[211]*w11[13] + Q11[212]*w11[14] + Q11[213]*w11[15] + Q11[214]*w11[16] + Q11[215]*w11[17] + w12[11];
w13[12] = + Q11[216]*w11[0] + Q11[217]*w11[1] + Q11[218]*w11[2] + Q11[219]*w11[3] + Q11[220]*w11[4] + Q11[221]*w11[5] + Q11[222]*w11[6] + Q11[223]*w11[7] + Q11[224]*w11[8] + Q11[225]*w11[9] + Q11[226]*w11[10] + Q11[227]*w11[11] + Q11[228]*w11[12] + Q11[229]*w11[13] + Q11[230]*w11[14] + Q11[231]*w11[15] + Q11[232]*w11[16] + Q11[233]*w11[17] + w12[12];
w13[13] = + Q11[234]*w11[0] + Q11[235]*w11[1] + Q11[236]*w11[2] + Q11[237]*w11[3] + Q11[238]*w11[4] + Q11[239]*w11[5] + Q11[240]*w11[6] + Q11[241]*w11[7] + Q11[242]*w11[8] + Q11[243]*w11[9] + Q11[244]*w11[10] + Q11[245]*w11[11] + Q11[246]*w11[12] + Q11[247]*w11[13] + Q11[248]*w11[14] + Q11[249]*w11[15] + Q11[250]*w11[16] + Q11[251]*w11[17] + w12[13];
w13[14] = + Q11[252]*w11[0] + Q11[253]*w11[1] + Q11[254]*w11[2] + Q11[255]*w11[3] + Q11[256]*w11[4] + Q11[257]*w11[5] + Q11[258]*w11[6] + Q11[259]*w11[7] + Q11[260]*w11[8] + Q11[261]*w11[9] + Q11[262]*w11[10] + Q11[263]*w11[11] + Q11[264]*w11[12] + Q11[265]*w11[13] + Q11[266]*w11[14] + Q11[267]*w11[15] + Q11[268]*w11[16] + Q11[269]*w11[17] + w12[14];
w13[15] = + Q11[270]*w11[0] + Q11[271]*w11[1] + Q11[272]*w11[2] + Q11[273]*w11[3] + Q11[274]*w11[4] + Q11[275]*w11[5] + Q11[276]*w11[6] + Q11[277]*w11[7] + Q11[278]*w11[8] + Q11[279]*w11[9] + Q11[280]*w11[10] + Q11[281]*w11[11] + Q11[282]*w11[12] + Q11[283]*w11[13] + Q11[284]*w11[14] + Q11[285]*w11[15] + Q11[286]*w11[16] + Q11[287]*w11[17] + w12[15];
w13[16] = + Q11[288]*w11[0] + Q11[289]*w11[1] + Q11[290]*w11[2] + Q11[291]*w11[3] + Q11[292]*w11[4] + Q11[293]*w11[5] + Q11[294]*w11[6] + Q11[295]*w11[7] + Q11[296]*w11[8] + Q11[297]*w11[9] + Q11[298]*w11[10] + Q11[299]*w11[11] + Q11[300]*w11[12] + Q11[301]*w11[13] + Q11[302]*w11[14] + Q11[303]*w11[15] + Q11[304]*w11[16] + Q11[305]*w11[17] + w12[16];
w13[17] = + Q11[306]*w11[0] + Q11[307]*w11[1] + Q11[308]*w11[2] + Q11[309]*w11[3] + Q11[310]*w11[4] + Q11[311]*w11[5] + Q11[312]*w11[6] + Q11[313]*w11[7] + Q11[314]*w11[8] + Q11[315]*w11[9] + Q11[316]*w11[10] + Q11[317]*w11[11] + Q11[318]*w11[12] + Q11[319]*w11[13] + Q11[320]*w11[14] + Q11[321]*w11[15] + Q11[322]*w11[16] + Q11[323]*w11[17] + w12[17];
}

void acado_macASbar( real_t* const Gx1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8] + Gx1[9]*w11[9] + Gx1[10]*w11[10] + Gx1[11]*w11[11] + Gx1[12]*w11[12] + Gx1[13]*w11[13] + Gx1[14]*w11[14] + Gx1[15]*w11[15] + Gx1[16]*w11[16] + Gx1[17]*w11[17];
w12[1] += + Gx1[18]*w11[0] + Gx1[19]*w11[1] + Gx1[20]*w11[2] + Gx1[21]*w11[3] + Gx1[22]*w11[4] + Gx1[23]*w11[5] + Gx1[24]*w11[6] + Gx1[25]*w11[7] + Gx1[26]*w11[8] + Gx1[27]*w11[9] + Gx1[28]*w11[10] + Gx1[29]*w11[11] + Gx1[30]*w11[12] + Gx1[31]*w11[13] + Gx1[32]*w11[14] + Gx1[33]*w11[15] + Gx1[34]*w11[16] + Gx1[35]*w11[17];
w12[2] += + Gx1[36]*w11[0] + Gx1[37]*w11[1] + Gx1[38]*w11[2] + Gx1[39]*w11[3] + Gx1[40]*w11[4] + Gx1[41]*w11[5] + Gx1[42]*w11[6] + Gx1[43]*w11[7] + Gx1[44]*w11[8] + Gx1[45]*w11[9] + Gx1[46]*w11[10] + Gx1[47]*w11[11] + Gx1[48]*w11[12] + Gx1[49]*w11[13] + Gx1[50]*w11[14] + Gx1[51]*w11[15] + Gx1[52]*w11[16] + Gx1[53]*w11[17];
w12[3] += + Gx1[54]*w11[0] + Gx1[55]*w11[1] + Gx1[56]*w11[2] + Gx1[57]*w11[3] + Gx1[58]*w11[4] + Gx1[59]*w11[5] + Gx1[60]*w11[6] + Gx1[61]*w11[7] + Gx1[62]*w11[8] + Gx1[63]*w11[9] + Gx1[64]*w11[10] + Gx1[65]*w11[11] + Gx1[66]*w11[12] + Gx1[67]*w11[13] + Gx1[68]*w11[14] + Gx1[69]*w11[15] + Gx1[70]*w11[16] + Gx1[71]*w11[17];
w12[4] += + Gx1[72]*w11[0] + Gx1[73]*w11[1] + Gx1[74]*w11[2] + Gx1[75]*w11[3] + Gx1[76]*w11[4] + Gx1[77]*w11[5] + Gx1[78]*w11[6] + Gx1[79]*w11[7] + Gx1[80]*w11[8] + Gx1[81]*w11[9] + Gx1[82]*w11[10] + Gx1[83]*w11[11] + Gx1[84]*w11[12] + Gx1[85]*w11[13] + Gx1[86]*w11[14] + Gx1[87]*w11[15] + Gx1[88]*w11[16] + Gx1[89]*w11[17];
w12[5] += + Gx1[90]*w11[0] + Gx1[91]*w11[1] + Gx1[92]*w11[2] + Gx1[93]*w11[3] + Gx1[94]*w11[4] + Gx1[95]*w11[5] + Gx1[96]*w11[6] + Gx1[97]*w11[7] + Gx1[98]*w11[8] + Gx1[99]*w11[9] + Gx1[100]*w11[10] + Gx1[101]*w11[11] + Gx1[102]*w11[12] + Gx1[103]*w11[13] + Gx1[104]*w11[14] + Gx1[105]*w11[15] + Gx1[106]*w11[16] + Gx1[107]*w11[17];
w12[6] += + Gx1[108]*w11[0] + Gx1[109]*w11[1] + Gx1[110]*w11[2] + Gx1[111]*w11[3] + Gx1[112]*w11[4] + Gx1[113]*w11[5] + Gx1[114]*w11[6] + Gx1[115]*w11[7] + Gx1[116]*w11[8] + Gx1[117]*w11[9] + Gx1[118]*w11[10] + Gx1[119]*w11[11] + Gx1[120]*w11[12] + Gx1[121]*w11[13] + Gx1[122]*w11[14] + Gx1[123]*w11[15] + Gx1[124]*w11[16] + Gx1[125]*w11[17];
w12[7] += + Gx1[126]*w11[0] + Gx1[127]*w11[1] + Gx1[128]*w11[2] + Gx1[129]*w11[3] + Gx1[130]*w11[4] + Gx1[131]*w11[5] + Gx1[132]*w11[6] + Gx1[133]*w11[7] + Gx1[134]*w11[8] + Gx1[135]*w11[9] + Gx1[136]*w11[10] + Gx1[137]*w11[11] + Gx1[138]*w11[12] + Gx1[139]*w11[13] + Gx1[140]*w11[14] + Gx1[141]*w11[15] + Gx1[142]*w11[16] + Gx1[143]*w11[17];
w12[8] += + Gx1[144]*w11[0] + Gx1[145]*w11[1] + Gx1[146]*w11[2] + Gx1[147]*w11[3] + Gx1[148]*w11[4] + Gx1[149]*w11[5] + Gx1[150]*w11[6] + Gx1[151]*w11[7] + Gx1[152]*w11[8] + Gx1[153]*w11[9] + Gx1[154]*w11[10] + Gx1[155]*w11[11] + Gx1[156]*w11[12] + Gx1[157]*w11[13] + Gx1[158]*w11[14] + Gx1[159]*w11[15] + Gx1[160]*w11[16] + Gx1[161]*w11[17];
w12[9] += + Gx1[162]*w11[0] + Gx1[163]*w11[1] + Gx1[164]*w11[2] + Gx1[165]*w11[3] + Gx1[166]*w11[4] + Gx1[167]*w11[5] + Gx1[168]*w11[6] + Gx1[169]*w11[7] + Gx1[170]*w11[8] + Gx1[171]*w11[9] + Gx1[172]*w11[10] + Gx1[173]*w11[11] + Gx1[174]*w11[12] + Gx1[175]*w11[13] + Gx1[176]*w11[14] + Gx1[177]*w11[15] + Gx1[178]*w11[16] + Gx1[179]*w11[17];
w12[10] += + Gx1[180]*w11[0] + Gx1[181]*w11[1] + Gx1[182]*w11[2] + Gx1[183]*w11[3] + Gx1[184]*w11[4] + Gx1[185]*w11[5] + Gx1[186]*w11[6] + Gx1[187]*w11[7] + Gx1[188]*w11[8] + Gx1[189]*w11[9] + Gx1[190]*w11[10] + Gx1[191]*w11[11] + Gx1[192]*w11[12] + Gx1[193]*w11[13] + Gx1[194]*w11[14] + Gx1[195]*w11[15] + Gx1[196]*w11[16] + Gx1[197]*w11[17];
w12[11] += + Gx1[198]*w11[0] + Gx1[199]*w11[1] + Gx1[200]*w11[2] + Gx1[201]*w11[3] + Gx1[202]*w11[4] + Gx1[203]*w11[5] + Gx1[204]*w11[6] + Gx1[205]*w11[7] + Gx1[206]*w11[8] + Gx1[207]*w11[9] + Gx1[208]*w11[10] + Gx1[209]*w11[11] + Gx1[210]*w11[12] + Gx1[211]*w11[13] + Gx1[212]*w11[14] + Gx1[213]*w11[15] + Gx1[214]*w11[16] + Gx1[215]*w11[17];
w12[12] += + Gx1[216]*w11[0] + Gx1[217]*w11[1] + Gx1[218]*w11[2] + Gx1[219]*w11[3] + Gx1[220]*w11[4] + Gx1[221]*w11[5] + Gx1[222]*w11[6] + Gx1[223]*w11[7] + Gx1[224]*w11[8] + Gx1[225]*w11[9] + Gx1[226]*w11[10] + Gx1[227]*w11[11] + Gx1[228]*w11[12] + Gx1[229]*w11[13] + Gx1[230]*w11[14] + Gx1[231]*w11[15] + Gx1[232]*w11[16] + Gx1[233]*w11[17];
w12[13] += + Gx1[234]*w11[0] + Gx1[235]*w11[1] + Gx1[236]*w11[2] + Gx1[237]*w11[3] + Gx1[238]*w11[4] + Gx1[239]*w11[5] + Gx1[240]*w11[6] + Gx1[241]*w11[7] + Gx1[242]*w11[8] + Gx1[243]*w11[9] + Gx1[244]*w11[10] + Gx1[245]*w11[11] + Gx1[246]*w11[12] + Gx1[247]*w11[13] + Gx1[248]*w11[14] + Gx1[249]*w11[15] + Gx1[250]*w11[16] + Gx1[251]*w11[17];
w12[14] += + Gx1[252]*w11[0] + Gx1[253]*w11[1] + Gx1[254]*w11[2] + Gx1[255]*w11[3] + Gx1[256]*w11[4] + Gx1[257]*w11[5] + Gx1[258]*w11[6] + Gx1[259]*w11[7] + Gx1[260]*w11[8] + Gx1[261]*w11[9] + Gx1[262]*w11[10] + Gx1[263]*w11[11] + Gx1[264]*w11[12] + Gx1[265]*w11[13] + Gx1[266]*w11[14] + Gx1[267]*w11[15] + Gx1[268]*w11[16] + Gx1[269]*w11[17];
w12[15] += + Gx1[270]*w11[0] + Gx1[271]*w11[1] + Gx1[272]*w11[2] + Gx1[273]*w11[3] + Gx1[274]*w11[4] + Gx1[275]*w11[5] + Gx1[276]*w11[6] + Gx1[277]*w11[7] + Gx1[278]*w11[8] + Gx1[279]*w11[9] + Gx1[280]*w11[10] + Gx1[281]*w11[11] + Gx1[282]*w11[12] + Gx1[283]*w11[13] + Gx1[284]*w11[14] + Gx1[285]*w11[15] + Gx1[286]*w11[16] + Gx1[287]*w11[17];
w12[16] += + Gx1[288]*w11[0] + Gx1[289]*w11[1] + Gx1[290]*w11[2] + Gx1[291]*w11[3] + Gx1[292]*w11[4] + Gx1[293]*w11[5] + Gx1[294]*w11[6] + Gx1[295]*w11[7] + Gx1[296]*w11[8] + Gx1[297]*w11[9] + Gx1[298]*w11[10] + Gx1[299]*w11[11] + Gx1[300]*w11[12] + Gx1[301]*w11[13] + Gx1[302]*w11[14] + Gx1[303]*w11[15] + Gx1[304]*w11[16] + Gx1[305]*w11[17];
w12[17] += + Gx1[306]*w11[0] + Gx1[307]*w11[1] + Gx1[308]*w11[2] + Gx1[309]*w11[3] + Gx1[310]*w11[4] + Gx1[311]*w11[5] + Gx1[312]*w11[6] + Gx1[313]*w11[7] + Gx1[314]*w11[8] + Gx1[315]*w11[9] + Gx1[316]*w11[10] + Gx1[317]*w11[11] + Gx1[318]*w11[12] + Gx1[319]*w11[13] + Gx1[320]*w11[14] + Gx1[321]*w11[15] + Gx1[322]*w11[16] + Gx1[323]*w11[17];
}

void acado_expansionStep( real_t* const Gx1, real_t* const Gu1, real_t* const U1, real_t* const w11, real_t* const w12 )
{
w12[0] += + Gx1[0]*w11[0] + Gx1[1]*w11[1] + Gx1[2]*w11[2] + Gx1[3]*w11[3] + Gx1[4]*w11[4] + Gx1[5]*w11[5] + Gx1[6]*w11[6] + Gx1[7]*w11[7] + Gx1[8]*w11[8] + Gx1[9]*w11[9] + Gx1[10]*w11[10] + Gx1[11]*w11[11] + Gx1[12]*w11[12] + Gx1[13]*w11[13] + Gx1[14]*w11[14] + Gx1[15]*w11[15] + Gx1[16]*w11[16] + Gx1[17]*w11[17];
w12[1] += + Gx1[18]*w11[0] + Gx1[19]*w11[1] + Gx1[20]*w11[2] + Gx1[21]*w11[3] + Gx1[22]*w11[4] + Gx1[23]*w11[5] + Gx1[24]*w11[6] + Gx1[25]*w11[7] + Gx1[26]*w11[8] + Gx1[27]*w11[9] + Gx1[28]*w11[10] + Gx1[29]*w11[11] + Gx1[30]*w11[12] + Gx1[31]*w11[13] + Gx1[32]*w11[14] + Gx1[33]*w11[15] + Gx1[34]*w11[16] + Gx1[35]*w11[17];
w12[2] += + Gx1[36]*w11[0] + Gx1[37]*w11[1] + Gx1[38]*w11[2] + Gx1[39]*w11[3] + Gx1[40]*w11[4] + Gx1[41]*w11[5] + Gx1[42]*w11[6] + Gx1[43]*w11[7] + Gx1[44]*w11[8] + Gx1[45]*w11[9] + Gx1[46]*w11[10] + Gx1[47]*w11[11] + Gx1[48]*w11[12] + Gx1[49]*w11[13] + Gx1[50]*w11[14] + Gx1[51]*w11[15] + Gx1[52]*w11[16] + Gx1[53]*w11[17];
w12[3] += + Gx1[54]*w11[0] + Gx1[55]*w11[1] + Gx1[56]*w11[2] + Gx1[57]*w11[3] + Gx1[58]*w11[4] + Gx1[59]*w11[5] + Gx1[60]*w11[6] + Gx1[61]*w11[7] + Gx1[62]*w11[8] + Gx1[63]*w11[9] + Gx1[64]*w11[10] + Gx1[65]*w11[11] + Gx1[66]*w11[12] + Gx1[67]*w11[13] + Gx1[68]*w11[14] + Gx1[69]*w11[15] + Gx1[70]*w11[16] + Gx1[71]*w11[17];
w12[4] += + Gx1[72]*w11[0] + Gx1[73]*w11[1] + Gx1[74]*w11[2] + Gx1[75]*w11[3] + Gx1[76]*w11[4] + Gx1[77]*w11[5] + Gx1[78]*w11[6] + Gx1[79]*w11[7] + Gx1[80]*w11[8] + Gx1[81]*w11[9] + Gx1[82]*w11[10] + Gx1[83]*w11[11] + Gx1[84]*w11[12] + Gx1[85]*w11[13] + Gx1[86]*w11[14] + Gx1[87]*w11[15] + Gx1[88]*w11[16] + Gx1[89]*w11[17];
w12[5] += + Gx1[90]*w11[0] + Gx1[91]*w11[1] + Gx1[92]*w11[2] + Gx1[93]*w11[3] + Gx1[94]*w11[4] + Gx1[95]*w11[5] + Gx1[96]*w11[6] + Gx1[97]*w11[7] + Gx1[98]*w11[8] + Gx1[99]*w11[9] + Gx1[100]*w11[10] + Gx1[101]*w11[11] + Gx1[102]*w11[12] + Gx1[103]*w11[13] + Gx1[104]*w11[14] + Gx1[105]*w11[15] + Gx1[106]*w11[16] + Gx1[107]*w11[17];
w12[6] += + Gx1[108]*w11[0] + Gx1[109]*w11[1] + Gx1[110]*w11[2] + Gx1[111]*w11[3] + Gx1[112]*w11[4] + Gx1[113]*w11[5] + Gx1[114]*w11[6] + Gx1[115]*w11[7] + Gx1[116]*w11[8] + Gx1[117]*w11[9] + Gx1[118]*w11[10] + Gx1[119]*w11[11] + Gx1[120]*w11[12] + Gx1[121]*w11[13] + Gx1[122]*w11[14] + Gx1[123]*w11[15] + Gx1[124]*w11[16] + Gx1[125]*w11[17];
w12[7] += + Gx1[126]*w11[0] + Gx1[127]*w11[1] + Gx1[128]*w11[2] + Gx1[129]*w11[3] + Gx1[130]*w11[4] + Gx1[131]*w11[5] + Gx1[132]*w11[6] + Gx1[133]*w11[7] + Gx1[134]*w11[8] + Gx1[135]*w11[9] + Gx1[136]*w11[10] + Gx1[137]*w11[11] + Gx1[138]*w11[12] + Gx1[139]*w11[13] + Gx1[140]*w11[14] + Gx1[141]*w11[15] + Gx1[142]*w11[16] + Gx1[143]*w11[17];
w12[8] += + Gx1[144]*w11[0] + Gx1[145]*w11[1] + Gx1[146]*w11[2] + Gx1[147]*w11[3] + Gx1[148]*w11[4] + Gx1[149]*w11[5] + Gx1[150]*w11[6] + Gx1[151]*w11[7] + Gx1[152]*w11[8] + Gx1[153]*w11[9] + Gx1[154]*w11[10] + Gx1[155]*w11[11] + Gx1[156]*w11[12] + Gx1[157]*w11[13] + Gx1[158]*w11[14] + Gx1[159]*w11[15] + Gx1[160]*w11[16] + Gx1[161]*w11[17];
w12[9] += + Gx1[162]*w11[0] + Gx1[163]*w11[1] + Gx1[164]*w11[2] + Gx1[165]*w11[3] + Gx1[166]*w11[4] + Gx1[167]*w11[5] + Gx1[168]*w11[6] + Gx1[169]*w11[7] + Gx1[170]*w11[8] + Gx1[171]*w11[9] + Gx1[172]*w11[10] + Gx1[173]*w11[11] + Gx1[174]*w11[12] + Gx1[175]*w11[13] + Gx1[176]*w11[14] + Gx1[177]*w11[15] + Gx1[178]*w11[16] + Gx1[179]*w11[17];
w12[10] += + Gx1[180]*w11[0] + Gx1[181]*w11[1] + Gx1[182]*w11[2] + Gx1[183]*w11[3] + Gx1[184]*w11[4] + Gx1[185]*w11[5] + Gx1[186]*w11[6] + Gx1[187]*w11[7] + Gx1[188]*w11[8] + Gx1[189]*w11[9] + Gx1[190]*w11[10] + Gx1[191]*w11[11] + Gx1[192]*w11[12] + Gx1[193]*w11[13] + Gx1[194]*w11[14] + Gx1[195]*w11[15] + Gx1[196]*w11[16] + Gx1[197]*w11[17];
w12[11] += + Gx1[198]*w11[0] + Gx1[199]*w11[1] + Gx1[200]*w11[2] + Gx1[201]*w11[3] + Gx1[202]*w11[4] + Gx1[203]*w11[5] + Gx1[204]*w11[6] + Gx1[205]*w11[7] + Gx1[206]*w11[8] + Gx1[207]*w11[9] + Gx1[208]*w11[10] + Gx1[209]*w11[11] + Gx1[210]*w11[12] + Gx1[211]*w11[13] + Gx1[212]*w11[14] + Gx1[213]*w11[15] + Gx1[214]*w11[16] + Gx1[215]*w11[17];
w12[12] += + Gx1[216]*w11[0] + Gx1[217]*w11[1] + Gx1[218]*w11[2] + Gx1[219]*w11[3] + Gx1[220]*w11[4] + Gx1[221]*w11[5] + Gx1[222]*w11[6] + Gx1[223]*w11[7] + Gx1[224]*w11[8] + Gx1[225]*w11[9] + Gx1[226]*w11[10] + Gx1[227]*w11[11] + Gx1[228]*w11[12] + Gx1[229]*w11[13] + Gx1[230]*w11[14] + Gx1[231]*w11[15] + Gx1[232]*w11[16] + Gx1[233]*w11[17];
w12[13] += + Gx1[234]*w11[0] + Gx1[235]*w11[1] + Gx1[236]*w11[2] + Gx1[237]*w11[3] + Gx1[238]*w11[4] + Gx1[239]*w11[5] + Gx1[240]*w11[6] + Gx1[241]*w11[7] + Gx1[242]*w11[8] + Gx1[243]*w11[9] + Gx1[244]*w11[10] + Gx1[245]*w11[11] + Gx1[246]*w11[12] + Gx1[247]*w11[13] + Gx1[248]*w11[14] + Gx1[249]*w11[15] + Gx1[250]*w11[16] + Gx1[251]*w11[17];
w12[14] += + Gx1[252]*w11[0] + Gx1[253]*w11[1] + Gx1[254]*w11[2] + Gx1[255]*w11[3] + Gx1[256]*w11[4] + Gx1[257]*w11[5] + Gx1[258]*w11[6] + Gx1[259]*w11[7] + Gx1[260]*w11[8] + Gx1[261]*w11[9] + Gx1[262]*w11[10] + Gx1[263]*w11[11] + Gx1[264]*w11[12] + Gx1[265]*w11[13] + Gx1[266]*w11[14] + Gx1[267]*w11[15] + Gx1[268]*w11[16] + Gx1[269]*w11[17];
w12[15] += + Gx1[270]*w11[0] + Gx1[271]*w11[1] + Gx1[272]*w11[2] + Gx1[273]*w11[3] + Gx1[274]*w11[4] + Gx1[275]*w11[5] + Gx1[276]*w11[6] + Gx1[277]*w11[7] + Gx1[278]*w11[8] + Gx1[279]*w11[9] + Gx1[280]*w11[10] + Gx1[281]*w11[11] + Gx1[282]*w11[12] + Gx1[283]*w11[13] + Gx1[284]*w11[14] + Gx1[285]*w11[15] + Gx1[286]*w11[16] + Gx1[287]*w11[17];
w12[16] += + Gx1[288]*w11[0] + Gx1[289]*w11[1] + Gx1[290]*w11[2] + Gx1[291]*w11[3] + Gx1[292]*w11[4] + Gx1[293]*w11[5] + Gx1[294]*w11[6] + Gx1[295]*w11[7] + Gx1[296]*w11[8] + Gx1[297]*w11[9] + Gx1[298]*w11[10] + Gx1[299]*w11[11] + Gx1[300]*w11[12] + Gx1[301]*w11[13] + Gx1[302]*w11[14] + Gx1[303]*w11[15] + Gx1[304]*w11[16] + Gx1[305]*w11[17];
w12[17] += + Gx1[306]*w11[0] + Gx1[307]*w11[1] + Gx1[308]*w11[2] + Gx1[309]*w11[3] + Gx1[310]*w11[4] + Gx1[311]*w11[5] + Gx1[312]*w11[6] + Gx1[313]*w11[7] + Gx1[314]*w11[8] + Gx1[315]*w11[9] + Gx1[316]*w11[10] + Gx1[317]*w11[11] + Gx1[318]*w11[12] + Gx1[319]*w11[13] + Gx1[320]*w11[14] + Gx1[321]*w11[15] + Gx1[322]*w11[16] + Gx1[323]*w11[17];
w12[0] += + Gu1[0]*U1[0] + Gu1[1]*U1[1] + Gu1[2]*U1[2] + Gu1[3]*U1[3] + Gu1[4]*U1[4] + Gu1[5]*U1[5];
w12[1] += + Gu1[6]*U1[0] + Gu1[7]*U1[1] + Gu1[8]*U1[2] + Gu1[9]*U1[3] + Gu1[10]*U1[4] + Gu1[11]*U1[5];
w12[2] += + Gu1[12]*U1[0] + Gu1[13]*U1[1] + Gu1[14]*U1[2] + Gu1[15]*U1[3] + Gu1[16]*U1[4] + Gu1[17]*U1[5];
w12[3] += + Gu1[18]*U1[0] + Gu1[19]*U1[1] + Gu1[20]*U1[2] + Gu1[21]*U1[3] + Gu1[22]*U1[4] + Gu1[23]*U1[5];
w12[4] += + Gu1[24]*U1[0] + Gu1[25]*U1[1] + Gu1[26]*U1[2] + Gu1[27]*U1[3] + Gu1[28]*U1[4] + Gu1[29]*U1[5];
w12[5] += + Gu1[30]*U1[0] + Gu1[31]*U1[1] + Gu1[32]*U1[2] + Gu1[33]*U1[3] + Gu1[34]*U1[4] + Gu1[35]*U1[5];
w12[6] += + Gu1[36]*U1[0] + Gu1[37]*U1[1] + Gu1[38]*U1[2] + Gu1[39]*U1[3] + Gu1[40]*U1[4] + Gu1[41]*U1[5];
w12[7] += + Gu1[42]*U1[0] + Gu1[43]*U1[1] + Gu1[44]*U1[2] + Gu1[45]*U1[3] + Gu1[46]*U1[4] + Gu1[47]*U1[5];
w12[8] += + Gu1[48]*U1[0] + Gu1[49]*U1[1] + Gu1[50]*U1[2] + Gu1[51]*U1[3] + Gu1[52]*U1[4] + Gu1[53]*U1[5];
w12[9] += + Gu1[54]*U1[0] + Gu1[55]*U1[1] + Gu1[56]*U1[2] + Gu1[57]*U1[3] + Gu1[58]*U1[4] + Gu1[59]*U1[5];
w12[10] += + Gu1[60]*U1[0] + Gu1[61]*U1[1] + Gu1[62]*U1[2] + Gu1[63]*U1[3] + Gu1[64]*U1[4] + Gu1[65]*U1[5];
w12[11] += + Gu1[66]*U1[0] + Gu1[67]*U1[1] + Gu1[68]*U1[2] + Gu1[69]*U1[3] + Gu1[70]*U1[4] + Gu1[71]*U1[5];
w12[12] += + Gu1[72]*U1[0] + Gu1[73]*U1[1] + Gu1[74]*U1[2] + Gu1[75]*U1[3] + Gu1[76]*U1[4] + Gu1[77]*U1[5];
w12[13] += + Gu1[78]*U1[0] + Gu1[79]*U1[1] + Gu1[80]*U1[2] + Gu1[81]*U1[3] + Gu1[82]*U1[4] + Gu1[83]*U1[5];
w12[14] += + Gu1[84]*U1[0] + Gu1[85]*U1[1] + Gu1[86]*U1[2] + Gu1[87]*U1[3] + Gu1[88]*U1[4] + Gu1[89]*U1[5];
w12[15] += + Gu1[90]*U1[0] + Gu1[91]*U1[1] + Gu1[92]*U1[2] + Gu1[93]*U1[3] + Gu1[94]*U1[4] + Gu1[95]*U1[5];
w12[16] += + Gu1[96]*U1[0] + Gu1[97]*U1[1] + Gu1[98]*U1[2] + Gu1[99]*U1[3] + Gu1[100]*U1[4] + Gu1[101]*U1[5];
w12[17] += + Gu1[102]*U1[0] + Gu1[103]*U1[1] + Gu1[104]*U1[2] + Gu1[105]*U1[3] + Gu1[106]*U1[4] + Gu1[107]*U1[5];
}

void acado_copyHTH( int iRow, int iCol )
{
acadoWorkspace.H[(iRow * 360) + (iCol * 6)] = acadoWorkspace.H[(iCol * 360) + (iRow * 6)];
acadoWorkspace.H[(iRow * 360) + (iCol * 6 + 1)] = acadoWorkspace.H[(iCol * 360 + 60) + (iRow * 6)];
acadoWorkspace.H[(iRow * 360) + (iCol * 6 + 2)] = acadoWorkspace.H[(iCol * 360 + 120) + (iRow * 6)];
acadoWorkspace.H[(iRow * 360) + (iCol * 6 + 3)] = acadoWorkspace.H[(iCol * 360 + 180) + (iRow * 6)];
acadoWorkspace.H[(iRow * 360) + (iCol * 6 + 4)] = acadoWorkspace.H[(iCol * 360 + 240) + (iRow * 6)];
acadoWorkspace.H[(iRow * 360) + (iCol * 6 + 5)] = acadoWorkspace.H[(iCol * 360 + 300) + (iRow * 6)];
acadoWorkspace.H[(iRow * 360 + 60) + (iCol * 6)] = acadoWorkspace.H[(iCol * 360) + (iRow * 6 + 1)];
acadoWorkspace.H[(iRow * 360 + 60) + (iCol * 6 + 1)] = acadoWorkspace.H[(iCol * 360 + 60) + (iRow * 6 + 1)];
acadoWorkspace.H[(iRow * 360 + 60) + (iCol * 6 + 2)] = acadoWorkspace.H[(iCol * 360 + 120) + (iRow * 6 + 1)];
acadoWorkspace.H[(iRow * 360 + 60) + (iCol * 6 + 3)] = acadoWorkspace.H[(iCol * 360 + 180) + (iRow * 6 + 1)];
acadoWorkspace.H[(iRow * 360 + 60) + (iCol * 6 + 4)] = acadoWorkspace.H[(iCol * 360 + 240) + (iRow * 6 + 1)];
acadoWorkspace.H[(iRow * 360 + 60) + (iCol * 6 + 5)] = acadoWorkspace.H[(iCol * 360 + 300) + (iRow * 6 + 1)];
acadoWorkspace.H[(iRow * 360 + 120) + (iCol * 6)] = acadoWorkspace.H[(iCol * 360) + (iRow * 6 + 2)];
acadoWorkspace.H[(iRow * 360 + 120) + (iCol * 6 + 1)] = acadoWorkspace.H[(iCol * 360 + 60) + (iRow * 6 + 2)];
acadoWorkspace.H[(iRow * 360 + 120) + (iCol * 6 + 2)] = acadoWorkspace.H[(iCol * 360 + 120) + (iRow * 6 + 2)];
acadoWorkspace.H[(iRow * 360 + 120) + (iCol * 6 + 3)] = acadoWorkspace.H[(iCol * 360 + 180) + (iRow * 6 + 2)];
acadoWorkspace.H[(iRow * 360 + 120) + (iCol * 6 + 4)] = acadoWorkspace.H[(iCol * 360 + 240) + (iRow * 6 + 2)];
acadoWorkspace.H[(iRow * 360 + 120) + (iCol * 6 + 5)] = acadoWorkspace.H[(iCol * 360 + 300) + (iRow * 6 + 2)];
acadoWorkspace.H[(iRow * 360 + 180) + (iCol * 6)] = acadoWorkspace.H[(iCol * 360) + (iRow * 6 + 3)];
acadoWorkspace.H[(iRow * 360 + 180) + (iCol * 6 + 1)] = acadoWorkspace.H[(iCol * 360 + 60) + (iRow * 6 + 3)];
acadoWorkspace.H[(iRow * 360 + 180) + (iCol * 6 + 2)] = acadoWorkspace.H[(iCol * 360 + 120) + (iRow * 6 + 3)];
acadoWorkspace.H[(iRow * 360 + 180) + (iCol * 6 + 3)] = acadoWorkspace.H[(iCol * 360 + 180) + (iRow * 6 + 3)];
acadoWorkspace.H[(iRow * 360 + 180) + (iCol * 6 + 4)] = acadoWorkspace.H[(iCol * 360 + 240) + (iRow * 6 + 3)];
acadoWorkspace.H[(iRow * 360 + 180) + (iCol * 6 + 5)] = acadoWorkspace.H[(iCol * 360 + 300) + (iRow * 6 + 3)];
acadoWorkspace.H[(iRow * 360 + 240) + (iCol * 6)] = acadoWorkspace.H[(iCol * 360) + (iRow * 6 + 4)];
acadoWorkspace.H[(iRow * 360 + 240) + (iCol * 6 + 1)] = acadoWorkspace.H[(iCol * 360 + 60) + (iRow * 6 + 4)];
acadoWorkspace.H[(iRow * 360 + 240) + (iCol * 6 + 2)] = acadoWorkspace.H[(iCol * 360 + 120) + (iRow * 6 + 4)];
acadoWorkspace.H[(iRow * 360 + 240) + (iCol * 6 + 3)] = acadoWorkspace.H[(iCol * 360 + 180) + (iRow * 6 + 4)];
acadoWorkspace.H[(iRow * 360 + 240) + (iCol * 6 + 4)] = acadoWorkspace.H[(iCol * 360 + 240) + (iRow * 6 + 4)];
acadoWorkspace.H[(iRow * 360 + 240) + (iCol * 6 + 5)] = acadoWorkspace.H[(iCol * 360 + 300) + (iRow * 6 + 4)];
acadoWorkspace.H[(iRow * 360 + 300) + (iCol * 6)] = acadoWorkspace.H[(iCol * 360) + (iRow * 6 + 5)];
acadoWorkspace.H[(iRow * 360 + 300) + (iCol * 6 + 1)] = acadoWorkspace.H[(iCol * 360 + 60) + (iRow * 6 + 5)];
acadoWorkspace.H[(iRow * 360 + 300) + (iCol * 6 + 2)] = acadoWorkspace.H[(iCol * 360 + 120) + (iRow * 6 + 5)];
acadoWorkspace.H[(iRow * 360 + 300) + (iCol * 6 + 3)] = acadoWorkspace.H[(iCol * 360 + 180) + (iRow * 6 + 5)];
acadoWorkspace.H[(iRow * 360 + 300) + (iCol * 6 + 4)] = acadoWorkspace.H[(iCol * 360 + 240) + (iRow * 6 + 5)];
acadoWorkspace.H[(iRow * 360 + 300) + (iCol * 6 + 5)] = acadoWorkspace.H[(iCol * 360 + 300) + (iRow * 6 + 5)];
}

void acado_multRDy( real_t* const R2, real_t* const Dy1, real_t* const RDy1 )
{
RDy1[0] = + R2[0]*Dy1[0] + R2[1]*Dy1[1] + R2[2]*Dy1[2] + R2[3]*Dy1[3] + R2[4]*Dy1[4] + R2[5]*Dy1[5] + R2[6]*Dy1[6] + R2[7]*Dy1[7] + R2[8]*Dy1[8] + R2[9]*Dy1[9] + R2[10]*Dy1[10] + R2[11]*Dy1[11] + R2[12]*Dy1[12] + R2[13]*Dy1[13] + R2[14]*Dy1[14] + R2[15]*Dy1[15] + R2[16]*Dy1[16] + R2[17]*Dy1[17] + R2[18]*Dy1[18];
RDy1[1] = + R2[19]*Dy1[0] + R2[20]*Dy1[1] + R2[21]*Dy1[2] + R2[22]*Dy1[3] + R2[23]*Dy1[4] + R2[24]*Dy1[5] + R2[25]*Dy1[6] + R2[26]*Dy1[7] + R2[27]*Dy1[8] + R2[28]*Dy1[9] + R2[29]*Dy1[10] + R2[30]*Dy1[11] + R2[31]*Dy1[12] + R2[32]*Dy1[13] + R2[33]*Dy1[14] + R2[34]*Dy1[15] + R2[35]*Dy1[16] + R2[36]*Dy1[17] + R2[37]*Dy1[18];
RDy1[2] = + R2[38]*Dy1[0] + R2[39]*Dy1[1] + R2[40]*Dy1[2] + R2[41]*Dy1[3] + R2[42]*Dy1[4] + R2[43]*Dy1[5] + R2[44]*Dy1[6] + R2[45]*Dy1[7] + R2[46]*Dy1[8] + R2[47]*Dy1[9] + R2[48]*Dy1[10] + R2[49]*Dy1[11] + R2[50]*Dy1[12] + R2[51]*Dy1[13] + R2[52]*Dy1[14] + R2[53]*Dy1[15] + R2[54]*Dy1[16] + R2[55]*Dy1[17] + R2[56]*Dy1[18];
RDy1[3] = + R2[57]*Dy1[0] + R2[58]*Dy1[1] + R2[59]*Dy1[2] + R2[60]*Dy1[3] + R2[61]*Dy1[4] + R2[62]*Dy1[5] + R2[63]*Dy1[6] + R2[64]*Dy1[7] + R2[65]*Dy1[8] + R2[66]*Dy1[9] + R2[67]*Dy1[10] + R2[68]*Dy1[11] + R2[69]*Dy1[12] + R2[70]*Dy1[13] + R2[71]*Dy1[14] + R2[72]*Dy1[15] + R2[73]*Dy1[16] + R2[74]*Dy1[17] + R2[75]*Dy1[18];
RDy1[4] = + R2[76]*Dy1[0] + R2[77]*Dy1[1] + R2[78]*Dy1[2] + R2[79]*Dy1[3] + R2[80]*Dy1[4] + R2[81]*Dy1[5] + R2[82]*Dy1[6] + R2[83]*Dy1[7] + R2[84]*Dy1[8] + R2[85]*Dy1[9] + R2[86]*Dy1[10] + R2[87]*Dy1[11] + R2[88]*Dy1[12] + R2[89]*Dy1[13] + R2[90]*Dy1[14] + R2[91]*Dy1[15] + R2[92]*Dy1[16] + R2[93]*Dy1[17] + R2[94]*Dy1[18];
RDy1[5] = + R2[95]*Dy1[0] + R2[96]*Dy1[1] + R2[97]*Dy1[2] + R2[98]*Dy1[3] + R2[99]*Dy1[4] + R2[100]*Dy1[5] + R2[101]*Dy1[6] + R2[102]*Dy1[7] + R2[103]*Dy1[8] + R2[104]*Dy1[9] + R2[105]*Dy1[10] + R2[106]*Dy1[11] + R2[107]*Dy1[12] + R2[108]*Dy1[13] + R2[109]*Dy1[14] + R2[110]*Dy1[15] + R2[111]*Dy1[16] + R2[112]*Dy1[17] + R2[113]*Dy1[18];
}

void acado_multQDy( real_t* const Q2, real_t* const Dy1, real_t* const QDy1 )
{
QDy1[0] = + Q2[0]*Dy1[0] + Q2[1]*Dy1[1] + Q2[2]*Dy1[2] + Q2[3]*Dy1[3] + Q2[4]*Dy1[4] + Q2[5]*Dy1[5] + Q2[6]*Dy1[6] + Q2[7]*Dy1[7] + Q2[8]*Dy1[8] + Q2[9]*Dy1[9] + Q2[10]*Dy1[10] + Q2[11]*Dy1[11] + Q2[12]*Dy1[12] + Q2[13]*Dy1[13] + Q2[14]*Dy1[14] + Q2[15]*Dy1[15] + Q2[16]*Dy1[16] + Q2[17]*Dy1[17] + Q2[18]*Dy1[18];
QDy1[1] = + Q2[19]*Dy1[0] + Q2[20]*Dy1[1] + Q2[21]*Dy1[2] + Q2[22]*Dy1[3] + Q2[23]*Dy1[4] + Q2[24]*Dy1[5] + Q2[25]*Dy1[6] + Q2[26]*Dy1[7] + Q2[27]*Dy1[8] + Q2[28]*Dy1[9] + Q2[29]*Dy1[10] + Q2[30]*Dy1[11] + Q2[31]*Dy1[12] + Q2[32]*Dy1[13] + Q2[33]*Dy1[14] + Q2[34]*Dy1[15] + Q2[35]*Dy1[16] + Q2[36]*Dy1[17] + Q2[37]*Dy1[18];
QDy1[2] = + Q2[38]*Dy1[0] + Q2[39]*Dy1[1] + Q2[40]*Dy1[2] + Q2[41]*Dy1[3] + Q2[42]*Dy1[4] + Q2[43]*Dy1[5] + Q2[44]*Dy1[6] + Q2[45]*Dy1[7] + Q2[46]*Dy1[8] + Q2[47]*Dy1[9] + Q2[48]*Dy1[10] + Q2[49]*Dy1[11] + Q2[50]*Dy1[12] + Q2[51]*Dy1[13] + Q2[52]*Dy1[14] + Q2[53]*Dy1[15] + Q2[54]*Dy1[16] + Q2[55]*Dy1[17] + Q2[56]*Dy1[18];
QDy1[3] = + Q2[57]*Dy1[0] + Q2[58]*Dy1[1] + Q2[59]*Dy1[2] + Q2[60]*Dy1[3] + Q2[61]*Dy1[4] + Q2[62]*Dy1[5] + Q2[63]*Dy1[6] + Q2[64]*Dy1[7] + Q2[65]*Dy1[8] + Q2[66]*Dy1[9] + Q2[67]*Dy1[10] + Q2[68]*Dy1[11] + Q2[69]*Dy1[12] + Q2[70]*Dy1[13] + Q2[71]*Dy1[14] + Q2[72]*Dy1[15] + Q2[73]*Dy1[16] + Q2[74]*Dy1[17] + Q2[75]*Dy1[18];
QDy1[4] = + Q2[76]*Dy1[0] + Q2[77]*Dy1[1] + Q2[78]*Dy1[2] + Q2[79]*Dy1[3] + Q2[80]*Dy1[4] + Q2[81]*Dy1[5] + Q2[82]*Dy1[6] + Q2[83]*Dy1[7] + Q2[84]*Dy1[8] + Q2[85]*Dy1[9] + Q2[86]*Dy1[10] + Q2[87]*Dy1[11] + Q2[88]*Dy1[12] + Q2[89]*Dy1[13] + Q2[90]*Dy1[14] + Q2[91]*Dy1[15] + Q2[92]*Dy1[16] + Q2[93]*Dy1[17] + Q2[94]*Dy1[18];
QDy1[5] = + Q2[95]*Dy1[0] + Q2[96]*Dy1[1] + Q2[97]*Dy1[2] + Q2[98]*Dy1[3] + Q2[99]*Dy1[4] + Q2[100]*Dy1[5] + Q2[101]*Dy1[6] + Q2[102]*Dy1[7] + Q2[103]*Dy1[8] + Q2[104]*Dy1[9] + Q2[105]*Dy1[10] + Q2[106]*Dy1[11] + Q2[107]*Dy1[12] + Q2[108]*Dy1[13] + Q2[109]*Dy1[14] + Q2[110]*Dy1[15] + Q2[111]*Dy1[16] + Q2[112]*Dy1[17] + Q2[113]*Dy1[18];
QDy1[6] = + Q2[114]*Dy1[0] + Q2[115]*Dy1[1] + Q2[116]*Dy1[2] + Q2[117]*Dy1[3] + Q2[118]*Dy1[4] + Q2[119]*Dy1[5] + Q2[120]*Dy1[6] + Q2[121]*Dy1[7] + Q2[122]*Dy1[8] + Q2[123]*Dy1[9] + Q2[124]*Dy1[10] + Q2[125]*Dy1[11] + Q2[126]*Dy1[12] + Q2[127]*Dy1[13] + Q2[128]*Dy1[14] + Q2[129]*Dy1[15] + Q2[130]*Dy1[16] + Q2[131]*Dy1[17] + Q2[132]*Dy1[18];
QDy1[7] = + Q2[133]*Dy1[0] + Q2[134]*Dy1[1] + Q2[135]*Dy1[2] + Q2[136]*Dy1[3] + Q2[137]*Dy1[4] + Q2[138]*Dy1[5] + Q2[139]*Dy1[6] + Q2[140]*Dy1[7] + Q2[141]*Dy1[8] + Q2[142]*Dy1[9] + Q2[143]*Dy1[10] + Q2[144]*Dy1[11] + Q2[145]*Dy1[12] + Q2[146]*Dy1[13] + Q2[147]*Dy1[14] + Q2[148]*Dy1[15] + Q2[149]*Dy1[16] + Q2[150]*Dy1[17] + Q2[151]*Dy1[18];
QDy1[8] = + Q2[152]*Dy1[0] + Q2[153]*Dy1[1] + Q2[154]*Dy1[2] + Q2[155]*Dy1[3] + Q2[156]*Dy1[4] + Q2[157]*Dy1[5] + Q2[158]*Dy1[6] + Q2[159]*Dy1[7] + Q2[160]*Dy1[8] + Q2[161]*Dy1[9] + Q2[162]*Dy1[10] + Q2[163]*Dy1[11] + Q2[164]*Dy1[12] + Q2[165]*Dy1[13] + Q2[166]*Dy1[14] + Q2[167]*Dy1[15] + Q2[168]*Dy1[16] + Q2[169]*Dy1[17] + Q2[170]*Dy1[18];
QDy1[9] = + Q2[171]*Dy1[0] + Q2[172]*Dy1[1] + Q2[173]*Dy1[2] + Q2[174]*Dy1[3] + Q2[175]*Dy1[4] + Q2[176]*Dy1[5] + Q2[177]*Dy1[6] + Q2[178]*Dy1[7] + Q2[179]*Dy1[8] + Q2[180]*Dy1[9] + Q2[181]*Dy1[10] + Q2[182]*Dy1[11] + Q2[183]*Dy1[12] + Q2[184]*Dy1[13] + Q2[185]*Dy1[14] + Q2[186]*Dy1[15] + Q2[187]*Dy1[16] + Q2[188]*Dy1[17] + Q2[189]*Dy1[18];
QDy1[10] = + Q2[190]*Dy1[0] + Q2[191]*Dy1[1] + Q2[192]*Dy1[2] + Q2[193]*Dy1[3] + Q2[194]*Dy1[4] + Q2[195]*Dy1[5] + Q2[196]*Dy1[6] + Q2[197]*Dy1[7] + Q2[198]*Dy1[8] + Q2[199]*Dy1[9] + Q2[200]*Dy1[10] + Q2[201]*Dy1[11] + Q2[202]*Dy1[12] + Q2[203]*Dy1[13] + Q2[204]*Dy1[14] + Q2[205]*Dy1[15] + Q2[206]*Dy1[16] + Q2[207]*Dy1[17] + Q2[208]*Dy1[18];
QDy1[11] = + Q2[209]*Dy1[0] + Q2[210]*Dy1[1] + Q2[211]*Dy1[2] + Q2[212]*Dy1[3] + Q2[213]*Dy1[4] + Q2[214]*Dy1[5] + Q2[215]*Dy1[6] + Q2[216]*Dy1[7] + Q2[217]*Dy1[8] + Q2[218]*Dy1[9] + Q2[219]*Dy1[10] + Q2[220]*Dy1[11] + Q2[221]*Dy1[12] + Q2[222]*Dy1[13] + Q2[223]*Dy1[14] + Q2[224]*Dy1[15] + Q2[225]*Dy1[16] + Q2[226]*Dy1[17] + Q2[227]*Dy1[18];
QDy1[12] = + Q2[228]*Dy1[0] + Q2[229]*Dy1[1] + Q2[230]*Dy1[2] + Q2[231]*Dy1[3] + Q2[232]*Dy1[4] + Q2[233]*Dy1[5] + Q2[234]*Dy1[6] + Q2[235]*Dy1[7] + Q2[236]*Dy1[8] + Q2[237]*Dy1[9] + Q2[238]*Dy1[10] + Q2[239]*Dy1[11] + Q2[240]*Dy1[12] + Q2[241]*Dy1[13] + Q2[242]*Dy1[14] + Q2[243]*Dy1[15] + Q2[244]*Dy1[16] + Q2[245]*Dy1[17] + Q2[246]*Dy1[18];
QDy1[13] = + Q2[247]*Dy1[0] + Q2[248]*Dy1[1] + Q2[249]*Dy1[2] + Q2[250]*Dy1[3] + Q2[251]*Dy1[4] + Q2[252]*Dy1[5] + Q2[253]*Dy1[6] + Q2[254]*Dy1[7] + Q2[255]*Dy1[8] + Q2[256]*Dy1[9] + Q2[257]*Dy1[10] + Q2[258]*Dy1[11] + Q2[259]*Dy1[12] + Q2[260]*Dy1[13] + Q2[261]*Dy1[14] + Q2[262]*Dy1[15] + Q2[263]*Dy1[16] + Q2[264]*Dy1[17] + Q2[265]*Dy1[18];
QDy1[14] = + Q2[266]*Dy1[0] + Q2[267]*Dy1[1] + Q2[268]*Dy1[2] + Q2[269]*Dy1[3] + Q2[270]*Dy1[4] + Q2[271]*Dy1[5] + Q2[272]*Dy1[6] + Q2[273]*Dy1[7] + Q2[274]*Dy1[8] + Q2[275]*Dy1[9] + Q2[276]*Dy1[10] + Q2[277]*Dy1[11] + Q2[278]*Dy1[12] + Q2[279]*Dy1[13] + Q2[280]*Dy1[14] + Q2[281]*Dy1[15] + Q2[282]*Dy1[16] + Q2[283]*Dy1[17] + Q2[284]*Dy1[18];
QDy1[15] = + Q2[285]*Dy1[0] + Q2[286]*Dy1[1] + Q2[287]*Dy1[2] + Q2[288]*Dy1[3] + Q2[289]*Dy1[4] + Q2[290]*Dy1[5] + Q2[291]*Dy1[6] + Q2[292]*Dy1[7] + Q2[293]*Dy1[8] + Q2[294]*Dy1[9] + Q2[295]*Dy1[10] + Q2[296]*Dy1[11] + Q2[297]*Dy1[12] + Q2[298]*Dy1[13] + Q2[299]*Dy1[14] + Q2[300]*Dy1[15] + Q2[301]*Dy1[16] + Q2[302]*Dy1[17] + Q2[303]*Dy1[18];
QDy1[16] = + Q2[304]*Dy1[0] + Q2[305]*Dy1[1] + Q2[306]*Dy1[2] + Q2[307]*Dy1[3] + Q2[308]*Dy1[4] + Q2[309]*Dy1[5] + Q2[310]*Dy1[6] + Q2[311]*Dy1[7] + Q2[312]*Dy1[8] + Q2[313]*Dy1[9] + Q2[314]*Dy1[10] + Q2[315]*Dy1[11] + Q2[316]*Dy1[12] + Q2[317]*Dy1[13] + Q2[318]*Dy1[14] + Q2[319]*Dy1[15] + Q2[320]*Dy1[16] + Q2[321]*Dy1[17] + Q2[322]*Dy1[18];
QDy1[17] = + Q2[323]*Dy1[0] + Q2[324]*Dy1[1] + Q2[325]*Dy1[2] + Q2[326]*Dy1[3] + Q2[327]*Dy1[4] + Q2[328]*Dy1[5] + Q2[329]*Dy1[6] + Q2[330]*Dy1[7] + Q2[331]*Dy1[8] + Q2[332]*Dy1[9] + Q2[333]*Dy1[10] + Q2[334]*Dy1[11] + Q2[335]*Dy1[12] + Q2[336]*Dy1[13] + Q2[337]*Dy1[14] + Q2[338]*Dy1[15] + Q2[339]*Dy1[16] + Q2[340]*Dy1[17] + Q2[341]*Dy1[18];
}

void acado_condensePrep(  )
{
int lRun1;
int lRun2;
int lRun3;
int lRun4;
int lRun5;
/** Row vector of size: 20 */
static const int xBoundIndices[ 20 ] = 
{ 30, 31, 48, 49, 66, 67, 84, 85, 102, 103, 120, 121, 138, 139, 156, 157, 174, 175, 192, 193 };
/* Column: 0 */
acado_moveGuE( acadoWorkspace.evGu, acadoWorkspace.E );
acado_multGxGu( &(acadoWorkspace.evGx[ 324 ]), acadoWorkspace.E, &(acadoWorkspace.E[ 108 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.E[ 108 ]), &(acadoWorkspace.E[ 216 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 972 ]), &(acadoWorkspace.E[ 216 ]), &(acadoWorkspace.E[ 324 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.E[ 324 ]), &(acadoWorkspace.E[ 432 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1620 ]), &(acadoWorkspace.E[ 432 ]), &(acadoWorkspace.E[ 540 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1944 ]), &(acadoWorkspace.E[ 540 ]), &(acadoWorkspace.E[ 648 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 2268 ]), &(acadoWorkspace.E[ 648 ]), &(acadoWorkspace.E[ 756 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 2592 ]), &(acadoWorkspace.E[ 756 ]), &(acadoWorkspace.E[ 864 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 2916 ]), &(acadoWorkspace.E[ 864 ]), &(acadoWorkspace.E[ 972 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 972 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 972 ]), acadoWorkspace.W1, 9, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 2916 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 2916 ]), &(acadoWorkspace.E[ 864 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 864 ]), acadoWorkspace.W1, 8, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 2592 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 2592 ]), &(acadoWorkspace.E[ 756 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 756 ]), acadoWorkspace.W1, 7, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 2268 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 2268 ]), &(acadoWorkspace.E[ 648 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 648 ]), acadoWorkspace.W1, 6, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1944 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1944 ]), &(acadoWorkspace.E[ 540 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 540 ]), acadoWorkspace.W1, 5, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1620 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1620 ]), &(acadoWorkspace.E[ 432 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.W1, 4, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1296 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1296 ]), &(acadoWorkspace.E[ 324 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 324 ]), acadoWorkspace.W1, 3, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 972 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 972 ]), &(acadoWorkspace.E[ 216 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 216 ]), acadoWorkspace.W1, 2, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 648 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 648 ]), &(acadoWorkspace.E[ 108 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 108 ]), acadoWorkspace.W1, 1, 0 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 324 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 324 ]), acadoWorkspace.E, acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( acadoWorkspace.R1, acadoWorkspace.evGu, acadoWorkspace.W1, 0 );

/* Column: 1 */
acado_moveGuE( &(acadoWorkspace.evGu[ 108 ]), &(acadoWorkspace.E[ 1080 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.E[ 1080 ]), &(acadoWorkspace.E[ 1188 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 972 ]), &(acadoWorkspace.E[ 1188 ]), &(acadoWorkspace.E[ 1296 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.E[ 1296 ]), &(acadoWorkspace.E[ 1404 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1620 ]), &(acadoWorkspace.E[ 1404 ]), &(acadoWorkspace.E[ 1512 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1944 ]), &(acadoWorkspace.E[ 1512 ]), &(acadoWorkspace.E[ 1620 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 2268 ]), &(acadoWorkspace.E[ 1620 ]), &(acadoWorkspace.E[ 1728 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 2592 ]), &(acadoWorkspace.E[ 1728 ]), &(acadoWorkspace.E[ 1836 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 2916 ]), &(acadoWorkspace.E[ 1836 ]), &(acadoWorkspace.E[ 1944 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 1944 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 972 ]), acadoWorkspace.W1, 9, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 2916 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 2916 ]), &(acadoWorkspace.E[ 1836 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 864 ]), acadoWorkspace.W1, 8, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 2592 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 2592 ]), &(acadoWorkspace.E[ 1728 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 756 ]), acadoWorkspace.W1, 7, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 2268 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 2268 ]), &(acadoWorkspace.E[ 1620 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 648 ]), acadoWorkspace.W1, 6, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1944 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1944 ]), &(acadoWorkspace.E[ 1512 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 540 ]), acadoWorkspace.W1, 5, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1620 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1620 ]), &(acadoWorkspace.E[ 1404 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.W1, 4, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1296 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1296 ]), &(acadoWorkspace.E[ 1296 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 324 ]), acadoWorkspace.W1, 3, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 972 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 972 ]), &(acadoWorkspace.E[ 1188 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 216 ]), acadoWorkspace.W1, 2, 1 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 648 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 648 ]), &(acadoWorkspace.E[ 1080 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 36 ]), &(acadoWorkspace.evGu[ 108 ]), acadoWorkspace.W1, 1 );

/* Column: 2 */
acado_moveGuE( &(acadoWorkspace.evGu[ 216 ]), &(acadoWorkspace.E[ 2052 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 972 ]), &(acadoWorkspace.E[ 2052 ]), &(acadoWorkspace.E[ 2160 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.E[ 2160 ]), &(acadoWorkspace.E[ 2268 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1620 ]), &(acadoWorkspace.E[ 2268 ]), &(acadoWorkspace.E[ 2376 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1944 ]), &(acadoWorkspace.E[ 2376 ]), &(acadoWorkspace.E[ 2484 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 2268 ]), &(acadoWorkspace.E[ 2484 ]), &(acadoWorkspace.E[ 2592 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 2592 ]), &(acadoWorkspace.E[ 2592 ]), &(acadoWorkspace.E[ 2700 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 2916 ]), &(acadoWorkspace.E[ 2700 ]), &(acadoWorkspace.E[ 2808 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 2808 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 972 ]), acadoWorkspace.W1, 9, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 2916 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 2916 ]), &(acadoWorkspace.E[ 2700 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 864 ]), acadoWorkspace.W1, 8, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 2592 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 2592 ]), &(acadoWorkspace.E[ 2592 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 756 ]), acadoWorkspace.W1, 7, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 2268 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 2268 ]), &(acadoWorkspace.E[ 2484 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 648 ]), acadoWorkspace.W1, 6, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1944 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1944 ]), &(acadoWorkspace.E[ 2376 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 540 ]), acadoWorkspace.W1, 5, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1620 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1620 ]), &(acadoWorkspace.E[ 2268 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.W1, 4, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1296 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1296 ]), &(acadoWorkspace.E[ 2160 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 324 ]), acadoWorkspace.W1, 3, 2 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 972 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 972 ]), &(acadoWorkspace.E[ 2052 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 72 ]), &(acadoWorkspace.evGu[ 216 ]), acadoWorkspace.W1, 2 );

/* Column: 3 */
acado_moveGuE( &(acadoWorkspace.evGu[ 324 ]), &(acadoWorkspace.E[ 2916 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.E[ 2916 ]), &(acadoWorkspace.E[ 3024 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1620 ]), &(acadoWorkspace.E[ 3024 ]), &(acadoWorkspace.E[ 3132 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1944 ]), &(acadoWorkspace.E[ 3132 ]), &(acadoWorkspace.E[ 3240 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 2268 ]), &(acadoWorkspace.E[ 3240 ]), &(acadoWorkspace.E[ 3348 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 2592 ]), &(acadoWorkspace.E[ 3348 ]), &(acadoWorkspace.E[ 3456 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 2916 ]), &(acadoWorkspace.E[ 3456 ]), &(acadoWorkspace.E[ 3564 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 3564 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 972 ]), acadoWorkspace.W1, 9, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 2916 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 2916 ]), &(acadoWorkspace.E[ 3456 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 864 ]), acadoWorkspace.W1, 8, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 2592 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 2592 ]), &(acadoWorkspace.E[ 3348 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 756 ]), acadoWorkspace.W1, 7, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 2268 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 2268 ]), &(acadoWorkspace.E[ 3240 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 648 ]), acadoWorkspace.W1, 6, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1944 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1944 ]), &(acadoWorkspace.E[ 3132 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 540 ]), acadoWorkspace.W1, 5, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1620 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1620 ]), &(acadoWorkspace.E[ 3024 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.W1, 4, 3 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1296 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1296 ]), &(acadoWorkspace.E[ 2916 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 108 ]), &(acadoWorkspace.evGu[ 324 ]), acadoWorkspace.W1, 3 );

/* Column: 4 */
acado_moveGuE( &(acadoWorkspace.evGu[ 432 ]), &(acadoWorkspace.E[ 3672 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1620 ]), &(acadoWorkspace.E[ 3672 ]), &(acadoWorkspace.E[ 3780 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1944 ]), &(acadoWorkspace.E[ 3780 ]), &(acadoWorkspace.E[ 3888 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 2268 ]), &(acadoWorkspace.E[ 3888 ]), &(acadoWorkspace.E[ 3996 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 2592 ]), &(acadoWorkspace.E[ 3996 ]), &(acadoWorkspace.E[ 4104 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 2916 ]), &(acadoWorkspace.E[ 4104 ]), &(acadoWorkspace.E[ 4212 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 4212 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 972 ]), acadoWorkspace.W1, 9, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 2916 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 2916 ]), &(acadoWorkspace.E[ 4104 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 864 ]), acadoWorkspace.W1, 8, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 2592 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 2592 ]), &(acadoWorkspace.E[ 3996 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 756 ]), acadoWorkspace.W1, 7, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 2268 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 2268 ]), &(acadoWorkspace.E[ 3888 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 648 ]), acadoWorkspace.W1, 6, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1944 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1944 ]), &(acadoWorkspace.E[ 3780 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 540 ]), acadoWorkspace.W1, 5, 4 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1620 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1620 ]), &(acadoWorkspace.E[ 3672 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 144 ]), &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.W1, 4 );

/* Column: 5 */
acado_moveGuE( &(acadoWorkspace.evGu[ 540 ]), &(acadoWorkspace.E[ 4320 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 1944 ]), &(acadoWorkspace.E[ 4320 ]), &(acadoWorkspace.E[ 4428 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 2268 ]), &(acadoWorkspace.E[ 4428 ]), &(acadoWorkspace.E[ 4536 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 2592 ]), &(acadoWorkspace.E[ 4536 ]), &(acadoWorkspace.E[ 4644 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 2916 ]), &(acadoWorkspace.E[ 4644 ]), &(acadoWorkspace.E[ 4752 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 4752 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 972 ]), acadoWorkspace.W1, 9, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 2916 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 2916 ]), &(acadoWorkspace.E[ 4644 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 864 ]), acadoWorkspace.W1, 8, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 2592 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 2592 ]), &(acadoWorkspace.E[ 4536 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 756 ]), acadoWorkspace.W1, 7, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 2268 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 2268 ]), &(acadoWorkspace.E[ 4428 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 648 ]), acadoWorkspace.W1, 6, 5 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 1944 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 1944 ]), &(acadoWorkspace.E[ 4320 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 180 ]), &(acadoWorkspace.evGu[ 540 ]), acadoWorkspace.W1, 5 );

/* Column: 6 */
acado_moveGuE( &(acadoWorkspace.evGu[ 648 ]), &(acadoWorkspace.E[ 4860 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 2268 ]), &(acadoWorkspace.E[ 4860 ]), &(acadoWorkspace.E[ 4968 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 2592 ]), &(acadoWorkspace.E[ 4968 ]), &(acadoWorkspace.E[ 5076 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 2916 ]), &(acadoWorkspace.E[ 5076 ]), &(acadoWorkspace.E[ 5184 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 5184 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 972 ]), acadoWorkspace.W1, 9, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 2916 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 2916 ]), &(acadoWorkspace.E[ 5076 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 864 ]), acadoWorkspace.W1, 8, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 2592 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 2592 ]), &(acadoWorkspace.E[ 4968 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 756 ]), acadoWorkspace.W1, 7, 6 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 2268 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 2268 ]), &(acadoWorkspace.E[ 4860 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 216 ]), &(acadoWorkspace.evGu[ 648 ]), acadoWorkspace.W1, 6 );

/* Column: 7 */
acado_moveGuE( &(acadoWorkspace.evGu[ 756 ]), &(acadoWorkspace.E[ 5292 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 2592 ]), &(acadoWorkspace.E[ 5292 ]), &(acadoWorkspace.E[ 5400 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 2916 ]), &(acadoWorkspace.E[ 5400 ]), &(acadoWorkspace.E[ 5508 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 5508 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 972 ]), acadoWorkspace.W1, 9, 7 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 2916 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 2916 ]), &(acadoWorkspace.E[ 5400 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 864 ]), acadoWorkspace.W1, 8, 7 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 2592 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 2592 ]), &(acadoWorkspace.E[ 5292 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 252 ]), &(acadoWorkspace.evGu[ 756 ]), acadoWorkspace.W1, 7 );

/* Column: 8 */
acado_moveGuE( &(acadoWorkspace.evGu[ 864 ]), &(acadoWorkspace.E[ 5616 ]) );
acado_multGxGu( &(acadoWorkspace.evGx[ 2916 ]), &(acadoWorkspace.E[ 5616 ]), &(acadoWorkspace.E[ 5724 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 5724 ]), acadoWorkspace.W1 );
acado_multBTW1( &(acadoWorkspace.evGu[ 972 ]), acadoWorkspace.W1, 9, 8 );
acado_multGxTGu( &(acadoWorkspace.evGx[ 2916 ]), acadoWorkspace.W1, acadoWorkspace.W2 );
acado_multQEW2( &(acadoWorkspace.Q1[ 2916 ]), &(acadoWorkspace.E[ 5616 ]), acadoWorkspace.W2, acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 288 ]), &(acadoWorkspace.evGu[ 864 ]), acadoWorkspace.W1, 8 );

/* Column: 9 */
acado_moveGuE( &(acadoWorkspace.evGu[ 972 ]), &(acadoWorkspace.E[ 5832 ]) );

acado_multGxGu( acadoWorkspace.QN1, &(acadoWorkspace.E[ 5832 ]), acadoWorkspace.W1 );
acado_multBTW1_R1( &(acadoWorkspace.R1[ 324 ]), &(acadoWorkspace.evGu[ 972 ]), acadoWorkspace.W1, 9 );

acado_copyHTH( 0, 1 );
acado_copyHTH( 0, 2 );
acado_copyHTH( 1, 2 );
acado_copyHTH( 0, 3 );
acado_copyHTH( 1, 3 );
acado_copyHTH( 2, 3 );
acado_copyHTH( 0, 4 );
acado_copyHTH( 1, 4 );
acado_copyHTH( 2, 4 );
acado_copyHTH( 3, 4 );
acado_copyHTH( 0, 5 );
acado_copyHTH( 1, 5 );
acado_copyHTH( 2, 5 );
acado_copyHTH( 3, 5 );
acado_copyHTH( 4, 5 );
acado_copyHTH( 0, 6 );
acado_copyHTH( 1, 6 );
acado_copyHTH( 2, 6 );
acado_copyHTH( 3, 6 );
acado_copyHTH( 4, 6 );
acado_copyHTH( 5, 6 );
acado_copyHTH( 0, 7 );
acado_copyHTH( 1, 7 );
acado_copyHTH( 2, 7 );
acado_copyHTH( 3, 7 );
acado_copyHTH( 4, 7 );
acado_copyHTH( 5, 7 );
acado_copyHTH( 6, 7 );
acado_copyHTH( 0, 8 );
acado_copyHTH( 1, 8 );
acado_copyHTH( 2, 8 );
acado_copyHTH( 3, 8 );
acado_copyHTH( 4, 8 );
acado_copyHTH( 5, 8 );
acado_copyHTH( 6, 8 );
acado_copyHTH( 7, 8 );
acado_copyHTH( 0, 9 );
acado_copyHTH( 1, 9 );
acado_copyHTH( 2, 9 );
acado_copyHTH( 3, 9 );
acado_copyHTH( 4, 9 );
acado_copyHTH( 5, 9 );
acado_copyHTH( 6, 9 );
acado_copyHTH( 7, 9 );
acado_copyHTH( 8, 9 );

for (lRun1 = 0; lRun1 < 180; ++lRun1)
acadoWorkspace.sbar[lRun1 + 18] = acadoWorkspace.d[lRun1];

acadoWorkspace.lb[0] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[0];
acadoWorkspace.lb[1] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[1];
acadoWorkspace.lb[2] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[2];
acadoWorkspace.lb[3] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[3];
acadoWorkspace.lb[4] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[4];
acadoWorkspace.lb[5] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[5];
acadoWorkspace.lb[6] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[6];
acadoWorkspace.lb[7] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[7];
acadoWorkspace.lb[8] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[8];
acadoWorkspace.lb[9] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[9];
acadoWorkspace.lb[10] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[10];
acadoWorkspace.lb[11] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[11];
acadoWorkspace.lb[12] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[12];
acadoWorkspace.lb[13] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[13];
acadoWorkspace.lb[14] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[14];
acadoWorkspace.lb[15] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[15];
acadoWorkspace.lb[16] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[16];
acadoWorkspace.lb[17] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[17];
acadoWorkspace.lb[18] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[18];
acadoWorkspace.lb[19] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[19];
acadoWorkspace.lb[20] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[20];
acadoWorkspace.lb[21] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[21];
acadoWorkspace.lb[22] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[22];
acadoWorkspace.lb[23] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[23];
acadoWorkspace.lb[24] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[24];
acadoWorkspace.lb[25] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[25];
acadoWorkspace.lb[26] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[26];
acadoWorkspace.lb[27] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[27];
acadoWorkspace.lb[28] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[28];
acadoWorkspace.lb[29] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[29];
acadoWorkspace.lb[30] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[30];
acadoWorkspace.lb[31] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[31];
acadoWorkspace.lb[32] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[32];
acadoWorkspace.lb[33] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[33];
acadoWorkspace.lb[34] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[34];
acadoWorkspace.lb[35] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[35];
acadoWorkspace.lb[36] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[36];
acadoWorkspace.lb[37] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[37];
acadoWorkspace.lb[38] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[38];
acadoWorkspace.lb[39] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[39];
acadoWorkspace.lb[40] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[40];
acadoWorkspace.lb[41] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[41];
acadoWorkspace.lb[42] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[42];
acadoWorkspace.lb[43] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[43];
acadoWorkspace.lb[44] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[44];
acadoWorkspace.lb[45] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[45];
acadoWorkspace.lb[46] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[46];
acadoWorkspace.lb[47] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[47];
acadoWorkspace.lb[48] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[48];
acadoWorkspace.lb[49] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[49];
acadoWorkspace.lb[50] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[50];
acadoWorkspace.lb[51] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[51];
acadoWorkspace.lb[52] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[52];
acadoWorkspace.lb[53] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[53];
acadoWorkspace.lb[54] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[54];
acadoWorkspace.lb[55] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[55];
acadoWorkspace.lb[56] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[56];
acadoWorkspace.lb[57] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[57];
acadoWorkspace.lb[58] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[58];
acadoWorkspace.lb[59] = (real_t)-6.5000000000000002e-01 - acadoVariables.u[59];
acadoWorkspace.ub[0] = (real_t)6.5000000000000002e-01 - acadoVariables.u[0];
acadoWorkspace.ub[1] = (real_t)6.5000000000000002e-01 - acadoVariables.u[1];
acadoWorkspace.ub[2] = (real_t)6.5000000000000002e-01 - acadoVariables.u[2];
acadoWorkspace.ub[3] = (real_t)6.5000000000000002e-01 - acadoVariables.u[3];
acadoWorkspace.ub[4] = (real_t)6.5000000000000002e-01 - acadoVariables.u[4];
acadoWorkspace.ub[5] = (real_t)6.5000000000000002e-01 - acadoVariables.u[5];
acadoWorkspace.ub[6] = (real_t)6.5000000000000002e-01 - acadoVariables.u[6];
acadoWorkspace.ub[7] = (real_t)6.5000000000000002e-01 - acadoVariables.u[7];
acadoWorkspace.ub[8] = (real_t)6.5000000000000002e-01 - acadoVariables.u[8];
acadoWorkspace.ub[9] = (real_t)6.5000000000000002e-01 - acadoVariables.u[9];
acadoWorkspace.ub[10] = (real_t)6.5000000000000002e-01 - acadoVariables.u[10];
acadoWorkspace.ub[11] = (real_t)6.5000000000000002e-01 - acadoVariables.u[11];
acadoWorkspace.ub[12] = (real_t)6.5000000000000002e-01 - acadoVariables.u[12];
acadoWorkspace.ub[13] = (real_t)6.5000000000000002e-01 - acadoVariables.u[13];
acadoWorkspace.ub[14] = (real_t)6.5000000000000002e-01 - acadoVariables.u[14];
acadoWorkspace.ub[15] = (real_t)6.5000000000000002e-01 - acadoVariables.u[15];
acadoWorkspace.ub[16] = (real_t)6.5000000000000002e-01 - acadoVariables.u[16];
acadoWorkspace.ub[17] = (real_t)6.5000000000000002e-01 - acadoVariables.u[17];
acadoWorkspace.ub[18] = (real_t)6.5000000000000002e-01 - acadoVariables.u[18];
acadoWorkspace.ub[19] = (real_t)6.5000000000000002e-01 - acadoVariables.u[19];
acadoWorkspace.ub[20] = (real_t)6.5000000000000002e-01 - acadoVariables.u[20];
acadoWorkspace.ub[21] = (real_t)6.5000000000000002e-01 - acadoVariables.u[21];
acadoWorkspace.ub[22] = (real_t)6.5000000000000002e-01 - acadoVariables.u[22];
acadoWorkspace.ub[23] = (real_t)6.5000000000000002e-01 - acadoVariables.u[23];
acadoWorkspace.ub[24] = (real_t)6.5000000000000002e-01 - acadoVariables.u[24];
acadoWorkspace.ub[25] = (real_t)6.5000000000000002e-01 - acadoVariables.u[25];
acadoWorkspace.ub[26] = (real_t)6.5000000000000002e-01 - acadoVariables.u[26];
acadoWorkspace.ub[27] = (real_t)6.5000000000000002e-01 - acadoVariables.u[27];
acadoWorkspace.ub[28] = (real_t)6.5000000000000002e-01 - acadoVariables.u[28];
acadoWorkspace.ub[29] = (real_t)6.5000000000000002e-01 - acadoVariables.u[29];
acadoWorkspace.ub[30] = (real_t)6.5000000000000002e-01 - acadoVariables.u[30];
acadoWorkspace.ub[31] = (real_t)6.5000000000000002e-01 - acadoVariables.u[31];
acadoWorkspace.ub[32] = (real_t)6.5000000000000002e-01 - acadoVariables.u[32];
acadoWorkspace.ub[33] = (real_t)6.5000000000000002e-01 - acadoVariables.u[33];
acadoWorkspace.ub[34] = (real_t)6.5000000000000002e-01 - acadoVariables.u[34];
acadoWorkspace.ub[35] = (real_t)6.5000000000000002e-01 - acadoVariables.u[35];
acadoWorkspace.ub[36] = (real_t)6.5000000000000002e-01 - acadoVariables.u[36];
acadoWorkspace.ub[37] = (real_t)6.5000000000000002e-01 - acadoVariables.u[37];
acadoWorkspace.ub[38] = (real_t)6.5000000000000002e-01 - acadoVariables.u[38];
acadoWorkspace.ub[39] = (real_t)6.5000000000000002e-01 - acadoVariables.u[39];
acadoWorkspace.ub[40] = (real_t)6.5000000000000002e-01 - acadoVariables.u[40];
acadoWorkspace.ub[41] = (real_t)6.5000000000000002e-01 - acadoVariables.u[41];
acadoWorkspace.ub[42] = (real_t)6.5000000000000002e-01 - acadoVariables.u[42];
acadoWorkspace.ub[43] = (real_t)6.5000000000000002e-01 - acadoVariables.u[43];
acadoWorkspace.ub[44] = (real_t)6.5000000000000002e-01 - acadoVariables.u[44];
acadoWorkspace.ub[45] = (real_t)6.5000000000000002e-01 - acadoVariables.u[45];
acadoWorkspace.ub[46] = (real_t)6.5000000000000002e-01 - acadoVariables.u[46];
acadoWorkspace.ub[47] = (real_t)6.5000000000000002e-01 - acadoVariables.u[47];
acadoWorkspace.ub[48] = (real_t)6.5000000000000002e-01 - acadoVariables.u[48];
acadoWorkspace.ub[49] = (real_t)6.5000000000000002e-01 - acadoVariables.u[49];
acadoWorkspace.ub[50] = (real_t)6.5000000000000002e-01 - acadoVariables.u[50];
acadoWorkspace.ub[51] = (real_t)6.5000000000000002e-01 - acadoVariables.u[51];
acadoWorkspace.ub[52] = (real_t)6.5000000000000002e-01 - acadoVariables.u[52];
acadoWorkspace.ub[53] = (real_t)6.5000000000000002e-01 - acadoVariables.u[53];
acadoWorkspace.ub[54] = (real_t)6.5000000000000002e-01 - acadoVariables.u[54];
acadoWorkspace.ub[55] = (real_t)6.5000000000000002e-01 - acadoVariables.u[55];
acadoWorkspace.ub[56] = (real_t)6.5000000000000002e-01 - acadoVariables.u[56];
acadoWorkspace.ub[57] = (real_t)6.5000000000000002e-01 - acadoVariables.u[57];
acadoWorkspace.ub[58] = (real_t)6.5000000000000002e-01 - acadoVariables.u[58];
acadoWorkspace.ub[59] = (real_t)6.5000000000000002e-01 - acadoVariables.u[59];

for (lRun1 = 0; lRun1 < 20; ++lRun1)
{
lRun3 = xBoundIndices[ lRun1 ] - 18;
lRun4 = ((lRun3) / (18)) + (1);
for (lRun2 = 0; lRun2 < lRun4; ++lRun2)
{
lRun5 = ((((((lRun2) * (lRun2 * -1 + 19)) / (2)) + (lRun4)) - (1)) * (18)) + ((lRun3) % (18));
acadoWorkspace.A[(lRun1 * 60) + (lRun2 * 6)] = acadoWorkspace.E[lRun5 * 6];
acadoWorkspace.A[(lRun1 * 60) + (lRun2 * 6 + 1)] = acadoWorkspace.E[lRun5 * 6 + 1];
acadoWorkspace.A[(lRun1 * 60) + (lRun2 * 6 + 2)] = acadoWorkspace.E[lRun5 * 6 + 2];
acadoWorkspace.A[(lRun1 * 60) + (lRun2 * 6 + 3)] = acadoWorkspace.E[lRun5 * 6 + 3];
acadoWorkspace.A[(lRun1 * 60) + (lRun2 * 6 + 4)] = acadoWorkspace.E[lRun5 * 6 + 4];
acadoWorkspace.A[(lRun1 * 60) + (lRun2 * 6 + 5)] = acadoWorkspace.E[lRun5 * 6 + 5];
}
}

}

void acado_condenseFdb(  )
{
int lRun1;
real_t tmp;

acadoWorkspace.Dx0[0] = acadoVariables.x0[0] - acadoVariables.x[0];
acadoWorkspace.Dx0[1] = acadoVariables.x0[1] - acadoVariables.x[1];
acadoWorkspace.Dx0[2] = acadoVariables.x0[2] - acadoVariables.x[2];
acadoWorkspace.Dx0[3] = acadoVariables.x0[3] - acadoVariables.x[3];
acadoWorkspace.Dx0[4] = acadoVariables.x0[4] - acadoVariables.x[4];
acadoWorkspace.Dx0[5] = acadoVariables.x0[5] - acadoVariables.x[5];
acadoWorkspace.Dx0[6] = acadoVariables.x0[6] - acadoVariables.x[6];
acadoWorkspace.Dx0[7] = acadoVariables.x0[7] - acadoVariables.x[7];
acadoWorkspace.Dx0[8] = acadoVariables.x0[8] - acadoVariables.x[8];
acadoWorkspace.Dx0[9] = acadoVariables.x0[9] - acadoVariables.x[9];
acadoWorkspace.Dx0[10] = acadoVariables.x0[10] - acadoVariables.x[10];
acadoWorkspace.Dx0[11] = acadoVariables.x0[11] - acadoVariables.x[11];
acadoWorkspace.Dx0[12] = acadoVariables.x0[12] - acadoVariables.x[12];
acadoWorkspace.Dx0[13] = acadoVariables.x0[13] - acadoVariables.x[13];
acadoWorkspace.Dx0[14] = acadoVariables.x0[14] - acadoVariables.x[14];
acadoWorkspace.Dx0[15] = acadoVariables.x0[15] - acadoVariables.x[15];
acadoWorkspace.Dx0[16] = acadoVariables.x0[16] - acadoVariables.x[16];
acadoWorkspace.Dx0[17] = acadoVariables.x0[17] - acadoVariables.x[17];
for (lRun1 = 0; lRun1 < 190; ++lRun1)
acadoWorkspace.Dy[lRun1] -= acadoVariables.y[lRun1];

acadoWorkspace.DyN[0] -= acadoVariables.yN[0];
acadoWorkspace.DyN[1] -= acadoVariables.yN[1];
acadoWorkspace.DyN[2] -= acadoVariables.yN[2];
acadoWorkspace.DyN[3] -= acadoVariables.yN[3];
acadoWorkspace.DyN[4] -= acadoVariables.yN[4];
acadoWorkspace.DyN[5] -= acadoVariables.yN[5];
acadoWorkspace.DyN[6] -= acadoVariables.yN[6];
acadoWorkspace.DyN[7] -= acadoVariables.yN[7];
acadoWorkspace.DyN[8] -= acadoVariables.yN[8];
acadoWorkspace.DyN[9] -= acadoVariables.yN[9];
acadoWorkspace.DyN[10] -= acadoVariables.yN[10];
acadoWorkspace.DyN[11] -= acadoVariables.yN[11];
acadoWorkspace.DyN[12] -= acadoVariables.yN[12];

acado_multRDy( acadoWorkspace.R2, acadoWorkspace.Dy, acadoWorkspace.g );
acado_multRDy( &(acadoWorkspace.R2[ 114 ]), &(acadoWorkspace.Dy[ 19 ]), &(acadoWorkspace.g[ 6 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 228 ]), &(acadoWorkspace.Dy[ 38 ]), &(acadoWorkspace.g[ 12 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 342 ]), &(acadoWorkspace.Dy[ 57 ]), &(acadoWorkspace.g[ 18 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 456 ]), &(acadoWorkspace.Dy[ 76 ]), &(acadoWorkspace.g[ 24 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 570 ]), &(acadoWorkspace.Dy[ 95 ]), &(acadoWorkspace.g[ 30 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 684 ]), &(acadoWorkspace.Dy[ 114 ]), &(acadoWorkspace.g[ 36 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 798 ]), &(acadoWorkspace.Dy[ 133 ]), &(acadoWorkspace.g[ 42 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 912 ]), &(acadoWorkspace.Dy[ 152 ]), &(acadoWorkspace.g[ 48 ]) );
acado_multRDy( &(acadoWorkspace.R2[ 1026 ]), &(acadoWorkspace.Dy[ 171 ]), &(acadoWorkspace.g[ 54 ]) );

acado_multQDy( acadoWorkspace.Q2, acadoWorkspace.Dy, acadoWorkspace.QDy );
acado_multQDy( &(acadoWorkspace.Q2[ 342 ]), &(acadoWorkspace.Dy[ 19 ]), &(acadoWorkspace.QDy[ 18 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 684 ]), &(acadoWorkspace.Dy[ 38 ]), &(acadoWorkspace.QDy[ 36 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1026 ]), &(acadoWorkspace.Dy[ 57 ]), &(acadoWorkspace.QDy[ 54 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1368 ]), &(acadoWorkspace.Dy[ 76 ]), &(acadoWorkspace.QDy[ 72 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 1710 ]), &(acadoWorkspace.Dy[ 95 ]), &(acadoWorkspace.QDy[ 90 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2052 ]), &(acadoWorkspace.Dy[ 114 ]), &(acadoWorkspace.QDy[ 108 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2394 ]), &(acadoWorkspace.Dy[ 133 ]), &(acadoWorkspace.QDy[ 126 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 2736 ]), &(acadoWorkspace.Dy[ 152 ]), &(acadoWorkspace.QDy[ 144 ]) );
acado_multQDy( &(acadoWorkspace.Q2[ 3078 ]), &(acadoWorkspace.Dy[ 171 ]), &(acadoWorkspace.QDy[ 162 ]) );

acadoWorkspace.QDy[180] = + acadoWorkspace.QN2[0]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[1]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[2]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[3]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[4]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[5]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[6]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[7]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[8]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[9]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[10]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[11]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[12]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[181] = + acadoWorkspace.QN2[13]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[14]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[15]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[16]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[17]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[18]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[19]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[20]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[21]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[22]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[23]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[24]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[25]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[182] = + acadoWorkspace.QN2[26]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[27]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[28]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[29]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[30]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[31]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[32]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[33]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[34]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[35]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[36]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[37]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[38]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[183] = + acadoWorkspace.QN2[39]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[40]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[41]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[42]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[43]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[44]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[45]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[46]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[47]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[48]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[49]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[50]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[51]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[184] = + acadoWorkspace.QN2[52]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[53]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[54]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[55]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[56]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[57]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[58]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[59]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[60]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[61]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[62]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[63]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[64]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[185] = + acadoWorkspace.QN2[65]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[66]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[67]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[68]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[69]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[70]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[71]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[72]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[73]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[74]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[75]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[76]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[77]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[186] = + acadoWorkspace.QN2[78]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[79]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[80]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[81]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[82]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[83]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[84]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[85]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[86]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[87]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[88]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[89]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[90]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[187] = + acadoWorkspace.QN2[91]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[92]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[93]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[94]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[95]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[96]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[97]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[98]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[99]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[100]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[101]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[102]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[103]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[188] = + acadoWorkspace.QN2[104]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[105]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[106]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[107]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[108]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[109]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[110]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[111]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[112]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[113]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[114]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[115]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[116]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[189] = + acadoWorkspace.QN2[117]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[118]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[119]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[120]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[121]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[122]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[123]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[124]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[125]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[126]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[127]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[128]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[129]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[190] = + acadoWorkspace.QN2[130]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[131]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[132]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[133]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[134]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[135]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[136]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[137]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[138]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[139]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[140]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[141]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[142]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[191] = + acadoWorkspace.QN2[143]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[144]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[145]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[146]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[147]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[148]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[149]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[150]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[151]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[152]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[153]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[154]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[155]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[192] = + acadoWorkspace.QN2[156]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[157]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[158]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[159]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[160]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[161]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[162]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[163]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[164]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[165]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[166]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[167]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[168]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[193] = + acadoWorkspace.QN2[169]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[170]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[171]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[172]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[173]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[174]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[175]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[176]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[177]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[178]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[179]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[180]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[181]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[194] = + acadoWorkspace.QN2[182]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[183]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[184]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[185]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[186]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[187]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[188]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[189]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[190]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[191]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[192]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[193]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[194]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[195] = + acadoWorkspace.QN2[195]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[196]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[197]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[198]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[199]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[200]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[201]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[202]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[203]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[204]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[205]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[206]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[207]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[196] = + acadoWorkspace.QN2[208]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[209]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[210]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[211]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[212]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[213]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[214]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[215]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[216]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[217]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[218]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[219]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[220]*acadoWorkspace.DyN[12];
acadoWorkspace.QDy[197] = + acadoWorkspace.QN2[221]*acadoWorkspace.DyN[0] + acadoWorkspace.QN2[222]*acadoWorkspace.DyN[1] + acadoWorkspace.QN2[223]*acadoWorkspace.DyN[2] + acadoWorkspace.QN2[224]*acadoWorkspace.DyN[3] + acadoWorkspace.QN2[225]*acadoWorkspace.DyN[4] + acadoWorkspace.QN2[226]*acadoWorkspace.DyN[5] + acadoWorkspace.QN2[227]*acadoWorkspace.DyN[6] + acadoWorkspace.QN2[228]*acadoWorkspace.DyN[7] + acadoWorkspace.QN2[229]*acadoWorkspace.DyN[8] + acadoWorkspace.QN2[230]*acadoWorkspace.DyN[9] + acadoWorkspace.QN2[231]*acadoWorkspace.DyN[10] + acadoWorkspace.QN2[232]*acadoWorkspace.DyN[11] + acadoWorkspace.QN2[233]*acadoWorkspace.DyN[12];

acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
acadoWorkspace.sbar[9] = acadoWorkspace.Dx0[9];
acadoWorkspace.sbar[10] = acadoWorkspace.Dx0[10];
acadoWorkspace.sbar[11] = acadoWorkspace.Dx0[11];
acadoWorkspace.sbar[12] = acadoWorkspace.Dx0[12];
acadoWorkspace.sbar[13] = acadoWorkspace.Dx0[13];
acadoWorkspace.sbar[14] = acadoWorkspace.Dx0[14];
acadoWorkspace.sbar[15] = acadoWorkspace.Dx0[15];
acadoWorkspace.sbar[16] = acadoWorkspace.Dx0[16];
acadoWorkspace.sbar[17] = acadoWorkspace.Dx0[17];
acado_macASbar( acadoWorkspace.evGx, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 18 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 972 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1620 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 1944 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 126 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2268 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2592 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 162 ]) );
acado_macASbar( &(acadoWorkspace.evGx[ 2916 ]), &(acadoWorkspace.sbar[ 162 ]), &(acadoWorkspace.sbar[ 180 ]) );

acadoWorkspace.w1[0] = + acadoWorkspace.QN1[0]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[1]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[2]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[3]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[4]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[5]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[6]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[7]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[8]*acadoWorkspace.sbar[188] + acadoWorkspace.QN1[9]*acadoWorkspace.sbar[189] + acadoWorkspace.QN1[10]*acadoWorkspace.sbar[190] + acadoWorkspace.QN1[11]*acadoWorkspace.sbar[191] + acadoWorkspace.QN1[12]*acadoWorkspace.sbar[192] + acadoWorkspace.QN1[13]*acadoWorkspace.sbar[193] + acadoWorkspace.QN1[14]*acadoWorkspace.sbar[194] + acadoWorkspace.QN1[15]*acadoWorkspace.sbar[195] + acadoWorkspace.QN1[16]*acadoWorkspace.sbar[196] + acadoWorkspace.QN1[17]*acadoWorkspace.sbar[197] + acadoWorkspace.QDy[180];
acadoWorkspace.w1[1] = + acadoWorkspace.QN1[18]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[19]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[20]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[21]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[22]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[23]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[24]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[25]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[26]*acadoWorkspace.sbar[188] + acadoWorkspace.QN1[27]*acadoWorkspace.sbar[189] + acadoWorkspace.QN1[28]*acadoWorkspace.sbar[190] + acadoWorkspace.QN1[29]*acadoWorkspace.sbar[191] + acadoWorkspace.QN1[30]*acadoWorkspace.sbar[192] + acadoWorkspace.QN1[31]*acadoWorkspace.sbar[193] + acadoWorkspace.QN1[32]*acadoWorkspace.sbar[194] + acadoWorkspace.QN1[33]*acadoWorkspace.sbar[195] + acadoWorkspace.QN1[34]*acadoWorkspace.sbar[196] + acadoWorkspace.QN1[35]*acadoWorkspace.sbar[197] + acadoWorkspace.QDy[181];
acadoWorkspace.w1[2] = + acadoWorkspace.QN1[36]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[37]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[38]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[39]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[40]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[41]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[42]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[43]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[44]*acadoWorkspace.sbar[188] + acadoWorkspace.QN1[45]*acadoWorkspace.sbar[189] + acadoWorkspace.QN1[46]*acadoWorkspace.sbar[190] + acadoWorkspace.QN1[47]*acadoWorkspace.sbar[191] + acadoWorkspace.QN1[48]*acadoWorkspace.sbar[192] + acadoWorkspace.QN1[49]*acadoWorkspace.sbar[193] + acadoWorkspace.QN1[50]*acadoWorkspace.sbar[194] + acadoWorkspace.QN1[51]*acadoWorkspace.sbar[195] + acadoWorkspace.QN1[52]*acadoWorkspace.sbar[196] + acadoWorkspace.QN1[53]*acadoWorkspace.sbar[197] + acadoWorkspace.QDy[182];
acadoWorkspace.w1[3] = + acadoWorkspace.QN1[54]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[55]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[56]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[57]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[58]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[59]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[60]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[61]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[62]*acadoWorkspace.sbar[188] + acadoWorkspace.QN1[63]*acadoWorkspace.sbar[189] + acadoWorkspace.QN1[64]*acadoWorkspace.sbar[190] + acadoWorkspace.QN1[65]*acadoWorkspace.sbar[191] + acadoWorkspace.QN1[66]*acadoWorkspace.sbar[192] + acadoWorkspace.QN1[67]*acadoWorkspace.sbar[193] + acadoWorkspace.QN1[68]*acadoWorkspace.sbar[194] + acadoWorkspace.QN1[69]*acadoWorkspace.sbar[195] + acadoWorkspace.QN1[70]*acadoWorkspace.sbar[196] + acadoWorkspace.QN1[71]*acadoWorkspace.sbar[197] + acadoWorkspace.QDy[183];
acadoWorkspace.w1[4] = + acadoWorkspace.QN1[72]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[73]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[74]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[75]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[76]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[77]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[78]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[79]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[80]*acadoWorkspace.sbar[188] + acadoWorkspace.QN1[81]*acadoWorkspace.sbar[189] + acadoWorkspace.QN1[82]*acadoWorkspace.sbar[190] + acadoWorkspace.QN1[83]*acadoWorkspace.sbar[191] + acadoWorkspace.QN1[84]*acadoWorkspace.sbar[192] + acadoWorkspace.QN1[85]*acadoWorkspace.sbar[193] + acadoWorkspace.QN1[86]*acadoWorkspace.sbar[194] + acadoWorkspace.QN1[87]*acadoWorkspace.sbar[195] + acadoWorkspace.QN1[88]*acadoWorkspace.sbar[196] + acadoWorkspace.QN1[89]*acadoWorkspace.sbar[197] + acadoWorkspace.QDy[184];
acadoWorkspace.w1[5] = + acadoWorkspace.QN1[90]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[91]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[92]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[93]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[94]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[95]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[96]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[97]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[98]*acadoWorkspace.sbar[188] + acadoWorkspace.QN1[99]*acadoWorkspace.sbar[189] + acadoWorkspace.QN1[100]*acadoWorkspace.sbar[190] + acadoWorkspace.QN1[101]*acadoWorkspace.sbar[191] + acadoWorkspace.QN1[102]*acadoWorkspace.sbar[192] + acadoWorkspace.QN1[103]*acadoWorkspace.sbar[193] + acadoWorkspace.QN1[104]*acadoWorkspace.sbar[194] + acadoWorkspace.QN1[105]*acadoWorkspace.sbar[195] + acadoWorkspace.QN1[106]*acadoWorkspace.sbar[196] + acadoWorkspace.QN1[107]*acadoWorkspace.sbar[197] + acadoWorkspace.QDy[185];
acadoWorkspace.w1[6] = + acadoWorkspace.QN1[108]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[109]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[110]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[111]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[112]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[113]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[114]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[115]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[116]*acadoWorkspace.sbar[188] + acadoWorkspace.QN1[117]*acadoWorkspace.sbar[189] + acadoWorkspace.QN1[118]*acadoWorkspace.sbar[190] + acadoWorkspace.QN1[119]*acadoWorkspace.sbar[191] + acadoWorkspace.QN1[120]*acadoWorkspace.sbar[192] + acadoWorkspace.QN1[121]*acadoWorkspace.sbar[193] + acadoWorkspace.QN1[122]*acadoWorkspace.sbar[194] + acadoWorkspace.QN1[123]*acadoWorkspace.sbar[195] + acadoWorkspace.QN1[124]*acadoWorkspace.sbar[196] + acadoWorkspace.QN1[125]*acadoWorkspace.sbar[197] + acadoWorkspace.QDy[186];
acadoWorkspace.w1[7] = + acadoWorkspace.QN1[126]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[127]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[128]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[129]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[130]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[131]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[132]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[133]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[134]*acadoWorkspace.sbar[188] + acadoWorkspace.QN1[135]*acadoWorkspace.sbar[189] + acadoWorkspace.QN1[136]*acadoWorkspace.sbar[190] + acadoWorkspace.QN1[137]*acadoWorkspace.sbar[191] + acadoWorkspace.QN1[138]*acadoWorkspace.sbar[192] + acadoWorkspace.QN1[139]*acadoWorkspace.sbar[193] + acadoWorkspace.QN1[140]*acadoWorkspace.sbar[194] + acadoWorkspace.QN1[141]*acadoWorkspace.sbar[195] + acadoWorkspace.QN1[142]*acadoWorkspace.sbar[196] + acadoWorkspace.QN1[143]*acadoWorkspace.sbar[197] + acadoWorkspace.QDy[187];
acadoWorkspace.w1[8] = + acadoWorkspace.QN1[144]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[145]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[146]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[147]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[148]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[149]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[150]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[151]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[152]*acadoWorkspace.sbar[188] + acadoWorkspace.QN1[153]*acadoWorkspace.sbar[189] + acadoWorkspace.QN1[154]*acadoWorkspace.sbar[190] + acadoWorkspace.QN1[155]*acadoWorkspace.sbar[191] + acadoWorkspace.QN1[156]*acadoWorkspace.sbar[192] + acadoWorkspace.QN1[157]*acadoWorkspace.sbar[193] + acadoWorkspace.QN1[158]*acadoWorkspace.sbar[194] + acadoWorkspace.QN1[159]*acadoWorkspace.sbar[195] + acadoWorkspace.QN1[160]*acadoWorkspace.sbar[196] + acadoWorkspace.QN1[161]*acadoWorkspace.sbar[197] + acadoWorkspace.QDy[188];
acadoWorkspace.w1[9] = + acadoWorkspace.QN1[162]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[163]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[164]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[165]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[166]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[167]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[168]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[169]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[170]*acadoWorkspace.sbar[188] + acadoWorkspace.QN1[171]*acadoWorkspace.sbar[189] + acadoWorkspace.QN1[172]*acadoWorkspace.sbar[190] + acadoWorkspace.QN1[173]*acadoWorkspace.sbar[191] + acadoWorkspace.QN1[174]*acadoWorkspace.sbar[192] + acadoWorkspace.QN1[175]*acadoWorkspace.sbar[193] + acadoWorkspace.QN1[176]*acadoWorkspace.sbar[194] + acadoWorkspace.QN1[177]*acadoWorkspace.sbar[195] + acadoWorkspace.QN1[178]*acadoWorkspace.sbar[196] + acadoWorkspace.QN1[179]*acadoWorkspace.sbar[197] + acadoWorkspace.QDy[189];
acadoWorkspace.w1[10] = + acadoWorkspace.QN1[180]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[181]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[182]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[183]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[184]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[185]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[186]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[187]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[188]*acadoWorkspace.sbar[188] + acadoWorkspace.QN1[189]*acadoWorkspace.sbar[189] + acadoWorkspace.QN1[190]*acadoWorkspace.sbar[190] + acadoWorkspace.QN1[191]*acadoWorkspace.sbar[191] + acadoWorkspace.QN1[192]*acadoWorkspace.sbar[192] + acadoWorkspace.QN1[193]*acadoWorkspace.sbar[193] + acadoWorkspace.QN1[194]*acadoWorkspace.sbar[194] + acadoWorkspace.QN1[195]*acadoWorkspace.sbar[195] + acadoWorkspace.QN1[196]*acadoWorkspace.sbar[196] + acadoWorkspace.QN1[197]*acadoWorkspace.sbar[197] + acadoWorkspace.QDy[190];
acadoWorkspace.w1[11] = + acadoWorkspace.QN1[198]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[199]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[200]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[201]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[202]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[203]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[204]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[205]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[206]*acadoWorkspace.sbar[188] + acadoWorkspace.QN1[207]*acadoWorkspace.sbar[189] + acadoWorkspace.QN1[208]*acadoWorkspace.sbar[190] + acadoWorkspace.QN1[209]*acadoWorkspace.sbar[191] + acadoWorkspace.QN1[210]*acadoWorkspace.sbar[192] + acadoWorkspace.QN1[211]*acadoWorkspace.sbar[193] + acadoWorkspace.QN1[212]*acadoWorkspace.sbar[194] + acadoWorkspace.QN1[213]*acadoWorkspace.sbar[195] + acadoWorkspace.QN1[214]*acadoWorkspace.sbar[196] + acadoWorkspace.QN1[215]*acadoWorkspace.sbar[197] + acadoWorkspace.QDy[191];
acadoWorkspace.w1[12] = + acadoWorkspace.QN1[216]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[217]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[218]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[219]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[220]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[221]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[222]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[223]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[224]*acadoWorkspace.sbar[188] + acadoWorkspace.QN1[225]*acadoWorkspace.sbar[189] + acadoWorkspace.QN1[226]*acadoWorkspace.sbar[190] + acadoWorkspace.QN1[227]*acadoWorkspace.sbar[191] + acadoWorkspace.QN1[228]*acadoWorkspace.sbar[192] + acadoWorkspace.QN1[229]*acadoWorkspace.sbar[193] + acadoWorkspace.QN1[230]*acadoWorkspace.sbar[194] + acadoWorkspace.QN1[231]*acadoWorkspace.sbar[195] + acadoWorkspace.QN1[232]*acadoWorkspace.sbar[196] + acadoWorkspace.QN1[233]*acadoWorkspace.sbar[197] + acadoWorkspace.QDy[192];
acadoWorkspace.w1[13] = + acadoWorkspace.QN1[234]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[235]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[236]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[237]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[238]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[239]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[240]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[241]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[242]*acadoWorkspace.sbar[188] + acadoWorkspace.QN1[243]*acadoWorkspace.sbar[189] + acadoWorkspace.QN1[244]*acadoWorkspace.sbar[190] + acadoWorkspace.QN1[245]*acadoWorkspace.sbar[191] + acadoWorkspace.QN1[246]*acadoWorkspace.sbar[192] + acadoWorkspace.QN1[247]*acadoWorkspace.sbar[193] + acadoWorkspace.QN1[248]*acadoWorkspace.sbar[194] + acadoWorkspace.QN1[249]*acadoWorkspace.sbar[195] + acadoWorkspace.QN1[250]*acadoWorkspace.sbar[196] + acadoWorkspace.QN1[251]*acadoWorkspace.sbar[197] + acadoWorkspace.QDy[193];
acadoWorkspace.w1[14] = + acadoWorkspace.QN1[252]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[253]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[254]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[255]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[256]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[257]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[258]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[259]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[260]*acadoWorkspace.sbar[188] + acadoWorkspace.QN1[261]*acadoWorkspace.sbar[189] + acadoWorkspace.QN1[262]*acadoWorkspace.sbar[190] + acadoWorkspace.QN1[263]*acadoWorkspace.sbar[191] + acadoWorkspace.QN1[264]*acadoWorkspace.sbar[192] + acadoWorkspace.QN1[265]*acadoWorkspace.sbar[193] + acadoWorkspace.QN1[266]*acadoWorkspace.sbar[194] + acadoWorkspace.QN1[267]*acadoWorkspace.sbar[195] + acadoWorkspace.QN1[268]*acadoWorkspace.sbar[196] + acadoWorkspace.QN1[269]*acadoWorkspace.sbar[197] + acadoWorkspace.QDy[194];
acadoWorkspace.w1[15] = + acadoWorkspace.QN1[270]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[271]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[272]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[273]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[274]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[275]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[276]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[277]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[278]*acadoWorkspace.sbar[188] + acadoWorkspace.QN1[279]*acadoWorkspace.sbar[189] + acadoWorkspace.QN1[280]*acadoWorkspace.sbar[190] + acadoWorkspace.QN1[281]*acadoWorkspace.sbar[191] + acadoWorkspace.QN1[282]*acadoWorkspace.sbar[192] + acadoWorkspace.QN1[283]*acadoWorkspace.sbar[193] + acadoWorkspace.QN1[284]*acadoWorkspace.sbar[194] + acadoWorkspace.QN1[285]*acadoWorkspace.sbar[195] + acadoWorkspace.QN1[286]*acadoWorkspace.sbar[196] + acadoWorkspace.QN1[287]*acadoWorkspace.sbar[197] + acadoWorkspace.QDy[195];
acadoWorkspace.w1[16] = + acadoWorkspace.QN1[288]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[289]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[290]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[291]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[292]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[293]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[294]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[295]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[296]*acadoWorkspace.sbar[188] + acadoWorkspace.QN1[297]*acadoWorkspace.sbar[189] + acadoWorkspace.QN1[298]*acadoWorkspace.sbar[190] + acadoWorkspace.QN1[299]*acadoWorkspace.sbar[191] + acadoWorkspace.QN1[300]*acadoWorkspace.sbar[192] + acadoWorkspace.QN1[301]*acadoWorkspace.sbar[193] + acadoWorkspace.QN1[302]*acadoWorkspace.sbar[194] + acadoWorkspace.QN1[303]*acadoWorkspace.sbar[195] + acadoWorkspace.QN1[304]*acadoWorkspace.sbar[196] + acadoWorkspace.QN1[305]*acadoWorkspace.sbar[197] + acadoWorkspace.QDy[196];
acadoWorkspace.w1[17] = + acadoWorkspace.QN1[306]*acadoWorkspace.sbar[180] + acadoWorkspace.QN1[307]*acadoWorkspace.sbar[181] + acadoWorkspace.QN1[308]*acadoWorkspace.sbar[182] + acadoWorkspace.QN1[309]*acadoWorkspace.sbar[183] + acadoWorkspace.QN1[310]*acadoWorkspace.sbar[184] + acadoWorkspace.QN1[311]*acadoWorkspace.sbar[185] + acadoWorkspace.QN1[312]*acadoWorkspace.sbar[186] + acadoWorkspace.QN1[313]*acadoWorkspace.sbar[187] + acadoWorkspace.QN1[314]*acadoWorkspace.sbar[188] + acadoWorkspace.QN1[315]*acadoWorkspace.sbar[189] + acadoWorkspace.QN1[316]*acadoWorkspace.sbar[190] + acadoWorkspace.QN1[317]*acadoWorkspace.sbar[191] + acadoWorkspace.QN1[318]*acadoWorkspace.sbar[192] + acadoWorkspace.QN1[319]*acadoWorkspace.sbar[193] + acadoWorkspace.QN1[320]*acadoWorkspace.sbar[194] + acadoWorkspace.QN1[321]*acadoWorkspace.sbar[195] + acadoWorkspace.QN1[322]*acadoWorkspace.sbar[196] + acadoWorkspace.QN1[323]*acadoWorkspace.sbar[197] + acadoWorkspace.QDy[197];
acado_macBTw1( &(acadoWorkspace.evGu[ 972 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 54 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2916 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 162 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2916 ]), &(acadoWorkspace.sbar[ 162 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 864 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 48 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2592 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 144 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2592 ]), &(acadoWorkspace.sbar[ 144 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 756 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 42 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 2268 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 126 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 2268 ]), &(acadoWorkspace.sbar[ 126 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 648 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 36 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1944 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 108 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1944 ]), &(acadoWorkspace.sbar[ 108 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 540 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 30 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1620 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 90 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1620 ]), &(acadoWorkspace.sbar[ 90 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 432 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 24 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 1296 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 72 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 1296 ]), &(acadoWorkspace.sbar[ 72 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 324 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 18 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 972 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 54 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 972 ]), &(acadoWorkspace.sbar[ 54 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 216 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 12 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 648 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 36 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 648 ]), &(acadoWorkspace.sbar[ 36 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( &(acadoWorkspace.evGu[ 108 ]), acadoWorkspace.w1, &(acadoWorkspace.g[ 6 ]) );
acado_macATw1QDy( &(acadoWorkspace.evGx[ 324 ]), acadoWorkspace.w1, &(acadoWorkspace.QDy[ 18 ]), acadoWorkspace.w2 );
acado_macQSbarW2( &(acadoWorkspace.Q1[ 324 ]), &(acadoWorkspace.sbar[ 18 ]), acadoWorkspace.w2, acadoWorkspace.w1 );
acado_macBTw1( acadoWorkspace.evGu, acadoWorkspace.w1, acadoWorkspace.g );


tmp = acadoWorkspace.sbar[30] + acadoVariables.x[30];
acadoWorkspace.lbA[0] = (real_t)-1.5710000000000000e+00 - tmp;
acadoWorkspace.ubA[0] = (real_t)1.5710000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[31] + acadoVariables.x[31];
acadoWorkspace.lbA[1] = (real_t)-1.5710000000000000e+00 - tmp;
acadoWorkspace.ubA[1] = (real_t)1.5710000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[48] + acadoVariables.x[48];
acadoWorkspace.lbA[2] = (real_t)-1.5710000000000000e+00 - tmp;
acadoWorkspace.ubA[2] = (real_t)1.5710000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[49] + acadoVariables.x[49];
acadoWorkspace.lbA[3] = (real_t)-1.5710000000000000e+00 - tmp;
acadoWorkspace.ubA[3] = (real_t)1.5710000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[66] + acadoVariables.x[66];
acadoWorkspace.lbA[4] = (real_t)-1.5710000000000000e+00 - tmp;
acadoWorkspace.ubA[4] = (real_t)1.5710000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[67] + acadoVariables.x[67];
acadoWorkspace.lbA[5] = (real_t)-1.5710000000000000e+00 - tmp;
acadoWorkspace.ubA[5] = (real_t)1.5710000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[84] + acadoVariables.x[84];
acadoWorkspace.lbA[6] = (real_t)-1.5710000000000000e+00 - tmp;
acadoWorkspace.ubA[6] = (real_t)1.5710000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[85] + acadoVariables.x[85];
acadoWorkspace.lbA[7] = (real_t)-1.5710000000000000e+00 - tmp;
acadoWorkspace.ubA[7] = (real_t)1.5710000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[102] + acadoVariables.x[102];
acadoWorkspace.lbA[8] = (real_t)-1.5710000000000000e+00 - tmp;
acadoWorkspace.ubA[8] = (real_t)1.5710000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[103] + acadoVariables.x[103];
acadoWorkspace.lbA[9] = (real_t)-1.5710000000000000e+00 - tmp;
acadoWorkspace.ubA[9] = (real_t)1.5710000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[120] + acadoVariables.x[120];
acadoWorkspace.lbA[10] = (real_t)-1.5710000000000000e+00 - tmp;
acadoWorkspace.ubA[10] = (real_t)1.5710000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[121] + acadoVariables.x[121];
acadoWorkspace.lbA[11] = (real_t)-1.5710000000000000e+00 - tmp;
acadoWorkspace.ubA[11] = (real_t)1.5710000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[138] + acadoVariables.x[138];
acadoWorkspace.lbA[12] = (real_t)-1.5710000000000000e+00 - tmp;
acadoWorkspace.ubA[12] = (real_t)1.5710000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[139] + acadoVariables.x[139];
acadoWorkspace.lbA[13] = (real_t)-1.5710000000000000e+00 - tmp;
acadoWorkspace.ubA[13] = (real_t)1.5710000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[156] + acadoVariables.x[156];
acadoWorkspace.lbA[14] = (real_t)-1.5710000000000000e+00 - tmp;
acadoWorkspace.ubA[14] = (real_t)1.5710000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[157] + acadoVariables.x[157];
acadoWorkspace.lbA[15] = (real_t)-1.5710000000000000e+00 - tmp;
acadoWorkspace.ubA[15] = (real_t)1.5710000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[174] + acadoVariables.x[174];
acadoWorkspace.lbA[16] = (real_t)-1.5710000000000000e+00 - tmp;
acadoWorkspace.ubA[16] = (real_t)1.5710000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[175] + acadoVariables.x[175];
acadoWorkspace.lbA[17] = (real_t)-1.5710000000000000e+00 - tmp;
acadoWorkspace.ubA[17] = (real_t)1.5710000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[192] + acadoVariables.x[192];
acadoWorkspace.lbA[18] = (real_t)-1.5710000000000000e+00 - tmp;
acadoWorkspace.ubA[18] = (real_t)1.5710000000000000e+00 - tmp;
tmp = acadoWorkspace.sbar[193] + acadoVariables.x[193];
acadoWorkspace.lbA[19] = (real_t)-1.5710000000000000e+00 - tmp;
acadoWorkspace.ubA[19] = (real_t)1.5710000000000000e+00 - tmp;

}

void acado_expand(  )
{
int lRun1;
acadoVariables.u[0] += acadoWorkspace.x[0];
acadoVariables.u[1] += acadoWorkspace.x[1];
acadoVariables.u[2] += acadoWorkspace.x[2];
acadoVariables.u[3] += acadoWorkspace.x[3];
acadoVariables.u[4] += acadoWorkspace.x[4];
acadoVariables.u[5] += acadoWorkspace.x[5];
acadoVariables.u[6] += acadoWorkspace.x[6];
acadoVariables.u[7] += acadoWorkspace.x[7];
acadoVariables.u[8] += acadoWorkspace.x[8];
acadoVariables.u[9] += acadoWorkspace.x[9];
acadoVariables.u[10] += acadoWorkspace.x[10];
acadoVariables.u[11] += acadoWorkspace.x[11];
acadoVariables.u[12] += acadoWorkspace.x[12];
acadoVariables.u[13] += acadoWorkspace.x[13];
acadoVariables.u[14] += acadoWorkspace.x[14];
acadoVariables.u[15] += acadoWorkspace.x[15];
acadoVariables.u[16] += acadoWorkspace.x[16];
acadoVariables.u[17] += acadoWorkspace.x[17];
acadoVariables.u[18] += acadoWorkspace.x[18];
acadoVariables.u[19] += acadoWorkspace.x[19];
acadoVariables.u[20] += acadoWorkspace.x[20];
acadoVariables.u[21] += acadoWorkspace.x[21];
acadoVariables.u[22] += acadoWorkspace.x[22];
acadoVariables.u[23] += acadoWorkspace.x[23];
acadoVariables.u[24] += acadoWorkspace.x[24];
acadoVariables.u[25] += acadoWorkspace.x[25];
acadoVariables.u[26] += acadoWorkspace.x[26];
acadoVariables.u[27] += acadoWorkspace.x[27];
acadoVariables.u[28] += acadoWorkspace.x[28];
acadoVariables.u[29] += acadoWorkspace.x[29];
acadoVariables.u[30] += acadoWorkspace.x[30];
acadoVariables.u[31] += acadoWorkspace.x[31];
acadoVariables.u[32] += acadoWorkspace.x[32];
acadoVariables.u[33] += acadoWorkspace.x[33];
acadoVariables.u[34] += acadoWorkspace.x[34];
acadoVariables.u[35] += acadoWorkspace.x[35];
acadoVariables.u[36] += acadoWorkspace.x[36];
acadoVariables.u[37] += acadoWorkspace.x[37];
acadoVariables.u[38] += acadoWorkspace.x[38];
acadoVariables.u[39] += acadoWorkspace.x[39];
acadoVariables.u[40] += acadoWorkspace.x[40];
acadoVariables.u[41] += acadoWorkspace.x[41];
acadoVariables.u[42] += acadoWorkspace.x[42];
acadoVariables.u[43] += acadoWorkspace.x[43];
acadoVariables.u[44] += acadoWorkspace.x[44];
acadoVariables.u[45] += acadoWorkspace.x[45];
acadoVariables.u[46] += acadoWorkspace.x[46];
acadoVariables.u[47] += acadoWorkspace.x[47];
acadoVariables.u[48] += acadoWorkspace.x[48];
acadoVariables.u[49] += acadoWorkspace.x[49];
acadoVariables.u[50] += acadoWorkspace.x[50];
acadoVariables.u[51] += acadoWorkspace.x[51];
acadoVariables.u[52] += acadoWorkspace.x[52];
acadoVariables.u[53] += acadoWorkspace.x[53];
acadoVariables.u[54] += acadoWorkspace.x[54];
acadoVariables.u[55] += acadoWorkspace.x[55];
acadoVariables.u[56] += acadoWorkspace.x[56];
acadoVariables.u[57] += acadoWorkspace.x[57];
acadoVariables.u[58] += acadoWorkspace.x[58];
acadoVariables.u[59] += acadoWorkspace.x[59];
acadoWorkspace.sbar[0] = acadoWorkspace.Dx0[0];
acadoWorkspace.sbar[1] = acadoWorkspace.Dx0[1];
acadoWorkspace.sbar[2] = acadoWorkspace.Dx0[2];
acadoWorkspace.sbar[3] = acadoWorkspace.Dx0[3];
acadoWorkspace.sbar[4] = acadoWorkspace.Dx0[4];
acadoWorkspace.sbar[5] = acadoWorkspace.Dx0[5];
acadoWorkspace.sbar[6] = acadoWorkspace.Dx0[6];
acadoWorkspace.sbar[7] = acadoWorkspace.Dx0[7];
acadoWorkspace.sbar[8] = acadoWorkspace.Dx0[8];
acadoWorkspace.sbar[9] = acadoWorkspace.Dx0[9];
acadoWorkspace.sbar[10] = acadoWorkspace.Dx0[10];
acadoWorkspace.sbar[11] = acadoWorkspace.Dx0[11];
acadoWorkspace.sbar[12] = acadoWorkspace.Dx0[12];
acadoWorkspace.sbar[13] = acadoWorkspace.Dx0[13];
acadoWorkspace.sbar[14] = acadoWorkspace.Dx0[14];
acadoWorkspace.sbar[15] = acadoWorkspace.Dx0[15];
acadoWorkspace.sbar[16] = acadoWorkspace.Dx0[16];
acadoWorkspace.sbar[17] = acadoWorkspace.Dx0[17];
for (lRun1 = 0; lRun1 < 180; ++lRun1)
acadoWorkspace.sbar[lRun1 + 18] = acadoWorkspace.d[lRun1];

acado_expansionStep( acadoWorkspace.evGx, acadoWorkspace.evGu, acadoWorkspace.x, acadoWorkspace.sbar, &(acadoWorkspace.sbar[ 18 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 324 ]), &(acadoWorkspace.evGu[ 108 ]), &(acadoWorkspace.x[ 6 ]), &(acadoWorkspace.sbar[ 18 ]), &(acadoWorkspace.sbar[ 36 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 648 ]), &(acadoWorkspace.evGu[ 216 ]), &(acadoWorkspace.x[ 12 ]), &(acadoWorkspace.sbar[ 36 ]), &(acadoWorkspace.sbar[ 54 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 972 ]), &(acadoWorkspace.evGu[ 324 ]), &(acadoWorkspace.x[ 18 ]), &(acadoWorkspace.sbar[ 54 ]), &(acadoWorkspace.sbar[ 72 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1296 ]), &(acadoWorkspace.evGu[ 432 ]), &(acadoWorkspace.x[ 24 ]), &(acadoWorkspace.sbar[ 72 ]), &(acadoWorkspace.sbar[ 90 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1620 ]), &(acadoWorkspace.evGu[ 540 ]), &(acadoWorkspace.x[ 30 ]), &(acadoWorkspace.sbar[ 90 ]), &(acadoWorkspace.sbar[ 108 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 1944 ]), &(acadoWorkspace.evGu[ 648 ]), &(acadoWorkspace.x[ 36 ]), &(acadoWorkspace.sbar[ 108 ]), &(acadoWorkspace.sbar[ 126 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2268 ]), &(acadoWorkspace.evGu[ 756 ]), &(acadoWorkspace.x[ 42 ]), &(acadoWorkspace.sbar[ 126 ]), &(acadoWorkspace.sbar[ 144 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2592 ]), &(acadoWorkspace.evGu[ 864 ]), &(acadoWorkspace.x[ 48 ]), &(acadoWorkspace.sbar[ 144 ]), &(acadoWorkspace.sbar[ 162 ]) );
acado_expansionStep( &(acadoWorkspace.evGx[ 2916 ]), &(acadoWorkspace.evGu[ 972 ]), &(acadoWorkspace.x[ 54 ]), &(acadoWorkspace.sbar[ 162 ]), &(acadoWorkspace.sbar[ 180 ]) );
for (lRun1 = 0; lRun1 < 198; ++lRun1)
acadoVariables.x[lRun1] += acadoWorkspace.sbar[lRun1];

}

int acado_preparationStep(  )
{
int ret;

ret = acado_modelSimulation();
acado_evaluateObjective(  );
acado_condensePrep(  );
return ret;
}

int acado_feedbackStep(  )
{
int tmp;

acado_condenseFdb(  );

tmp = acado_solve( );

acado_expand(  );
return tmp;
}

int acado_initializeSolver(  )
{
int ret;

/* This is a function which must be called once before any other function call! */


ret = 0;

memset(&acadoWorkspace, 0, sizeof( acadoWorkspace ));
return ret;
}

void acado_initializeNodesByForwardSimulation(  )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoWorkspace.state[0] = acadoVariables.x[index * 18];
acadoWorkspace.state[1] = acadoVariables.x[index * 18 + 1];
acadoWorkspace.state[2] = acadoVariables.x[index * 18 + 2];
acadoWorkspace.state[3] = acadoVariables.x[index * 18 + 3];
acadoWorkspace.state[4] = acadoVariables.x[index * 18 + 4];
acadoWorkspace.state[5] = acadoVariables.x[index * 18 + 5];
acadoWorkspace.state[6] = acadoVariables.x[index * 18 + 6];
acadoWorkspace.state[7] = acadoVariables.x[index * 18 + 7];
acadoWorkspace.state[8] = acadoVariables.x[index * 18 + 8];
acadoWorkspace.state[9] = acadoVariables.x[index * 18 + 9];
acadoWorkspace.state[10] = acadoVariables.x[index * 18 + 10];
acadoWorkspace.state[11] = acadoVariables.x[index * 18 + 11];
acadoWorkspace.state[12] = acadoVariables.x[index * 18 + 12];
acadoWorkspace.state[13] = acadoVariables.x[index * 18 + 13];
acadoWorkspace.state[14] = acadoVariables.x[index * 18 + 14];
acadoWorkspace.state[15] = acadoVariables.x[index * 18 + 15];
acadoWorkspace.state[16] = acadoVariables.x[index * 18 + 16];
acadoWorkspace.state[17] = acadoVariables.x[index * 18 + 17];
acadoWorkspace.state[450] = acadoVariables.u[index * 6];
acadoWorkspace.state[451] = acadoVariables.u[index * 6 + 1];
acadoWorkspace.state[452] = acadoVariables.u[index * 6 + 2];
acadoWorkspace.state[453] = acadoVariables.u[index * 6 + 3];
acadoWorkspace.state[454] = acadoVariables.u[index * 6 + 4];
acadoWorkspace.state[455] = acadoVariables.u[index * 6 + 5];
acadoWorkspace.state[456] = acadoVariables.od[index * 9];
acadoWorkspace.state[457] = acadoVariables.od[index * 9 + 1];
acadoWorkspace.state[458] = acadoVariables.od[index * 9 + 2];
acadoWorkspace.state[459] = acadoVariables.od[index * 9 + 3];
acadoWorkspace.state[460] = acadoVariables.od[index * 9 + 4];
acadoWorkspace.state[461] = acadoVariables.od[index * 9 + 5];
acadoWorkspace.state[462] = acadoVariables.od[index * 9 + 6];
acadoWorkspace.state[463] = acadoVariables.od[index * 9 + 7];
acadoWorkspace.state[464] = acadoVariables.od[index * 9 + 8];

acado_integrate(acadoWorkspace.state, index == 0);

acadoVariables.x[index * 18 + 18] = acadoWorkspace.state[0];
acadoVariables.x[index * 18 + 19] = acadoWorkspace.state[1];
acadoVariables.x[index * 18 + 20] = acadoWorkspace.state[2];
acadoVariables.x[index * 18 + 21] = acadoWorkspace.state[3];
acadoVariables.x[index * 18 + 22] = acadoWorkspace.state[4];
acadoVariables.x[index * 18 + 23] = acadoWorkspace.state[5];
acadoVariables.x[index * 18 + 24] = acadoWorkspace.state[6];
acadoVariables.x[index * 18 + 25] = acadoWorkspace.state[7];
acadoVariables.x[index * 18 + 26] = acadoWorkspace.state[8];
acadoVariables.x[index * 18 + 27] = acadoWorkspace.state[9];
acadoVariables.x[index * 18 + 28] = acadoWorkspace.state[10];
acadoVariables.x[index * 18 + 29] = acadoWorkspace.state[11];
acadoVariables.x[index * 18 + 30] = acadoWorkspace.state[12];
acadoVariables.x[index * 18 + 31] = acadoWorkspace.state[13];
acadoVariables.x[index * 18 + 32] = acadoWorkspace.state[14];
acadoVariables.x[index * 18 + 33] = acadoWorkspace.state[15];
acadoVariables.x[index * 18 + 34] = acadoWorkspace.state[16];
acadoVariables.x[index * 18 + 35] = acadoWorkspace.state[17];
}
}

void acado_shiftStates( int strategy, real_t* const xEnd, real_t* const uEnd )
{
int index;
for (index = 0; index < 10; ++index)
{
acadoVariables.x[index * 18] = acadoVariables.x[index * 18 + 18];
acadoVariables.x[index * 18 + 1] = acadoVariables.x[index * 18 + 19];
acadoVariables.x[index * 18 + 2] = acadoVariables.x[index * 18 + 20];
acadoVariables.x[index * 18 + 3] = acadoVariables.x[index * 18 + 21];
acadoVariables.x[index * 18 + 4] = acadoVariables.x[index * 18 + 22];
acadoVariables.x[index * 18 + 5] = acadoVariables.x[index * 18 + 23];
acadoVariables.x[index * 18 + 6] = acadoVariables.x[index * 18 + 24];
acadoVariables.x[index * 18 + 7] = acadoVariables.x[index * 18 + 25];
acadoVariables.x[index * 18 + 8] = acadoVariables.x[index * 18 + 26];
acadoVariables.x[index * 18 + 9] = acadoVariables.x[index * 18 + 27];
acadoVariables.x[index * 18 + 10] = acadoVariables.x[index * 18 + 28];
acadoVariables.x[index * 18 + 11] = acadoVariables.x[index * 18 + 29];
acadoVariables.x[index * 18 + 12] = acadoVariables.x[index * 18 + 30];
acadoVariables.x[index * 18 + 13] = acadoVariables.x[index * 18 + 31];
acadoVariables.x[index * 18 + 14] = acadoVariables.x[index * 18 + 32];
acadoVariables.x[index * 18 + 15] = acadoVariables.x[index * 18 + 33];
acadoVariables.x[index * 18 + 16] = acadoVariables.x[index * 18 + 34];
acadoVariables.x[index * 18 + 17] = acadoVariables.x[index * 18 + 35];
}

if (strategy == 1 && xEnd != 0)
{
acadoVariables.x[180] = xEnd[0];
acadoVariables.x[181] = xEnd[1];
acadoVariables.x[182] = xEnd[2];
acadoVariables.x[183] = xEnd[3];
acadoVariables.x[184] = xEnd[4];
acadoVariables.x[185] = xEnd[5];
acadoVariables.x[186] = xEnd[6];
acadoVariables.x[187] = xEnd[7];
acadoVariables.x[188] = xEnd[8];
acadoVariables.x[189] = xEnd[9];
acadoVariables.x[190] = xEnd[10];
acadoVariables.x[191] = xEnd[11];
acadoVariables.x[192] = xEnd[12];
acadoVariables.x[193] = xEnd[13];
acadoVariables.x[194] = xEnd[14];
acadoVariables.x[195] = xEnd[15];
acadoVariables.x[196] = xEnd[16];
acadoVariables.x[197] = xEnd[17];
}
else if (strategy == 2) 
{
acadoWorkspace.state[0] = acadoVariables.x[180];
acadoWorkspace.state[1] = acadoVariables.x[181];
acadoWorkspace.state[2] = acadoVariables.x[182];
acadoWorkspace.state[3] = acadoVariables.x[183];
acadoWorkspace.state[4] = acadoVariables.x[184];
acadoWorkspace.state[5] = acadoVariables.x[185];
acadoWorkspace.state[6] = acadoVariables.x[186];
acadoWorkspace.state[7] = acadoVariables.x[187];
acadoWorkspace.state[8] = acadoVariables.x[188];
acadoWorkspace.state[9] = acadoVariables.x[189];
acadoWorkspace.state[10] = acadoVariables.x[190];
acadoWorkspace.state[11] = acadoVariables.x[191];
acadoWorkspace.state[12] = acadoVariables.x[192];
acadoWorkspace.state[13] = acadoVariables.x[193];
acadoWorkspace.state[14] = acadoVariables.x[194];
acadoWorkspace.state[15] = acadoVariables.x[195];
acadoWorkspace.state[16] = acadoVariables.x[196];
acadoWorkspace.state[17] = acadoVariables.x[197];
if (uEnd != 0)
{
acadoWorkspace.state[450] = uEnd[0];
acadoWorkspace.state[451] = uEnd[1];
acadoWorkspace.state[452] = uEnd[2];
acadoWorkspace.state[453] = uEnd[3];
acadoWorkspace.state[454] = uEnd[4];
acadoWorkspace.state[455] = uEnd[5];
}
else
{
acadoWorkspace.state[450] = acadoVariables.u[54];
acadoWorkspace.state[451] = acadoVariables.u[55];
acadoWorkspace.state[452] = acadoVariables.u[56];
acadoWorkspace.state[453] = acadoVariables.u[57];
acadoWorkspace.state[454] = acadoVariables.u[58];
acadoWorkspace.state[455] = acadoVariables.u[59];
}
acadoWorkspace.state[456] = acadoVariables.od[90];
acadoWorkspace.state[457] = acadoVariables.od[91];
acadoWorkspace.state[458] = acadoVariables.od[92];
acadoWorkspace.state[459] = acadoVariables.od[93];
acadoWorkspace.state[460] = acadoVariables.od[94];
acadoWorkspace.state[461] = acadoVariables.od[95];
acadoWorkspace.state[462] = acadoVariables.od[96];
acadoWorkspace.state[463] = acadoVariables.od[97];
acadoWorkspace.state[464] = acadoVariables.od[98];

acado_integrate(acadoWorkspace.state, 1);

acadoVariables.x[180] = acadoWorkspace.state[0];
acadoVariables.x[181] = acadoWorkspace.state[1];
acadoVariables.x[182] = acadoWorkspace.state[2];
acadoVariables.x[183] = acadoWorkspace.state[3];
acadoVariables.x[184] = acadoWorkspace.state[4];
acadoVariables.x[185] = acadoWorkspace.state[5];
acadoVariables.x[186] = acadoWorkspace.state[6];
acadoVariables.x[187] = acadoWorkspace.state[7];
acadoVariables.x[188] = acadoWorkspace.state[8];
acadoVariables.x[189] = acadoWorkspace.state[9];
acadoVariables.x[190] = acadoWorkspace.state[10];
acadoVariables.x[191] = acadoWorkspace.state[11];
acadoVariables.x[192] = acadoWorkspace.state[12];
acadoVariables.x[193] = acadoWorkspace.state[13];
acadoVariables.x[194] = acadoWorkspace.state[14];
acadoVariables.x[195] = acadoWorkspace.state[15];
acadoVariables.x[196] = acadoWorkspace.state[16];
acadoVariables.x[197] = acadoWorkspace.state[17];
}
}

void acado_shiftControls( real_t* const uEnd )
{
int index;
for (index = 0; index < 9; ++index)
{
acadoVariables.u[index * 6] = acadoVariables.u[index * 6 + 6];
acadoVariables.u[index * 6 + 1] = acadoVariables.u[index * 6 + 7];
acadoVariables.u[index * 6 + 2] = acadoVariables.u[index * 6 + 8];
acadoVariables.u[index * 6 + 3] = acadoVariables.u[index * 6 + 9];
acadoVariables.u[index * 6 + 4] = acadoVariables.u[index * 6 + 10];
acadoVariables.u[index * 6 + 5] = acadoVariables.u[index * 6 + 11];
}

if (uEnd != 0)
{
acadoVariables.u[54] = uEnd[0];
acadoVariables.u[55] = uEnd[1];
acadoVariables.u[56] = uEnd[2];
acadoVariables.u[57] = uEnd[3];
acadoVariables.u[58] = uEnd[4];
acadoVariables.u[59] = uEnd[5];
}
}

real_t acado_getKKT(  )
{
real_t kkt;

int index;
real_t prd;

kkt = + acadoWorkspace.g[0]*acadoWorkspace.x[0] + acadoWorkspace.g[1]*acadoWorkspace.x[1] + acadoWorkspace.g[2]*acadoWorkspace.x[2] + acadoWorkspace.g[3]*acadoWorkspace.x[3] + acadoWorkspace.g[4]*acadoWorkspace.x[4] + acadoWorkspace.g[5]*acadoWorkspace.x[5] + acadoWorkspace.g[6]*acadoWorkspace.x[6] + acadoWorkspace.g[7]*acadoWorkspace.x[7] + acadoWorkspace.g[8]*acadoWorkspace.x[8] + acadoWorkspace.g[9]*acadoWorkspace.x[9] + acadoWorkspace.g[10]*acadoWorkspace.x[10] + acadoWorkspace.g[11]*acadoWorkspace.x[11] + acadoWorkspace.g[12]*acadoWorkspace.x[12] + acadoWorkspace.g[13]*acadoWorkspace.x[13] + acadoWorkspace.g[14]*acadoWorkspace.x[14] + acadoWorkspace.g[15]*acadoWorkspace.x[15] + acadoWorkspace.g[16]*acadoWorkspace.x[16] + acadoWorkspace.g[17]*acadoWorkspace.x[17] + acadoWorkspace.g[18]*acadoWorkspace.x[18] + acadoWorkspace.g[19]*acadoWorkspace.x[19] + acadoWorkspace.g[20]*acadoWorkspace.x[20] + acadoWorkspace.g[21]*acadoWorkspace.x[21] + acadoWorkspace.g[22]*acadoWorkspace.x[22] + acadoWorkspace.g[23]*acadoWorkspace.x[23] + acadoWorkspace.g[24]*acadoWorkspace.x[24] + acadoWorkspace.g[25]*acadoWorkspace.x[25] + acadoWorkspace.g[26]*acadoWorkspace.x[26] + acadoWorkspace.g[27]*acadoWorkspace.x[27] + acadoWorkspace.g[28]*acadoWorkspace.x[28] + acadoWorkspace.g[29]*acadoWorkspace.x[29] + acadoWorkspace.g[30]*acadoWorkspace.x[30] + acadoWorkspace.g[31]*acadoWorkspace.x[31] + acadoWorkspace.g[32]*acadoWorkspace.x[32] + acadoWorkspace.g[33]*acadoWorkspace.x[33] + acadoWorkspace.g[34]*acadoWorkspace.x[34] + acadoWorkspace.g[35]*acadoWorkspace.x[35] + acadoWorkspace.g[36]*acadoWorkspace.x[36] + acadoWorkspace.g[37]*acadoWorkspace.x[37] + acadoWorkspace.g[38]*acadoWorkspace.x[38] + acadoWorkspace.g[39]*acadoWorkspace.x[39] + acadoWorkspace.g[40]*acadoWorkspace.x[40] + acadoWorkspace.g[41]*acadoWorkspace.x[41] + acadoWorkspace.g[42]*acadoWorkspace.x[42] + acadoWorkspace.g[43]*acadoWorkspace.x[43] + acadoWorkspace.g[44]*acadoWorkspace.x[44] + acadoWorkspace.g[45]*acadoWorkspace.x[45] + acadoWorkspace.g[46]*acadoWorkspace.x[46] + acadoWorkspace.g[47]*acadoWorkspace.x[47] + acadoWorkspace.g[48]*acadoWorkspace.x[48] + acadoWorkspace.g[49]*acadoWorkspace.x[49] + acadoWorkspace.g[50]*acadoWorkspace.x[50] + acadoWorkspace.g[51]*acadoWorkspace.x[51] + acadoWorkspace.g[52]*acadoWorkspace.x[52] + acadoWorkspace.g[53]*acadoWorkspace.x[53] + acadoWorkspace.g[54]*acadoWorkspace.x[54] + acadoWorkspace.g[55]*acadoWorkspace.x[55] + acadoWorkspace.g[56]*acadoWorkspace.x[56] + acadoWorkspace.g[57]*acadoWorkspace.x[57] + acadoWorkspace.g[58]*acadoWorkspace.x[58] + acadoWorkspace.g[59]*acadoWorkspace.x[59];
kkt = fabs( kkt );
for (index = 0; index < 60; ++index)
{
prd = acadoWorkspace.y[index];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lb[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ub[index] * prd);
}
for (index = 0; index < 20; ++index)
{
prd = acadoWorkspace.y[index + 60];
if (prd > 1e-12)
kkt += fabs(acadoWorkspace.lbA[index] * prd);
else if (prd < -1e-12)
kkt += fabs(acadoWorkspace.ubA[index] * prd);
}
return kkt;
}

real_t acado_getObjective(  )
{
real_t objVal;

int lRun1;
/** Row vector of size: 19 */
real_t tmpDy[ 19 ];

/** Row vector of size: 13 */
real_t tmpDyN[ 13 ];

for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
acadoWorkspace.objValueIn[0] = acadoVariables.x[lRun1 * 18];
acadoWorkspace.objValueIn[1] = acadoVariables.x[lRun1 * 18 + 1];
acadoWorkspace.objValueIn[2] = acadoVariables.x[lRun1 * 18 + 2];
acadoWorkspace.objValueIn[3] = acadoVariables.x[lRun1 * 18 + 3];
acadoWorkspace.objValueIn[4] = acadoVariables.x[lRun1 * 18 + 4];
acadoWorkspace.objValueIn[5] = acadoVariables.x[lRun1 * 18 + 5];
acadoWorkspace.objValueIn[6] = acadoVariables.x[lRun1 * 18 + 6];
acadoWorkspace.objValueIn[7] = acadoVariables.x[lRun1 * 18 + 7];
acadoWorkspace.objValueIn[8] = acadoVariables.x[lRun1 * 18 + 8];
acadoWorkspace.objValueIn[9] = acadoVariables.x[lRun1 * 18 + 9];
acadoWorkspace.objValueIn[10] = acadoVariables.x[lRun1 * 18 + 10];
acadoWorkspace.objValueIn[11] = acadoVariables.x[lRun1 * 18 + 11];
acadoWorkspace.objValueIn[12] = acadoVariables.x[lRun1 * 18 + 12];
acadoWorkspace.objValueIn[13] = acadoVariables.x[lRun1 * 18 + 13];
acadoWorkspace.objValueIn[14] = acadoVariables.x[lRun1 * 18 + 14];
acadoWorkspace.objValueIn[15] = acadoVariables.x[lRun1 * 18 + 15];
acadoWorkspace.objValueIn[16] = acadoVariables.x[lRun1 * 18 + 16];
acadoWorkspace.objValueIn[17] = acadoVariables.x[lRun1 * 18 + 17];
acadoWorkspace.objValueIn[18] = acadoVariables.u[lRun1 * 6];
acadoWorkspace.objValueIn[19] = acadoVariables.u[lRun1 * 6 + 1];
acadoWorkspace.objValueIn[20] = acadoVariables.u[lRun1 * 6 + 2];
acadoWorkspace.objValueIn[21] = acadoVariables.u[lRun1 * 6 + 3];
acadoWorkspace.objValueIn[22] = acadoVariables.u[lRun1 * 6 + 4];
acadoWorkspace.objValueIn[23] = acadoVariables.u[lRun1 * 6 + 5];
acadoWorkspace.objValueIn[24] = acadoVariables.od[lRun1 * 9];
acadoWorkspace.objValueIn[25] = acadoVariables.od[lRun1 * 9 + 1];
acadoWorkspace.objValueIn[26] = acadoVariables.od[lRun1 * 9 + 2];
acadoWorkspace.objValueIn[27] = acadoVariables.od[lRun1 * 9 + 3];
acadoWorkspace.objValueIn[28] = acadoVariables.od[lRun1 * 9 + 4];
acadoWorkspace.objValueIn[29] = acadoVariables.od[lRun1 * 9 + 5];
acadoWorkspace.objValueIn[30] = acadoVariables.od[lRun1 * 9 + 6];
acadoWorkspace.objValueIn[31] = acadoVariables.od[lRun1 * 9 + 7];
acadoWorkspace.objValueIn[32] = acadoVariables.od[lRun1 * 9 + 8];

acado_evaluateLSQ( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.Dy[lRun1 * 19] = acadoWorkspace.objValueOut[0] - acadoVariables.y[lRun1 * 19];
acadoWorkspace.Dy[lRun1 * 19 + 1] = acadoWorkspace.objValueOut[1] - acadoVariables.y[lRun1 * 19 + 1];
acadoWorkspace.Dy[lRun1 * 19 + 2] = acadoWorkspace.objValueOut[2] - acadoVariables.y[lRun1 * 19 + 2];
acadoWorkspace.Dy[lRun1 * 19 + 3] = acadoWorkspace.objValueOut[3] - acadoVariables.y[lRun1 * 19 + 3];
acadoWorkspace.Dy[lRun1 * 19 + 4] = acadoWorkspace.objValueOut[4] - acadoVariables.y[lRun1 * 19 + 4];
acadoWorkspace.Dy[lRun1 * 19 + 5] = acadoWorkspace.objValueOut[5] - acadoVariables.y[lRun1 * 19 + 5];
acadoWorkspace.Dy[lRun1 * 19 + 6] = acadoWorkspace.objValueOut[6] - acadoVariables.y[lRun1 * 19 + 6];
acadoWorkspace.Dy[lRun1 * 19 + 7] = acadoWorkspace.objValueOut[7] - acadoVariables.y[lRun1 * 19 + 7];
acadoWorkspace.Dy[lRun1 * 19 + 8] = acadoWorkspace.objValueOut[8] - acadoVariables.y[lRun1 * 19 + 8];
acadoWorkspace.Dy[lRun1 * 19 + 9] = acadoWorkspace.objValueOut[9] - acadoVariables.y[lRun1 * 19 + 9];
acadoWorkspace.Dy[lRun1 * 19 + 10] = acadoWorkspace.objValueOut[10] - acadoVariables.y[lRun1 * 19 + 10];
acadoWorkspace.Dy[lRun1 * 19 + 11] = acadoWorkspace.objValueOut[11] - acadoVariables.y[lRun1 * 19 + 11];
acadoWorkspace.Dy[lRun1 * 19 + 12] = acadoWorkspace.objValueOut[12] - acadoVariables.y[lRun1 * 19 + 12];
acadoWorkspace.Dy[lRun1 * 19 + 13] = acadoWorkspace.objValueOut[13] - acadoVariables.y[lRun1 * 19 + 13];
acadoWorkspace.Dy[lRun1 * 19 + 14] = acadoWorkspace.objValueOut[14] - acadoVariables.y[lRun1 * 19 + 14];
acadoWorkspace.Dy[lRun1 * 19 + 15] = acadoWorkspace.objValueOut[15] - acadoVariables.y[lRun1 * 19 + 15];
acadoWorkspace.Dy[lRun1 * 19 + 16] = acadoWorkspace.objValueOut[16] - acadoVariables.y[lRun1 * 19 + 16];
acadoWorkspace.Dy[lRun1 * 19 + 17] = acadoWorkspace.objValueOut[17] - acadoVariables.y[lRun1 * 19 + 17];
acadoWorkspace.Dy[lRun1 * 19 + 18] = acadoWorkspace.objValueOut[18] - acadoVariables.y[lRun1 * 19 + 18];
}
acadoWorkspace.objValueIn[0] = acadoVariables.x[180];
acadoWorkspace.objValueIn[1] = acadoVariables.x[181];
acadoWorkspace.objValueIn[2] = acadoVariables.x[182];
acadoWorkspace.objValueIn[3] = acadoVariables.x[183];
acadoWorkspace.objValueIn[4] = acadoVariables.x[184];
acadoWorkspace.objValueIn[5] = acadoVariables.x[185];
acadoWorkspace.objValueIn[6] = acadoVariables.x[186];
acadoWorkspace.objValueIn[7] = acadoVariables.x[187];
acadoWorkspace.objValueIn[8] = acadoVariables.x[188];
acadoWorkspace.objValueIn[9] = acadoVariables.x[189];
acadoWorkspace.objValueIn[10] = acadoVariables.x[190];
acadoWorkspace.objValueIn[11] = acadoVariables.x[191];
acadoWorkspace.objValueIn[12] = acadoVariables.x[192];
acadoWorkspace.objValueIn[13] = acadoVariables.x[193];
acadoWorkspace.objValueIn[14] = acadoVariables.x[194];
acadoWorkspace.objValueIn[15] = acadoVariables.x[195];
acadoWorkspace.objValueIn[16] = acadoVariables.x[196];
acadoWorkspace.objValueIn[17] = acadoVariables.x[197];
acadoWorkspace.objValueIn[18] = acadoVariables.od[90];
acadoWorkspace.objValueIn[19] = acadoVariables.od[91];
acadoWorkspace.objValueIn[20] = acadoVariables.od[92];
acadoWorkspace.objValueIn[21] = acadoVariables.od[93];
acadoWorkspace.objValueIn[22] = acadoVariables.od[94];
acadoWorkspace.objValueIn[23] = acadoVariables.od[95];
acadoWorkspace.objValueIn[24] = acadoVariables.od[96];
acadoWorkspace.objValueIn[25] = acadoVariables.od[97];
acadoWorkspace.objValueIn[26] = acadoVariables.od[98];
acado_evaluateLSQEndTerm( acadoWorkspace.objValueIn, acadoWorkspace.objValueOut );
acadoWorkspace.DyN[0] = acadoWorkspace.objValueOut[0] - acadoVariables.yN[0];
acadoWorkspace.DyN[1] = acadoWorkspace.objValueOut[1] - acadoVariables.yN[1];
acadoWorkspace.DyN[2] = acadoWorkspace.objValueOut[2] - acadoVariables.yN[2];
acadoWorkspace.DyN[3] = acadoWorkspace.objValueOut[3] - acadoVariables.yN[3];
acadoWorkspace.DyN[4] = acadoWorkspace.objValueOut[4] - acadoVariables.yN[4];
acadoWorkspace.DyN[5] = acadoWorkspace.objValueOut[5] - acadoVariables.yN[5];
acadoWorkspace.DyN[6] = acadoWorkspace.objValueOut[6] - acadoVariables.yN[6];
acadoWorkspace.DyN[7] = acadoWorkspace.objValueOut[7] - acadoVariables.yN[7];
acadoWorkspace.DyN[8] = acadoWorkspace.objValueOut[8] - acadoVariables.yN[8];
acadoWorkspace.DyN[9] = acadoWorkspace.objValueOut[9] - acadoVariables.yN[9];
acadoWorkspace.DyN[10] = acadoWorkspace.objValueOut[10] - acadoVariables.yN[10];
acadoWorkspace.DyN[11] = acadoWorkspace.objValueOut[11] - acadoVariables.yN[11];
acadoWorkspace.DyN[12] = acadoWorkspace.objValueOut[12] - acadoVariables.yN[12];
objVal = 0.0000000000000000e+00;
for (lRun1 = 0; lRun1 < 10; ++lRun1)
{
tmpDy[0] = + acadoWorkspace.Dy[lRun1 * 19]*acadoVariables.W[0];
tmpDy[1] = + acadoWorkspace.Dy[lRun1 * 19 + 1]*acadoVariables.W[20];
tmpDy[2] = + acadoWorkspace.Dy[lRun1 * 19 + 2]*acadoVariables.W[40];
tmpDy[3] = + acadoWorkspace.Dy[lRun1 * 19 + 3]*acadoVariables.W[60];
tmpDy[4] = + acadoWorkspace.Dy[lRun1 * 19 + 4]*acadoVariables.W[80];
tmpDy[5] = + acadoWorkspace.Dy[lRun1 * 19 + 5]*acadoVariables.W[100];
tmpDy[6] = + acadoWorkspace.Dy[lRun1 * 19 + 6]*acadoVariables.W[120];
tmpDy[7] = + acadoWorkspace.Dy[lRun1 * 19 + 7]*acadoVariables.W[140];
tmpDy[8] = + acadoWorkspace.Dy[lRun1 * 19 + 8]*acadoVariables.W[160];
tmpDy[9] = + acadoWorkspace.Dy[lRun1 * 19 + 9]*acadoVariables.W[180];
tmpDy[10] = + acadoWorkspace.Dy[lRun1 * 19 + 10]*acadoVariables.W[200];
tmpDy[11] = + acadoWorkspace.Dy[lRun1 * 19 + 11]*acadoVariables.W[220];
tmpDy[12] = + acadoWorkspace.Dy[lRun1 * 19 + 12]*acadoVariables.W[240];
tmpDy[13] = + acadoWorkspace.Dy[lRun1 * 19 + 13]*acadoVariables.W[260];
tmpDy[14] = + acadoWorkspace.Dy[lRun1 * 19 + 14]*acadoVariables.W[280];
tmpDy[15] = + acadoWorkspace.Dy[lRun1 * 19 + 15]*acadoVariables.W[300];
tmpDy[16] = + acadoWorkspace.Dy[lRun1 * 19 + 16]*acadoVariables.W[320];
tmpDy[17] = + acadoWorkspace.Dy[lRun1 * 19 + 17]*acadoVariables.W[340];
tmpDy[18] = + acadoWorkspace.Dy[lRun1 * 19 + 18]*acadoVariables.W[360];
objVal += + acadoWorkspace.Dy[lRun1 * 19]*tmpDy[0] + acadoWorkspace.Dy[lRun1 * 19 + 1]*tmpDy[1] + acadoWorkspace.Dy[lRun1 * 19 + 2]*tmpDy[2] + acadoWorkspace.Dy[lRun1 * 19 + 3]*tmpDy[3] + acadoWorkspace.Dy[lRun1 * 19 + 4]*tmpDy[4] + acadoWorkspace.Dy[lRun1 * 19 + 5]*tmpDy[5] + acadoWorkspace.Dy[lRun1 * 19 + 6]*tmpDy[6] + acadoWorkspace.Dy[lRun1 * 19 + 7]*tmpDy[7] + acadoWorkspace.Dy[lRun1 * 19 + 8]*tmpDy[8] + acadoWorkspace.Dy[lRun1 * 19 + 9]*tmpDy[9] + acadoWorkspace.Dy[lRun1 * 19 + 10]*tmpDy[10] + acadoWorkspace.Dy[lRun1 * 19 + 11]*tmpDy[11] + acadoWorkspace.Dy[lRun1 * 19 + 12]*tmpDy[12] + acadoWorkspace.Dy[lRun1 * 19 + 13]*tmpDy[13] + acadoWorkspace.Dy[lRun1 * 19 + 14]*tmpDy[14] + acadoWorkspace.Dy[lRun1 * 19 + 15]*tmpDy[15] + acadoWorkspace.Dy[lRun1 * 19 + 16]*tmpDy[16] + acadoWorkspace.Dy[lRun1 * 19 + 17]*tmpDy[17] + acadoWorkspace.Dy[lRun1 * 19 + 18]*tmpDy[18];
}

tmpDyN[0] = + acadoWorkspace.DyN[0]*acadoVariables.WN[0];
tmpDyN[1] = + acadoWorkspace.DyN[1]*acadoVariables.WN[14];
tmpDyN[2] = + acadoWorkspace.DyN[2]*acadoVariables.WN[28];
tmpDyN[3] = + acadoWorkspace.DyN[3]*acadoVariables.WN[42];
tmpDyN[4] = + acadoWorkspace.DyN[4]*acadoVariables.WN[56];
tmpDyN[5] = + acadoWorkspace.DyN[5]*acadoVariables.WN[70];
tmpDyN[6] = + acadoWorkspace.DyN[6]*acadoVariables.WN[84];
tmpDyN[7] = + acadoWorkspace.DyN[7]*acadoVariables.WN[98];
tmpDyN[8] = + acadoWorkspace.DyN[8]*acadoVariables.WN[112];
tmpDyN[9] = + acadoWorkspace.DyN[9]*acadoVariables.WN[126];
tmpDyN[10] = + acadoWorkspace.DyN[10]*acadoVariables.WN[140];
tmpDyN[11] = + acadoWorkspace.DyN[11]*acadoVariables.WN[154];
tmpDyN[12] = + acadoWorkspace.DyN[12]*acadoVariables.WN[168];
objVal += + acadoWorkspace.DyN[0]*tmpDyN[0] + acadoWorkspace.DyN[1]*tmpDyN[1] + acadoWorkspace.DyN[2]*tmpDyN[2] + acadoWorkspace.DyN[3]*tmpDyN[3] + acadoWorkspace.DyN[4]*tmpDyN[4] + acadoWorkspace.DyN[5]*tmpDyN[5] + acadoWorkspace.DyN[6]*tmpDyN[6] + acadoWorkspace.DyN[7]*tmpDyN[7] + acadoWorkspace.DyN[8]*tmpDyN[8] + acadoWorkspace.DyN[9]*tmpDyN[9] + acadoWorkspace.DyN[10]*tmpDyN[10] + acadoWorkspace.DyN[11]*tmpDyN[11] + acadoWorkspace.DyN[12]*tmpDyN[12];

objVal *= 0.5;
return objVal;
}

