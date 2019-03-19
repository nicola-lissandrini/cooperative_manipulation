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


void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 3;

/* Compute outputs: */
out[0] = xd[1];
out[1] = u[0];
out[2] = u[1];
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = (real_t)(0.0000000000000000e+00);
out[1] = (real_t)(1.0000000000000000e+00);
out[2] = (real_t)(0.0000000000000000e+00);
out[3] = (real_t)(0.0000000000000000e+00);
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = (real_t)(0.0000000000000000e+00);
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = (real_t)(1.0000000000000000e+00);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
out[14] = (real_t)(1.0000000000000000e+00);
}



void acado_solve_dim3_triangular( real_t* const A, real_t* const b )
{

b[2] = b[2]/A[8];
b[1] -= + A[5]*b[2];
b[1] = b[1]/A[4];
b[0] -= + A[2]*b[2];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t acado_solve_dim3_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 3; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
if(fabs(A[3]) > fabs(A[0]) && fabs(A[3]) > fabs(A[6])) {
acadoWorkspace.rk_dim3_swap = A[0];
A[0] = A[3];
A[3] = acadoWorkspace.rk_dim3_swap;
acadoWorkspace.rk_dim3_swap = A[1];
A[1] = A[4];
A[4] = acadoWorkspace.rk_dim3_swap;
acadoWorkspace.rk_dim3_swap = A[2];
A[2] = A[5];
A[5] = acadoWorkspace.rk_dim3_swap;
acadoWorkspace.rk_dim3_swap = b[0];
b[0] = b[1];
b[1] = acadoWorkspace.rk_dim3_swap;
intSwap = rk_perm[0];
rk_perm[0] = rk_perm[1];
rk_perm[1] = intSwap;
}
else if(fabs(A[6]) > fabs(A[0]) && fabs(A[6]) > fabs(A[3])) {
acadoWorkspace.rk_dim3_swap = A[0];
A[0] = A[6];
A[6] = acadoWorkspace.rk_dim3_swap;
acadoWorkspace.rk_dim3_swap = A[1];
A[1] = A[7];
A[7] = acadoWorkspace.rk_dim3_swap;
acadoWorkspace.rk_dim3_swap = A[2];
A[2] = A[8];
A[8] = acadoWorkspace.rk_dim3_swap;
acadoWorkspace.rk_dim3_swap = b[0];
b[0] = b[2];
b[2] = acadoWorkspace.rk_dim3_swap;
intSwap = rk_perm[0];
rk_perm[0] = rk_perm[2];
rk_perm[2] = intSwap;
}

A[3] = -A[3]/A[0];
A[4] += + A[3]*A[1];
A[5] += + A[3]*A[2];
b[1] += + A[3]*b[0];

A[6] = -A[6]/A[0];
A[7] += + A[6]*A[1];
A[8] += + A[6]*A[2];
b[2] += + A[6]*b[0];

det = + det*A[0];

if(fabs(A[7]) > fabs(A[4])) {
acadoWorkspace.rk_dim3_swap = A[3];
A[3] = A[6];
A[6] = acadoWorkspace.rk_dim3_swap;
acadoWorkspace.rk_dim3_swap = A[4];
A[4] = A[7];
A[7] = acadoWorkspace.rk_dim3_swap;
acadoWorkspace.rk_dim3_swap = A[5];
A[5] = A[8];
A[8] = acadoWorkspace.rk_dim3_swap;
acadoWorkspace.rk_dim3_swap = b[1];
b[1] = b[2];
b[2] = acadoWorkspace.rk_dim3_swap;
intSwap = rk_perm[1];
rk_perm[1] = rk_perm[2];
rk_perm[2] = intSwap;
}

A[7] = -A[7]/A[4];
A[8] += + A[7]*A[5];
b[2] += + A[7]*b[1];

det = + det*A[4];

det = + det*A[8];

det = fabs(det);
acado_solve_dim3_triangular( A, b );
return det;
}

void acado_solve_dim3_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

acadoWorkspace.rk_dim3_bPerm[0] = b[rk_perm[0]];
acadoWorkspace.rk_dim3_bPerm[1] = b[rk_perm[1]];
acadoWorkspace.rk_dim3_bPerm[2] = b[rk_perm[2]];
acadoWorkspace.rk_dim3_bPerm[1] += A[3]*acadoWorkspace.rk_dim3_bPerm[0];

acadoWorkspace.rk_dim3_bPerm[2] += A[6]*acadoWorkspace.rk_dim3_bPerm[0];
acadoWorkspace.rk_dim3_bPerm[2] += A[7]*acadoWorkspace.rk_dim3_bPerm[1];


acado_solve_dim3_triangular( A, acadoWorkspace.rk_dim3_bPerm );
b[0] = acadoWorkspace.rk_dim3_bPerm[0];
b[1] = acadoWorkspace.rk_dim3_bPerm[1];
b[2] = acadoWorkspace.rk_dim3_bPerm[2];
}



/** Column vector of size: 1 */
static const real_t acado_Ah_mat[ 1 ] = 
{ 2.5000000000000001e-02 };


/* Fixed step size:0.05 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int run;
int run1;
int tmp_index1;
int tmp_index2;

real_t det;

acadoWorkspace.rk_ttt = 0.0000000000000000e+00;
acadoWorkspace.rk_xxx[3] = rk_eta[18];
acadoWorkspace.rk_xxx[4] = rk_eta[19];
acadoWorkspace.rk_xxx[5] = rk_eta[20];

for (run = 0; run < 2; ++run)
{
if( run > 0 ) {
for (i = 0; i < 3; ++i)
{
acadoWorkspace.rk_diffsPrev2[i * 5] = rk_eta[i * 3 + 3];
acadoWorkspace.rk_diffsPrev2[i * 5 + 1] = rk_eta[i * 3 + 4];
acadoWorkspace.rk_diffsPrev2[i * 5 + 2] = rk_eta[i * 3 + 5];
acadoWorkspace.rk_diffsPrev2[i * 5 + 3] = rk_eta[i * 2 + 12];
acadoWorkspace.rk_diffsPrev2[i * 5 + 4] = rk_eta[i * 2 + 13];
}
}
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 3; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1]*acadoWorkspace.rk_kkk[tmp_index1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 15 ]) );
for (j = 0; j < 3; ++j)
{
tmp_index1 = (run1 * 3) + (j);
acadoWorkspace.rk_A[tmp_index1 * 3] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 15) + (j * 5)];
acadoWorkspace.rk_A[tmp_index1 * 3 + 1] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 15) + (j * 5 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 3 + 2] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 15) + (j * 5 + 2)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 3) + (j)] -= 1.0000000000000000e+00;
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 3] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 3 + 1] = acadoWorkspace.rk_kkk[run1 + 1] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 3 + 2] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[2];
}
det = acado_solve_dim3_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim3_perm );
for (j = 0; j < 1; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 3];
acadoWorkspace.rk_kkk[j + 1] += acadoWorkspace.rk_b[j * 3 + 1];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 3 + 2];
}
}
}
for (i = 0; i < 5; ++i)
{
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 3; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1]*acadoWorkspace.rk_kkk[tmp_index1];
}
acado_rhs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_rhsTemp );
acadoWorkspace.rk_b[run1 * 3] = acadoWorkspace.rk_kkk[run1] - acadoWorkspace.rk_rhsTemp[0];
acadoWorkspace.rk_b[run1 * 3 + 1] = acadoWorkspace.rk_kkk[run1 + 1] - acadoWorkspace.rk_rhsTemp[1];
acadoWorkspace.rk_b[run1 * 3 + 2] = acadoWorkspace.rk_kkk[run1 + 2] - acadoWorkspace.rk_rhsTemp[2];
}
acado_solve_dim3_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim3_perm );
for (j = 0; j < 1; ++j)
{
acadoWorkspace.rk_kkk[j] += acadoWorkspace.rk_b[j * 3];
acadoWorkspace.rk_kkk[j + 1] += acadoWorkspace.rk_b[j * 3 + 1];
acadoWorkspace.rk_kkk[j + 2] += acadoWorkspace.rk_b[j * 3 + 2];
}
}
for (run1 = 0; run1 < 1; ++run1)
{
for (j = 0; j < 3; ++j)
{
acadoWorkspace.rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
acadoWorkspace.rk_xxx[j] += + acado_Ah_mat[run1]*acadoWorkspace.rk_kkk[tmp_index1];
}
acado_diffs( acadoWorkspace.rk_xxx, &(acadoWorkspace.rk_diffsTemp2[ run1 * 15 ]) );
for (j = 0; j < 3; ++j)
{
tmp_index1 = (run1 * 3) + (j);
acadoWorkspace.rk_A[tmp_index1 * 3] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 15) + (j * 5)];
acadoWorkspace.rk_A[tmp_index1 * 3 + 1] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 15) + (j * 5 + 1)];
acadoWorkspace.rk_A[tmp_index1 * 3 + 2] = + acado_Ah_mat[run1]*acadoWorkspace.rk_diffsTemp2[(run1 * 15) + (j * 5 + 2)];
if( 0 == run1 ) acadoWorkspace.rk_A[(tmp_index1 * 3) + (j)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 3; ++run1)
{
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_b[i * 3] = - acadoWorkspace.rk_diffsTemp2[(i * 15) + (run1)];
acadoWorkspace.rk_b[i * 3 + 1] = - acadoWorkspace.rk_diffsTemp2[(i * 15) + (run1 + 5)];
acadoWorkspace.rk_b[i * 3 + 2] = - acadoWorkspace.rk_diffsTemp2[(i * 15) + (run1 + 10)];
}
if( 0 == run1 ) {
det = acado_solve_dim3_system( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim3_perm );
}
 else {
acado_solve_dim3_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim3_perm );
}
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 3];
acadoWorkspace.rk_diffK[i + 1] = acadoWorkspace.rk_b[i * 3 + 1];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 3 + 2];
}
for (i = 0; i < 3; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 5) + (run1)] = (i == run1-0);
acadoWorkspace.rk_diffsNew2[(i * 5) + (run1)] += + acadoWorkspace.rk_diffK[i]*(real_t)5.0000000000000003e-02;
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (i = 0; i < 1; ++i)
{
for (j = 0; j < 3; ++j)
{
tmp_index1 = (i * 3) + (j);
tmp_index2 = (run1) + (j * 5);
acadoWorkspace.rk_b[tmp_index1] = - acadoWorkspace.rk_diffsTemp2[(i * 15) + (tmp_index2 + 3)];
}
}
acado_solve_dim3_system_reuse( acadoWorkspace.rk_A, acadoWorkspace.rk_b, acadoWorkspace.rk_dim3_perm );
for (i = 0; i < 1; ++i)
{
acadoWorkspace.rk_diffK[i] = acadoWorkspace.rk_b[i * 3];
acadoWorkspace.rk_diffK[i + 1] = acadoWorkspace.rk_b[i * 3 + 1];
acadoWorkspace.rk_diffK[i + 2] = acadoWorkspace.rk_b[i * 3 + 2];
}
for (i = 0; i < 3; ++i)
{
acadoWorkspace.rk_diffsNew2[(i * 5) + (run1 + 3)] = + acadoWorkspace.rk_diffK[i]*(real_t)5.0000000000000003e-02;
}
}
rk_eta[0] += + acadoWorkspace.rk_kkk[0]*(real_t)5.0000000000000003e-02;
rk_eta[1] += + acadoWorkspace.rk_kkk[1]*(real_t)5.0000000000000003e-02;
rk_eta[2] += + acadoWorkspace.rk_kkk[2]*(real_t)5.0000000000000003e-02;
if( run == 0 ) {
for (i = 0; i < 3; ++i)
{
for (j = 0; j < 3; ++j)
{
tmp_index2 = (j) + (i * 3);
rk_eta[tmp_index2 + 3] = acadoWorkspace.rk_diffsNew2[(i * 5) + (j)];
}
for (j = 0; j < 2; ++j)
{
tmp_index2 = (j) + (i * 2);
rk_eta[tmp_index2 + 12] = acadoWorkspace.rk_diffsNew2[(i * 5) + (j + 3)];
}
}
}
else {
for (i = 0; i < 3; ++i)
{
for (j = 0; j < 3; ++j)
{
tmp_index2 = (j) + (i * 3);
rk_eta[tmp_index2 + 3] = + acadoWorkspace.rk_diffsNew2[i * 5]*acadoWorkspace.rk_diffsPrev2[j];
rk_eta[tmp_index2 + 3] += + acadoWorkspace.rk_diffsNew2[i * 5 + 1]*acadoWorkspace.rk_diffsPrev2[j + 5];
rk_eta[tmp_index2 + 3] += + acadoWorkspace.rk_diffsNew2[i * 5 + 2]*acadoWorkspace.rk_diffsPrev2[j + 10];
}
for (j = 0; j < 2; ++j)
{
tmp_index2 = (j) + (i * 2);
rk_eta[tmp_index2 + 12] = acadoWorkspace.rk_diffsNew2[(i * 5) + (j + 3)];
rk_eta[tmp_index2 + 12] += + acadoWorkspace.rk_diffsNew2[i * 5]*acadoWorkspace.rk_diffsPrev2[j + 3];
rk_eta[tmp_index2 + 12] += + acadoWorkspace.rk_diffsNew2[i * 5 + 1]*acadoWorkspace.rk_diffsPrev2[j + 8];
rk_eta[tmp_index2 + 12] += + acadoWorkspace.rk_diffsNew2[i * 5 + 2]*acadoWorkspace.rk_diffsPrev2[j + 13];
}
}
}
resetIntegrator = 0;
acadoWorkspace.rk_ttt += 5.0000000000000000e-01;
}
for (i = 0; i < 3; ++i)
{
}
if( det < 1e-12 ) {
error = 2;
} else if( det < 1e-6 ) {
error = 1;
} else {
error = 0;
}
return error;
}



