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


real_t rk_dim28_swap;

/** Column vector of size: 28 */
real_t rk_dim28_bPerm[ 28 ];

/** Column vector of size: 22 */
real_t auxVar[ 22 ];

real_t rk_ttt;

/** Row vector of size: 32 */
real_t rk_xxx[ 32 ];

/** Matrix of size: 14 x 2 (row major format) */
real_t rk_kkk[ 28 ];

/** Matrix of size: 28 x 28 (row major format) */
real_t rk_A[ 784 ];

/** Column vector of size: 28 */
real_t rk_b[ 28 ];

/** Row vector of size: 28 */
int rk_dim28_perm[ 28 ];

/** Column vector of size: 14 */
real_t rk_rhsTemp[ 14 ];

/** Matrix of size: 2 x 308 (row major format) */
real_t rk_diffsTemp2[ 616 ];

/** Matrix of size: 14 x 2 (row major format) */
real_t rk_diffK[ 28 ];

/** Matrix of size: 14 x 22 (row major format) */
real_t rk_diffsNew2[ 308 ];

#pragma omp threadprivate( auxVar, rk_ttt, rk_xxx, rk_kkk, rk_diffK, rk_rhsTemp, rk_dim28_perm, rk_A, rk_b, rk_diffsNew2, rk_diffsTemp2, rk_dim28_swap, rk_dim28_bPerm )

void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 14;
/* Vector of auxiliary variables; number of elements: 14. */
real_t* a = auxVar;

/* Compute intermediate quantities: */
a[0] = xd[10];
a[1] = ((real_t)(5.1134634999999998e-02)*a[0]);
a[2] = xd[11];
a[3] = (a[1]+((real_t)(-5.1484328000000003e-02)*a[2]));
a[4] = xd[12];
a[5] = (a[3]+((real_t)(5.1273007000000002e-02)*a[4]));
a[6] = xd[13];
a[7] = (a[5]+((real_t)(-5.1385595999999999e-02)*a[6]));
a[8] = ((real_t)(3.8261752999999998e-01)+a[7]);
a[9] = ((real_t)(-5.1819004000000002e-02)*a[0]);
a[10] = (a[9]+((real_t)(5.1844250000000001e-02)*a[2]));
a[11] = (a[10]+((real_t)(-5.1987289999999999e-02)*a[4]));
a[12] = (a[11]+((real_t)(5.1781394000000001e-02)*a[6]));
a[13] = ((real_t)(3.8267942999999999e-01)+a[12]);

/* Compute outputs: */
out[0] = xd[7];
out[1] = xd[8];
out[2] = xd[9];
out[3] = ((real_t)(5.0000000000000000e-01)*(((((real_t)(0.0000000000000000e+00)-u[1])*xd[4])-(u[2]*xd[5]))-(u[3]*xd[6])));
out[4] = ((real_t)(5.0000000000000000e-01)*(((u[1]*xd[3])+(u[3]*xd[5]))-(u[2]*xd[6])));
out[5] = ((real_t)(5.0000000000000000e-01)*(((u[2]*xd[3])-(u[3]*xd[4]))+(u[1]*xd[6])));
out[6] = ((real_t)(5.0000000000000000e-01)*(((u[3]*xd[3])+(u[2]*xd[4]))-(u[1]*xd[5])));
out[7] = ((((real_t)(2.0000000000000000e+00)*((xd[3]*xd[5])+(xd[4]*xd[6])))*u[0])-(a[8]*xd[7]));
out[8] = ((((real_t)(2.0000000000000000e+00)*((xd[5]*xd[6])-(xd[3]*xd[4])))*u[0])-(a[13]*xd[8]));
out[9] = (((((real_t)(1.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[4])*xd[4]))-(((real_t)(2.0000000000000000e+00)*xd[5])*xd[5]))*u[0])-(real_t)(9.8065999999999995e+00));
out[10] = u[4];
out[11] = u[5];
out[12] = u[6];
out[13] = u[7];
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 14;
/* Vector of auxiliary variables; number of elements: 22. */
real_t* a = auxVar;

/* Compute intermediate quantities: */
a[0] = xd[10];
a[1] = ((real_t)(5.1134634999999998e-02)*a[0]);
a[2] = xd[11];
a[3] = (a[1]+((real_t)(-5.1484328000000003e-02)*a[2]));
a[4] = xd[12];
a[5] = (a[3]+((real_t)(5.1273007000000002e-02)*a[4]));
a[6] = xd[13];
a[7] = (a[5]+((real_t)(-5.1385595999999999e-02)*a[6]));
a[8] = ((real_t)(3.8261752999999998e-01)+a[7]);
a[9] = (real_t)(5.1134634999999998e-02);
a[10] = (real_t)(-5.1484328000000003e-02);
a[11] = (real_t)(5.1273007000000002e-02);
a[12] = (real_t)(-5.1385595999999999e-02);
a[13] = ((real_t)(-5.1819004000000002e-02)*a[0]);
a[14] = (a[13]+((real_t)(5.1844250000000001e-02)*a[2]));
a[15] = (a[14]+((real_t)(-5.1987289999999999e-02)*a[4]));
a[16] = (a[15]+((real_t)(5.1781394000000001e-02)*a[6]));
a[17] = ((real_t)(3.8267942999999999e-01)+a[16]);
a[18] = (real_t)(-5.1819004000000002e-02);
a[19] = (real_t)(5.1844250000000001e-02);
a[20] = (real_t)(-5.1987289999999999e-02);
a[21] = (real_t)(5.1781394000000001e-02);

/* Compute outputs: */
out[0] = (real_t)(0.0000000000000000e+00);
out[1] = (real_t)(0.0000000000000000e+00);
out[2] = (real_t)(0.0000000000000000e+00);
out[3] = (real_t)(0.0000000000000000e+00);
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = (real_t)(0.0000000000000000e+00);
out[7] = (real_t)(1.0000000000000000e+00);
out[8] = (real_t)(0.0000000000000000e+00);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(0.0000000000000000e+00);
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
out[30] = (real_t)(1.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
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
out[51] = (real_t)(0.0000000000000000e+00);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(1.0000000000000000e+00);
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
out[70] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[1]));
out[71] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[2]));
out[72] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[3]));
out[73] = (real_t)(0.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = (real_t)(0.0000000000000000e+00);
out[77] = (real_t)(0.0000000000000000e+00);
out[78] = (real_t)(0.0000000000000000e+00);
out[79] = (real_t)(0.0000000000000000e+00);
out[80] = (real_t)(0.0000000000000000e+00);
out[81] = ((real_t)(5.0000000000000000e-01)*(((real_t)(0.0000000000000000e+00)-(real_t)(1.0000000000000000e+00))*xd[4]));
out[82] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-xd[5]));
out[83] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-xd[6]));
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(0.0000000000000000e+00);
out[86] = (real_t)(0.0000000000000000e+00);
out[87] = (real_t)(0.0000000000000000e+00);
out[88] = (real_t)(0.0000000000000000e+00);
out[89] = (real_t)(0.0000000000000000e+00);
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = ((real_t)(5.0000000000000000e-01)*u[1]);
out[92] = (real_t)(0.0000000000000000e+00);
out[93] = ((real_t)(5.0000000000000000e-01)*u[3]);
out[94] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[2]));
out[95] = (real_t)(0.0000000000000000e+00);
out[96] = (real_t)(0.0000000000000000e+00);
out[97] = (real_t)(0.0000000000000000e+00);
out[98] = (real_t)(0.0000000000000000e+00);
out[99] = (real_t)(0.0000000000000000e+00);
out[100] = (real_t)(0.0000000000000000e+00);
out[101] = (real_t)(0.0000000000000000e+00);
out[102] = (real_t)(0.0000000000000000e+00);
out[103] = ((real_t)(5.0000000000000000e-01)*xd[3]);
out[104] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-xd[6]));
out[105] = ((real_t)(5.0000000000000000e-01)*xd[5]);
out[106] = (real_t)(0.0000000000000000e+00);
out[107] = (real_t)(0.0000000000000000e+00);
out[108] = (real_t)(0.0000000000000000e+00);
out[109] = (real_t)(0.0000000000000000e+00);
out[110] = (real_t)(0.0000000000000000e+00);
out[111] = (real_t)(0.0000000000000000e+00);
out[112] = (real_t)(0.0000000000000000e+00);
out[113] = ((real_t)(5.0000000000000000e-01)*u[2]);
out[114] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[3]));
out[115] = (real_t)(0.0000000000000000e+00);
out[116] = ((real_t)(5.0000000000000000e-01)*u[1]);
out[117] = (real_t)(0.0000000000000000e+00);
out[118] = (real_t)(0.0000000000000000e+00);
out[119] = (real_t)(0.0000000000000000e+00);
out[120] = (real_t)(0.0000000000000000e+00);
out[121] = (real_t)(0.0000000000000000e+00);
out[122] = (real_t)(0.0000000000000000e+00);
out[123] = (real_t)(0.0000000000000000e+00);
out[124] = (real_t)(0.0000000000000000e+00);
out[125] = ((real_t)(5.0000000000000000e-01)*xd[6]);
out[126] = ((real_t)(5.0000000000000000e-01)*xd[3]);
out[127] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-xd[4]));
out[128] = (real_t)(0.0000000000000000e+00);
out[129] = (real_t)(0.0000000000000000e+00);
out[130] = (real_t)(0.0000000000000000e+00);
out[131] = (real_t)(0.0000000000000000e+00);
out[132] = (real_t)(0.0000000000000000e+00);
out[133] = (real_t)(0.0000000000000000e+00);
out[134] = (real_t)(0.0000000000000000e+00);
out[135] = ((real_t)(5.0000000000000000e-01)*u[3]);
out[136] = ((real_t)(5.0000000000000000e-01)*u[2]);
out[137] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-u[1]));
out[138] = (real_t)(0.0000000000000000e+00);
out[139] = (real_t)(0.0000000000000000e+00);
out[140] = (real_t)(0.0000000000000000e+00);
out[141] = (real_t)(0.0000000000000000e+00);
out[142] = (real_t)(0.0000000000000000e+00);
out[143] = (real_t)(0.0000000000000000e+00);
out[144] = (real_t)(0.0000000000000000e+00);
out[145] = (real_t)(0.0000000000000000e+00);
out[146] = (real_t)(0.0000000000000000e+00);
out[147] = ((real_t)(5.0000000000000000e-01)*((real_t)(0.0000000000000000e+00)-xd[5]));
out[148] = ((real_t)(5.0000000000000000e-01)*xd[4]);
out[149] = ((real_t)(5.0000000000000000e-01)*xd[3]);
out[150] = (real_t)(0.0000000000000000e+00);
out[151] = (real_t)(0.0000000000000000e+00);
out[152] = (real_t)(0.0000000000000000e+00);
out[153] = (real_t)(0.0000000000000000e+00);
out[154] = (real_t)(0.0000000000000000e+00);
out[155] = (real_t)(0.0000000000000000e+00);
out[156] = (real_t)(0.0000000000000000e+00);
out[157] = (((real_t)(2.0000000000000000e+00)*xd[5])*u[0]);
out[158] = (((real_t)(2.0000000000000000e+00)*xd[6])*u[0]);
out[159] = (((real_t)(2.0000000000000000e+00)*xd[3])*u[0]);
out[160] = (((real_t)(2.0000000000000000e+00)*xd[4])*u[0]);
out[161] = ((real_t)(0.0000000000000000e+00)-a[8]);
out[162] = (real_t)(0.0000000000000000e+00);
out[163] = (real_t)(0.0000000000000000e+00);
out[164] = ((real_t)(0.0000000000000000e+00)-(a[9]*xd[7]));
out[165] = ((real_t)(0.0000000000000000e+00)-(a[10]*xd[7]));
out[166] = ((real_t)(0.0000000000000000e+00)-(a[11]*xd[7]));
out[167] = ((real_t)(0.0000000000000000e+00)-(a[12]*xd[7]));
out[168] = ((real_t)(2.0000000000000000e+00)*((xd[3]*xd[5])+(xd[4]*xd[6])));
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
out[179] = (((real_t)(2.0000000000000000e+00)*((real_t)(0.0000000000000000e+00)-xd[4]))*u[0]);
out[180] = (((real_t)(2.0000000000000000e+00)*((real_t)(0.0000000000000000e+00)-xd[3]))*u[0]);
out[181] = (((real_t)(2.0000000000000000e+00)*xd[6])*u[0]);
out[182] = (((real_t)(2.0000000000000000e+00)*xd[5])*u[0]);
out[183] = (real_t)(0.0000000000000000e+00);
out[184] = ((real_t)(0.0000000000000000e+00)-a[17]);
out[185] = (real_t)(0.0000000000000000e+00);
out[186] = ((real_t)(0.0000000000000000e+00)-(a[18]*xd[8]));
out[187] = ((real_t)(0.0000000000000000e+00)-(a[19]*xd[8]));
out[188] = ((real_t)(0.0000000000000000e+00)-(a[20]*xd[8]));
out[189] = ((real_t)(0.0000000000000000e+00)-(a[21]*xd[8]));
out[190] = ((real_t)(2.0000000000000000e+00)*((xd[5]*xd[6])-(xd[3]*xd[4])));
out[191] = (real_t)(0.0000000000000000e+00);
out[192] = (real_t)(0.0000000000000000e+00);
out[193] = (real_t)(0.0000000000000000e+00);
out[194] = (real_t)(0.0000000000000000e+00);
out[195] = (real_t)(0.0000000000000000e+00);
out[196] = (real_t)(0.0000000000000000e+00);
out[197] = (real_t)(0.0000000000000000e+00);
out[198] = (real_t)(0.0000000000000000e+00);
out[199] = (real_t)(0.0000000000000000e+00);
out[200] = (real_t)(0.0000000000000000e+00);
out[201] = (real_t)(0.0000000000000000e+00);
out[202] = (((real_t)(0.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[4])+((real_t)(2.0000000000000000e+00)*xd[4])))*u[0]);
out[203] = (((real_t)(0.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[5])+((real_t)(2.0000000000000000e+00)*xd[5])))*u[0]);
out[204] = (real_t)(0.0000000000000000e+00);
out[205] = (real_t)(0.0000000000000000e+00);
out[206] = (real_t)(0.0000000000000000e+00);
out[207] = (real_t)(0.0000000000000000e+00);
out[208] = (real_t)(0.0000000000000000e+00);
out[209] = (real_t)(0.0000000000000000e+00);
out[210] = (real_t)(0.0000000000000000e+00);
out[211] = (real_t)(0.0000000000000000e+00);
out[212] = (((real_t)(1.0000000000000000e+00)-(((real_t)(2.0000000000000000e+00)*xd[4])*xd[4]))-(((real_t)(2.0000000000000000e+00)*xd[5])*xd[5]));
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
out[238] = (real_t)(1.0000000000000000e+00);
out[239] = (real_t)(0.0000000000000000e+00);
out[240] = (real_t)(0.0000000000000000e+00);
out[241] = (real_t)(0.0000000000000000e+00);
out[242] = (real_t)(0.0000000000000000e+00);
out[243] = (real_t)(0.0000000000000000e+00);
out[244] = (real_t)(0.0000000000000000e+00);
out[245] = (real_t)(0.0000000000000000e+00);
out[246] = (real_t)(0.0000000000000000e+00);
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
out[261] = (real_t)(1.0000000000000000e+00);
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
out[284] = (real_t)(1.0000000000000000e+00);
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
out[307] = (real_t)(1.0000000000000000e+00);
}



void acado_solve_dim28_triangular( real_t* const A, real_t* const b )
{

b[27] = b[27]/A[783];
b[26] -= + A[755]*b[27];
b[26] = b[26]/A[754];
b[25] -= + A[727]*b[27];
b[25] -= + A[726]*b[26];
b[25] = b[25]/A[725];
b[24] -= + A[699]*b[27];
b[24] -= + A[698]*b[26];
b[24] -= + A[697]*b[25];
b[24] = b[24]/A[696];
b[23] -= + A[671]*b[27];
b[23] -= + A[670]*b[26];
b[23] -= + A[669]*b[25];
b[23] -= + A[668]*b[24];
b[23] = b[23]/A[667];
b[22] -= + A[643]*b[27];
b[22] -= + A[642]*b[26];
b[22] -= + A[641]*b[25];
b[22] -= + A[640]*b[24];
b[22] -= + A[639]*b[23];
b[22] = b[22]/A[638];
b[21] -= + A[615]*b[27];
b[21] -= + A[614]*b[26];
b[21] -= + A[613]*b[25];
b[21] -= + A[612]*b[24];
b[21] -= + A[611]*b[23];
b[21] -= + A[610]*b[22];
b[21] = b[21]/A[609];
b[20] -= + A[587]*b[27];
b[20] -= + A[586]*b[26];
b[20] -= + A[585]*b[25];
b[20] -= + A[584]*b[24];
b[20] -= + A[583]*b[23];
b[20] -= + A[582]*b[22];
b[20] -= + A[581]*b[21];
b[20] = b[20]/A[580];
b[19] -= + A[559]*b[27];
b[19] -= + A[558]*b[26];
b[19] -= + A[557]*b[25];
b[19] -= + A[556]*b[24];
b[19] -= + A[555]*b[23];
b[19] -= + A[554]*b[22];
b[19] -= + A[553]*b[21];
b[19] -= + A[552]*b[20];
b[19] = b[19]/A[551];
b[18] -= + A[531]*b[27];
b[18] -= + A[530]*b[26];
b[18] -= + A[529]*b[25];
b[18] -= + A[528]*b[24];
b[18] -= + A[527]*b[23];
b[18] -= + A[526]*b[22];
b[18] -= + A[525]*b[21];
b[18] -= + A[524]*b[20];
b[18] -= + A[523]*b[19];
b[18] = b[18]/A[522];
b[17] -= + A[503]*b[27];
b[17] -= + A[502]*b[26];
b[17] -= + A[501]*b[25];
b[17] -= + A[500]*b[24];
b[17] -= + A[499]*b[23];
b[17] -= + A[498]*b[22];
b[17] -= + A[497]*b[21];
b[17] -= + A[496]*b[20];
b[17] -= + A[495]*b[19];
b[17] -= + A[494]*b[18];
b[17] = b[17]/A[493];
b[16] -= + A[475]*b[27];
b[16] -= + A[474]*b[26];
b[16] -= + A[473]*b[25];
b[16] -= + A[472]*b[24];
b[16] -= + A[471]*b[23];
b[16] -= + A[470]*b[22];
b[16] -= + A[469]*b[21];
b[16] -= + A[468]*b[20];
b[16] -= + A[467]*b[19];
b[16] -= + A[466]*b[18];
b[16] -= + A[465]*b[17];
b[16] = b[16]/A[464];
b[15] -= + A[447]*b[27];
b[15] -= + A[446]*b[26];
b[15] -= + A[445]*b[25];
b[15] -= + A[444]*b[24];
b[15] -= + A[443]*b[23];
b[15] -= + A[442]*b[22];
b[15] -= + A[441]*b[21];
b[15] -= + A[440]*b[20];
b[15] -= + A[439]*b[19];
b[15] -= + A[438]*b[18];
b[15] -= + A[437]*b[17];
b[15] -= + A[436]*b[16];
b[15] = b[15]/A[435];
b[14] -= + A[419]*b[27];
b[14] -= + A[418]*b[26];
b[14] -= + A[417]*b[25];
b[14] -= + A[416]*b[24];
b[14] -= + A[415]*b[23];
b[14] -= + A[414]*b[22];
b[14] -= + A[413]*b[21];
b[14] -= + A[412]*b[20];
b[14] -= + A[411]*b[19];
b[14] -= + A[410]*b[18];
b[14] -= + A[409]*b[17];
b[14] -= + A[408]*b[16];
b[14] -= + A[407]*b[15];
b[14] = b[14]/A[406];
b[13] -= + A[391]*b[27];
b[13] -= + A[390]*b[26];
b[13] -= + A[389]*b[25];
b[13] -= + A[388]*b[24];
b[13] -= + A[387]*b[23];
b[13] -= + A[386]*b[22];
b[13] -= + A[385]*b[21];
b[13] -= + A[384]*b[20];
b[13] -= + A[383]*b[19];
b[13] -= + A[382]*b[18];
b[13] -= + A[381]*b[17];
b[13] -= + A[380]*b[16];
b[13] -= + A[379]*b[15];
b[13] -= + A[378]*b[14];
b[13] = b[13]/A[377];
b[12] -= + A[363]*b[27];
b[12] -= + A[362]*b[26];
b[12] -= + A[361]*b[25];
b[12] -= + A[360]*b[24];
b[12] -= + A[359]*b[23];
b[12] -= + A[358]*b[22];
b[12] -= + A[357]*b[21];
b[12] -= + A[356]*b[20];
b[12] -= + A[355]*b[19];
b[12] -= + A[354]*b[18];
b[12] -= + A[353]*b[17];
b[12] -= + A[352]*b[16];
b[12] -= + A[351]*b[15];
b[12] -= + A[350]*b[14];
b[12] -= + A[349]*b[13];
b[12] = b[12]/A[348];
b[11] -= + A[335]*b[27];
b[11] -= + A[334]*b[26];
b[11] -= + A[333]*b[25];
b[11] -= + A[332]*b[24];
b[11] -= + A[331]*b[23];
b[11] -= + A[330]*b[22];
b[11] -= + A[329]*b[21];
b[11] -= + A[328]*b[20];
b[11] -= + A[327]*b[19];
b[11] -= + A[326]*b[18];
b[11] -= + A[325]*b[17];
b[11] -= + A[324]*b[16];
b[11] -= + A[323]*b[15];
b[11] -= + A[322]*b[14];
b[11] -= + A[321]*b[13];
b[11] -= + A[320]*b[12];
b[11] = b[11]/A[319];
b[10] -= + A[307]*b[27];
b[10] -= + A[306]*b[26];
b[10] -= + A[305]*b[25];
b[10] -= + A[304]*b[24];
b[10] -= + A[303]*b[23];
b[10] -= + A[302]*b[22];
b[10] -= + A[301]*b[21];
b[10] -= + A[300]*b[20];
b[10] -= + A[299]*b[19];
b[10] -= + A[298]*b[18];
b[10] -= + A[297]*b[17];
b[10] -= + A[296]*b[16];
b[10] -= + A[295]*b[15];
b[10] -= + A[294]*b[14];
b[10] -= + A[293]*b[13];
b[10] -= + A[292]*b[12];
b[10] -= + A[291]*b[11];
b[10] = b[10]/A[290];
b[9] -= + A[279]*b[27];
b[9] -= + A[278]*b[26];
b[9] -= + A[277]*b[25];
b[9] -= + A[276]*b[24];
b[9] -= + A[275]*b[23];
b[9] -= + A[274]*b[22];
b[9] -= + A[273]*b[21];
b[9] -= + A[272]*b[20];
b[9] -= + A[271]*b[19];
b[9] -= + A[270]*b[18];
b[9] -= + A[269]*b[17];
b[9] -= + A[268]*b[16];
b[9] -= + A[267]*b[15];
b[9] -= + A[266]*b[14];
b[9] -= + A[265]*b[13];
b[9] -= + A[264]*b[12];
b[9] -= + A[263]*b[11];
b[9] -= + A[262]*b[10];
b[9] = b[9]/A[261];
b[8] -= + A[251]*b[27];
b[8] -= + A[250]*b[26];
b[8] -= + A[249]*b[25];
b[8] -= + A[248]*b[24];
b[8] -= + A[247]*b[23];
b[8] -= + A[246]*b[22];
b[8] -= + A[245]*b[21];
b[8] -= + A[244]*b[20];
b[8] -= + A[243]*b[19];
b[8] -= + A[242]*b[18];
b[8] -= + A[241]*b[17];
b[8] -= + A[240]*b[16];
b[8] -= + A[239]*b[15];
b[8] -= + A[238]*b[14];
b[8] -= + A[237]*b[13];
b[8] -= + A[236]*b[12];
b[8] -= + A[235]*b[11];
b[8] -= + A[234]*b[10];
b[8] -= + A[233]*b[9];
b[8] = b[8]/A[232];
b[7] -= + A[223]*b[27];
b[7] -= + A[222]*b[26];
b[7] -= + A[221]*b[25];
b[7] -= + A[220]*b[24];
b[7] -= + A[219]*b[23];
b[7] -= + A[218]*b[22];
b[7] -= + A[217]*b[21];
b[7] -= + A[216]*b[20];
b[7] -= + A[215]*b[19];
b[7] -= + A[214]*b[18];
b[7] -= + A[213]*b[17];
b[7] -= + A[212]*b[16];
b[7] -= + A[211]*b[15];
b[7] -= + A[210]*b[14];
b[7] -= + A[209]*b[13];
b[7] -= + A[208]*b[12];
b[7] -= + A[207]*b[11];
b[7] -= + A[206]*b[10];
b[7] -= + A[205]*b[9];
b[7] -= + A[204]*b[8];
b[7] = b[7]/A[203];
b[6] -= + A[195]*b[27];
b[6] -= + A[194]*b[26];
b[6] -= + A[193]*b[25];
b[6] -= + A[192]*b[24];
b[6] -= + A[191]*b[23];
b[6] -= + A[190]*b[22];
b[6] -= + A[189]*b[21];
b[6] -= + A[188]*b[20];
b[6] -= + A[187]*b[19];
b[6] -= + A[186]*b[18];
b[6] -= + A[185]*b[17];
b[6] -= + A[184]*b[16];
b[6] -= + A[183]*b[15];
b[6] -= + A[182]*b[14];
b[6] -= + A[181]*b[13];
b[6] -= + A[180]*b[12];
b[6] -= + A[179]*b[11];
b[6] -= + A[178]*b[10];
b[6] -= + A[177]*b[9];
b[6] -= + A[176]*b[8];
b[6] -= + A[175]*b[7];
b[6] = b[6]/A[174];
b[5] -= + A[167]*b[27];
b[5] -= + A[166]*b[26];
b[5] -= + A[165]*b[25];
b[5] -= + A[164]*b[24];
b[5] -= + A[163]*b[23];
b[5] -= + A[162]*b[22];
b[5] -= + A[161]*b[21];
b[5] -= + A[160]*b[20];
b[5] -= + A[159]*b[19];
b[5] -= + A[158]*b[18];
b[5] -= + A[157]*b[17];
b[5] -= + A[156]*b[16];
b[5] -= + A[155]*b[15];
b[5] -= + A[154]*b[14];
b[5] -= + A[153]*b[13];
b[5] -= + A[152]*b[12];
b[5] -= + A[151]*b[11];
b[5] -= + A[150]*b[10];
b[5] -= + A[149]*b[9];
b[5] -= + A[148]*b[8];
b[5] -= + A[147]*b[7];
b[5] -= + A[146]*b[6];
b[5] = b[5]/A[145];
b[4] -= + A[139]*b[27];
b[4] -= + A[138]*b[26];
b[4] -= + A[137]*b[25];
b[4] -= + A[136]*b[24];
b[4] -= + A[135]*b[23];
b[4] -= + A[134]*b[22];
b[4] -= + A[133]*b[21];
b[4] -= + A[132]*b[20];
b[4] -= + A[131]*b[19];
b[4] -= + A[130]*b[18];
b[4] -= + A[129]*b[17];
b[4] -= + A[128]*b[16];
b[4] -= + A[127]*b[15];
b[4] -= + A[126]*b[14];
b[4] -= + A[125]*b[13];
b[4] -= + A[124]*b[12];
b[4] -= + A[123]*b[11];
b[4] -= + A[122]*b[10];
b[4] -= + A[121]*b[9];
b[4] -= + A[120]*b[8];
b[4] -= + A[119]*b[7];
b[4] -= + A[118]*b[6];
b[4] -= + A[117]*b[5];
b[4] = b[4]/A[116];
b[3] -= + A[111]*b[27];
b[3] -= + A[110]*b[26];
b[3] -= + A[109]*b[25];
b[3] -= + A[108]*b[24];
b[3] -= + A[107]*b[23];
b[3] -= + A[106]*b[22];
b[3] -= + A[105]*b[21];
b[3] -= + A[104]*b[20];
b[3] -= + A[103]*b[19];
b[3] -= + A[102]*b[18];
b[3] -= + A[101]*b[17];
b[3] -= + A[100]*b[16];
b[3] -= + A[99]*b[15];
b[3] -= + A[98]*b[14];
b[3] -= + A[97]*b[13];
b[3] -= + A[96]*b[12];
b[3] -= + A[95]*b[11];
b[3] -= + A[94]*b[10];
b[3] -= + A[93]*b[9];
b[3] -= + A[92]*b[8];
b[3] -= + A[91]*b[7];
b[3] -= + A[90]*b[6];
b[3] -= + A[89]*b[5];
b[3] -= + A[88]*b[4];
b[3] = b[3]/A[87];
b[2] -= + A[83]*b[27];
b[2] -= + A[82]*b[26];
b[2] -= + A[81]*b[25];
b[2] -= + A[80]*b[24];
b[2] -= + A[79]*b[23];
b[2] -= + A[78]*b[22];
b[2] -= + A[77]*b[21];
b[2] -= + A[76]*b[20];
b[2] -= + A[75]*b[19];
b[2] -= + A[74]*b[18];
b[2] -= + A[73]*b[17];
b[2] -= + A[72]*b[16];
b[2] -= + A[71]*b[15];
b[2] -= + A[70]*b[14];
b[2] -= + A[69]*b[13];
b[2] -= + A[68]*b[12];
b[2] -= + A[67]*b[11];
b[2] -= + A[66]*b[10];
b[2] -= + A[65]*b[9];
b[2] -= + A[64]*b[8];
b[2] -= + A[63]*b[7];
b[2] -= + A[62]*b[6];
b[2] -= + A[61]*b[5];
b[2] -= + A[60]*b[4];
b[2] -= + A[59]*b[3];
b[2] = b[2]/A[58];
b[1] -= + A[55]*b[27];
b[1] -= + A[54]*b[26];
b[1] -= + A[53]*b[25];
b[1] -= + A[52]*b[24];
b[1] -= + A[51]*b[23];
b[1] -= + A[50]*b[22];
b[1] -= + A[49]*b[21];
b[1] -= + A[48]*b[20];
b[1] -= + A[47]*b[19];
b[1] -= + A[46]*b[18];
b[1] -= + A[45]*b[17];
b[1] -= + A[44]*b[16];
b[1] -= + A[43]*b[15];
b[1] -= + A[42]*b[14];
b[1] -= + A[41]*b[13];
b[1] -= + A[40]*b[12];
b[1] -= + A[39]*b[11];
b[1] -= + A[38]*b[10];
b[1] -= + A[37]*b[9];
b[1] -= + A[36]*b[8];
b[1] -= + A[35]*b[7];
b[1] -= + A[34]*b[6];
b[1] -= + A[33]*b[5];
b[1] -= + A[32]*b[4];
b[1] -= + A[31]*b[3];
b[1] -= + A[30]*b[2];
b[1] = b[1]/A[29];
b[0] -= + A[27]*b[27];
b[0] -= + A[26]*b[26];
b[0] -= + A[25]*b[25];
b[0] -= + A[24]*b[24];
b[0] -= + A[23]*b[23];
b[0] -= + A[22]*b[22];
b[0] -= + A[21]*b[21];
b[0] -= + A[20]*b[20];
b[0] -= + A[19]*b[19];
b[0] -= + A[18]*b[18];
b[0] -= + A[17]*b[17];
b[0] -= + A[16]*b[16];
b[0] -= + A[15]*b[15];
b[0] -= + A[14]*b[14];
b[0] -= + A[13]*b[13];
b[0] -= + A[12]*b[12];
b[0] -= + A[11]*b[11];
b[0] -= + A[10]*b[10];
b[0] -= + A[9]*b[9];
b[0] -= + A[8]*b[8];
b[0] -= + A[7]*b[7];
b[0] -= + A[6]*b[6];
b[0] -= + A[5]*b[5];
b[0] -= + A[4]*b[4];
b[0] -= + A[3]*b[3];
b[0] -= + A[2]*b[2];
b[0] -= + A[1]*b[1];
b[0] = b[0]/A[0];
}

real_t acado_solve_dim28_system( real_t* const A, real_t* const b, int* const rk_perm )
{
real_t det;

int i;
int j;
int k;

int indexMax;

int intSwap;

real_t valueMax;

real_t temp;

for (i = 0; i < 28; ++i)
{
rk_perm[i] = i;
}
det = 1.0000000000000000e+00;
for( i=0; i < (27); i++ ) {
	indexMax = i;
	valueMax = fabs(A[i*28+i]);
	for( j=(i+1); j < 28; j++ ) {
		temp = fabs(A[j*28+i]);
		if( temp > valueMax ) {
			indexMax = j;
			valueMax = temp;
		}
	}
	if( indexMax > i ) {
for (k = 0; k < 28; ++k)
{
	rk_dim28_swap = A[i*28+k];
	A[i*28+k] = A[indexMax*28+k];
	A[indexMax*28+k] = rk_dim28_swap;
}
	rk_dim28_swap = b[i];
	b[i] = b[indexMax];
	b[indexMax] = rk_dim28_swap;
	intSwap = rk_perm[i];
	rk_perm[i] = rk_perm[indexMax];
	rk_perm[indexMax] = intSwap;
	}
	det *= A[i*28+i];
	for( j=i+1; j < 28; j++ ) {
		A[j*28+i] = -A[j*28+i]/A[i*28+i];
		for( k=i+1; k < 28; k++ ) {
			A[j*28+k] += A[j*28+i] * A[i*28+k];
		}
		b[j] += A[j*28+i] * b[i];
	}
}
det *= A[783];
det = fabs(det);
acado_solve_dim28_triangular( A, b );
return det;
}

void acado_solve_dim28_system_reuse( real_t* const A, real_t* const b, int* const rk_perm )
{

rk_dim28_bPerm[0] = b[rk_perm[0]];
rk_dim28_bPerm[1] = b[rk_perm[1]];
rk_dim28_bPerm[2] = b[rk_perm[2]];
rk_dim28_bPerm[3] = b[rk_perm[3]];
rk_dim28_bPerm[4] = b[rk_perm[4]];
rk_dim28_bPerm[5] = b[rk_perm[5]];
rk_dim28_bPerm[6] = b[rk_perm[6]];
rk_dim28_bPerm[7] = b[rk_perm[7]];
rk_dim28_bPerm[8] = b[rk_perm[8]];
rk_dim28_bPerm[9] = b[rk_perm[9]];
rk_dim28_bPerm[10] = b[rk_perm[10]];
rk_dim28_bPerm[11] = b[rk_perm[11]];
rk_dim28_bPerm[12] = b[rk_perm[12]];
rk_dim28_bPerm[13] = b[rk_perm[13]];
rk_dim28_bPerm[14] = b[rk_perm[14]];
rk_dim28_bPerm[15] = b[rk_perm[15]];
rk_dim28_bPerm[16] = b[rk_perm[16]];
rk_dim28_bPerm[17] = b[rk_perm[17]];
rk_dim28_bPerm[18] = b[rk_perm[18]];
rk_dim28_bPerm[19] = b[rk_perm[19]];
rk_dim28_bPerm[20] = b[rk_perm[20]];
rk_dim28_bPerm[21] = b[rk_perm[21]];
rk_dim28_bPerm[22] = b[rk_perm[22]];
rk_dim28_bPerm[23] = b[rk_perm[23]];
rk_dim28_bPerm[24] = b[rk_perm[24]];
rk_dim28_bPerm[25] = b[rk_perm[25]];
rk_dim28_bPerm[26] = b[rk_perm[26]];
rk_dim28_bPerm[27] = b[rk_perm[27]];
rk_dim28_bPerm[1] += A[28]*rk_dim28_bPerm[0];

rk_dim28_bPerm[2] += A[56]*rk_dim28_bPerm[0];
rk_dim28_bPerm[2] += A[57]*rk_dim28_bPerm[1];

rk_dim28_bPerm[3] += A[84]*rk_dim28_bPerm[0];
rk_dim28_bPerm[3] += A[85]*rk_dim28_bPerm[1];
rk_dim28_bPerm[3] += A[86]*rk_dim28_bPerm[2];

rk_dim28_bPerm[4] += A[112]*rk_dim28_bPerm[0];
rk_dim28_bPerm[4] += A[113]*rk_dim28_bPerm[1];
rk_dim28_bPerm[4] += A[114]*rk_dim28_bPerm[2];
rk_dim28_bPerm[4] += A[115]*rk_dim28_bPerm[3];

rk_dim28_bPerm[5] += A[140]*rk_dim28_bPerm[0];
rk_dim28_bPerm[5] += A[141]*rk_dim28_bPerm[1];
rk_dim28_bPerm[5] += A[142]*rk_dim28_bPerm[2];
rk_dim28_bPerm[5] += A[143]*rk_dim28_bPerm[3];
rk_dim28_bPerm[5] += A[144]*rk_dim28_bPerm[4];

rk_dim28_bPerm[6] += A[168]*rk_dim28_bPerm[0];
rk_dim28_bPerm[6] += A[169]*rk_dim28_bPerm[1];
rk_dim28_bPerm[6] += A[170]*rk_dim28_bPerm[2];
rk_dim28_bPerm[6] += A[171]*rk_dim28_bPerm[3];
rk_dim28_bPerm[6] += A[172]*rk_dim28_bPerm[4];
rk_dim28_bPerm[6] += A[173]*rk_dim28_bPerm[5];

rk_dim28_bPerm[7] += A[196]*rk_dim28_bPerm[0];
rk_dim28_bPerm[7] += A[197]*rk_dim28_bPerm[1];
rk_dim28_bPerm[7] += A[198]*rk_dim28_bPerm[2];
rk_dim28_bPerm[7] += A[199]*rk_dim28_bPerm[3];
rk_dim28_bPerm[7] += A[200]*rk_dim28_bPerm[4];
rk_dim28_bPerm[7] += A[201]*rk_dim28_bPerm[5];
rk_dim28_bPerm[7] += A[202]*rk_dim28_bPerm[6];

rk_dim28_bPerm[8] += A[224]*rk_dim28_bPerm[0];
rk_dim28_bPerm[8] += A[225]*rk_dim28_bPerm[1];
rk_dim28_bPerm[8] += A[226]*rk_dim28_bPerm[2];
rk_dim28_bPerm[8] += A[227]*rk_dim28_bPerm[3];
rk_dim28_bPerm[8] += A[228]*rk_dim28_bPerm[4];
rk_dim28_bPerm[8] += A[229]*rk_dim28_bPerm[5];
rk_dim28_bPerm[8] += A[230]*rk_dim28_bPerm[6];
rk_dim28_bPerm[8] += A[231]*rk_dim28_bPerm[7];

rk_dim28_bPerm[9] += A[252]*rk_dim28_bPerm[0];
rk_dim28_bPerm[9] += A[253]*rk_dim28_bPerm[1];
rk_dim28_bPerm[9] += A[254]*rk_dim28_bPerm[2];
rk_dim28_bPerm[9] += A[255]*rk_dim28_bPerm[3];
rk_dim28_bPerm[9] += A[256]*rk_dim28_bPerm[4];
rk_dim28_bPerm[9] += A[257]*rk_dim28_bPerm[5];
rk_dim28_bPerm[9] += A[258]*rk_dim28_bPerm[6];
rk_dim28_bPerm[9] += A[259]*rk_dim28_bPerm[7];
rk_dim28_bPerm[9] += A[260]*rk_dim28_bPerm[8];

rk_dim28_bPerm[10] += A[280]*rk_dim28_bPerm[0];
rk_dim28_bPerm[10] += A[281]*rk_dim28_bPerm[1];
rk_dim28_bPerm[10] += A[282]*rk_dim28_bPerm[2];
rk_dim28_bPerm[10] += A[283]*rk_dim28_bPerm[3];
rk_dim28_bPerm[10] += A[284]*rk_dim28_bPerm[4];
rk_dim28_bPerm[10] += A[285]*rk_dim28_bPerm[5];
rk_dim28_bPerm[10] += A[286]*rk_dim28_bPerm[6];
rk_dim28_bPerm[10] += A[287]*rk_dim28_bPerm[7];
rk_dim28_bPerm[10] += A[288]*rk_dim28_bPerm[8];
rk_dim28_bPerm[10] += A[289]*rk_dim28_bPerm[9];

rk_dim28_bPerm[11] += A[308]*rk_dim28_bPerm[0];
rk_dim28_bPerm[11] += A[309]*rk_dim28_bPerm[1];
rk_dim28_bPerm[11] += A[310]*rk_dim28_bPerm[2];
rk_dim28_bPerm[11] += A[311]*rk_dim28_bPerm[3];
rk_dim28_bPerm[11] += A[312]*rk_dim28_bPerm[4];
rk_dim28_bPerm[11] += A[313]*rk_dim28_bPerm[5];
rk_dim28_bPerm[11] += A[314]*rk_dim28_bPerm[6];
rk_dim28_bPerm[11] += A[315]*rk_dim28_bPerm[7];
rk_dim28_bPerm[11] += A[316]*rk_dim28_bPerm[8];
rk_dim28_bPerm[11] += A[317]*rk_dim28_bPerm[9];
rk_dim28_bPerm[11] += A[318]*rk_dim28_bPerm[10];

rk_dim28_bPerm[12] += A[336]*rk_dim28_bPerm[0];
rk_dim28_bPerm[12] += A[337]*rk_dim28_bPerm[1];
rk_dim28_bPerm[12] += A[338]*rk_dim28_bPerm[2];
rk_dim28_bPerm[12] += A[339]*rk_dim28_bPerm[3];
rk_dim28_bPerm[12] += A[340]*rk_dim28_bPerm[4];
rk_dim28_bPerm[12] += A[341]*rk_dim28_bPerm[5];
rk_dim28_bPerm[12] += A[342]*rk_dim28_bPerm[6];
rk_dim28_bPerm[12] += A[343]*rk_dim28_bPerm[7];
rk_dim28_bPerm[12] += A[344]*rk_dim28_bPerm[8];
rk_dim28_bPerm[12] += A[345]*rk_dim28_bPerm[9];
rk_dim28_bPerm[12] += A[346]*rk_dim28_bPerm[10];
rk_dim28_bPerm[12] += A[347]*rk_dim28_bPerm[11];

rk_dim28_bPerm[13] += A[364]*rk_dim28_bPerm[0];
rk_dim28_bPerm[13] += A[365]*rk_dim28_bPerm[1];
rk_dim28_bPerm[13] += A[366]*rk_dim28_bPerm[2];
rk_dim28_bPerm[13] += A[367]*rk_dim28_bPerm[3];
rk_dim28_bPerm[13] += A[368]*rk_dim28_bPerm[4];
rk_dim28_bPerm[13] += A[369]*rk_dim28_bPerm[5];
rk_dim28_bPerm[13] += A[370]*rk_dim28_bPerm[6];
rk_dim28_bPerm[13] += A[371]*rk_dim28_bPerm[7];
rk_dim28_bPerm[13] += A[372]*rk_dim28_bPerm[8];
rk_dim28_bPerm[13] += A[373]*rk_dim28_bPerm[9];
rk_dim28_bPerm[13] += A[374]*rk_dim28_bPerm[10];
rk_dim28_bPerm[13] += A[375]*rk_dim28_bPerm[11];
rk_dim28_bPerm[13] += A[376]*rk_dim28_bPerm[12];

rk_dim28_bPerm[14] += A[392]*rk_dim28_bPerm[0];
rk_dim28_bPerm[14] += A[393]*rk_dim28_bPerm[1];
rk_dim28_bPerm[14] += A[394]*rk_dim28_bPerm[2];
rk_dim28_bPerm[14] += A[395]*rk_dim28_bPerm[3];
rk_dim28_bPerm[14] += A[396]*rk_dim28_bPerm[4];
rk_dim28_bPerm[14] += A[397]*rk_dim28_bPerm[5];
rk_dim28_bPerm[14] += A[398]*rk_dim28_bPerm[6];
rk_dim28_bPerm[14] += A[399]*rk_dim28_bPerm[7];
rk_dim28_bPerm[14] += A[400]*rk_dim28_bPerm[8];
rk_dim28_bPerm[14] += A[401]*rk_dim28_bPerm[9];
rk_dim28_bPerm[14] += A[402]*rk_dim28_bPerm[10];
rk_dim28_bPerm[14] += A[403]*rk_dim28_bPerm[11];
rk_dim28_bPerm[14] += A[404]*rk_dim28_bPerm[12];
rk_dim28_bPerm[14] += A[405]*rk_dim28_bPerm[13];

rk_dim28_bPerm[15] += A[420]*rk_dim28_bPerm[0];
rk_dim28_bPerm[15] += A[421]*rk_dim28_bPerm[1];
rk_dim28_bPerm[15] += A[422]*rk_dim28_bPerm[2];
rk_dim28_bPerm[15] += A[423]*rk_dim28_bPerm[3];
rk_dim28_bPerm[15] += A[424]*rk_dim28_bPerm[4];
rk_dim28_bPerm[15] += A[425]*rk_dim28_bPerm[5];
rk_dim28_bPerm[15] += A[426]*rk_dim28_bPerm[6];
rk_dim28_bPerm[15] += A[427]*rk_dim28_bPerm[7];
rk_dim28_bPerm[15] += A[428]*rk_dim28_bPerm[8];
rk_dim28_bPerm[15] += A[429]*rk_dim28_bPerm[9];
rk_dim28_bPerm[15] += A[430]*rk_dim28_bPerm[10];
rk_dim28_bPerm[15] += A[431]*rk_dim28_bPerm[11];
rk_dim28_bPerm[15] += A[432]*rk_dim28_bPerm[12];
rk_dim28_bPerm[15] += A[433]*rk_dim28_bPerm[13];
rk_dim28_bPerm[15] += A[434]*rk_dim28_bPerm[14];

rk_dim28_bPerm[16] += A[448]*rk_dim28_bPerm[0];
rk_dim28_bPerm[16] += A[449]*rk_dim28_bPerm[1];
rk_dim28_bPerm[16] += A[450]*rk_dim28_bPerm[2];
rk_dim28_bPerm[16] += A[451]*rk_dim28_bPerm[3];
rk_dim28_bPerm[16] += A[452]*rk_dim28_bPerm[4];
rk_dim28_bPerm[16] += A[453]*rk_dim28_bPerm[5];
rk_dim28_bPerm[16] += A[454]*rk_dim28_bPerm[6];
rk_dim28_bPerm[16] += A[455]*rk_dim28_bPerm[7];
rk_dim28_bPerm[16] += A[456]*rk_dim28_bPerm[8];
rk_dim28_bPerm[16] += A[457]*rk_dim28_bPerm[9];
rk_dim28_bPerm[16] += A[458]*rk_dim28_bPerm[10];
rk_dim28_bPerm[16] += A[459]*rk_dim28_bPerm[11];
rk_dim28_bPerm[16] += A[460]*rk_dim28_bPerm[12];
rk_dim28_bPerm[16] += A[461]*rk_dim28_bPerm[13];
rk_dim28_bPerm[16] += A[462]*rk_dim28_bPerm[14];
rk_dim28_bPerm[16] += A[463]*rk_dim28_bPerm[15];

rk_dim28_bPerm[17] += A[476]*rk_dim28_bPerm[0];
rk_dim28_bPerm[17] += A[477]*rk_dim28_bPerm[1];
rk_dim28_bPerm[17] += A[478]*rk_dim28_bPerm[2];
rk_dim28_bPerm[17] += A[479]*rk_dim28_bPerm[3];
rk_dim28_bPerm[17] += A[480]*rk_dim28_bPerm[4];
rk_dim28_bPerm[17] += A[481]*rk_dim28_bPerm[5];
rk_dim28_bPerm[17] += A[482]*rk_dim28_bPerm[6];
rk_dim28_bPerm[17] += A[483]*rk_dim28_bPerm[7];
rk_dim28_bPerm[17] += A[484]*rk_dim28_bPerm[8];
rk_dim28_bPerm[17] += A[485]*rk_dim28_bPerm[9];
rk_dim28_bPerm[17] += A[486]*rk_dim28_bPerm[10];
rk_dim28_bPerm[17] += A[487]*rk_dim28_bPerm[11];
rk_dim28_bPerm[17] += A[488]*rk_dim28_bPerm[12];
rk_dim28_bPerm[17] += A[489]*rk_dim28_bPerm[13];
rk_dim28_bPerm[17] += A[490]*rk_dim28_bPerm[14];
rk_dim28_bPerm[17] += A[491]*rk_dim28_bPerm[15];
rk_dim28_bPerm[17] += A[492]*rk_dim28_bPerm[16];

rk_dim28_bPerm[18] += A[504]*rk_dim28_bPerm[0];
rk_dim28_bPerm[18] += A[505]*rk_dim28_bPerm[1];
rk_dim28_bPerm[18] += A[506]*rk_dim28_bPerm[2];
rk_dim28_bPerm[18] += A[507]*rk_dim28_bPerm[3];
rk_dim28_bPerm[18] += A[508]*rk_dim28_bPerm[4];
rk_dim28_bPerm[18] += A[509]*rk_dim28_bPerm[5];
rk_dim28_bPerm[18] += A[510]*rk_dim28_bPerm[6];
rk_dim28_bPerm[18] += A[511]*rk_dim28_bPerm[7];
rk_dim28_bPerm[18] += A[512]*rk_dim28_bPerm[8];
rk_dim28_bPerm[18] += A[513]*rk_dim28_bPerm[9];
rk_dim28_bPerm[18] += A[514]*rk_dim28_bPerm[10];
rk_dim28_bPerm[18] += A[515]*rk_dim28_bPerm[11];
rk_dim28_bPerm[18] += A[516]*rk_dim28_bPerm[12];
rk_dim28_bPerm[18] += A[517]*rk_dim28_bPerm[13];
rk_dim28_bPerm[18] += A[518]*rk_dim28_bPerm[14];
rk_dim28_bPerm[18] += A[519]*rk_dim28_bPerm[15];
rk_dim28_bPerm[18] += A[520]*rk_dim28_bPerm[16];
rk_dim28_bPerm[18] += A[521]*rk_dim28_bPerm[17];

rk_dim28_bPerm[19] += A[532]*rk_dim28_bPerm[0];
rk_dim28_bPerm[19] += A[533]*rk_dim28_bPerm[1];
rk_dim28_bPerm[19] += A[534]*rk_dim28_bPerm[2];
rk_dim28_bPerm[19] += A[535]*rk_dim28_bPerm[3];
rk_dim28_bPerm[19] += A[536]*rk_dim28_bPerm[4];
rk_dim28_bPerm[19] += A[537]*rk_dim28_bPerm[5];
rk_dim28_bPerm[19] += A[538]*rk_dim28_bPerm[6];
rk_dim28_bPerm[19] += A[539]*rk_dim28_bPerm[7];
rk_dim28_bPerm[19] += A[540]*rk_dim28_bPerm[8];
rk_dim28_bPerm[19] += A[541]*rk_dim28_bPerm[9];
rk_dim28_bPerm[19] += A[542]*rk_dim28_bPerm[10];
rk_dim28_bPerm[19] += A[543]*rk_dim28_bPerm[11];
rk_dim28_bPerm[19] += A[544]*rk_dim28_bPerm[12];
rk_dim28_bPerm[19] += A[545]*rk_dim28_bPerm[13];
rk_dim28_bPerm[19] += A[546]*rk_dim28_bPerm[14];
rk_dim28_bPerm[19] += A[547]*rk_dim28_bPerm[15];
rk_dim28_bPerm[19] += A[548]*rk_dim28_bPerm[16];
rk_dim28_bPerm[19] += A[549]*rk_dim28_bPerm[17];
rk_dim28_bPerm[19] += A[550]*rk_dim28_bPerm[18];

rk_dim28_bPerm[20] += A[560]*rk_dim28_bPerm[0];
rk_dim28_bPerm[20] += A[561]*rk_dim28_bPerm[1];
rk_dim28_bPerm[20] += A[562]*rk_dim28_bPerm[2];
rk_dim28_bPerm[20] += A[563]*rk_dim28_bPerm[3];
rk_dim28_bPerm[20] += A[564]*rk_dim28_bPerm[4];
rk_dim28_bPerm[20] += A[565]*rk_dim28_bPerm[5];
rk_dim28_bPerm[20] += A[566]*rk_dim28_bPerm[6];
rk_dim28_bPerm[20] += A[567]*rk_dim28_bPerm[7];
rk_dim28_bPerm[20] += A[568]*rk_dim28_bPerm[8];
rk_dim28_bPerm[20] += A[569]*rk_dim28_bPerm[9];
rk_dim28_bPerm[20] += A[570]*rk_dim28_bPerm[10];
rk_dim28_bPerm[20] += A[571]*rk_dim28_bPerm[11];
rk_dim28_bPerm[20] += A[572]*rk_dim28_bPerm[12];
rk_dim28_bPerm[20] += A[573]*rk_dim28_bPerm[13];
rk_dim28_bPerm[20] += A[574]*rk_dim28_bPerm[14];
rk_dim28_bPerm[20] += A[575]*rk_dim28_bPerm[15];
rk_dim28_bPerm[20] += A[576]*rk_dim28_bPerm[16];
rk_dim28_bPerm[20] += A[577]*rk_dim28_bPerm[17];
rk_dim28_bPerm[20] += A[578]*rk_dim28_bPerm[18];
rk_dim28_bPerm[20] += A[579]*rk_dim28_bPerm[19];

rk_dim28_bPerm[21] += A[588]*rk_dim28_bPerm[0];
rk_dim28_bPerm[21] += A[589]*rk_dim28_bPerm[1];
rk_dim28_bPerm[21] += A[590]*rk_dim28_bPerm[2];
rk_dim28_bPerm[21] += A[591]*rk_dim28_bPerm[3];
rk_dim28_bPerm[21] += A[592]*rk_dim28_bPerm[4];
rk_dim28_bPerm[21] += A[593]*rk_dim28_bPerm[5];
rk_dim28_bPerm[21] += A[594]*rk_dim28_bPerm[6];
rk_dim28_bPerm[21] += A[595]*rk_dim28_bPerm[7];
rk_dim28_bPerm[21] += A[596]*rk_dim28_bPerm[8];
rk_dim28_bPerm[21] += A[597]*rk_dim28_bPerm[9];
rk_dim28_bPerm[21] += A[598]*rk_dim28_bPerm[10];
rk_dim28_bPerm[21] += A[599]*rk_dim28_bPerm[11];
rk_dim28_bPerm[21] += A[600]*rk_dim28_bPerm[12];
rk_dim28_bPerm[21] += A[601]*rk_dim28_bPerm[13];
rk_dim28_bPerm[21] += A[602]*rk_dim28_bPerm[14];
rk_dim28_bPerm[21] += A[603]*rk_dim28_bPerm[15];
rk_dim28_bPerm[21] += A[604]*rk_dim28_bPerm[16];
rk_dim28_bPerm[21] += A[605]*rk_dim28_bPerm[17];
rk_dim28_bPerm[21] += A[606]*rk_dim28_bPerm[18];
rk_dim28_bPerm[21] += A[607]*rk_dim28_bPerm[19];
rk_dim28_bPerm[21] += A[608]*rk_dim28_bPerm[20];

rk_dim28_bPerm[22] += A[616]*rk_dim28_bPerm[0];
rk_dim28_bPerm[22] += A[617]*rk_dim28_bPerm[1];
rk_dim28_bPerm[22] += A[618]*rk_dim28_bPerm[2];
rk_dim28_bPerm[22] += A[619]*rk_dim28_bPerm[3];
rk_dim28_bPerm[22] += A[620]*rk_dim28_bPerm[4];
rk_dim28_bPerm[22] += A[621]*rk_dim28_bPerm[5];
rk_dim28_bPerm[22] += A[622]*rk_dim28_bPerm[6];
rk_dim28_bPerm[22] += A[623]*rk_dim28_bPerm[7];
rk_dim28_bPerm[22] += A[624]*rk_dim28_bPerm[8];
rk_dim28_bPerm[22] += A[625]*rk_dim28_bPerm[9];
rk_dim28_bPerm[22] += A[626]*rk_dim28_bPerm[10];
rk_dim28_bPerm[22] += A[627]*rk_dim28_bPerm[11];
rk_dim28_bPerm[22] += A[628]*rk_dim28_bPerm[12];
rk_dim28_bPerm[22] += A[629]*rk_dim28_bPerm[13];
rk_dim28_bPerm[22] += A[630]*rk_dim28_bPerm[14];
rk_dim28_bPerm[22] += A[631]*rk_dim28_bPerm[15];
rk_dim28_bPerm[22] += A[632]*rk_dim28_bPerm[16];
rk_dim28_bPerm[22] += A[633]*rk_dim28_bPerm[17];
rk_dim28_bPerm[22] += A[634]*rk_dim28_bPerm[18];
rk_dim28_bPerm[22] += A[635]*rk_dim28_bPerm[19];
rk_dim28_bPerm[22] += A[636]*rk_dim28_bPerm[20];
rk_dim28_bPerm[22] += A[637]*rk_dim28_bPerm[21];

rk_dim28_bPerm[23] += A[644]*rk_dim28_bPerm[0];
rk_dim28_bPerm[23] += A[645]*rk_dim28_bPerm[1];
rk_dim28_bPerm[23] += A[646]*rk_dim28_bPerm[2];
rk_dim28_bPerm[23] += A[647]*rk_dim28_bPerm[3];
rk_dim28_bPerm[23] += A[648]*rk_dim28_bPerm[4];
rk_dim28_bPerm[23] += A[649]*rk_dim28_bPerm[5];
rk_dim28_bPerm[23] += A[650]*rk_dim28_bPerm[6];
rk_dim28_bPerm[23] += A[651]*rk_dim28_bPerm[7];
rk_dim28_bPerm[23] += A[652]*rk_dim28_bPerm[8];
rk_dim28_bPerm[23] += A[653]*rk_dim28_bPerm[9];
rk_dim28_bPerm[23] += A[654]*rk_dim28_bPerm[10];
rk_dim28_bPerm[23] += A[655]*rk_dim28_bPerm[11];
rk_dim28_bPerm[23] += A[656]*rk_dim28_bPerm[12];
rk_dim28_bPerm[23] += A[657]*rk_dim28_bPerm[13];
rk_dim28_bPerm[23] += A[658]*rk_dim28_bPerm[14];
rk_dim28_bPerm[23] += A[659]*rk_dim28_bPerm[15];
rk_dim28_bPerm[23] += A[660]*rk_dim28_bPerm[16];
rk_dim28_bPerm[23] += A[661]*rk_dim28_bPerm[17];
rk_dim28_bPerm[23] += A[662]*rk_dim28_bPerm[18];
rk_dim28_bPerm[23] += A[663]*rk_dim28_bPerm[19];
rk_dim28_bPerm[23] += A[664]*rk_dim28_bPerm[20];
rk_dim28_bPerm[23] += A[665]*rk_dim28_bPerm[21];
rk_dim28_bPerm[23] += A[666]*rk_dim28_bPerm[22];

rk_dim28_bPerm[24] += A[672]*rk_dim28_bPerm[0];
rk_dim28_bPerm[24] += A[673]*rk_dim28_bPerm[1];
rk_dim28_bPerm[24] += A[674]*rk_dim28_bPerm[2];
rk_dim28_bPerm[24] += A[675]*rk_dim28_bPerm[3];
rk_dim28_bPerm[24] += A[676]*rk_dim28_bPerm[4];
rk_dim28_bPerm[24] += A[677]*rk_dim28_bPerm[5];
rk_dim28_bPerm[24] += A[678]*rk_dim28_bPerm[6];
rk_dim28_bPerm[24] += A[679]*rk_dim28_bPerm[7];
rk_dim28_bPerm[24] += A[680]*rk_dim28_bPerm[8];
rk_dim28_bPerm[24] += A[681]*rk_dim28_bPerm[9];
rk_dim28_bPerm[24] += A[682]*rk_dim28_bPerm[10];
rk_dim28_bPerm[24] += A[683]*rk_dim28_bPerm[11];
rk_dim28_bPerm[24] += A[684]*rk_dim28_bPerm[12];
rk_dim28_bPerm[24] += A[685]*rk_dim28_bPerm[13];
rk_dim28_bPerm[24] += A[686]*rk_dim28_bPerm[14];
rk_dim28_bPerm[24] += A[687]*rk_dim28_bPerm[15];
rk_dim28_bPerm[24] += A[688]*rk_dim28_bPerm[16];
rk_dim28_bPerm[24] += A[689]*rk_dim28_bPerm[17];
rk_dim28_bPerm[24] += A[690]*rk_dim28_bPerm[18];
rk_dim28_bPerm[24] += A[691]*rk_dim28_bPerm[19];
rk_dim28_bPerm[24] += A[692]*rk_dim28_bPerm[20];
rk_dim28_bPerm[24] += A[693]*rk_dim28_bPerm[21];
rk_dim28_bPerm[24] += A[694]*rk_dim28_bPerm[22];
rk_dim28_bPerm[24] += A[695]*rk_dim28_bPerm[23];

rk_dim28_bPerm[25] += A[700]*rk_dim28_bPerm[0];
rk_dim28_bPerm[25] += A[701]*rk_dim28_bPerm[1];
rk_dim28_bPerm[25] += A[702]*rk_dim28_bPerm[2];
rk_dim28_bPerm[25] += A[703]*rk_dim28_bPerm[3];
rk_dim28_bPerm[25] += A[704]*rk_dim28_bPerm[4];
rk_dim28_bPerm[25] += A[705]*rk_dim28_bPerm[5];
rk_dim28_bPerm[25] += A[706]*rk_dim28_bPerm[6];
rk_dim28_bPerm[25] += A[707]*rk_dim28_bPerm[7];
rk_dim28_bPerm[25] += A[708]*rk_dim28_bPerm[8];
rk_dim28_bPerm[25] += A[709]*rk_dim28_bPerm[9];
rk_dim28_bPerm[25] += A[710]*rk_dim28_bPerm[10];
rk_dim28_bPerm[25] += A[711]*rk_dim28_bPerm[11];
rk_dim28_bPerm[25] += A[712]*rk_dim28_bPerm[12];
rk_dim28_bPerm[25] += A[713]*rk_dim28_bPerm[13];
rk_dim28_bPerm[25] += A[714]*rk_dim28_bPerm[14];
rk_dim28_bPerm[25] += A[715]*rk_dim28_bPerm[15];
rk_dim28_bPerm[25] += A[716]*rk_dim28_bPerm[16];
rk_dim28_bPerm[25] += A[717]*rk_dim28_bPerm[17];
rk_dim28_bPerm[25] += A[718]*rk_dim28_bPerm[18];
rk_dim28_bPerm[25] += A[719]*rk_dim28_bPerm[19];
rk_dim28_bPerm[25] += A[720]*rk_dim28_bPerm[20];
rk_dim28_bPerm[25] += A[721]*rk_dim28_bPerm[21];
rk_dim28_bPerm[25] += A[722]*rk_dim28_bPerm[22];
rk_dim28_bPerm[25] += A[723]*rk_dim28_bPerm[23];
rk_dim28_bPerm[25] += A[724]*rk_dim28_bPerm[24];

rk_dim28_bPerm[26] += A[728]*rk_dim28_bPerm[0];
rk_dim28_bPerm[26] += A[729]*rk_dim28_bPerm[1];
rk_dim28_bPerm[26] += A[730]*rk_dim28_bPerm[2];
rk_dim28_bPerm[26] += A[731]*rk_dim28_bPerm[3];
rk_dim28_bPerm[26] += A[732]*rk_dim28_bPerm[4];
rk_dim28_bPerm[26] += A[733]*rk_dim28_bPerm[5];
rk_dim28_bPerm[26] += A[734]*rk_dim28_bPerm[6];
rk_dim28_bPerm[26] += A[735]*rk_dim28_bPerm[7];
rk_dim28_bPerm[26] += A[736]*rk_dim28_bPerm[8];
rk_dim28_bPerm[26] += A[737]*rk_dim28_bPerm[9];
rk_dim28_bPerm[26] += A[738]*rk_dim28_bPerm[10];
rk_dim28_bPerm[26] += A[739]*rk_dim28_bPerm[11];
rk_dim28_bPerm[26] += A[740]*rk_dim28_bPerm[12];
rk_dim28_bPerm[26] += A[741]*rk_dim28_bPerm[13];
rk_dim28_bPerm[26] += A[742]*rk_dim28_bPerm[14];
rk_dim28_bPerm[26] += A[743]*rk_dim28_bPerm[15];
rk_dim28_bPerm[26] += A[744]*rk_dim28_bPerm[16];
rk_dim28_bPerm[26] += A[745]*rk_dim28_bPerm[17];
rk_dim28_bPerm[26] += A[746]*rk_dim28_bPerm[18];
rk_dim28_bPerm[26] += A[747]*rk_dim28_bPerm[19];
rk_dim28_bPerm[26] += A[748]*rk_dim28_bPerm[20];
rk_dim28_bPerm[26] += A[749]*rk_dim28_bPerm[21];
rk_dim28_bPerm[26] += A[750]*rk_dim28_bPerm[22];
rk_dim28_bPerm[26] += A[751]*rk_dim28_bPerm[23];
rk_dim28_bPerm[26] += A[752]*rk_dim28_bPerm[24];
rk_dim28_bPerm[26] += A[753]*rk_dim28_bPerm[25];

rk_dim28_bPerm[27] += A[756]*rk_dim28_bPerm[0];
rk_dim28_bPerm[27] += A[757]*rk_dim28_bPerm[1];
rk_dim28_bPerm[27] += A[758]*rk_dim28_bPerm[2];
rk_dim28_bPerm[27] += A[759]*rk_dim28_bPerm[3];
rk_dim28_bPerm[27] += A[760]*rk_dim28_bPerm[4];
rk_dim28_bPerm[27] += A[761]*rk_dim28_bPerm[5];
rk_dim28_bPerm[27] += A[762]*rk_dim28_bPerm[6];
rk_dim28_bPerm[27] += A[763]*rk_dim28_bPerm[7];
rk_dim28_bPerm[27] += A[764]*rk_dim28_bPerm[8];
rk_dim28_bPerm[27] += A[765]*rk_dim28_bPerm[9];
rk_dim28_bPerm[27] += A[766]*rk_dim28_bPerm[10];
rk_dim28_bPerm[27] += A[767]*rk_dim28_bPerm[11];
rk_dim28_bPerm[27] += A[768]*rk_dim28_bPerm[12];
rk_dim28_bPerm[27] += A[769]*rk_dim28_bPerm[13];
rk_dim28_bPerm[27] += A[770]*rk_dim28_bPerm[14];
rk_dim28_bPerm[27] += A[771]*rk_dim28_bPerm[15];
rk_dim28_bPerm[27] += A[772]*rk_dim28_bPerm[16];
rk_dim28_bPerm[27] += A[773]*rk_dim28_bPerm[17];
rk_dim28_bPerm[27] += A[774]*rk_dim28_bPerm[18];
rk_dim28_bPerm[27] += A[775]*rk_dim28_bPerm[19];
rk_dim28_bPerm[27] += A[776]*rk_dim28_bPerm[20];
rk_dim28_bPerm[27] += A[777]*rk_dim28_bPerm[21];
rk_dim28_bPerm[27] += A[778]*rk_dim28_bPerm[22];
rk_dim28_bPerm[27] += A[779]*rk_dim28_bPerm[23];
rk_dim28_bPerm[27] += A[780]*rk_dim28_bPerm[24];
rk_dim28_bPerm[27] += A[781]*rk_dim28_bPerm[25];
rk_dim28_bPerm[27] += A[782]*rk_dim28_bPerm[26];


acado_solve_dim28_triangular( A, rk_dim28_bPerm );
b[0] = rk_dim28_bPerm[0];
b[1] = rk_dim28_bPerm[1];
b[2] = rk_dim28_bPerm[2];
b[3] = rk_dim28_bPerm[3];
b[4] = rk_dim28_bPerm[4];
b[5] = rk_dim28_bPerm[5];
b[6] = rk_dim28_bPerm[6];
b[7] = rk_dim28_bPerm[7];
b[8] = rk_dim28_bPerm[8];
b[9] = rk_dim28_bPerm[9];
b[10] = rk_dim28_bPerm[10];
b[11] = rk_dim28_bPerm[11];
b[12] = rk_dim28_bPerm[12];
b[13] = rk_dim28_bPerm[13];
b[14] = rk_dim28_bPerm[14];
b[15] = rk_dim28_bPerm[15];
b[16] = rk_dim28_bPerm[16];
b[17] = rk_dim28_bPerm[17];
b[18] = rk_dim28_bPerm[18];
b[19] = rk_dim28_bPerm[19];
b[20] = rk_dim28_bPerm[20];
b[21] = rk_dim28_bPerm[21];
b[22] = rk_dim28_bPerm[22];
b[23] = rk_dim28_bPerm[23];
b[24] = rk_dim28_bPerm[24];
b[25] = rk_dim28_bPerm[25];
b[26] = rk_dim28_bPerm[26];
b[27] = rk_dim28_bPerm[27];
}



/** Matrix of size: 2 x 2 (row major format) */
static const real_t acado_Ah_mat[ 4 ] = 
{ 2.5000000000000001e-02, 5.3867513459481292e-02, 
-3.8675134594812867e-03, 2.5000000000000001e-02 };


/* Fixed step size:0.1 */
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

rk_ttt = 0.0000000000000000e+00;
rk_xxx[14] = rk_eta[322];
rk_xxx[15] = rk_eta[323];
rk_xxx[16] = rk_eta[324];
rk_xxx[17] = rk_eta[325];
rk_xxx[18] = rk_eta[326];
rk_xxx[19] = rk_eta[327];
rk_xxx[20] = rk_eta[328];
rk_xxx[21] = rk_eta[329];
rk_xxx[22] = rk_eta[330];
rk_xxx[23] = rk_eta[331];
rk_xxx[24] = rk_eta[332];
rk_xxx[25] = rk_eta[333];
rk_xxx[26] = rk_eta[334];
rk_xxx[27] = rk_eta[335];
rk_xxx[28] = rk_eta[336];
rk_xxx[29] = rk_eta[337];
rk_xxx[30] = rk_eta[338];
rk_xxx[31] = rk_eta[339];

for (run = 0; run < 1; ++run)
{
if( resetIntegrator ) {
for (i = 0; i < 1; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 14; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1 * 2]*rk_kkk[tmp_index1 * 2];
rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 308 ]) );
for (j = 0; j < 14; ++j)
{
tmp_index1 = (run1 * 14) + (j);
rk_A[tmp_index1 * 28] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22)];
rk_A[tmp_index1 * 28 + 1] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 1)];
rk_A[tmp_index1 * 28 + 2] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 2)];
rk_A[tmp_index1 * 28 + 3] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 3)];
rk_A[tmp_index1 * 28 + 4] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 4)];
rk_A[tmp_index1 * 28 + 5] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 5)];
rk_A[tmp_index1 * 28 + 6] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 6)];
rk_A[tmp_index1 * 28 + 7] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 7)];
rk_A[tmp_index1 * 28 + 8] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 8)];
rk_A[tmp_index1 * 28 + 9] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 9)];
rk_A[tmp_index1 * 28 + 10] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 10)];
rk_A[tmp_index1 * 28 + 11] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 11)];
rk_A[tmp_index1 * 28 + 12] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 12)];
rk_A[tmp_index1 * 28 + 13] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 13)];
if( 0 == run1 ) rk_A[(tmp_index1 * 28) + (j)] -= 1.0000000000000000e+00;
rk_A[tmp_index1 * 28 + 14] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22)];
rk_A[tmp_index1 * 28 + 15] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 1)];
rk_A[tmp_index1 * 28 + 16] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 2)];
rk_A[tmp_index1 * 28 + 17] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 3)];
rk_A[tmp_index1 * 28 + 18] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 4)];
rk_A[tmp_index1 * 28 + 19] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 5)];
rk_A[tmp_index1 * 28 + 20] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 6)];
rk_A[tmp_index1 * 28 + 21] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 7)];
rk_A[tmp_index1 * 28 + 22] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 8)];
rk_A[tmp_index1 * 28 + 23] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 9)];
rk_A[tmp_index1 * 28 + 24] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 10)];
rk_A[tmp_index1 * 28 + 25] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 11)];
rk_A[tmp_index1 * 28 + 26] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 12)];
rk_A[tmp_index1 * 28 + 27] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 13)];
if( 1 == run1 ) rk_A[(tmp_index1 * 28) + (j + 14)] -= 1.0000000000000000e+00;
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 14] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 14 + 1] = rk_kkk[run1 + 2] - rk_rhsTemp[1];
rk_b[run1 * 14 + 2] = rk_kkk[run1 + 4] - rk_rhsTemp[2];
rk_b[run1 * 14 + 3] = rk_kkk[run1 + 6] - rk_rhsTemp[3];
rk_b[run1 * 14 + 4] = rk_kkk[run1 + 8] - rk_rhsTemp[4];
rk_b[run1 * 14 + 5] = rk_kkk[run1 + 10] - rk_rhsTemp[5];
rk_b[run1 * 14 + 6] = rk_kkk[run1 + 12] - rk_rhsTemp[6];
rk_b[run1 * 14 + 7] = rk_kkk[run1 + 14] - rk_rhsTemp[7];
rk_b[run1 * 14 + 8] = rk_kkk[run1 + 16] - rk_rhsTemp[8];
rk_b[run1 * 14 + 9] = rk_kkk[run1 + 18] - rk_rhsTemp[9];
rk_b[run1 * 14 + 10] = rk_kkk[run1 + 20] - rk_rhsTemp[10];
rk_b[run1 * 14 + 11] = rk_kkk[run1 + 22] - rk_rhsTemp[11];
rk_b[run1 * 14 + 12] = rk_kkk[run1 + 24] - rk_rhsTemp[12];
rk_b[run1 * 14 + 13] = rk_kkk[run1 + 26] - rk_rhsTemp[13];
}
det = acado_solve_dim28_system( rk_A, rk_b, rk_dim28_perm );
for (j = 0; j < 2; ++j)
{
rk_kkk[j] += rk_b[j * 14];
rk_kkk[j + 2] += rk_b[j * 14 + 1];
rk_kkk[j + 4] += rk_b[j * 14 + 2];
rk_kkk[j + 6] += rk_b[j * 14 + 3];
rk_kkk[j + 8] += rk_b[j * 14 + 4];
rk_kkk[j + 10] += rk_b[j * 14 + 5];
rk_kkk[j + 12] += rk_b[j * 14 + 6];
rk_kkk[j + 14] += rk_b[j * 14 + 7];
rk_kkk[j + 16] += rk_b[j * 14 + 8];
rk_kkk[j + 18] += rk_b[j * 14 + 9];
rk_kkk[j + 20] += rk_b[j * 14 + 10];
rk_kkk[j + 22] += rk_b[j * 14 + 11];
rk_kkk[j + 24] += rk_b[j * 14 + 12];
rk_kkk[j + 26] += rk_b[j * 14 + 13];
}
}
}
for (i = 0; i < 5; ++i)
{
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 14; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1 * 2]*rk_kkk[tmp_index1 * 2];
rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*rk_kkk[tmp_index1 * 2 + 1];
}
acado_rhs( rk_xxx, rk_rhsTemp );
rk_b[run1 * 14] = rk_kkk[run1] - rk_rhsTemp[0];
rk_b[run1 * 14 + 1] = rk_kkk[run1 + 2] - rk_rhsTemp[1];
rk_b[run1 * 14 + 2] = rk_kkk[run1 + 4] - rk_rhsTemp[2];
rk_b[run1 * 14 + 3] = rk_kkk[run1 + 6] - rk_rhsTemp[3];
rk_b[run1 * 14 + 4] = rk_kkk[run1 + 8] - rk_rhsTemp[4];
rk_b[run1 * 14 + 5] = rk_kkk[run1 + 10] - rk_rhsTemp[5];
rk_b[run1 * 14 + 6] = rk_kkk[run1 + 12] - rk_rhsTemp[6];
rk_b[run1 * 14 + 7] = rk_kkk[run1 + 14] - rk_rhsTemp[7];
rk_b[run1 * 14 + 8] = rk_kkk[run1 + 16] - rk_rhsTemp[8];
rk_b[run1 * 14 + 9] = rk_kkk[run1 + 18] - rk_rhsTemp[9];
rk_b[run1 * 14 + 10] = rk_kkk[run1 + 20] - rk_rhsTemp[10];
rk_b[run1 * 14 + 11] = rk_kkk[run1 + 22] - rk_rhsTemp[11];
rk_b[run1 * 14 + 12] = rk_kkk[run1 + 24] - rk_rhsTemp[12];
rk_b[run1 * 14 + 13] = rk_kkk[run1 + 26] - rk_rhsTemp[13];
}
acado_solve_dim28_system_reuse( rk_A, rk_b, rk_dim28_perm );
for (j = 0; j < 2; ++j)
{
rk_kkk[j] += rk_b[j * 14];
rk_kkk[j + 2] += rk_b[j * 14 + 1];
rk_kkk[j + 4] += rk_b[j * 14 + 2];
rk_kkk[j + 6] += rk_b[j * 14 + 3];
rk_kkk[j + 8] += rk_b[j * 14 + 4];
rk_kkk[j + 10] += rk_b[j * 14 + 5];
rk_kkk[j + 12] += rk_b[j * 14 + 6];
rk_kkk[j + 14] += rk_b[j * 14 + 7];
rk_kkk[j + 16] += rk_b[j * 14 + 8];
rk_kkk[j + 18] += rk_b[j * 14 + 9];
rk_kkk[j + 20] += rk_b[j * 14 + 10];
rk_kkk[j + 22] += rk_b[j * 14 + 11];
rk_kkk[j + 24] += rk_b[j * 14 + 12];
rk_kkk[j + 26] += rk_b[j * 14 + 13];
}
}
for (run1 = 0; run1 < 2; ++run1)
{
for (j = 0; j < 14; ++j)
{
rk_xxx[j] = rk_eta[j];
tmp_index1 = j;
rk_xxx[j] += + acado_Ah_mat[run1 * 2]*rk_kkk[tmp_index1 * 2];
rk_xxx[j] += + acado_Ah_mat[run1 * 2 + 1]*rk_kkk[tmp_index1 * 2 + 1];
}
acado_diffs( rk_xxx, &(rk_diffsTemp2[ run1 * 308 ]) );
for (j = 0; j < 14; ++j)
{
tmp_index1 = (run1 * 14) + (j);
rk_A[tmp_index1 * 28] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22)];
rk_A[tmp_index1 * 28 + 1] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 1)];
rk_A[tmp_index1 * 28 + 2] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 2)];
rk_A[tmp_index1 * 28 + 3] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 3)];
rk_A[tmp_index1 * 28 + 4] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 4)];
rk_A[tmp_index1 * 28 + 5] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 5)];
rk_A[tmp_index1 * 28 + 6] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 6)];
rk_A[tmp_index1 * 28 + 7] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 7)];
rk_A[tmp_index1 * 28 + 8] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 8)];
rk_A[tmp_index1 * 28 + 9] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 9)];
rk_A[tmp_index1 * 28 + 10] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 10)];
rk_A[tmp_index1 * 28 + 11] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 11)];
rk_A[tmp_index1 * 28 + 12] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 12)];
rk_A[tmp_index1 * 28 + 13] = + acado_Ah_mat[run1 * 2]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 13)];
if( 0 == run1 ) rk_A[(tmp_index1 * 28) + (j)] -= 1.0000000000000000e+00;
rk_A[tmp_index1 * 28 + 14] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22)];
rk_A[tmp_index1 * 28 + 15] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 1)];
rk_A[tmp_index1 * 28 + 16] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 2)];
rk_A[tmp_index1 * 28 + 17] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 3)];
rk_A[tmp_index1 * 28 + 18] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 4)];
rk_A[tmp_index1 * 28 + 19] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 5)];
rk_A[tmp_index1 * 28 + 20] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 6)];
rk_A[tmp_index1 * 28 + 21] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 7)];
rk_A[tmp_index1 * 28 + 22] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 8)];
rk_A[tmp_index1 * 28 + 23] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 9)];
rk_A[tmp_index1 * 28 + 24] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 10)];
rk_A[tmp_index1 * 28 + 25] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 11)];
rk_A[tmp_index1 * 28 + 26] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 12)];
rk_A[tmp_index1 * 28 + 27] = + acado_Ah_mat[run1 * 2 + 1]*rk_diffsTemp2[(run1 * 308) + (j * 22 + 13)];
if( 1 == run1 ) rk_A[(tmp_index1 * 28) + (j + 14)] -= 1.0000000000000000e+00;
}
}
for (run1 = 0; run1 < 14; ++run1)
{
for (i = 0; i < 2; ++i)
{
rk_b[i * 14] = - rk_diffsTemp2[(i * 308) + (run1)];
rk_b[i * 14 + 1] = - rk_diffsTemp2[(i * 308) + (run1 + 22)];
rk_b[i * 14 + 2] = - rk_diffsTemp2[(i * 308) + (run1 + 44)];
rk_b[i * 14 + 3] = - rk_diffsTemp2[(i * 308) + (run1 + 66)];
rk_b[i * 14 + 4] = - rk_diffsTemp2[(i * 308) + (run1 + 88)];
rk_b[i * 14 + 5] = - rk_diffsTemp2[(i * 308) + (run1 + 110)];
rk_b[i * 14 + 6] = - rk_diffsTemp2[(i * 308) + (run1 + 132)];
rk_b[i * 14 + 7] = - rk_diffsTemp2[(i * 308) + (run1 + 154)];
rk_b[i * 14 + 8] = - rk_diffsTemp2[(i * 308) + (run1 + 176)];
rk_b[i * 14 + 9] = - rk_diffsTemp2[(i * 308) + (run1 + 198)];
rk_b[i * 14 + 10] = - rk_diffsTemp2[(i * 308) + (run1 + 220)];
rk_b[i * 14 + 11] = - rk_diffsTemp2[(i * 308) + (run1 + 242)];
rk_b[i * 14 + 12] = - rk_diffsTemp2[(i * 308) + (run1 + 264)];
rk_b[i * 14 + 13] = - rk_diffsTemp2[(i * 308) + (run1 + 286)];
}
if( 0 == run1 ) {
det = acado_solve_dim28_system( rk_A, rk_b, rk_dim28_perm );
}
 else {
acado_solve_dim28_system_reuse( rk_A, rk_b, rk_dim28_perm );
}
for (i = 0; i < 2; ++i)
{
rk_diffK[i] = rk_b[i * 14];
rk_diffK[i + 2] = rk_b[i * 14 + 1];
rk_diffK[i + 4] = rk_b[i * 14 + 2];
rk_diffK[i + 6] = rk_b[i * 14 + 3];
rk_diffK[i + 8] = rk_b[i * 14 + 4];
rk_diffK[i + 10] = rk_b[i * 14 + 5];
rk_diffK[i + 12] = rk_b[i * 14 + 6];
rk_diffK[i + 14] = rk_b[i * 14 + 7];
rk_diffK[i + 16] = rk_b[i * 14 + 8];
rk_diffK[i + 18] = rk_b[i * 14 + 9];
rk_diffK[i + 20] = rk_b[i * 14 + 10];
rk_diffK[i + 22] = rk_b[i * 14 + 11];
rk_diffK[i + 24] = rk_b[i * 14 + 12];
rk_diffK[i + 26] = rk_b[i * 14 + 13];
}
for (i = 0; i < 14; ++i)
{
rk_diffsNew2[(i * 22) + (run1)] = (i == run1-0);
rk_diffsNew2[(i * 22) + (run1)] += + rk_diffK[i * 2]*(real_t)5.0000000000000003e-02 + rk_diffK[i * 2 + 1]*(real_t)5.0000000000000003e-02;
}
}
for (run1 = 0; run1 < 8; ++run1)
{
for (i = 0; i < 2; ++i)
{
for (j = 0; j < 14; ++j)
{
tmp_index1 = (i * 14) + (j);
tmp_index2 = (run1) + (j * 22);
rk_b[tmp_index1] = - rk_diffsTemp2[(i * 308) + (tmp_index2 + 14)];
}
}
acado_solve_dim28_system_reuse( rk_A, rk_b, rk_dim28_perm );
for (i = 0; i < 2; ++i)
{
rk_diffK[i] = rk_b[i * 14];
rk_diffK[i + 2] = rk_b[i * 14 + 1];
rk_diffK[i + 4] = rk_b[i * 14 + 2];
rk_diffK[i + 6] = rk_b[i * 14 + 3];
rk_diffK[i + 8] = rk_b[i * 14 + 4];
rk_diffK[i + 10] = rk_b[i * 14 + 5];
rk_diffK[i + 12] = rk_b[i * 14 + 6];
rk_diffK[i + 14] = rk_b[i * 14 + 7];
rk_diffK[i + 16] = rk_b[i * 14 + 8];
rk_diffK[i + 18] = rk_b[i * 14 + 9];
rk_diffK[i + 20] = rk_b[i * 14 + 10];
rk_diffK[i + 22] = rk_b[i * 14 + 11];
rk_diffK[i + 24] = rk_b[i * 14 + 12];
rk_diffK[i + 26] = rk_b[i * 14 + 13];
}
for (i = 0; i < 14; ++i)
{
rk_diffsNew2[(i * 22) + (run1 + 14)] = + rk_diffK[i * 2]*(real_t)5.0000000000000003e-02 + rk_diffK[i * 2 + 1]*(real_t)5.0000000000000003e-02;
}
}
rk_eta[0] += + rk_kkk[0]*(real_t)5.0000000000000003e-02 + rk_kkk[1]*(real_t)5.0000000000000003e-02;
rk_eta[1] += + rk_kkk[2]*(real_t)5.0000000000000003e-02 + rk_kkk[3]*(real_t)5.0000000000000003e-02;
rk_eta[2] += + rk_kkk[4]*(real_t)5.0000000000000003e-02 + rk_kkk[5]*(real_t)5.0000000000000003e-02;
rk_eta[3] += + rk_kkk[6]*(real_t)5.0000000000000003e-02 + rk_kkk[7]*(real_t)5.0000000000000003e-02;
rk_eta[4] += + rk_kkk[8]*(real_t)5.0000000000000003e-02 + rk_kkk[9]*(real_t)5.0000000000000003e-02;
rk_eta[5] += + rk_kkk[10]*(real_t)5.0000000000000003e-02 + rk_kkk[11]*(real_t)5.0000000000000003e-02;
rk_eta[6] += + rk_kkk[12]*(real_t)5.0000000000000003e-02 + rk_kkk[13]*(real_t)5.0000000000000003e-02;
rk_eta[7] += + rk_kkk[14]*(real_t)5.0000000000000003e-02 + rk_kkk[15]*(real_t)5.0000000000000003e-02;
rk_eta[8] += + rk_kkk[16]*(real_t)5.0000000000000003e-02 + rk_kkk[17]*(real_t)5.0000000000000003e-02;
rk_eta[9] += + rk_kkk[18]*(real_t)5.0000000000000003e-02 + rk_kkk[19]*(real_t)5.0000000000000003e-02;
rk_eta[10] += + rk_kkk[20]*(real_t)5.0000000000000003e-02 + rk_kkk[21]*(real_t)5.0000000000000003e-02;
rk_eta[11] += + rk_kkk[22]*(real_t)5.0000000000000003e-02 + rk_kkk[23]*(real_t)5.0000000000000003e-02;
rk_eta[12] += + rk_kkk[24]*(real_t)5.0000000000000003e-02 + rk_kkk[25]*(real_t)5.0000000000000003e-02;
rk_eta[13] += + rk_kkk[26]*(real_t)5.0000000000000003e-02 + rk_kkk[27]*(real_t)5.0000000000000003e-02;
for (i = 0; i < 14; ++i)
{
for (j = 0; j < 14; ++j)
{
tmp_index2 = (j) + (i * 14);
rk_eta[tmp_index2 + 14] = rk_diffsNew2[(i * 22) + (j)];
}
for (j = 0; j < 8; ++j)
{
tmp_index2 = (j) + (i * 8);
rk_eta[tmp_index2 + 210] = rk_diffsNew2[(i * 22) + (j + 14)];
}
}
resetIntegrator = 0;
rk_ttt += 1.0000000000000000e+00;
}
for (i = 0; i < 14; ++i)
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



