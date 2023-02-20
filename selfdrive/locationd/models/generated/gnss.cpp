#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_1452444098184337597) {
   out_1452444098184337597[0] = delta_x[0] + nom_x[0];
   out_1452444098184337597[1] = delta_x[1] + nom_x[1];
   out_1452444098184337597[2] = delta_x[2] + nom_x[2];
   out_1452444098184337597[3] = delta_x[3] + nom_x[3];
   out_1452444098184337597[4] = delta_x[4] + nom_x[4];
   out_1452444098184337597[5] = delta_x[5] + nom_x[5];
   out_1452444098184337597[6] = delta_x[6] + nom_x[6];
   out_1452444098184337597[7] = delta_x[7] + nom_x[7];
   out_1452444098184337597[8] = delta_x[8] + nom_x[8];
   out_1452444098184337597[9] = delta_x[9] + nom_x[9];
   out_1452444098184337597[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_7629255097489212520) {
   out_7629255097489212520[0] = -nom_x[0] + true_x[0];
   out_7629255097489212520[1] = -nom_x[1] + true_x[1];
   out_7629255097489212520[2] = -nom_x[2] + true_x[2];
   out_7629255097489212520[3] = -nom_x[3] + true_x[3];
   out_7629255097489212520[4] = -nom_x[4] + true_x[4];
   out_7629255097489212520[5] = -nom_x[5] + true_x[5];
   out_7629255097489212520[6] = -nom_x[6] + true_x[6];
   out_7629255097489212520[7] = -nom_x[7] + true_x[7];
   out_7629255097489212520[8] = -nom_x[8] + true_x[8];
   out_7629255097489212520[9] = -nom_x[9] + true_x[9];
   out_7629255097489212520[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_4438101212553421742) {
   out_4438101212553421742[0] = 1.0;
   out_4438101212553421742[1] = 0;
   out_4438101212553421742[2] = 0;
   out_4438101212553421742[3] = 0;
   out_4438101212553421742[4] = 0;
   out_4438101212553421742[5] = 0;
   out_4438101212553421742[6] = 0;
   out_4438101212553421742[7] = 0;
   out_4438101212553421742[8] = 0;
   out_4438101212553421742[9] = 0;
   out_4438101212553421742[10] = 0;
   out_4438101212553421742[11] = 0;
   out_4438101212553421742[12] = 1.0;
   out_4438101212553421742[13] = 0;
   out_4438101212553421742[14] = 0;
   out_4438101212553421742[15] = 0;
   out_4438101212553421742[16] = 0;
   out_4438101212553421742[17] = 0;
   out_4438101212553421742[18] = 0;
   out_4438101212553421742[19] = 0;
   out_4438101212553421742[20] = 0;
   out_4438101212553421742[21] = 0;
   out_4438101212553421742[22] = 0;
   out_4438101212553421742[23] = 0;
   out_4438101212553421742[24] = 1.0;
   out_4438101212553421742[25] = 0;
   out_4438101212553421742[26] = 0;
   out_4438101212553421742[27] = 0;
   out_4438101212553421742[28] = 0;
   out_4438101212553421742[29] = 0;
   out_4438101212553421742[30] = 0;
   out_4438101212553421742[31] = 0;
   out_4438101212553421742[32] = 0;
   out_4438101212553421742[33] = 0;
   out_4438101212553421742[34] = 0;
   out_4438101212553421742[35] = 0;
   out_4438101212553421742[36] = 1.0;
   out_4438101212553421742[37] = 0;
   out_4438101212553421742[38] = 0;
   out_4438101212553421742[39] = 0;
   out_4438101212553421742[40] = 0;
   out_4438101212553421742[41] = 0;
   out_4438101212553421742[42] = 0;
   out_4438101212553421742[43] = 0;
   out_4438101212553421742[44] = 0;
   out_4438101212553421742[45] = 0;
   out_4438101212553421742[46] = 0;
   out_4438101212553421742[47] = 0;
   out_4438101212553421742[48] = 1.0;
   out_4438101212553421742[49] = 0;
   out_4438101212553421742[50] = 0;
   out_4438101212553421742[51] = 0;
   out_4438101212553421742[52] = 0;
   out_4438101212553421742[53] = 0;
   out_4438101212553421742[54] = 0;
   out_4438101212553421742[55] = 0;
   out_4438101212553421742[56] = 0;
   out_4438101212553421742[57] = 0;
   out_4438101212553421742[58] = 0;
   out_4438101212553421742[59] = 0;
   out_4438101212553421742[60] = 1.0;
   out_4438101212553421742[61] = 0;
   out_4438101212553421742[62] = 0;
   out_4438101212553421742[63] = 0;
   out_4438101212553421742[64] = 0;
   out_4438101212553421742[65] = 0;
   out_4438101212553421742[66] = 0;
   out_4438101212553421742[67] = 0;
   out_4438101212553421742[68] = 0;
   out_4438101212553421742[69] = 0;
   out_4438101212553421742[70] = 0;
   out_4438101212553421742[71] = 0;
   out_4438101212553421742[72] = 1.0;
   out_4438101212553421742[73] = 0;
   out_4438101212553421742[74] = 0;
   out_4438101212553421742[75] = 0;
   out_4438101212553421742[76] = 0;
   out_4438101212553421742[77] = 0;
   out_4438101212553421742[78] = 0;
   out_4438101212553421742[79] = 0;
   out_4438101212553421742[80] = 0;
   out_4438101212553421742[81] = 0;
   out_4438101212553421742[82] = 0;
   out_4438101212553421742[83] = 0;
   out_4438101212553421742[84] = 1.0;
   out_4438101212553421742[85] = 0;
   out_4438101212553421742[86] = 0;
   out_4438101212553421742[87] = 0;
   out_4438101212553421742[88] = 0;
   out_4438101212553421742[89] = 0;
   out_4438101212553421742[90] = 0;
   out_4438101212553421742[91] = 0;
   out_4438101212553421742[92] = 0;
   out_4438101212553421742[93] = 0;
   out_4438101212553421742[94] = 0;
   out_4438101212553421742[95] = 0;
   out_4438101212553421742[96] = 1.0;
   out_4438101212553421742[97] = 0;
   out_4438101212553421742[98] = 0;
   out_4438101212553421742[99] = 0;
   out_4438101212553421742[100] = 0;
   out_4438101212553421742[101] = 0;
   out_4438101212553421742[102] = 0;
   out_4438101212553421742[103] = 0;
   out_4438101212553421742[104] = 0;
   out_4438101212553421742[105] = 0;
   out_4438101212553421742[106] = 0;
   out_4438101212553421742[107] = 0;
   out_4438101212553421742[108] = 1.0;
   out_4438101212553421742[109] = 0;
   out_4438101212553421742[110] = 0;
   out_4438101212553421742[111] = 0;
   out_4438101212553421742[112] = 0;
   out_4438101212553421742[113] = 0;
   out_4438101212553421742[114] = 0;
   out_4438101212553421742[115] = 0;
   out_4438101212553421742[116] = 0;
   out_4438101212553421742[117] = 0;
   out_4438101212553421742[118] = 0;
   out_4438101212553421742[119] = 0;
   out_4438101212553421742[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_2174527054572435340) {
   out_2174527054572435340[0] = dt*state[3] + state[0];
   out_2174527054572435340[1] = dt*state[4] + state[1];
   out_2174527054572435340[2] = dt*state[5] + state[2];
   out_2174527054572435340[3] = state[3];
   out_2174527054572435340[4] = state[4];
   out_2174527054572435340[5] = state[5];
   out_2174527054572435340[6] = dt*state[7] + state[6];
   out_2174527054572435340[7] = dt*state[8] + state[7];
   out_2174527054572435340[8] = state[8];
   out_2174527054572435340[9] = state[9];
   out_2174527054572435340[10] = state[10];
}
void F_fun(double *state, double dt, double *out_4591488534622109259) {
   out_4591488534622109259[0] = 1;
   out_4591488534622109259[1] = 0;
   out_4591488534622109259[2] = 0;
   out_4591488534622109259[3] = dt;
   out_4591488534622109259[4] = 0;
   out_4591488534622109259[5] = 0;
   out_4591488534622109259[6] = 0;
   out_4591488534622109259[7] = 0;
   out_4591488534622109259[8] = 0;
   out_4591488534622109259[9] = 0;
   out_4591488534622109259[10] = 0;
   out_4591488534622109259[11] = 0;
   out_4591488534622109259[12] = 1;
   out_4591488534622109259[13] = 0;
   out_4591488534622109259[14] = 0;
   out_4591488534622109259[15] = dt;
   out_4591488534622109259[16] = 0;
   out_4591488534622109259[17] = 0;
   out_4591488534622109259[18] = 0;
   out_4591488534622109259[19] = 0;
   out_4591488534622109259[20] = 0;
   out_4591488534622109259[21] = 0;
   out_4591488534622109259[22] = 0;
   out_4591488534622109259[23] = 0;
   out_4591488534622109259[24] = 1;
   out_4591488534622109259[25] = 0;
   out_4591488534622109259[26] = 0;
   out_4591488534622109259[27] = dt;
   out_4591488534622109259[28] = 0;
   out_4591488534622109259[29] = 0;
   out_4591488534622109259[30] = 0;
   out_4591488534622109259[31] = 0;
   out_4591488534622109259[32] = 0;
   out_4591488534622109259[33] = 0;
   out_4591488534622109259[34] = 0;
   out_4591488534622109259[35] = 0;
   out_4591488534622109259[36] = 1;
   out_4591488534622109259[37] = 0;
   out_4591488534622109259[38] = 0;
   out_4591488534622109259[39] = 0;
   out_4591488534622109259[40] = 0;
   out_4591488534622109259[41] = 0;
   out_4591488534622109259[42] = 0;
   out_4591488534622109259[43] = 0;
   out_4591488534622109259[44] = 0;
   out_4591488534622109259[45] = 0;
   out_4591488534622109259[46] = 0;
   out_4591488534622109259[47] = 0;
   out_4591488534622109259[48] = 1;
   out_4591488534622109259[49] = 0;
   out_4591488534622109259[50] = 0;
   out_4591488534622109259[51] = 0;
   out_4591488534622109259[52] = 0;
   out_4591488534622109259[53] = 0;
   out_4591488534622109259[54] = 0;
   out_4591488534622109259[55] = 0;
   out_4591488534622109259[56] = 0;
   out_4591488534622109259[57] = 0;
   out_4591488534622109259[58] = 0;
   out_4591488534622109259[59] = 0;
   out_4591488534622109259[60] = 1;
   out_4591488534622109259[61] = 0;
   out_4591488534622109259[62] = 0;
   out_4591488534622109259[63] = 0;
   out_4591488534622109259[64] = 0;
   out_4591488534622109259[65] = 0;
   out_4591488534622109259[66] = 0;
   out_4591488534622109259[67] = 0;
   out_4591488534622109259[68] = 0;
   out_4591488534622109259[69] = 0;
   out_4591488534622109259[70] = 0;
   out_4591488534622109259[71] = 0;
   out_4591488534622109259[72] = 1;
   out_4591488534622109259[73] = dt;
   out_4591488534622109259[74] = 0;
   out_4591488534622109259[75] = 0;
   out_4591488534622109259[76] = 0;
   out_4591488534622109259[77] = 0;
   out_4591488534622109259[78] = 0;
   out_4591488534622109259[79] = 0;
   out_4591488534622109259[80] = 0;
   out_4591488534622109259[81] = 0;
   out_4591488534622109259[82] = 0;
   out_4591488534622109259[83] = 0;
   out_4591488534622109259[84] = 1;
   out_4591488534622109259[85] = dt;
   out_4591488534622109259[86] = 0;
   out_4591488534622109259[87] = 0;
   out_4591488534622109259[88] = 0;
   out_4591488534622109259[89] = 0;
   out_4591488534622109259[90] = 0;
   out_4591488534622109259[91] = 0;
   out_4591488534622109259[92] = 0;
   out_4591488534622109259[93] = 0;
   out_4591488534622109259[94] = 0;
   out_4591488534622109259[95] = 0;
   out_4591488534622109259[96] = 1;
   out_4591488534622109259[97] = 0;
   out_4591488534622109259[98] = 0;
   out_4591488534622109259[99] = 0;
   out_4591488534622109259[100] = 0;
   out_4591488534622109259[101] = 0;
   out_4591488534622109259[102] = 0;
   out_4591488534622109259[103] = 0;
   out_4591488534622109259[104] = 0;
   out_4591488534622109259[105] = 0;
   out_4591488534622109259[106] = 0;
   out_4591488534622109259[107] = 0;
   out_4591488534622109259[108] = 1;
   out_4591488534622109259[109] = 0;
   out_4591488534622109259[110] = 0;
   out_4591488534622109259[111] = 0;
   out_4591488534622109259[112] = 0;
   out_4591488534622109259[113] = 0;
   out_4591488534622109259[114] = 0;
   out_4591488534622109259[115] = 0;
   out_4591488534622109259[116] = 0;
   out_4591488534622109259[117] = 0;
   out_4591488534622109259[118] = 0;
   out_4591488534622109259[119] = 0;
   out_4591488534622109259[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_2110618736992863902) {
   out_2110618736992863902[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_2543314074998286150) {
   out_2543314074998286150[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2543314074998286150[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2543314074998286150[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_2543314074998286150[3] = 0;
   out_2543314074998286150[4] = 0;
   out_2543314074998286150[5] = 0;
   out_2543314074998286150[6] = 1;
   out_2543314074998286150[7] = 0;
   out_2543314074998286150[8] = 0;
   out_2543314074998286150[9] = 0;
   out_2543314074998286150[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_7105290432145972618) {
   out_7105290432145972618[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_9148244151572780200) {
   out_9148244151572780200[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_9148244151572780200[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_9148244151572780200[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_9148244151572780200[3] = 0;
   out_9148244151572780200[4] = 0;
   out_9148244151572780200[5] = 0;
   out_9148244151572780200[6] = 1;
   out_9148244151572780200[7] = 0;
   out_9148244151572780200[8] = 0;
   out_9148244151572780200[9] = 1;
   out_9148244151572780200[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_8426460279896045301) {
   out_8426460279896045301[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_4953432613900121427) {
   out_4953432613900121427[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4953432613900121427[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4953432613900121427[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4953432613900121427[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4953432613900121427[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4953432613900121427[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4953432613900121427[6] = 0;
   out_4953432613900121427[7] = 1;
   out_4953432613900121427[8] = 0;
   out_4953432613900121427[9] = 0;
   out_4953432613900121427[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_8426460279896045301) {
   out_8426460279896045301[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_4953432613900121427) {
   out_4953432613900121427[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4953432613900121427[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4953432613900121427[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4953432613900121427[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4953432613900121427[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4953432613900121427[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_4953432613900121427[6] = 0;
   out_4953432613900121427[7] = 1;
   out_4953432613900121427[8] = 0;
   out_4953432613900121427[9] = 0;
   out_4953432613900121427[10] = 0;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_1452444098184337597) {
  err_fun(nom_x, delta_x, out_1452444098184337597);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_7629255097489212520) {
  inv_err_fun(nom_x, true_x, out_7629255097489212520);
}
void gnss_H_mod_fun(double *state, double *out_4438101212553421742) {
  H_mod_fun(state, out_4438101212553421742);
}
void gnss_f_fun(double *state, double dt, double *out_2174527054572435340) {
  f_fun(state,  dt, out_2174527054572435340);
}
void gnss_F_fun(double *state, double dt, double *out_4591488534622109259) {
  F_fun(state,  dt, out_4591488534622109259);
}
void gnss_h_6(double *state, double *sat_pos, double *out_2110618736992863902) {
  h_6(state, sat_pos, out_2110618736992863902);
}
void gnss_H_6(double *state, double *sat_pos, double *out_2543314074998286150) {
  H_6(state, sat_pos, out_2543314074998286150);
}
void gnss_h_20(double *state, double *sat_pos, double *out_7105290432145972618) {
  h_20(state, sat_pos, out_7105290432145972618);
}
void gnss_H_20(double *state, double *sat_pos, double *out_9148244151572780200) {
  H_20(state, sat_pos, out_9148244151572780200);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_8426460279896045301) {
  h_7(state, sat_pos_vel, out_8426460279896045301);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_4953432613900121427) {
  H_7(state, sat_pos_vel, out_4953432613900121427);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_8426460279896045301) {
  h_21(state, sat_pos_vel, out_8426460279896045301);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_4953432613900121427) {
  H_21(state, sat_pos_vel, out_4953432613900121427);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
