#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7621706757976548531) {
   out_7621706757976548531[0] = delta_x[0] + nom_x[0];
   out_7621706757976548531[1] = delta_x[1] + nom_x[1];
   out_7621706757976548531[2] = delta_x[2] + nom_x[2];
   out_7621706757976548531[3] = delta_x[3] + nom_x[3];
   out_7621706757976548531[4] = delta_x[4] + nom_x[4];
   out_7621706757976548531[5] = delta_x[5] + nom_x[5];
   out_7621706757976548531[6] = delta_x[6] + nom_x[6];
   out_7621706757976548531[7] = delta_x[7] + nom_x[7];
   out_7621706757976548531[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_9039208659539758108) {
   out_9039208659539758108[0] = -nom_x[0] + true_x[0];
   out_9039208659539758108[1] = -nom_x[1] + true_x[1];
   out_9039208659539758108[2] = -nom_x[2] + true_x[2];
   out_9039208659539758108[3] = -nom_x[3] + true_x[3];
   out_9039208659539758108[4] = -nom_x[4] + true_x[4];
   out_9039208659539758108[5] = -nom_x[5] + true_x[5];
   out_9039208659539758108[6] = -nom_x[6] + true_x[6];
   out_9039208659539758108[7] = -nom_x[7] + true_x[7];
   out_9039208659539758108[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_1961184534032960867) {
   out_1961184534032960867[0] = 1.0;
   out_1961184534032960867[1] = 0;
   out_1961184534032960867[2] = 0;
   out_1961184534032960867[3] = 0;
   out_1961184534032960867[4] = 0;
   out_1961184534032960867[5] = 0;
   out_1961184534032960867[6] = 0;
   out_1961184534032960867[7] = 0;
   out_1961184534032960867[8] = 0;
   out_1961184534032960867[9] = 0;
   out_1961184534032960867[10] = 1.0;
   out_1961184534032960867[11] = 0;
   out_1961184534032960867[12] = 0;
   out_1961184534032960867[13] = 0;
   out_1961184534032960867[14] = 0;
   out_1961184534032960867[15] = 0;
   out_1961184534032960867[16] = 0;
   out_1961184534032960867[17] = 0;
   out_1961184534032960867[18] = 0;
   out_1961184534032960867[19] = 0;
   out_1961184534032960867[20] = 1.0;
   out_1961184534032960867[21] = 0;
   out_1961184534032960867[22] = 0;
   out_1961184534032960867[23] = 0;
   out_1961184534032960867[24] = 0;
   out_1961184534032960867[25] = 0;
   out_1961184534032960867[26] = 0;
   out_1961184534032960867[27] = 0;
   out_1961184534032960867[28] = 0;
   out_1961184534032960867[29] = 0;
   out_1961184534032960867[30] = 1.0;
   out_1961184534032960867[31] = 0;
   out_1961184534032960867[32] = 0;
   out_1961184534032960867[33] = 0;
   out_1961184534032960867[34] = 0;
   out_1961184534032960867[35] = 0;
   out_1961184534032960867[36] = 0;
   out_1961184534032960867[37] = 0;
   out_1961184534032960867[38] = 0;
   out_1961184534032960867[39] = 0;
   out_1961184534032960867[40] = 1.0;
   out_1961184534032960867[41] = 0;
   out_1961184534032960867[42] = 0;
   out_1961184534032960867[43] = 0;
   out_1961184534032960867[44] = 0;
   out_1961184534032960867[45] = 0;
   out_1961184534032960867[46] = 0;
   out_1961184534032960867[47] = 0;
   out_1961184534032960867[48] = 0;
   out_1961184534032960867[49] = 0;
   out_1961184534032960867[50] = 1.0;
   out_1961184534032960867[51] = 0;
   out_1961184534032960867[52] = 0;
   out_1961184534032960867[53] = 0;
   out_1961184534032960867[54] = 0;
   out_1961184534032960867[55] = 0;
   out_1961184534032960867[56] = 0;
   out_1961184534032960867[57] = 0;
   out_1961184534032960867[58] = 0;
   out_1961184534032960867[59] = 0;
   out_1961184534032960867[60] = 1.0;
   out_1961184534032960867[61] = 0;
   out_1961184534032960867[62] = 0;
   out_1961184534032960867[63] = 0;
   out_1961184534032960867[64] = 0;
   out_1961184534032960867[65] = 0;
   out_1961184534032960867[66] = 0;
   out_1961184534032960867[67] = 0;
   out_1961184534032960867[68] = 0;
   out_1961184534032960867[69] = 0;
   out_1961184534032960867[70] = 1.0;
   out_1961184534032960867[71] = 0;
   out_1961184534032960867[72] = 0;
   out_1961184534032960867[73] = 0;
   out_1961184534032960867[74] = 0;
   out_1961184534032960867[75] = 0;
   out_1961184534032960867[76] = 0;
   out_1961184534032960867[77] = 0;
   out_1961184534032960867[78] = 0;
   out_1961184534032960867[79] = 0;
   out_1961184534032960867[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_2459567307932433368) {
   out_2459567307932433368[0] = state[0];
   out_2459567307932433368[1] = state[1];
   out_2459567307932433368[2] = state[2];
   out_2459567307932433368[3] = state[3];
   out_2459567307932433368[4] = state[4];
   out_2459567307932433368[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8000000000000007*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_2459567307932433368[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_2459567307932433368[7] = state[7];
   out_2459567307932433368[8] = state[8];
}
void F_fun(double *state, double dt, double *out_362343134846257641) {
   out_362343134846257641[0] = 1;
   out_362343134846257641[1] = 0;
   out_362343134846257641[2] = 0;
   out_362343134846257641[3] = 0;
   out_362343134846257641[4] = 0;
   out_362343134846257641[5] = 0;
   out_362343134846257641[6] = 0;
   out_362343134846257641[7] = 0;
   out_362343134846257641[8] = 0;
   out_362343134846257641[9] = 0;
   out_362343134846257641[10] = 1;
   out_362343134846257641[11] = 0;
   out_362343134846257641[12] = 0;
   out_362343134846257641[13] = 0;
   out_362343134846257641[14] = 0;
   out_362343134846257641[15] = 0;
   out_362343134846257641[16] = 0;
   out_362343134846257641[17] = 0;
   out_362343134846257641[18] = 0;
   out_362343134846257641[19] = 0;
   out_362343134846257641[20] = 1;
   out_362343134846257641[21] = 0;
   out_362343134846257641[22] = 0;
   out_362343134846257641[23] = 0;
   out_362343134846257641[24] = 0;
   out_362343134846257641[25] = 0;
   out_362343134846257641[26] = 0;
   out_362343134846257641[27] = 0;
   out_362343134846257641[28] = 0;
   out_362343134846257641[29] = 0;
   out_362343134846257641[30] = 1;
   out_362343134846257641[31] = 0;
   out_362343134846257641[32] = 0;
   out_362343134846257641[33] = 0;
   out_362343134846257641[34] = 0;
   out_362343134846257641[35] = 0;
   out_362343134846257641[36] = 0;
   out_362343134846257641[37] = 0;
   out_362343134846257641[38] = 0;
   out_362343134846257641[39] = 0;
   out_362343134846257641[40] = 1;
   out_362343134846257641[41] = 0;
   out_362343134846257641[42] = 0;
   out_362343134846257641[43] = 0;
   out_362343134846257641[44] = 0;
   out_362343134846257641[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_362343134846257641[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_362343134846257641[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_362343134846257641[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_362343134846257641[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_362343134846257641[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_362343134846257641[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_362343134846257641[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_362343134846257641[53] = -9.8000000000000007*dt;
   out_362343134846257641[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_362343134846257641[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_362343134846257641[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_362343134846257641[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_362343134846257641[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_362343134846257641[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_362343134846257641[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_362343134846257641[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_362343134846257641[62] = 0;
   out_362343134846257641[63] = 0;
   out_362343134846257641[64] = 0;
   out_362343134846257641[65] = 0;
   out_362343134846257641[66] = 0;
   out_362343134846257641[67] = 0;
   out_362343134846257641[68] = 0;
   out_362343134846257641[69] = 0;
   out_362343134846257641[70] = 1;
   out_362343134846257641[71] = 0;
   out_362343134846257641[72] = 0;
   out_362343134846257641[73] = 0;
   out_362343134846257641[74] = 0;
   out_362343134846257641[75] = 0;
   out_362343134846257641[76] = 0;
   out_362343134846257641[77] = 0;
   out_362343134846257641[78] = 0;
   out_362343134846257641[79] = 0;
   out_362343134846257641[80] = 1;
}
void h_25(double *state, double *unused, double *out_6462515553101370206) {
   out_6462515553101370206[0] = state[6];
}
void H_25(double *state, double *unused, double *out_7992465279185128394) {
   out_7992465279185128394[0] = 0;
   out_7992465279185128394[1] = 0;
   out_7992465279185128394[2] = 0;
   out_7992465279185128394[3] = 0;
   out_7992465279185128394[4] = 0;
   out_7992465279185128394[5] = 0;
   out_7992465279185128394[6] = 1;
   out_7992465279185128394[7] = 0;
   out_7992465279185128394[8] = 0;
}
void h_24(double *state, double *unused, double *out_7830134677132051528) {
   out_7830134677132051528[0] = state[4];
   out_7830134677132051528[1] = state[5];
}
void H_24(double *state, double *unused, double *out_3245955952218619360) {
   out_3245955952218619360[0] = 0;
   out_3245955952218619360[1] = 0;
   out_3245955952218619360[2] = 0;
   out_3245955952218619360[3] = 0;
   out_3245955952218619360[4] = 1;
   out_3245955952218619360[5] = 0;
   out_3245955952218619360[6] = 0;
   out_3245955952218619360[7] = 0;
   out_3245955952218619360[8] = 0;
   out_3245955952218619360[9] = 0;
   out_3245955952218619360[10] = 0;
   out_3245955952218619360[11] = 0;
   out_3245955952218619360[12] = 0;
   out_3245955952218619360[13] = 0;
   out_3245955952218619360[14] = 1;
   out_3245955952218619360[15] = 0;
   out_3245955952218619360[16] = 0;
   out_3245955952218619360[17] = 0;
}
void h_30(double *state, double *unused, double *out_2825066024847347856) {
   out_2825066024847347856[0] = state[4];
}
void H_30(double *state, double *unused, double *out_7935945836017174595) {
   out_7935945836017174595[0] = 0;
   out_7935945836017174595[1] = 0;
   out_7935945836017174595[2] = 0;
   out_7935945836017174595[3] = 0;
   out_7935945836017174595[4] = 1;
   out_7935945836017174595[5] = 0;
   out_7935945836017174595[6] = 0;
   out_7935945836017174595[7] = 0;
   out_7935945836017174595[8] = 0;
}
void h_26(double *state, double *unused, double *out_7826739292508193501) {
   out_7826739292508193501[0] = state[7];
}
void H_26(double *state, double *unused, double *out_7149752824763622621) {
   out_7149752824763622621[0] = 0;
   out_7149752824763622621[1] = 0;
   out_7149752824763622621[2] = 0;
   out_7149752824763622621[3] = 0;
   out_7149752824763622621[4] = 0;
   out_7149752824763622621[5] = 0;
   out_7149752824763622621[6] = 0;
   out_7149752824763622621[7] = 1;
   out_7149752824763622621[8] = 0;
}
void h_27(double *state, double *unused, double *out_3002259348601340645) {
   out_3002259348601340645[0] = state[3];
}
void H_27(double *state, double *unused, double *out_8336034925891952110) {
   out_8336034925891952110[0] = 0;
   out_8336034925891952110[1] = 0;
   out_8336034925891952110[2] = 0;
   out_8336034925891952110[3] = 1;
   out_8336034925891952110[4] = 0;
   out_8336034925891952110[5] = 0;
   out_8336034925891952110[6] = 0;
   out_8336034925891952110[7] = 0;
   out_8336034925891952110[8] = 0;
}
void h_29(double *state, double *unused, double *out_2176607242813497789) {
   out_2176607242813497789[0] = state[1];
}
void H_29(double *state, double *unused, double *out_7425714491702782411) {
   out_7425714491702782411[0] = 0;
   out_7425714491702782411[1] = 1;
   out_7425714491702782411[2] = 0;
   out_7425714491702782411[3] = 0;
   out_7425714491702782411[4] = 0;
   out_7425714491702782411[5] = 0;
   out_7425714491702782411[6] = 0;
   out_7425714491702782411[7] = 0;
   out_7425714491702782411[8] = 0;
}
void h_28(double *state, double *unused, double *out_8003932616262186290) {
   out_8003932616262186290[0] = state[0];
}
void H_28(double *state, double *unused, double *out_5938630564937238631) {
   out_5938630564937238631[0] = 1;
   out_5938630564937238631[1] = 0;
   out_5938630564937238631[2] = 0;
   out_5938630564937238631[3] = 0;
   out_5938630564937238631[4] = 0;
   out_5938630564937238631[5] = 0;
   out_5938630564937238631[6] = 0;
   out_5938630564937238631[7] = 0;
   out_5938630564937238631[8] = 0;
}
void h_31(double *state, double *unused, double *out_6737709615385876095) {
   out_6737709615385876095[0] = state[8];
}
void H_31(double *state, double *unused, double *out_3624753858077720694) {
   out_3624753858077720694[0] = 0;
   out_3624753858077720694[1] = 0;
   out_3624753858077720694[2] = 0;
   out_3624753858077720694[3] = 0;
   out_3624753858077720694[4] = 0;
   out_3624753858077720694[5] = 0;
   out_3624753858077720694[6] = 0;
   out_3624753858077720694[7] = 0;
   out_3624753858077720694[8] = 1;
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

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_7621706757976548531) {
  err_fun(nom_x, delta_x, out_7621706757976548531);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_9039208659539758108) {
  inv_err_fun(nom_x, true_x, out_9039208659539758108);
}
void car_H_mod_fun(double *state, double *out_1961184534032960867) {
  H_mod_fun(state, out_1961184534032960867);
}
void car_f_fun(double *state, double dt, double *out_2459567307932433368) {
  f_fun(state,  dt, out_2459567307932433368);
}
void car_F_fun(double *state, double dt, double *out_362343134846257641) {
  F_fun(state,  dt, out_362343134846257641);
}
void car_h_25(double *state, double *unused, double *out_6462515553101370206) {
  h_25(state, unused, out_6462515553101370206);
}
void car_H_25(double *state, double *unused, double *out_7992465279185128394) {
  H_25(state, unused, out_7992465279185128394);
}
void car_h_24(double *state, double *unused, double *out_7830134677132051528) {
  h_24(state, unused, out_7830134677132051528);
}
void car_H_24(double *state, double *unused, double *out_3245955952218619360) {
  H_24(state, unused, out_3245955952218619360);
}
void car_h_30(double *state, double *unused, double *out_2825066024847347856) {
  h_30(state, unused, out_2825066024847347856);
}
void car_H_30(double *state, double *unused, double *out_7935945836017174595) {
  H_30(state, unused, out_7935945836017174595);
}
void car_h_26(double *state, double *unused, double *out_7826739292508193501) {
  h_26(state, unused, out_7826739292508193501);
}
void car_H_26(double *state, double *unused, double *out_7149752824763622621) {
  H_26(state, unused, out_7149752824763622621);
}
void car_h_27(double *state, double *unused, double *out_3002259348601340645) {
  h_27(state, unused, out_3002259348601340645);
}
void car_H_27(double *state, double *unused, double *out_8336034925891952110) {
  H_27(state, unused, out_8336034925891952110);
}
void car_h_29(double *state, double *unused, double *out_2176607242813497789) {
  h_29(state, unused, out_2176607242813497789);
}
void car_H_29(double *state, double *unused, double *out_7425714491702782411) {
  H_29(state, unused, out_7425714491702782411);
}
void car_h_28(double *state, double *unused, double *out_8003932616262186290) {
  h_28(state, unused, out_8003932616262186290);
}
void car_H_28(double *state, double *unused, double *out_5938630564937238631) {
  H_28(state, unused, out_5938630564937238631);
}
void car_h_31(double *state, double *unused, double *out_6737709615385876095) {
  h_31(state, unused, out_6737709615385876095);
}
void car_H_31(double *state, double *unused, double *out_3624753858077720694) {
  H_31(state, unused, out_3624753858077720694);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_init(car);
