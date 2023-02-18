#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_7621706757976548531);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_9039208659539758108);
void car_H_mod_fun(double *state, double *out_1961184534032960867);
void car_f_fun(double *state, double dt, double *out_2459567307932433368);
void car_F_fun(double *state, double dt, double *out_362343134846257641);
void car_h_25(double *state, double *unused, double *out_6462515553101370206);
void car_H_25(double *state, double *unused, double *out_7992465279185128394);
void car_h_24(double *state, double *unused, double *out_7830134677132051528);
void car_H_24(double *state, double *unused, double *out_3245955952218619360);
void car_h_30(double *state, double *unused, double *out_2825066024847347856);
void car_H_30(double *state, double *unused, double *out_7935945836017174595);
void car_h_26(double *state, double *unused, double *out_7826739292508193501);
void car_H_26(double *state, double *unused, double *out_7149752824763622621);
void car_h_27(double *state, double *unused, double *out_3002259348601340645);
void car_H_27(double *state, double *unused, double *out_8336034925891952110);
void car_h_29(double *state, double *unused, double *out_2176607242813497789);
void car_H_29(double *state, double *unused, double *out_7425714491702782411);
void car_h_28(double *state, double *unused, double *out_8003932616262186290);
void car_H_28(double *state, double *unused, double *out_5938630564937238631);
void car_h_31(double *state, double *unused, double *out_6737709615385876095);
void car_H_31(double *state, double *unused, double *out_3624753858077720694);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}