#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_1452444098184337597);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_7629255097489212520);
void gnss_H_mod_fun(double *state, double *out_4438101212553421742);
void gnss_f_fun(double *state, double dt, double *out_2174527054572435340);
void gnss_F_fun(double *state, double dt, double *out_4591488534622109259);
void gnss_h_6(double *state, double *sat_pos, double *out_2110618736992863902);
void gnss_H_6(double *state, double *sat_pos, double *out_2543314074998286150);
void gnss_h_20(double *state, double *sat_pos, double *out_7105290432145972618);
void gnss_H_20(double *state, double *sat_pos, double *out_9148244151572780200);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_8426460279896045301);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_4953432613900121427);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_8426460279896045301);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_4953432613900121427);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}