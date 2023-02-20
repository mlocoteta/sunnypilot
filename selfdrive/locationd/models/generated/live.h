#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_4230235865906954894);
void live_err_fun(double *nom_x, double *delta_x, double *out_7285454058321316757);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_3027376436415471136);
void live_H_mod_fun(double *state, double *out_2182370620574098344);
void live_f_fun(double *state, double dt, double *out_5392690204489221927);
void live_F_fun(double *state, double dt, double *out_4187189026783364532);
void live_h_4(double *state, double *unused, double *out_7646675726508551857);
void live_H_4(double *state, double *unused, double *out_1754981452174722276);
void live_h_9(double *state, double *unused, double *out_2762814744364720524);
void live_H_9(double *state, double *unused, double *out_5532237483089725194);
void live_h_10(double *state, double *unused, double *out_6463768574348619745);
void live_H_10(double *state, double *unused, double *out_7009897717026957061);
void live_h_12(double *state, double *unused, double *out_7053766938194549604);
void live_H_12(double *state, double *unused, double *out_8136239829217455272);
void live_h_35(double *state, double *unused, double *out_2211334445259474650);
void live_H_35(double *state, double *unused, double *out_1611680605197885100);
void live_h_32(double *state, double *unused, double *out_6811700224214412955);
void live_H_32(double *state, double *unused, double *out_5697416293294808200);
void live_h_13(double *state, double *unused, double *out_6028616001746047803);
void live_H_13(double *state, double *unused, double *out_1733507159111608554);
void live_h_14(double *state, double *unused, double *out_2762814744364720524);
void live_H_14(double *state, double *unused, double *out_5532237483089725194);
void live_h_33(double *state, double *unused, double *out_6754864634829922112);
void live_H_33(double *state, double *unused, double *out_4762237609836742704);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}