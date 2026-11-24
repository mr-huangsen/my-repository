#pragma once

typedef struct DPConfig{
    int rows = 0;
    int cols = 0;
    double sample_s = 0;
    double sample_l = 0;

    double w_cost_obs = 0;
    double w_cost_dl = 0;
    double w_cost_ddl = 0;
    double w_cost_dddl = 0;
    double w_cost_ref = 0;

}DPConfig;

typedef struct QPConfig{
    double w_cost_dl = 0;
    double w_cost_ddl = 0;
    double w_cost_dddl = 0;
    double w_cost_ref = 0;
    double w_cost_center = 0;

    double w_cost_end_l = 0;
    double w_cost_end_dl = 0;
    double w_cost_end_ddl = 0;

    int obs_width = 0;
    int obs_length = 0;

    double host_d1 = 0;
    double host_d2 = 0;
    double host_w = 0;

    double delta_dl_max = 0;
    double delta_ddl_max = 0;
    double dl_max = 0;
    double ddl_max = 0;

    int size = 0;

}QPConfig;