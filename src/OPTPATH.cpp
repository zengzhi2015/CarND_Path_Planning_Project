//
// Created by zhi on 10/1/17.
//

#include "OPTPATH.h"

double OPTPATH::vmax_at_T(double ego_s_at_T, double ego_d_at_T, double T, vector<vector<double >> sensor_fusion) {
    double min_distance = 9999999.0;
    double nearest_car_velocity = 49.0/2.24; // initial value is the speed limit
    for(auto vehical_info: sensor_fusion) {
        double v_veh = sqrt(vehical_info[3]*vehical_info[3]+vehical_info[4]*vehical_info[4]);
        double s_veh = vehical_info[5] + v_veh*T;
        double d_veh = vehical_info[6];
        if(abs(ego_d_at_T-d_veh)<1 & ego_s_at_T<s_veh & s_veh-ego_s_at_T<50 & s_veh-ego_s_at_T<min_distance) {
            // if the two cars car near enough
            min_distance = s_veh-ego_s_at_T;
            nearest_car_velocity = min(v_veh,nearest_car_velocity);
        }
    }
    return nearest_car_velocity;
}

void OPTPATH::cal_optimal_path(vector<double> &start_s, vector<double> &start_d, double T,
                               vector<vector<double >> sensor_fusion, JMT &opt_jmt_s, JMT &opt_jmt_d) {
    ;
}