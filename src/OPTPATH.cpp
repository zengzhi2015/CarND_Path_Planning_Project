//
// Created by zhi on 10/1/17.
//

#include "OPTPATH.h"

double OPTPATH::vmax_at_T(double ego_s_at_T, double ego_d_at_T, double T, vector<vector<double >> sensor_fusion) {
    double min_distance = 9999999.0;
    double nearest_car_velocity = 47.0/2.24; // initial value is the speed limit
    for(auto vehical_info: sensor_fusion) {
        double v_veh = sqrt(vehical_info[3]*vehical_info[3]+vehical_info[4]*vehical_info[4]);
        double s_veh = vehical_info[5] + v_veh*T;
        double d_veh = vehical_info[6];
        if(abs(ego_d_at_T-d_veh)<2 & ego_s_at_T<s_veh & s_veh-ego_s_at_T<30 & s_veh-ego_s_at_T<min_distance) {
            // if the two cars car near enough
            min_distance = s_veh-ego_s_at_T;
            nearest_car_velocity = min(v_veh,nearest_car_velocity);
        }
    }
    return nearest_car_velocity;
}

void OPTPATH::cal_optimal_path(vector<double> &start_s, vector<double> &start_d, double T,
                               vector<vector<double >> sensor_fusion, JMT &opt_jmt_s, JMT &opt_jmt_d) {
    // initialization
    vector<double> end_p_s;
    for(double s=start_s[0];s<start_s[0]+T*50/2.24;s+=3) {
        end_p_s.push_back(s);
    }
    vector<double> end_p_d = {2,6,10};

    // optimization
    double min_cost = 9999999;
    vector<double> opt_end_s;
    vector<double> opt_end_d;

    for(auto p_d: end_p_d) {
        for(auto p_s: end_p_s) {
            double max_end_v_s = vmax_at_T(p_s,p_d,T,sensor_fusion);
            //vector<double> end_v_s = {max_end_v_s,max_end_v_s*0.9,max_end_v_s*0.8,max_end_v_s*0.7, max_end_v_s*0.6,max_end_v_s*0.5,max_end_v_s*0.4, max_end_v_s*0.3};
            vector<double> end_v_s = {max_end_v_s,max_end_v_s*0.95,max_end_v_s*0.75,max_end_v_s*0.5};
            for(auto v_s: end_v_s) {
                vector<double> temp_end_s = {p_s,v_s,0.0};
                vector<double> temp_end_d = {p_d,0.0,0.0};
                JMT temp_jmt_s, temp_jmt_d;
                temp_jmt_s.cal_coefficients(start_s,temp_end_s,T);
                temp_jmt_d.cal_coefficients(start_d,temp_end_d,T);
                COST cost;
                double temp_cost = cost.total_cost(temp_jmt_s,temp_jmt_d,T,sensor_fusion);
                if(temp_cost<min_cost) {
                    min_cost = temp_cost;
                    opt_end_s.clear();
                    opt_end_s.assign(temp_end_s.begin(),temp_end_s.end());
                    opt_end_d.clear();
                    opt_end_d.assign(temp_end_d.begin(),temp_end_d.end());
                }
            }
        }
    }

    opt_jmt_s.cal_coefficients(start_s,opt_end_s,T);
    opt_jmt_d.cal_coefficients(start_d,opt_end_d,T);
}