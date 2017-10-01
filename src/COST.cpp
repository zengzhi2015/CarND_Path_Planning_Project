//
// Created by zhi on 9/30/17.
//

#include "COST.h"

//double COST::collision_cost(vector<double> path_points_s, vector<double> path_points_d,
//                            vector<vector<double >> sensor_fusion) {
//    double delta_t = 0.02;
//    for(int i=0;i<path_points_s.size();i++){
//        double s_ego = path_points_s[i];
//        double d_ego = path_points_d[i];
//        for(auto vehical_info: sensor_fusion) {
//            double v_veh = sqrt(sensor_fusion[3]*sensor_fusion[3]+sensor_fusion[4]*sensor_fusion[4]);
//            double s_veh = sensor_fusion[5] + v_veh*(double)i*delta_t;
//            double d_veh = sensor_fusion[6];
//        }
//    }
//    return 0;
//}

double COST::collision_cost(JMT &jmt_s,JMT &jmt_d, double T, vector<vector<double >> sensor_fusion) {
    for(double t=0;t<T;t+=0.02){
        double s_ego = jmt_s.F(t);
        double d_ego = jmt_d.F(t);
        for(auto vehical_info: sensor_fusion) {
            double v_veh = sqrt(vehical_info[3]*vehical_info[3]+vehical_info[4]*vehical_info[4]);
            double s_veh = vehical_info[5] + v_veh*(double)t;
            double d_veh = vehical_info[6];
            if(abs(d_ego-d_veh)<2 & abs(s_ego-s_veh)<4) {
                return 1.0;
            }
        }
    }
    return 0.0;
}

double COST::speed_cost(JMT &jmt_s, double T) {
    for(double t=0;t<T;t+=0.02){
        if(jmt_s.dF(t)>49.0/2.24) {
            return 1.0;
        }
    }
    return 0.0;
}

double COST::acceleration_cost(JMT &jmt_s, double T) {
    for(double t=0;t<T;t+=0.02){
        if(jmt_s.ddF(t)>10.0/2.24) {
            return 1.0;
        }
    }
    return 0.0;
}

double COST::jerk_cost(JMT &jmt_s, double T) {
    for(double t=0;t<T;t+=0.02){
        if(jmt_s.dddF(t)>10.0/2.24) {
            return 1.0;
        }
    }
    return 0.0;
}

double COST::efficiency_cost(JMT &jmt_s, double T) {
    return (T*49.0/2.24-jmt_s.F(T))/(T*49.0/2.24);
}