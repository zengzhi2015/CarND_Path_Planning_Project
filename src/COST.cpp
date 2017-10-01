//
// Created by zhi on 9/30/17.
//

#include "COST.h"

double COST::collision_cost(JMT &jmt_s,JMT &jmt_d, double T, vector<vector<double >> sensor_fusion) {
    for(double t=0;t<T;t+=0.02){
        double s_ego = jmt_s.F(t);
        double d_ego = jmt_d.F(t);
        if(d_ego < 1 | d_ego>11) {
            return 1.0;
        }
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
        if(jmt_s.ddF(t)>10.0) {
            return 1.0;
        }
    }
    return 0.0;
}

double COST::jerk_cost(JMT &jmt_s, double T) {
    for(double t=0;t<T;t+=0.02){
        if(jmt_s.dddF(t)>10.0) {
            return 1.0;
        }
    }
    return 0.0;
}

double COST::efficiency_cost(JMT &jmt_s, double T) {
    return abs(T*49.0/2.24+jmt_s.F(0)-jmt_s.F(T))/(T*49.0/2.24);
}

double COST::total_cost(JMT &jmt_s, JMT &jmt_d, double T, vector<vector<double >> sensor_fusion) {
    return collision_cost(jmt_s, jmt_d, T, sensor_fusion) * 10 +
            speed_cost(jmt_s,T)*10 +
            acceleration_cost(jmt_s,T)*10 +
            jerk_cost(jmt_s,T)*10 +
            efficiency_cost(jmt_s,T);
}