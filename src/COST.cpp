//
// Created by zhi on 9/30/17.
//

#include "COST.h"
#include <iostream>

double COST::collision_cost(JMT &jmt_s,JMT &jmt_d, double T, vector<vector<double >> sensor_fusion) {
    double min_distence = 999999;
    for(double t=0;t<T;t+=0.02){
        double s_ego = jmt_s.F(t);
        double d_ego = jmt_d.F(t);
        // check whether crash with other cars
        for(auto vehical_info: sensor_fusion) {
            double v_veh = sqrt(vehical_info[3]*vehical_info[3]+vehical_info[4]*vehical_info[4]);
            double s_veh = vehical_info[5] + v_veh*t;
            double d_veh = vehical_info[6];
            if(abs(d_ego-d_veh)<2 & abs(s_ego-s_veh)<min_distence) {
                min_distence = abs(s_ego-s_veh);
            }
        }
    }
    if(min_distence<30) {
        return 5.0/min_distence;
    }
    return 0.0;
}

double COST::speed_cost(JMT &jmt_s, double T) {
    for(double t=0;t<T;t+=0.02){
        if(jmt_s.dF(t)>48.0/2.24 | jmt_s.dF(t)<0) {
            return 1.0;
        }
    }
    return 0.0;
}

double COST::acceleration_cost(JMT &jmt_s, double T) {
    for(double t=0;t<T;t+=0.02){
        if(abs(jmt_s.ddF(t))>9.0) {
            return 1.0;
        }
    }
    return 0.0;
}

double COST::jerk_cost(JMT &jmt_s, double T) {
    for(double t=0;t<T;t+=0.02){
        if(abs(jmt_s.dddF(t))>9.0) {
            return 1.0;
        }
    }
    return 0.0;
}

double COST::efficiency_cost(JMT &jmt_s, double T) {
    return abs(T*48.0/2.24+jmt_s.F(0)-jmt_s.F(T))/(T*48.0/2.24);
}

double COST::total_cost(JMT &jmt_s, JMT &jmt_d, double T, vector<vector<double >> sensor_fusion) {
    return collision_cost(jmt_s, jmt_d, T, sensor_fusion) * 10 +
            speed_cost(jmt_s,T)*10 +
            acceleration_cost(jmt_s,T)*10 +
            jerk_cost(jmt_s,T)*10 +
            efficiency_cost(jmt_s,T);
}