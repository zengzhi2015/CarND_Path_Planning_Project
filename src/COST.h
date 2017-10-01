//
// Created by zhi on 9/30/17.
//

#ifndef PATH_PLANNING_COST_H
#define PATH_PLANNING_COST_H

#include <math.h>
#include <vector>
#include "JMT.h"

using namespace std;

class COST {
public:
    double collision_cost(JMT &jmt_s,JMT &jmt_d, double T, vector<vector<double >> sensor_fusion);
    double speed_cost(JMT &jmt_s,double T);
    double acceleration_cost(JMT &jmt_s,double T);
    double jerk_cost(JMT &jmt_s,double T);
    double efficiency_cost(JMT &jmt_s,double T);
};


#endif //PATH_PLANNING_COST_H
