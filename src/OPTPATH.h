//
// Created by zhi on 10/1/17.
//

#ifndef PATH_PLANNING_OPTPATH_H
#define PATH_PLANNING_OPTPATH_H

#include "JMT.h"
#include "COST.h"

class OPTPATH {
public:
    // Calculate the maximum speed given the position of the ego car at T seconds later
    double vmax_at_T(double ego_s_at_T, double ego_d_at_T, double T, vector<vector<double >> sensor_fusion);
public:
    void cal_optimal_path(vector<double> &start_s, vector<double> &start_d, double T,
                          vector<vector<double >> sensor_fusion,
                          JMT &opt_jmt_s, JMT &opt_jmt_d);
};


#endif //PATH_PLANNING_OPTPATH_H
