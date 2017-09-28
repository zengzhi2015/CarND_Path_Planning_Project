//
// Created by zhi on 9/28/17.
//

#ifndef PATH_PLANNING_JMT_H
#define PATH_PLANNING_JMT_H

#include <cmath>
#include <vector>
#include "Eigen-3.3/Eigen/Dense"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

class JMT {
private:
    vector<double> coefficients;
    vector<double> d_coefficients;
    vector<double> dd_coefficients;
    vector<double> ddd_coefficients;
public:
    void cal_coefficients(vector< double> start, vector <double> end, double T);
    double F(double x); // output the result of interpolation
    double dF(double x); // first derivative;
    double ddF(double x);
    double dddF(double x);
};


#endif //PATH_PLANNING_JMT_H
