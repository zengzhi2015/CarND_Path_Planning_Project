//
// Created by zhi on 9/28/17.
//

#include "JMT.h"

void JMT::cal_coefficients(vector<double> start, vector<double> end, double T) {
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT
    an array of length 6, each value corresponding to a coefficent in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > cal_coefficients( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
*/

    MatrixXd A = MatrixXd(3, 3);
    A << T*T*T, T*T*T*T, T*T*T*T*T,
            3*T*T, 4*T*T*T,5*T*T*T*T,
            6*T, 12*T*T, 20*T*T*T;

    MatrixXd B = MatrixXd(3,1);
    B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
            end[1]-(start[1]+start[2]*T),
            end[2]-start[2];

    MatrixXd Ai = A.inverse();

    MatrixXd C = Ai*B;

    coefficients.clear();
    coefficients = {start[0], start[1], .5*start[2]};
    for(int i = 0; i < C.size(); i++)
    {
        coefficients.push_back(C.data()[i]);
    }

    // calculate coefficients of differentiated polynomials
    d_coefficients.clear();
    for(int i = 1; i < coefficients.size(); i++) {
        d_coefficients.push_back(coefficients[i]*(double)i);
    }
    dd_coefficients.clear();
    for(int i = 1; i < d_coefficients.size(); i++) {
        dd_coefficients.push_back(d_coefficients[i]*(double)i);
    }
    ddd_coefficients.clear();
    for(int i = 1; i < dd_coefficients.size(); i++) {
        ddd_coefficients.push_back(dd_coefficients[i]*(double)i);
    }
}

double JMT::F(double x) {
    double result = 0;
    for(int i=0;i<coefficients.size();i++) {
        result += coefficients[i]*pow(x,i);
    }
    return result;
}

double JMT::dF(double x) {
    double result = 0;
    for(int i=0;i<d_coefficients.size();i++) {
        result += d_coefficients[i]*pow(x,i);
    }
    return result;
}

double JMT::ddF(double x) {
    double result = 0;
    for(int i=0;i<dd_coefficients.size();i++) {
        result += dd_coefficients[i]*pow(x,i);
    }
    return result;
}

double JMT::dddF(double x) {
    double result = 0;
    for(int i=0;i<ddd_coefficients.size();i++) {
        result += ddd_coefficients[i]*pow(x,i);
    }
    return result;
}
