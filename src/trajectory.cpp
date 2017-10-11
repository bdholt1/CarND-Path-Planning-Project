#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <vector>

using namespace Eigen;
using namespace std;

vector<double> JMT(vector< double> start, vector <double> end, double T)
{
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
    */

    vector<double> coef = {start[0], start[1], 0.5 * start[2], 0, 0, 0};

    double T2 = T*T;
    double T3 = T*T*T;
    double T4 = T*T*T*T;
    double T5 = T*T*T*T*T;

    MatrixXd A(3,3);
    A << T3, T4, T5,
         3*T2, 4*T3, 5*T4,
         6*T, 12*T2, 20*T3;

    VectorXd b(3);
    b[0] = end[0] - (start[0] + start[1]*T + 0.5*start[2]*T2);
    b[1] = end[1] - (start[1] + start[2]*T);
    b[2] = end[2] - start[2];

    VectorXd x = A.colPivHouseholderQr().solve(b);

    coef[3] = x[0];
    coef[4] = x[1];
    coef[5] = x[2];

    return coef;

}