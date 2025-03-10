#include "trajectory_utils.hpp"
#include <Eigen/Dense>

// Compute Quintic Polynomial Coefficients
std::array<double, 6> computeQuinticCoefficients(double t0, double t1, double p0, double p1, double v0, double v1, double a0, double a1) {
    Eigen::Matrix<double, 6, 6> A;
    Eigen::Matrix<double, 6, 1> B;

    A << 1, t0, pow(t0, 2), pow(t0, 3), pow(t0, 4), pow(t0, 5),
         0, 1, 2*t0, 3*pow(t0, 2), 4*pow(t0, 3), 5*pow(t0, 4),
         0, 0, 2, 6*t0, 12*pow(t0, 2), 20*pow(t0, 3),
         1, t1, pow(t1, 2), pow(t1, 3), pow(t1, 4), pow(t1, 5),
         0, 1, 2*t1, 3*pow(t1, 2), 4*pow(t1, 3), 5*pow(t1, 4),
         0, 0, 2, 6*t1, 12*pow(t1, 2), 20*pow(t1, 3);

    B << p0, v0, a0, p1, v1, a1;

    Eigen::Matrix<double, 6, 1> coeffs = A.colPivHouseholderQr().solve(B);
    return {coeffs(0), coeffs(1), coeffs(2), coeffs(3), coeffs(4), coeffs(5)};
}

// Evaluate Quintic Polynomial at a given time t
double evaluateQuintic(double t, double t0, double t1, std::array<double, 6> coeffs) {
    double time = t - t0;
    return coeffs[0] + coeffs[1] * time + coeffs[2] * pow(time, 2) +
           coeffs[3] * pow(time, 3) + coeffs[4] * pow(time, 4) + coeffs[5] * pow(time, 5);
}

// Trapezoidal Velocity Profile
double trapezoidalVelocityProfile(double t, double t_total, double v_max, double a_max) {
    double t_accel = v_max / a_max;
    double t_decel = t_total - t_accel;
    
    if (t < t_accel)
        return a_max * t; // Acceleration phase
    else if (t < t_decel)
        return v_max; // Constant velocity phase
    else
        return v_max - a_max * (t - t_decel); // Deceleration phase
}