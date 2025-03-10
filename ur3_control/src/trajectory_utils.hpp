#ifndef TRAJECTORY_UTILS_HPP
#define TRAJECTORY_UTILS_HPP

#include <vector>
#include <array>

std::array<double, 6> computeQuinticCoefficients(double t0, double t1, double p0, double p1, double v0, double v1, double a0, double a1);
double evaluateQuintic(double t, double t0, double t1, std::array<double, 6> coeffs);
double trapezoidalVelocityProfile(double t, double t_total, double v_max, double a_max);

#endif