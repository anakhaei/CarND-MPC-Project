#ifndef TOOLS_H
#define TOOLS_H
#include <cppad/cppad.hpp>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <iostream>


// void rotate(const double& in_x, const double& in_y, double& out_x,
//             double& out_y, const double& theta);

void GlobalToLocal(const double& veh_x, const double& veh_y,
                   const double& veh_psi, const double& in_x,
                   const double& in_y, double& out_x, double& out_y);

void GlobalToLocal(const double& veh_x, const double& veh_y,
                   const double& veh_psi, const std::vector<double>& in_x,
                   const std::vector<double>& in_y, std::vector<double>& out_x,
std::vector<double>& out_y);

void LocalToGlobal(const double& veh_x, const double& veh_y,
                   const double& veh_psi, const double& in_x,
                   const double& in_y, double& out_x, double& out_y);

void LocalToGlobal(const double& veh_x, const double& veh_y,
                   const double& veh_psi, const std::vector<double>& in_x,
                   const std::vector<double>& in_y, std::vector<double>& out_x,
std::vector<double>& out_y);

double polyeval(Eigen::VectorXd coeffs, double x);
CppAD::AD<double> polyeval(Eigen::VectorXd coeffs, CppAD::AD<double> x);

CppAD::AD<double>  polyslope(Eigen::VectorXd coeffs, CppAD::AD<double> x);

CppAD::AD<double>  polyCurvature(Eigen::VectorXd coeffs, CppAD::AD<double> x);

double  polyCurvature(Eigen::VectorXd coeffs, double x);

Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order);

// double velocityRef(Eigen::VectorXd coeffs, double x);

CppAD::AD<double> velocityRef(Eigen::VectorXd coeffs, CppAD::AD<double> x);
//CppAD::AD<double>  polyCurvatureVel(Eigen::VectorXd coeffs, CppAD::AD<double> x);

#endif