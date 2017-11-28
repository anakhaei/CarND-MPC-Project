#ifndef TOOLS_H
#define TOOLS_H
#include <cppad/cppad.hpp>
#include <vector>


void rotate(const double& in_x, const double& in_y, double& out_x,
            double& out_y, const double& theta);

void GlobalToLocal(const double& veh_x, const double& veh_y,
                   const double& veh_psi, const double& in_x,
                   const double& in_y, double& out_x, double& out_y);

void GlobalToLocal(const double& veh_x, const double& veh_y,
                   const double& veh_psi, const std::vector<double>& in_x,
                   const std::vector<double>& in_y, std::vector<double>& out_x,
std::vector<double>& out_y);

#endif