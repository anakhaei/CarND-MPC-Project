#include "tools.h"

void rotate(const double &in_x, const double &in_y, double &out_x,
            double &out_y, const double &theta) {
  double s = sin(theta);
  double c = cos(theta);

  out_x = in_x * c - in_y * s;
  out_y = in_x * s + in_y * c;
}

void GlobalToLocal(const double &veh_x, const double &veh_y,
                   const double &veh_psi, const double &in_x,
                   const double &in_y, double &out_x, double &out_y) {
//   double delta_x = in_x - veh_x;
//   double delta_y = in_y - veh_y;
//   rotate(delta_x, delta_y, out_x, out_y, -veh_psi);
  double s = sin(veh_psi);
  double c = cos(veh_psi);

  out_x= in_x*c+in_y*s -veh_x*c - veh_y*s;
  out_y= -in_x*s + in_y*c + veh_x*s - veh_y*c;
}

void GlobalToLocal(const double &veh_x, const double &veh_y,
                   const double &veh_psi, const std::vector<double> &in_x,
                   const std::vector<double> &in_y, std::vector<double> &out_x,
                   std::vector<double> &out_y) {
  out_x.resize(in_x.size());
  out_y.resize(in_x.size());
  assert(in_x.size() == in_y.size() && in_x.size() == out_x.size() &&
         out_x.size() == out_y.size());
  for (size_t i = 0; i < in_x.size(); i++) {
    GlobalToLocal(veh_x, veh_y, veh_psi, in_x[i], in_y[i], out_x[i], out_y[i]);
  }
}