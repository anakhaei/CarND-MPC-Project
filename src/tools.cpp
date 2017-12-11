#include "tools.h"

void GlobalToLocal(const double &veh_x, const double &veh_y,
                   const double &veh_psi, const double &in_x,
                   const double &in_y, double &out_x, double &out_y)
{
  //   double delta_x = in_x - veh_x;
  //   double delta_y = in_y - veh_y;
  //   rotate(delta_x, delta_y, out_x, out_y, -veh_psi);
  //initial Implementation
  // double s = sin(veh_psi);
  // double c = cos(veh_psi);

  // out_x= in_x*c+in_y*s -veh_x*c - veh_y*s;
  // out_y= -in_x*s + in_y*c + veh_x*s - veh_y*c;

  //Updated:
  const double cospsi = cos(-veh_psi);
  const double sinpsi = sin(-veh_psi);
  const double dx = in_x - veh_x;
  const double dy = in_y - veh_y;
  out_x = dx * cospsi - dy * sinpsi;
  out_y = dy * cospsi + dx * sinpsi;
}

void GlobalToLocal(const double &veh_x, const double &veh_y,
                   const double &veh_psi, const std::vector<double> &in_x,
                   const std::vector<double> &in_y, std::vector<double> &out_x,
                   std::vector<double> &out_y)
{
  out_x.resize(in_x.size());
  out_y.resize(in_x.size());
  assert(in_x.size() == in_y.size() && in_x.size() == out_x.size() &&
         out_x.size() == out_y.size());
  for (size_t i = 0; i < in_x.size(); i++)
  {
    GlobalToLocal(veh_x, veh_y, veh_psi, in_x[i], in_y[i], out_x[i], out_y[i]);
  }
}

void LocalToGlobal(const double &veh_x, const double &veh_y,
                   const double &veh_psi, const double &in_x,
                   const double &in_y, double &out_x, double &out_y)
{
  //   double delta_x = in_x - veh_x;
  //   double delta_y = in_y - veh_y;
  //   rotate(delta_x, delta_y, out_x, out_y, -veh_psi);
  double s = sin(veh_psi);
  double c = cos(veh_psi);

  out_x = in_x * c - in_y * s + veh_x;
  out_y = in_x * s + in_y * c + veh_y;
}

void LocalToGlobal(const double &veh_x, const double &veh_y,
                   const double &veh_psi, const std::vector<double> &in_x,
                   const std::vector<double> &in_y, std::vector<double> &out_x,
                   std::vector<double> &out_y)
{
  out_x.resize(in_x.size());
  out_y.resize(in_x.size());
  assert(in_x.size() == in_y.size() && in_x.size() == out_x.size() &&
         out_x.size() == out_y.size());
  for (size_t i = 0; i < in_x.size(); i++)
  {
    LocalToGlobal(veh_x, veh_y, veh_psi, in_x[i], in_y[i], out_x[i], out_y[i]);
  }
}

// Evaluate a polynomial.
CppAD::AD<double> polyeval(Eigen::VectorXd coeffs, CppAD::AD<double> x)
{
  CppAD::AD<double> result = 0.0;
  for (int i = 0; i < coeffs.size(); i++)
  {
    result += coeffs[i] * CppAD::pow(x, i);
  }
  return result;
}
double polyeval(Eigen::VectorXd coeffs, double x)
{
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++)
  {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order)
{
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++)
  {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++)
  {
    for (int i = 0; i < order; i++)
    {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

CppAD::AD<double> polyslope(Eigen::VectorXd coeffs, CppAD::AD<double> x)
{
  CppAD::AD<double> result = 0.0;
  for (int i = 1; i < coeffs.size(); i++)
  {
    result += i * coeffs[i] * pow(x, i - 1);
  }
  return result;
}

CppAD::AD<double> polyCurvature(Eigen::VectorXd coeffs, CppAD::AD<double> x)
{
  CppAD::AD<double> result = 2 * coeffs[2] + 6 * coeffs[3] * x;
  return result;
}

double polyCurvature(Eigen::VectorXd coeffs, double x)
{
  double result = 2 * coeffs[2] + 6 * coeffs[3] * x;
  return result;
}

// double velocityRef(Eigen::VectorXd coeffs, double x){
//   double curvature = std::abs(polyCurvature(coeffs, x));
//   double c_max = 0.25;
//   double c_min = 0.001;
//   double v_min = 25 * 0.447;
//   double v_max = 60 * 0.447;
//   //double ref_velocity = (((v_max-v_min)/(c_min-c_max)*(curvature-c_max)+v_min))/0.447;
//   double ref_velocity = 19.4974 * std::pow (curvature, - 0.13904) ;
//   //std::cout << curvature << " ' " << ref_velocity<< std::endl;
//   return ref_velocity;
// }

CppAD::AD<double> velocityRef(Eigen::VectorXd coeffs, CppAD::AD<double> x)
{
  //double curvature = CppAD::abs(polyCurvature(coeffs, x));
  //   // double c_max = 0.25;
  //   // double c_min = 0.001;
  //   // double v_min = 25 * 0.447;
  //   // double v_max = 60 * 0.447;
  //   //double ref_velocity = (((v_max-v_min)/(c_min-c_max)*(curvature-c_max)+v_min))/0.447;
  CppAD::AD<double> ref_velocity = 19.4974 * CppAD::pow(CppAD::abs(polyCurvature(coeffs, x)), -0.13904);
  return ref_velocity;
}
