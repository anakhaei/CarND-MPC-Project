#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "tools.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos)
  {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos)
  {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
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

int main()
{
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2')
    {
      string s = hasData(sdata);
      if (s != "")
      {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry")
        {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];

          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];

          cout << "px="<<px<<", py="<< py<<", psi=" <<psi << ", v=" <<v<< ", steer_value" <<steer_value  << ", throttle_value" << throttle_value << endl;

          //converting mph to meter_p_s
          double v_m_per_s=v* 0.44704;

          // Predict state after latency
          double latency = 0.1;
          double pred_px = px + v_m_per_s * CppAD::cos(psi) * latency;
          double pred_py = py + v_m_per_s * CppAD::sin(psi) * latency;
          double pred_psi = psi + v_m_per_s * steer_value / Lf * latency; 
          double pred_v = v_m_per_s + throttle_value * latency; 

          // The cross track error is calculated by evaluating at polynomial at x, f(x)
          // and subtracting y.

          //converting global reference to local
          vector<double> ptsx_local;
          vector<double> ptsy_local;

          GlobalToLocal(pred_px, pred_py, pred_psi, ptsx, ptsy, ptsx_local, ptsy_local);

          Eigen::VectorXd ptsx_eig_local(ptsx.size());
          Eigen::VectorXd ptsy_eig_local(ptsx.size());

          for (unsigned int i = 0; i < ptsx.size(); i++)
          {
            ptsx_eig_local[i] = ptsx_local[i];
            ptsy_eig_local[i] = ptsy_local[i];
          }

          // The polynomial is fitted to a straight line so a polynomial with
          // order 1 is sufficient.
          auto coeffs = polyfit(ptsx_eig_local, ptsy_eig_local, 3);

          //converting global telemetry to local:
          double pred_px_local = 0;
          double pred_py_local = 0;
          double pred_psi_local = 0;


          double pred_cte = polyeval(coeffs, pred_px_local) - pred_py_local;

          // Due to the sign starting at 0, the orientation error is -f'(x). (toBeChecked)
          // derivative of coeffs[0] + coeffs[1] * x -> coeffs[1]
          double pred_epsi = pred_psi_local - atan(coeffs[1]);

          Eigen::VectorXd state(6);

          state << pred_px_local, pred_py_local, pred_psi_local, pred_v, pred_cte, pred_epsi;

          // cout << "ref path:"<< endl;
          // for (unsigned int i=0; i< ptsx_eig_local.size();  i++){
          //   cout << i << " :x =  " << ptsx_eig_local[i] << " ,y= " << ptsy_eig_local[i] << endl;
          // }

          auto vars = mpc.Solve(state, coeffs);

          std::vector<double> mpc_x;
          std::vector<double> mpc_y;
          std::vector<double> mpc_psi;
          std::vector<double> mpc_v;
          std::vector<double> mpc_delta;
          std::vector<double> mpc_a;

          //To be update
          size_t N = 40;
          size_t x_start = 0;
          size_t y_start = x_start + N;
          size_t psi_start = y_start + N;
          size_t v_start = psi_start + N;
          size_t cte_start = v_start + N;
          size_t epsi_start = cte_start + N;
          size_t delta_start = epsi_start + N;
          size_t a_start = delta_start + N - 1;

          for (unsigned int i = 0; i < N; i++)
          {
            mpc_x.push_back(vars[x_start + i]);
            mpc_y.push_back(vars[y_start + i]);
            mpc_psi.push_back(vars[psi_start + i]);
            mpc_v.push_back(vars[v_start + i]);
            if (i < N - 1)
            {
              mpc_delta.push_back(vars[delta_start + i]);
              mpc_a.push_back(vars[a_start + i]);
            }
          }

          // cout << "result path [x, y]:"<< endl;


          // for (unsigned int i=0; i< mpc_x.size();  i++){
          //   cout << i << " :x= " << mpc_x[i] << " ,y= " << mpc_y[i] << endl;
          // }

          // for (unsigned int i=0; i< mpc_delta.size();  i++){
          //   cout << i << " : delta:= " << mpc_delta[i]/deg2rad(25) << " , acc=" << mpc_a[i] << endl;
          // }

          //steer_value = -1 * mpc_delta[0] / deg2rad(25);
          //throttle_value = mpc_a[0];
          steer_value = 0;
          throttle_value = 0.1;

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].

          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (int i = 0; i < mpc_x.size(); i++)
          {
            mpc_x_vals.push_back(mpc_x[i]);
            mpc_y_vals.push_back(mpc_y[i]);
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          next_x_vals.resize(ptsx.size());
          next_y_vals.resize(ptsx.size());

          for (unsigned int i = 0; i < ptsx.size(); i++)
          {
            next_x_vals[i] = ptsx_local[i];
            next_y_vals[i] = ptsy_local[i];
          }

          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          //std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
