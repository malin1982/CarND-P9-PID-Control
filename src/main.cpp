#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>

using namespace std;
double p[3] = {0.0, 0.0, 0.0};

// twiddle as described in Sebastian's
// AI for Robotics
void twiddle(PID pid, double cte) {
  double dp[3] = {1.0, 1.0, 1.0};
  double tolerance = 0.0001;
  double best_err = abs(pid.TotalError());
  double err = 0.0;

  double sum = dp[0]+dp[1]+dp[2];
  //iteration counter
  int iter = 0;

  while (sum > tolerance) {
    for ( int i = 0; i < 3; i++) {
      p[i] += dp[i];
      pid.Kp = p[0];
      pid.Ki = p[1];
      pid.Kd = p[2];
      pid.UpdateError(cte);
      err = abs(pid.TotalError());

      if (err < best_err) {
        best_err = err;
        dp[i] *= 1.1;
      }
      else {
        p[i] -= 2 * dp[i];
        pid.Kp = p[0];
        pid.Ki = p[1];
        pid.Kd = p[2];
        pid.UpdateError(cte);
        err = abs(pid.TotalError());

        if (err < best_err) {
          best_err = err;
          dp[i] *= 1.1;
        }
        else {
          p[i] += dp[i];
          dp[i] *= 0.9;
        }
      }
    }
    sum = dp[0] + dp[1] + dp[2];
    iter += 1;
  }
}

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main()
{
  uWS::Hub h;

  //initial PID values found using Twiddle
  PID steer_pid;
  steer_pid.Init(0.11, 0.002, 1.5);

  PID speed_pid;
  speed_pid.Init(0.13, 0.0001, 0.8000);

  h.onMessage([&steer_pid, &speed_pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double throttle_value;

          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          /*
          * Calcuate steering value here, remember the steering value is [-1, 1].
          */

          steer_pid.UpdateError(cte);
          twiddle(steer_pid, cte);
          steer_value = steer_pid.TotalError();
          if (steer_value < -1) {
            steer_value = -1;
          } else if (steer_value > 1) {
            steer_value = 1;
          }
          double targetSpeed = 30.*(1.-std::abs(steer_value)) + 20.;
          speed_pid.UpdateError(speed - targetSpeed);
          throttle_value = speed_pid.TotalError();
          if (throttle_value < -1) {
            throttle_value = -1;
          } else if (throttle_value > 1) {
            throttle_value = 1;
          }
          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << "throttle Value: " << throttle_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
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

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
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
