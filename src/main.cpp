#include <uWS/uWS.h>
#include <iostream>
#include <cstdio>
#include "json.hpp"
#include "PID.h"
#include <math.h>

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

double p[3] = {0.1, 0.001, 4};
double dp[3] = {0.05, 0.0005, 0.5};
bool twiddle_flag = false;
bool twiddle_init = false;
int twiddle_state;
int twiddle_idx;
int iteration;
double best_error;

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

void twiddle(double error) {
  if (twiddle_state == 1) {
    p[twiddle_idx] += dp[twiddle_idx];
    twiddle_state = 2;
  } else if (twiddle_state == 2) {
    if (best_error > error) {
      best_error = error;
      dp[twiddle_idx] *= 1.1;
      twiddle_idx = (twiddle_idx + 1) % 3;
      p[twiddle_idx] += dp[twiddle_idx];
      twiddle_state = 2;
    } else {
      p[twiddle_idx] -= dp[twiddle_idx] * 2;
      twiddle_state = 3;
    }
  } else if (twiddle_state == 3) {
    if (best_error > error) {
      best_error = error;
      dp[twiddle_idx] *= 1.1;
      twiddle_idx = (twiddle_idx + 1) % 3;
      p[twiddle_idx] += dp[twiddle_idx];
      twiddle_state = 2;
    } else {
      p[twiddle_idx] += dp[twiddle_idx];
      dp[twiddle_idx] *= 0.9;
      twiddle_idx = (twiddle_idx + 1) % 3;
      p[twiddle_idx] += dp[twiddle_idx];
      twiddle_state = 2;
    }
  }
}

int main()
{
  uWS::Hub h;

  PID pid;
  // TODO: Initialize the pid variable.
  pid.Init(0.1, 0.001, 5.0);

  h.onMessage([&pid](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          if (twiddle_flag) {
            iteration++;
            if (!twiddle_init) {
                twiddle_init = true;
                iteration = 0;
                best_error = 1e8;
                twiddle_state = 1;
                twiddle_idx = 0;
                pid.Init(p[0], p[1], p[2]);
                printf("%lf %lf %lf %lf\n", cte, p[0], p[1], p[2]);
            } else {
              if (cte >= 20 || (iteration > 100 && speed < 5)) {
                std::string msg = "42[\"reset\", {}]";
                std::cout << msg << std::endl;
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
                // best_error = 1e8;
                iteration = 0;
              }
                
              if (iteration >= 300) {
                iteration = 0;
                printf("%lf %lf %lf %lf %lf %lf %lf\n", cte, p[0], p[1], p[2], dp[0],dp[1],dp[2]);
                twiddle(std::abs(cte));
                pid.Init(p[0], p[1], p[2]);
                // pid.Init(p[0], p[1], p[2]);
              }
            }
                     
          }
          
          // DEBUG
          // std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          // std::cout << msg << std::endl;
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
