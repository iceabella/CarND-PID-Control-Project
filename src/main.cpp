#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <chrono>
#include <ctime>

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

  PID pid;
  // TODO: Initialize the pid variable.
  pid.Init(0.05,0,0,0.1,0,0);
  //bool prev_meas = false;
  
  // initialize variable to store previous time stamp
  std::chrono::time_point<std::chrono::system_clock> prev_time;
  
  // define constant used for steering angle calculation
  const double max_steer = 1.0;
  // define variables used for calculation of steering angle         
  //double sum_cte; 
  //double prev_cte;    
  
  // define constants used for throttle calculation
  const double v_nominal = 20;
  const double max_throttle = 0.3;
  // define variables used to store previous data for throttle calculation
  //double v_sum_cte; 
  //double v_prev_cte;

  h.onMessage([&pid, &prev_time, &max_steer, &v_nominal, &max_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
          
          
          // Initialize values          
          if(!pid.prev_meas_){
            pid.s_prev_cte_ = cte;
            // define start time as now
            prev_time = std::chrono::system_clock::now();
            pid.v_prev_cte_ = v_nominal - speed; 
            
            // we have got our first data
            pid.prev_meas_ = true; 
          } 
          // Get time interval  
          std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now()-prev_time;
          double dt = elapsed_seconds.count();
          
          std::cout << "dt = " << dt << std::endl;
          
          // save time stamp of this data cte to be used in next cycle
          prev_time = std::chrono::system_clock::now();
          
          // Error between current and nominal speed       
          double v_cte = speed - v_nominal;
          
          // update errors for steering and throttle
          pid.UpdateError(cte, v_cte, dt);
          // get total steering error
          double steer_value = pid.TotalSteeringError();
 
          // Make sure steering values are within min and max limit
          if(steer_value > max_steer)
            steer_value = max_steer;
          if(steer_value < -max_steer)
            steer_value = -max_steer;
            
          // get total throttle error
          double throttle_value = pid.TotalThrottleError();
          
          // Make sure trottle values are within min and max limit
          if(throttle_value > max_throttle)
            throttle_value = max_throttle;
          if(throttle_value < -max_throttle)
            throttle_value = -max_throttle;
          std::cout << "throttle value= " << throttle_value << std::endl;
                      

          // DEBUG
          std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

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
