#include <uWS/uWS.h>
#include <iostream>
#include "json.hpp"
#include "PID.h"
#include <math.h>
#include <vector>

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

bool twiddle_on = false;
double best_error = 1000000;
bool state = 0;
int idx  = 0;
int iterations = 0;
std::vector<double> p = {0.27, 0.001, 3.0};
std::vector<double> dp = {0.05, 0.001, 0.05};

void twiddle(PID &pid)
{
	std::cout << "State: " << state << std:: endl;
	
	if(state == 0)
	{
		best_error = pid.TotalError();
		p[idx] += dp[idx];
		state = 1;
	}
	else if(state == 1)
	{
		if(pid.TotalError() < best_error)
		{
			best_error = pid.TotalError();
			dp[idx] *= 1.1;
			idx =(idx +1) % 3;
			p[idx] += dp[idx];
			state =1;
		}
		else{
			
			p[idx] -= 2* dp[idx];
			if (p[idx] < 0)
			{
				p[idx] = 0;
				idx = (idx + 1) % 3;
			}
			
			state = 2;
		}
	}
	else{
		if(pid.TotalError() < best_error)
		{
			best_error = pid.TotalError();
			
			dp[idx] *= 1.1;
			idx = (idx + 1) % 3;
			p[idx] += dp[idx];
			
			state = 1;
		}
		else{
			p[idx] += dp[idx];
			dp[idx] *= 0.9;
			idx = (idx + 1) % 3;
			p[idx] += dp[idx];
			
			state = 1;
		}
		
	}
	
	pid.Init(p[0],p[1],p[2]);
	double sum = 0;
	for (idx =0; idx <3; idx++)
	{
		sum += dp[idx];
	}
	if(sum < 0.001)
		std::cout << "Find the best PID parameters: p " <<p[0] << " i" << p[1]<< " d"<<p[2] << std::endl;
}

int main()
{
#if 0
  if (argc = 2){
	std::string argu = argv[1];
	std::cout << argu << std::endl;
	if (argu == "true")
	{
		twiddle_on = true;
		std::cout << "Using Twiddle to turn the best PID parameters!" << std::endl;
	}
  }
#endif
  uWS::Hub h;

  PID pid_;
 // TODO: Initialize the pid variable.
  
  const double Kp_s = 0.15;
  const double Ki_s = 0.00001;
  const double Kd_s = 1.0;
  pid_.Init(Kp_s, Ki_s, Kd_s);
  
  PID pid_throttle;
  const double Kp_t = 0.2;
  const double Ki_t = 0.001;
  const double Kd_t = 2.0;
  pid_throttle.Init(Kp_t, Ki_t, Kd_t);
  h.onMessage([&pid_, &pid_throttle](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
		  double speed_required = 30.0;
          /*
          * TODO: Calcuate steering value here, remember the steering value is
          * [-1, 1].
          * NOTE: Feel free to play around with the throttle and speed. Maybe use
          * another PID controller to control the speed!
          */
// 		    if ((speed > 50) & (throttle_value > 0.0)) {
//             pid_.Kp = 0.04;
//             pid_.Ki = 0.0012;
//             pid_.Kd = 0.8;
//           } else { // have to reset to defaults once car slows down
//             pid_.Kp = 0.035;
//             pid_.Ki = 0.0084;
//             //pid_.Ki = 0.00;
//             pid_.Kd = 1.16;
//           }
//           
          pid_.UpdateError(cte);
          
          // addressing issues with I
          if (pid_.i_error > 40) {
              pid_.i_error = 40;
          } else if (pid_.i_error < -40) {
              pid_.i_error = -40;
          }
          if (cte == 0) {
            pid_.i_error = 0;
          }
          
          steer_value = pid_.TotalError();
          if (steer_value > 1.0)
            steer_value = 1.0;
          if (steer_value < -1.0)
            steer_value = -1.0;          
          
		  double throttle_error = speed - speed_required;
		  pid_throttle.UpdateError(throttle_error);
		  throttle_value = pid_throttle.TotalError();
		  if (throttle_value > 0.3)
			  throttle_value = 0.3;
		  else if (throttle_value < -0.3)
			  throttle_value = -0.3;
		  
          // update throttle
          //pid_throttle.UpdateError(abs(cte));
          //throttle_value = 1.0 + (pid_throttle.TotalError());
//           double min_speed = 40;
//           double min_throttle = 0.3;
//           double max_breaking = -0.3;
		  
//           if (throttle_value < min_throttle) { 
//             if (speed < min_speed)
//               throttle_value = min_throttle;
//             else {
//               if (throttle_value < max_breaking)
//                 throttle_value = max_breaking;
//             }
//           }
          // DEBUG
		  if (!twiddle_on){
			std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
		  }
		  
		  if (twiddle_on)
		  {
			iterations ++;
			
			if ((iterations > 1000) || ((speed<speed_required*0.5) && iterations > 80)) {
              if ((speed<speed_required*0.5) && iterations > 80) { //probably crash
                best_error = 1000000;
                std::string msg = "42[\"reset\", {}]";
                std::cout << msg << std::endl;
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              }

              twiddle(pid_);
			  std::cout << "PID error: " << pid_.TotalError() << ", Best Error: " << best_error << std::endl;
              std::cout << "P VECTOR: " << p[0] << "\t" << p[1] << "\t" << p[2] << std::endl;
			  iterations = 0;
            }
			
			
			json msgJson;
            msgJson["steering_angle"] = steer_value;
            msgJson["throttle"] = throttle_value;
            auto msg = "42[\"steer\"," + msgJson.dump() + "]";
            std::cout << msg << std::endl;
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
		  }
		  else{
 			json msgJson;
			msgJson["steering_angle"] = steer_value;
			msgJson["throttle"] = throttle_value;
			auto msg = "42[\"steer\"," + msgJson.dump() + "]";
			std::cout << msg << std::endl;
			ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
		  }
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
