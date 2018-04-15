#include <uWS/uWS.h>
#include <iostream>
#include <fstream>
#include "json.hpp"
#include "PID.h"
#include "twiddle.hpp"
#include <math.h>
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

int main(int argc, char *argv[])
{
  uWS::Hub h;

  PID pid;
  Twiddle twiddle;
  
  // Initialize the pid variable.
  double init_Kp = atof(argv[1]);
  double init_Ki = atof(argv[2]);
  double init_Kd = atof(argv[3]);
  pid.Init(init_Kp, init_Ki, init_Kd);
  
  int apply_twiddle = atof(argv[4]);
  if(apply_twiddle!=0){
    int free_init = atof(argv[5]);
    int tuning_range = atof(argv[6]);
    twiddle.Init(init_Kp/10, init_Ki/10, init_Kd/10, apply_twiddle, free_init, tuning_range);
  }
  else{
    twiddle.Init(init_Kp/10, init_Ki/10, init_Kd/10, apply_twiddle, 0, 0);
  }

  // File to store sim values
  ofstream out_file;
  
  if(twiddle.write_file){
  
    // open file
    out_file.open ("data_record.txt");
  
    // write the file headers
    out_file << "timestep" << "\t";
    out_file << "Kp" << "\t";
    out_file << "Ki" << "\t";
    out_file << "Kd" << "\t";
    out_file << "cte" << "\t";
    out_file << "total_error" << "\t";
    out_file << "steering" << "\t";
    out_file << "throttle" << "\t";
    out_file << "speed" << "\t";
    out_file << "distance" << "\n";
  }
  
  h.onMessage([&pid, &twiddle, &out_file](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
          // double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value;
          double throttle = 0.35;
          
          
          //--> CODE INIT
          /*
           * CALC STEERING VALUE
           */
          pid.UpdateError(cte);
          steer_value = pid.TotalError();
          
          if(twiddle.apply_twiddle){
          /*
           * TWIDDLE PARAMETER TUNNING
           */
          twiddle.TrackError(cte, speed, pid);
          if (twiddle.Restart()) {
            std::string msg = "42[\"reset\",{}]";
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            
            cout << "Restart sim" << endl;
            out_file.close();
            
          }
          
          /*
           * PRINT RESULTS
           */
          std::cout << "\n";
          std::cout << "Time step: " << twiddle.timestep << endl;

          std::cout << "Twiddle best error:   " << twiddle.best_error << "\t";
          std::cout << "Longest distance: " << twiddle.longest_distance << "\t";
          std::cout << "Max speed:        " << twiddle.max_speed  << endl;
          
          std::cout << "Twiddle actual error: " << twiddle.actual_error << "\t";
          std::cout << "Actual distance:  " << twiddle.distance << "\t";
          std::cout << "Actual Max speed: " << twiddle.actual_max_speed  << endl;
            
          std::cout << "\n";
          cout << "PID Best params: " << twiddle.best_Ks[0] << "\t" << twiddle.best_Ks[1] << "\t" << twiddle.best_Ks[2] << endl;
          cout << "PID Params:      " << pid.Kp << "\t" << pid.Ki << "\t" << pid.Kd << endl;
          cout << "Twiddle Params:  " << twiddle.ds[0] << "\t" << twiddle.ds[1] << "\t" << twiddle.ds[2] << endl;
          cout << "Twiddle quality: " << twiddle.quality << endl;
          // std::cout << " CTE: " << cte << " Steering Value: " << steer_value << std::endl;
          }

          /*
           * WRITE TO FILE
           */
          if(twiddle.write_file){
            out_file << twiddle.timestep << "\t";
            out_file << pid.Kp << "\t";
            out_file << pid.Ki << "\t";
            out_file << pid.Kd << "\t";
            out_file << cte << "\t";
            out_file << twiddle.rm_error << "\t";
            out_file << steer_value << "\t";
            out_file << throttle << "\t";
            out_file << speed << "\t";
            out_file << twiddle.distance << "\n";
          }
          
          //<-- CODE END
          
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle;
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
