#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
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
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
		 // what we get from the simulator:
          vector<double> ptsx = j[1]["ptsx"];// The global x positions of the waypoints.
          vector<double> ptsy = j[1]["ptsy"];//The global y positions of the waypoints. This corresponds to the z coordinate in Unity since y is the up-down direction.
          double px = j[1]["x"];//The global x position of the vehicle.
          double py = j[1]["y"];//The global y position of the vehicle.
          double psi = j[1]["psi"];//The orientation of the vehicle in radians converted from the Unity format to the standard format expected in most mathemetical functions
          double v = j[1]["speed"];// The current velocity in mph.
		  double delta = j[1]["steering_angle"];//The current steering angle in radians.
		  double a = j[1]["throttle"];//The current throttle value [-1, 1].


		// Transform all the points from simulator (global Cooridinates)to the vehicle's orientation
		  for (int i = 0; i < ptsx.size(); i++) {
			  //Shift car reference angle to 90-degree
			  double x = ptsx[i] - px;
			  double y = ptsy[i] - py;
			  ptsx_car[i] = x * cos(-psi) - y * sin(-psi);
			  ptsy_car[i] = x * sin(-psi) + y * cos(-psi);
		  }

		  // Transrom the points vectors to Eigen vectors for polyfit in car Coordinates
		  Eigen::VectorXd ptsx_transform(ptsx.size()); 
		  Eigen::VectorXd ptsy_transform(ptsy.size());

		  // Fitting the transformed points to  a 3rd - order polynomial
		  auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

	      // Calculate Initial cte, now transformed points are in vehicle coordinates, so x & y equal 0 below.
		  // otherwise y shoud be subtracted from the polyeval value
		  double cte = polyeval(coeffs, 0); // Our target: it to turn cte =0

		  // Calculate the Initial orientation error
		 // double epsi = psi - -atan(coeffs[1]+ 2 * px * coeffs[2] + 3 * coeffs[3] * pow(px,2))
		  // Since initially  x = 0 in the vehicle coordinates, so the higher orders are zero and also psi=0 (initially)
	

		  // double epsi = psi - -atan(coeffs[1]+ 2 * px * coeffs[2] + 3 * coeffs[3] * pow(px,2))
		  double epsi = -atan(coeffs[1]); // simplification of the above equation of epsi, and our target: is to turn epsi = 0

		// Initial state values (in car coordinates)
		  Eigen::VectorXd state(6);
		  state << 0, 0, 0, v, cte, epsi;//  vector state -->[ x,y,psi,v,cte,epsi]




          /*
          * TODO: Calculate steering angle and throttle using MPC.
          *
          * Both are in between [-1, 1].
          *
          */

		  // Call Solve for new actuations (and to show predicted x and y in the future)
		  auto vars = mpc.Solve(state, coeffs);


		  // FOR VISUAL DISPLAY PURPOSE: The next_x and next_y variables display a line projection in yellow (waypoints/reference line) according to polynomial fit.
		  vector<double> next_x_vals;
		  vector<double> next_y_vals;
		  // add (x,y) points to list here, points are in reference to the vehicle's coordinate system
		  // the points in the simulator are connected by a Yellow line (the line the car is trying to follow)
		  double poly_inc = 2.5;// distance in x
		  int num_points = 25; // wanna see 25-points of the future

		  for (int i = 1; i < num_points; i++) {
			  next_x_vals.push_back(poly_inc * i);
			  next_y_vals.push_back(polyeval(coeffs, poly_inc * i));
		  }


		  // // FOR VISUAL DISPLAY PURPOSE: The mpc_x_vals and mpc_y_vals variables display a line projection in Green representing the MPC predicted trajectory.
		  vector<double> mpc_x_vals = { state[0] };
		  vector<double> mpc_y_vals = { state[1] };
		  // for vars[], every Even value is added to x and every Odd value is added to y.
		  for (int i = 2; i < vars.size(); i += 2) {
			  mpc_x_vals.push_back(vars[i]);
			  mpc_y_vals.push_back(vars[i + 1]);
		  }


		  // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
		  // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          double steer_value = vars[0] / (deg2rad(25) * Lf);;
          double throttle_value = vars[1];

		  double Lf = 2.67;

          json msgJson;
          
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Green line
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
          //.. add (x,y) points to list here, points are in reference to the vehicle's coordinate system
          // the points in the simulator are connected by a Yellow line
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
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
      } else {
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
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
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
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
