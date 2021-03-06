#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"

using namespace std;

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
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }

  }

  return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if (angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if(next_wp == 0)
  {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point

  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; i++)
  {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};

}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  // at the very beginning the ego car starts from lane 1
  int lane = 1;
  // velocity to start with in each cycle
  double ref_vel = 0.0;
  // whether or not the car is in the state of lane changing
//  bool change_lane = false;

  h.onMessage([&max_s, &lane, &ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();

          json msgJson;

          double car_s_now = fmod(car_s, max_s);

          double front_gap_zero = max_s;
          double front_gap_one = max_s;
          double front_gap_two = max_s;

          double rear_gap_zero = -max_s;
          double rear_gap_one = -max_s;
          double rear_gap_two = -max_s;

          if (prev_size > 0) {
            car_s = end_path_s;
          }

          bool too_close = false;
          car_s = fmod(car_s, max_s);

          // check other cars one by one
          for (int i = 0; i < sensor_fusion.size(); i++) {
            float d = sensor_fusion[i][6];
            double check_car_s = sensor_fusion[i][5];
            check_car_s = fmod(check_car_s, max_s);

            // I want to check the gap size between me and the car in front of me
            // in 3 lanes
            bool in_my_lane = d < (2 + 4 * lane + 2) && d > (2 + 4 * lane - 2);
            bool in_lane_zero = d < 4 && d > 0;
            bool in_lane_one = d < 8 && d > 4;
            bool in_lane_two = d < 12 && d > 8;

            double gap = check_car_s - car_s_now;
            // if the ego car is ahead of the surrounding car for more than half of the loop length
            // we might as well consider that the ego car is behind the surrounding car
            if (gap < - max_s / 2) {
              gap = max_s + gap;
            } else if (gap > max_s / 2) {
              gap -= max_s;
            }

            // update front gap or rear gap
            if (gap > 0) {
              if (in_lane_zero && gap < front_gap_zero) {
                front_gap_zero = gap;
              } else if (in_lane_one && gap < front_gap_one) {
                front_gap_one = gap;
              } else if (in_lane_two && gap < front_gap_two) {
                front_gap_two = gap;
              }
            } else {
              if (in_lane_zero && gap > rear_gap_zero) {
                rear_gap_zero = gap;
              } else if (in_lane_one && gap > rear_gap_one) {
                rear_gap_one = gap;
              } else if (in_lane_two && gap > rear_gap_two) {
                rear_gap_two = gap;
              }
            }

            // besides the front gap, I also want to check the feasibility of lane change

            // if car is in my lane
            if (in_my_lane) {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx * vx + vy * vy);

              // if car_s is at end_path_s, then check_car_s should be as follows
              check_car_s += (double) prev_size * .02 * check_speed;
              if (check_car_s > max_s) {
                check_car_s -= max_s;
              }

              gap = check_car_s - car_s;
              if (gap < - max_s / 2) {
                gap = max_s + gap;
              }

              // if the car is in front of me and too close
              if (gap > 0 && gap < 20) {
                // TODO: do some logic here. either lower the velocity or try to change lane
                too_close = true;
                // if too close and not in lane change mode
              }
            }
          }

          cout << "distance to the closest front car in lane 0: " << front_gap_zero << endl;
          cout << "distance to the closest front car in lane 1: " << front_gap_one << endl;
          cout << "distance to the closest front car in lane 2: " << front_gap_two << endl;
          cout << "distance to the closest rear car in lane 0: " << rear_gap_zero << endl;
          cout << "distance to the closest rear car in lane 1: " << rear_gap_one << endl;
          cout << "distance to the closest rear car in lane 2: " << rear_gap_two << endl;

          // Behavior Planning happens here
          // by default the ego car is in keep lane mode
          // the following branch includes prepare for lane change (slow down to keep distance)
          // and lane change
          double ref_rear_gap = -10;
          double ref_front_gap = 45;

          if (too_close) {
            ref_vel -= .224;

            if (lane == 0) {
              if ((front_gap_one > ref_front_gap) && (rear_gap_one < ref_rear_gap)) {
                lane += 1;
              }
            } else if (lane == 1) {
              bool is_zero_feasible = (front_gap_zero > ref_front_gap) && (rear_gap_zero < ref_rear_gap);
              bool is_two_feasible = (front_gap_two > ref_front_gap) && (rear_gap_two < ref_rear_gap);
              bool is_both_feasible = is_zero_feasible && is_two_feasible;

              if (is_both_feasible) {
                lane = (front_gap_zero > front_gap_two) ? 0 : 2;
              } else if (is_zero_feasible) {
                lane = 0;
              } else if (is_two_feasible) {
                lane = 2;
              }
            } else {
              if ((front_gap_one > ref_front_gap) && (rear_gap_one < ref_rear_gap)) {
                lane -= 1;
              }
            }
          } else {
            if (ref_vel < 49.5) {
              ref_vel += .224;
            }

            // lane 1 has two possible lanes to switch to
            // I reason that it has a priority. switch to it if possible
            // even if the current lane is wide open
            if (lane != 1) {
              if ((front_gap_one > ref_front_gap) && (rear_gap_one < ref_rear_gap)) {
                lane = 1;
              }
            }
          }

          //
          vector<double> ptsx;
          vector<double> ptsy;


          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if (prev_size < 2) {
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);
          } else {
            ref_x = previous_path_x[prev_size - 1];
            ref_y = previous_path_y[prev_size - 1];

            double ref_x_prev = previous_path_x[prev_size - 2];
            double ref_y_prev = previous_path_y[prev_size - 2];
            ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

            ptsx.push_back(ref_x_prev);
            ptsx.push_back(ref_x);

            ptsy.push_back(ref_y_prev);
            ptsy.push_back(ref_y);
          }

          int anchor_gap = 30;
          if (ref_vel < 40) {
            anchor_gap = 20;
          }

          vector<double> next_wp0 = getXY(car_s + anchor_gap, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + anchor_gap * 2, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + anchor_gap * 3, (2 + 4 * lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);

          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          // transform coordinates from map system to vehicle system
          for (int i = 0; i < ptsx.size(); i++) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
            ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
          }

          tk::spline s;

          // we fit the spline with only 5 points, 3 of them are on the same lane, straight line
          s.set_points(ptsx, ptsy);

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          for (int i = 0; i < prev_size; i++) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30.0;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);

          double x_add_on = 0;

          for (int i = 0; i < 50 - previous_path_x.size(); i++) {
            // evenly space out N points in the 30 meters range on the spline
            // remember we are in the vehicle coordinate system
            double N = target_dist / (.02 * ref_vel / 2.24);
            double x_pt = x_add_on + target_x / N;
            double y_pt = s(x_pt);

            x_add_on = x_pt;

            double x_ref = x_pt;
            double y_ref = y_pt;

            // transform coordinates from vehicle system to map system
            x_pt = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_pt = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_pt += ref_x;
            y_pt += ref_y;

            next_x_vals.push_back(x_pt);
            next_y_vals.push_back(y_pt);
          }
          //END

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          //this_thread::sleep_for(chrono::milliseconds(1000));
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
