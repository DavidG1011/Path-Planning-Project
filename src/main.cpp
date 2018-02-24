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
#include <string>
#include "spline.h"
#include <limits>

using namespace std;


// Best version

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

  if(angle > pi()/4)
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

  if(centerToPos <= centerToRef)
  {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
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

int lane = 1;
double current_speed = 0;

int main() 
{
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

  while (getline(in_map_, line)) 
  {
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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

            // Store size of previous path. (Assumes x size == y size)
            int size_prev = previous_path_x.size();


            // Check if there is a previous path to base car s on.
            if (size_prev > 0)
            {
              car_s = end_path_s;
            }

            // Sensor Fusion Data, a list of all other cars on the same side of the road.
            auto sensor_fusion = j[1]["sensor_fusion"];

            json msgJson;

            //############################################ Tunable parameters ##################################################
            
            // Toggle whether or not to output cost and best action data to terminal.
            bool debug = true;

            // Max speed car can travel at.
            double absolute_max_speed = 49.5;

            // Assumes equal default cost for lane changes. Could be changed to favor left passing for legality sake.
            vector<double> costs = {0,0.12,0.12};

            // Car ahead and behind detection distance for other lanes.
            double forward_distance = 25;
            // double forward_distance = 50;
            double rear_distance = 8;

            //*******************************************************
            //**  Make sure forward_distance > same_lane_distance. **
            //*******************************************************

            // Car in same lane detection distance.
            double same_lane_distance = 18;

            // Control how fast the ego car accelerates/decelerates.
            double accel_inc = 0.35;
            double decel_inc = 1.0;

            // Set how far apart way-points are for the spline function. Higher = smoother lines.
            double spl_waypoint_distance = 28.0;

            // Prevents ego car from accelerating if the acceleration would hit another car. 
            // Distance to car greater than this value = safe.
            double safe_to_accel = 13;

            // Prevents ego car from clipping other cars during lane changing. 
            // Distance to car greater than this value = safe.
            double safe_to_change = 14;

            //##################################################################################################################


            double current_desired_speed = absolute_max_speed;

            int best_index;

            bool speed_up = true;
            bool car_close = false;

            double s_to_compare = numeric_limits<double>::max();
            

            // Initialize x and y vectors to be used for spline calculation. 
            vector <double> spl_x;
            vector <double> spl_y; 


            // Store reference states for car x,y,yaw.
            double ref_x = car_x;
            double ref_y = car_y;
            // Convert to radians
            double ref_yaw = deg2rad(car_yaw);


            // Decide whether to use car or previous path end point as starting reference point.

            // If size of previous is less than 2, use car as starting point.
            if (size_prev < 2)
            {

              // Calculate points tangent to car.
              double prev_x = car_x - cos(car_yaw);
              double prev_y = car_y - sin(car_yaw);

              // Push back tangent x,y points and current car x,y points onto spline point vector.
              spl_x.push_back(prev_x);
              spl_x.push_back(car_x);
              spl_y.push_back(prev_y);
              spl_y.push_back(car_y);
            }
            // If size greater than 2, use the previous path end point as starting point.
            else
            { 
              // Update reference state to be previous path end point.
              ref_x = previous_path_x[size_prev-1];
              ref_y = previous_path_y[size_prev-1];

              // Calculate tangent points.
              double ref_x_prev = previous_path_x[size_prev-2];
              double ref_y_prev = previous_path_y[size_prev-2];
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              // Push back x,y points tangent to previous path end point onto spline point vector.
              spl_x.push_back(ref_x_prev);
              spl_x.push_back(ref_x);
              spl_y.push_back(ref_y_prev);
              spl_y.push_back(ref_y);
            }


            // Calculate future waypoints from 30m - 90m.
            vector <double> waypoints0 = getXY(car_s + spl_waypoint_distance,     (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector <double> waypoints1 = getXY(car_s + spl_waypoint_distance * 2, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector <double> waypoints2 = getXY(car_s + spl_waypoint_distance * 3, (2+4*lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

            // Push back waypoints onto spline vector.
            spl_x.push_back(waypoints0[0]);
            spl_x.push_back(waypoints1[0]);
            spl_x.push_back(waypoints2[0]);

            spl_y.push_back(waypoints0[1]);
            spl_y.push_back(waypoints1[1]);
            spl_y.push_back(waypoints2[1]);


            // Loop over points in spline vector and shift coordinate reference from global to local (car) coordinates.
            for (int n = 0; n < spl_x.size(); n++)
            {
              double shift_x = spl_x[n] - ref_x;
              double shift_y = spl_y[n] - ref_y;

              spl_x[n] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
              spl_y[n] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));
            }

            // Initialize spline.
            tk::spline spl;

            // Pass desired spline x,y points to the spline function.
            spl.set_points(spl_x, spl_y);


            // Declare next x,y points that will be passed to the planner.
            vector<double> next_x_vals;
            vector<double> next_y_vals;


            // Push back previous path points onto next x,y value vectors.
            for (int n = 0; n < previous_path_x.size(); n++)
            { 
              next_x_vals.push_back(previous_path_x[n]);
              next_y_vals.push_back(previous_path_y[n]);
            }


            // Calculate 30 meter way-points to use for moving at desired speed
            double waypoint_x = 30;
            double waypoint_y = spl(waypoint_x);
            double waypoint_dist = sqrt((waypoint_x) * (waypoint_x) + (waypoint_y) * (waypoint_y));
            double x_add_on = 0;


            vector<double> lanes_to_check = {0};

            if (lane == 0)
            {
              lanes_to_check.resize(1);
              lanes_to_check[0] = (1);
              costs[1] = (1);
            }
            else if (lane == 1)
            {
              lanes_to_check.resize(2);
              lanes_to_check[0] = (0);
              lanes_to_check[1] = (2);
            }
            else if (lane == 2)
            {
              lanes_to_check.resize(1);
              lanes_to_check[0] = (1);
              costs[2] = (1);
            }

            for (int n = 0; n < sensor_fusion.size(); n++)
            {
              
                float d = sensor_fusion[n][6];

                for (int i = 0; i < lanes_to_check.size(); i++)
                {

                  if (d < (2+4*lanes_to_check[i]+2) && d > (2+4*lanes_to_check[i]-2))
                  {
                    double vx = sensor_fusion[n][3];
                    double vy = sensor_fusion[n][4];
                    double other_car_speed = sqrt(vx*vx+vy*vy);
                    double s_to_check = sensor_fusion[n][5];
                    s_to_check += ((double)size_prev*.02*other_car_speed);
                    
                    //Measures if a car is close in other lane. Forward facing and rear distance are set independently. 
                    if (s_to_check > car_s && (s_to_check - car_s) <= forward_distance 
                        || s_to_check < car_s && (car_s - s_to_check) <= rear_distance)
                    {
                        if (lanes_to_check.size() == 2)
                        { 
                          //costs[i + 1] = ((forward_distance - (s_to_check - car_s)) / forward_distance);
                          costs[i + 1] = (1);
                        }
                        else if (lanes_to_check[i] = 1 && lane == 0)
                        {
                          costs[2] = (1);
                        }
                        else if (lanes_to_check[i] = 1 && lane == 2)
                        { 
                          costs[1] = (1);
                        }
                    }
                    
                  }
                } 
            }

            for (int n = 0; n < sensor_fusion.size(); n++)
            {
            
              float d = sensor_fusion[n][6];

              if (d < (2+4*lane+2) && d > (2+4*lane-2))
              {
                double vx = sensor_fusion[n][3];
                double vy = sensor_fusion[n][4];
                double other_car_speed = sqrt(vx*vx+vy*vy);
                double other_car_s = sensor_fusion[n][5];
                other_car_s += ((double)size_prev*.02*other_car_speed);

                if (other_car_s > car_s && (other_car_s - car_s) <= same_lane_distance)
                {
                  //car_close = true;
                  current_desired_speed = ((other_car_speed * 2.2369) - 0.5);
                  s_to_compare = other_car_s;
                }

              }
            }


            if (current_speed > current_desired_speed)
            {
              current_speed -= decel_inc;

              if (current_speed < current_desired_speed)
              {
                current_speed = current_desired_speed;
              }

              costs[0] = ((absolute_max_speed - current_speed) / absolute_max_speed);
            }
            else if (current_speed < current_desired_speed)
            {

              if ((s_to_compare - car_s) >= safe_to_accel)
              {
                current_speed += accel_inc;
              } 

              if (current_speed > absolute_max_speed)
              {
                current_speed = absolute_max_speed;
              }
            }


            // if (starting)
            // {
            //   costs[0] = 0.0;

            //   if (current_speed > 45)
            //   { // if (starting)
            // {
            //   costs[0] = 0.0;

            //   if (current_speed > 45)
            //   {
            //     starting = false;
            //   }
            // }
            // else
            // {
            //   costs[0] = ((absolute_max_speed - current_speed) / absolute_max_speed);
            // }
            //     starting = false;
            //   }
            // }
            // else
            // {
            //   costs[0] = ((absolute_max_speed - current_speed) / absolute_max_speed);
            // }
            

            int min_pos = 0;

            for (int n = 0; n < 3; ++n)
            {
              if (costs[n] < costs[min_pos])
              {
                  min_pos = n;
              }
            }

            best_index = min_pos;


            if (debug)
            {

              cout << "Costs" << endl << "Stay: " << costs[0] << endl << "Left Change:  "<< costs[1] << endl \
                   << "Right Change: " << costs[2] << endl;

              if (best_index == 0)
              {
                cout << "Lowest Cost: Stay in lane" << endl;
              }
              else if (best_index == 1)
              {
                cout << "Lowest Cost: Change to left <---- " << endl;
              }
              else if (best_index == 2)
              {
                cout << "Lowest Cost: Change to right ----> " << endl;
              }

              cout << endl;
            }


            // iteration++;


            // Hold for 50 iterations to prevent rapid lane changing.
            // if (iteration > 50)
            // {
            //   if (best_index == 1)
            //   {
            //     if ((s_to_compare - car_s) >= safe_to_change)
            //     {
            //       lane--;
            //       iteration = 0;
            //     }
            //   }
            //   else if (best_index == 2)
            //   {
            //     if ((s_to_compare - car_s) >= safe_to_change)
            //     {
            //       lane++;
            //       iteration = 0;
            //     }
            //   }
            // }

            if (best_index == 1)
            {
              if ((s_to_compare - car_s) >= safe_to_change)
              {
                lane--;
                //iteration = 0;
              }
            }
            else if (best_index == 2)
            {
              if ((s_to_compare - car_s) >= safe_to_change)
              {
                lane++;
                //iteration = 0;
              }
            }


            // if (current_speed > current_desired_speed)
            // {
            //   current_speed -= decel_inc;

            //   if (current_speed < current_desired_speed)
            //   {
            //     current_speed = current_desired_speed;
            //   }
            // }
            // else if (current_speed < current_desired_speed)
            // {

            //   if ((s_to_compare - car_s) >= safe_to_accel)
            //   {
            //     current_speed += accel_inc;
            //   } 

            //   if (current_speed > absolute_max_speed)
            //   {
            //     current_speed = absolute_max_speed;
            //   }
            // }


            for(int i = 1; i <= 50 - previous_path_x.size(); i++)
            {

              // Calculate way-point distance needed to travel at desired speed.
              double N = (waypoint_dist/(.02*current_speed/2.2369));
              double x_point = x_add_on+(waypoint_x)/N;
              double y_point = spl(x_point);

              x_add_on = x_point;

              // Set reference x,y points
              double x_ref = x_point;
              double y_ref = y_point;

              // Change coordinate points back to global perspective after changing them earlier.
              x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

              x_point += ref_x;
              y_point += ref_y;

              // Push back final x,y values to the simulation. 
              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);

            }

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
