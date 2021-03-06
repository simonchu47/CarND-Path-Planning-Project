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
#include "vehicle.h"
#include "spline.h"

#define TIME_STEP 0.02
#define PREDICT_TIME_SPAN 1
#define PREFERRED_SPEED 45 //unit:MPH
// define the coefficient of MPH to MPS
// which is 1.609344*1000/60/60=0.44704
#define MPH_TO_MPS 0.44704
#define AVAILABLE_LANES 3

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

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

// Transform speed from Cartesian coordinates to Frenet s,d coordinates
vector<double> getFrenetSpeed(double x, double y, double theta, double vx, double vy, const vector<double> &maps_x, const vector<double> &maps_y)
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
	//double x_x = x - maps_x[prev_wp];
	//double x_y = y - maps_y[prev_wp];
	double x_x = vx;
	double x_y = vy;

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
	double frenet_s = distance(0,0,proj_x,proj_y);
	
	return {frenet_s,frenet_d};

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

  Vehicle ego;
  vector<double> road_data = {PREFERRED_SPEED*MPH_TO_MPS, AVAILABLE_LANES}; 
  ego.configure(road_data);

  h.onMessage([&ego, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds		

		map<int ,vector<Vehicle>> predictions;
		for (int i = 0; i < sensor_fusion.size(); ++i) {
		  int v_id = sensor_fusion[i][0];
 		  double x = sensor_fusion[i][1];
		  double y = sensor_fusion[i][2];
		  double vx = sensor_fusion[i][3];
		  double vy = sensor_fusion[i][4];
		  double s = sensor_fusion[i][5];
		  double d = sensor_fusion[i][6];
		  double theta = atan2(y,x);

		  vector<double> vs_vd = getFrenetSpeed(x, y, theta, vx, vy, map_waypoints_x, map_waypoints_y);
		  Vehicle non_ego = Vehicle(s, d, vs_vd[0], vs_vd[1], 0.0, 0.0, "CS", 0);

		  vector<Vehicle> pred = non_ego.generate_predictions(PREDICT_TIME_SPAN, 1);
		  predictions[v_id] = pred;
		}

		double car_yaw_rad = deg2rad(car_yaw);
		double car_vx = car_speed*MPH_TO_MPS*cos(car_yaw_rad);
		double car_vy = car_speed*MPH_TO_MPS*sin(car_yaw_rad);
                vector<double> car_vs_vd = getFrenetSpeed(car_x, car_y, car_yaw_rad, car_vx, car_vy, map_waypoints_x, map_waypoints_y);

		ego.update_state(car_s, car_d, car_vs_vd[0], car_vs_vd[1]);

		vector<Vehicle> traj = ego.choose_next_state(predictions);

		if (traj.size() > 0) {
		    // Update the state and goal_lane for ego
		    ego.state = traj[1].state;
		    ego.goal_lane = traj[1].goal_lane;

		    // The anchor points used for spline
		    vector<double> ptsx;
		    vector<double> ptsy;
		    double ref_x;
		    double ref_y;
		    double ref_yaw;
		    int prev_size = previous_path_x.size();

		    // Use the last two points of the previous path for the first two anchor points
                    // Or generate the anchor points according to the current position and heading
		    if (prev_size < 2) {
		        double prev_car_x = car_x - cos(car_yaw_rad);
		        double prev_car_y = car_y - sin(car_yaw_rad);
		        ptsx.push_back(prev_car_x);
		        ptsx.push_back(car_x);
		        ptsy.push_back(prev_car_y);
		        ptsy.push_back(car_y);
		        ref_x = car_x;
		        ref_y = car_y;
		        ref_yaw = car_yaw_rad;
		    } else {
		        ref_x = previous_path_x[prev_size - 1];
		        ref_y = previous_path_y[prev_size - 1];
		        double prev_ref_x = previous_path_x[prev_size - 2];
		        double prev_ref_y = previous_path_y[prev_size - 2];
		        ref_yaw = atan2(ref_y - prev_ref_y, ref_x - prev_ref_x);
		        ptsx.push_back(prev_ref_x);
		        ptsx.push_back(ref_x);
		        ptsy.push_back(prev_ref_y);
		        ptsy.push_back(ref_y);
		    }

                    // Extending from the end of the jactory a bit to set the third waypoint wp1
		    double wp1_time = PREDICT_TIME_SPAN*1.2;
		    double wp1_s = traj[1].s + traj[1].vs*wp1_time;
                    double wp1_d = traj[1].d;
		    vector<double> wp1 = getXY(wp1_s, wp1_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

		    // Extend the wp1 to get the second way point
		    double wp2_time = PREDICT_TIME_SPAN*2;
		    double wp2_s = traj[1].s + traj[1].vs*wp2_time;
                    double wp2_d = traj[1].d;
		    vector<double> wp2 = getXY(wp2_s, wp2_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);

		    ptsx.push_back(wp1[0]);
		    ptsx.push_back(wp2[0]);

		    ptsy.push_back(wp1[1]);
		    ptsy.push_back(wp2[1]);

		    // Transform these anchor points to reference point perspective
		    for (int i = 0 ; i < ptsx.size(); ++i) {
		        double shift_x = ptsx[i] - ref_x;
		        double shift_y = ptsy[i] - ref_y;
		        ptsx[i] = shift_x*cos(0 - ref_yaw) - shift_y*sin(0 - ref_yaw);
		        ptsy[i] = shift_x*sin(0 - ref_yaw) + shift_y*cos(0 - ref_yaw);
		    }
		
		    // Check wether the x value is ascending or not, to prevent error
		    bool wp_sequence_wrong = false;
                    for (int i = 0 ; i < ptsx.size() - 1; ++i) {
		        if (ptsx[i] > ptsx[i + 1]) {
                            wp_sequence_wrong = true;
                            break;
                        }
                    }

                    if (!wp_sequence_wrong) {
		        // Using spline tool
		        // Create a spline
		        tk::spline s;
		
		        // Set the waypoints to the spline
		        s.set_points(ptsx, ptsy);
		
                        // Use the previous path points first
		        for (int i = 0; i < prev_size; ++i) {
		            next_x_vals.push_back(previous_path_x[i]);
		            next_y_vals.push_back(previous_path_y[i]);
		        }

                        // Calculate the x step length according to the best trajectory's s speed
                        double target_x = 30;
		        double target_y = s(target_x);
		        double target_dist = sqrt(target_x*target_x + target_y*target_y);
		        double N = target_dist/(0.02*traj[1].vs);
		        int remain_steps = PREDICT_TIME_SPAN/TIME_STEP - prev_size;
		        double x_addon = 0.0;
		        double x_step = target_x/N;
                        // Fill the remaining vacancies with the current path
		        for (int i = 1; i <= remain_steps; ++i) {
                            // Get the point position from the spline curve, and then transform back to original XY coordinate
		            x_addon += x_step;
		            double y_addon = s(x_addon);		   
		            double x_point = x_addon*cos(ref_yaw) - y_addon*sin(ref_yaw);
		            double y_point = x_addon*sin(ref_yaw) + y_addon*cos(ref_yaw);
		            x_point += ref_x;
		            y_point += ref_y;
                            // Add the point into the path
		            next_x_vals.push_back(x_point);
		            next_y_vals.push_back(y_point);
		        }
                    }
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
