#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

  map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};

  struct collider{

    bool collision ; // is there a collision?
    int  time; // time collision happens

  };

  int L = 1;

  //int preferred_buffer = 6; // impacts "keep lane" behavior.
  double preferred_buffer = 6;

  int lane;

  //int s;
  double s; // s direction position
  double d; // d direction position

  //float v;
  double vs; // velocity along s direction
  double vd; // velocity along d direction

  //float a;
  double as; // acceleration along s direction
  double ad; // acceleration along d direction

  double target_speed;

  int lanes_available;

  double max_acceleration;

  int goal_lane;

  int goal_s;

  string state;

  vector<string> actions = {"SPEEDUP", "MAINTAIN", "SLOWDOWN"};

  //bool LC_finished = true;

  /**
  * Constructor
  */
  Vehicle();
  //Vehicle(int lane, float s, float v, float a, string state="CS", int target_lane);
  Vehicle(double s, double d, double vs, double vd, double as, double ad, string state, int target_lane);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions);

  vector<string> successor_states();

  vector<vector<Vehicle>> generate_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<double> get_kinematics(map<int, vector<Vehicle>> predictions, int lane, double interval);

  vector<double> action(map<int, vector<Vehicle>> predictions, int lane, double interval, int choice);

  vector<Vehicle> constant_speed_trajectory();

  vector<vector<Vehicle>> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);

  vector<vector<Vehicle>> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<vector<Vehicle>> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  void increment(int dt);

  vector<double> position_at(double t);

  bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

  bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

  vector<Vehicle> generate_predictions(double interval=0.5, int horizon=2);

  void realize_next_state(vector<Vehicle> trajectory);

  void configure(vector<int> road_data);
 
  void update_state(double s, double d, double vs, double vd);

};

#endif
