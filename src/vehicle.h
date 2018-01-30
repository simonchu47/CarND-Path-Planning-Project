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

  double preferred_buffer;

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

  vector<string> actions = {"SPEEDUP", "HALF-SPEEDUP", "MAINTAIN", "HALF-SLOWDOWN", "SLOWDOWN"};

  /**
  * Constructor
  */
  Vehicle();

  Vehicle(double s, double d, double vs, double vd, double as, double ad, string state, int target_lane);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions);

  vector<string> successor_states();

  vector<vector<Vehicle>> generate_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<double> action(map<int, vector<Vehicle>> predictions, int lane, double interval, int choice);

  vector<vector<Vehicle>> keep_lane_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<vector<Vehicle>> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<vector<Vehicle>> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

  bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

  vector<Vehicle> generate_predictions(double interval=0.5, int horizon=2);

  vector<double> position_at(double t);

  void configure(vector<double> road_data);
 
  void update_state(double s, double d, double vs, double vd);

};

#endif
