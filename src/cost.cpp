#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>

const float REACH_GOAL = 1;
const float EFFICIENCY = 0.1;
const float COLLISION = 10;
const float S_OVER_ACCE = 0.001;
const float D_OVER_ACCE = 0.05;

#define CAR_LENGTH 5.0 //unit:m

float goal_distance_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data) {
    /*
    Cost increases based on distance of the virtual goal and final end of trajectory.
    */
    float cost;
    float distance = data["distance_to_goal"];
    if (distance > 0) {
	cost = 1 - exp(-(distance/vehicle.target_speed));
    } else {
        cost = 1;
    }

    return cost;
}

float inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data) {
    /*
    Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed.
    Cost also increases when a car is detected ahead on the intended lane. And its lower speed than target speed also generates costs. 
    */
    float cost = 0.0;
    
    float proposed_speed_intended = lane_speed(predictions, data["intended_lane"]);
    if (proposed_speed_intended < 0 || proposed_speed_intended > vehicle.target_speed) {
        proposed_speed_intended = vehicle.target_speed;
    }

    float proposed_speed_final = lane_speed(predictions, data["final_lane"]);
    if (proposed_speed_final < 0 || proposed_speed_final > vehicle.target_speed) {
        proposed_speed_final = vehicle.target_speed;
    }
    
    cost += (2.0*vehicle.target_speed - proposed_speed_intended - proposed_speed_final)/vehicle.target_speed;

    Vehicle trajectory_last = trajectory[1];
    Vehicle ego = vehicle;
    Vehicle non_ego_ahead;
    if (ego.get_vehicle_ahead(predictions, trajectory_last.goal_lane, non_ego_ahead)) {
        
        float non_ego_expect_s = non_ego_ahead.s;
	if (non_ego_expect_s < trajectory_last.s + ego.target_speed) {
	    cost += 1;
        }

	if (non_ego_ahead.vs < ego.target_speed) {
	    cost += (ego.target_speed - non_ego_ahead.vs) / ego.target_speed;
	}
    }

    return cost;
}

float collision_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data) {
    /*
    Cost becomes higher if the car ahead or behind gets closer to the end of the trajectory
    */
    float cost = 0.0;
    Vehicle trajectory_last = trajectory[1];
    Vehicle non_ego_ahead;
    Vehicle ego = vehicle;
    // A logistic function is used to calculate the cost according to the position difference
    if (ego.get_vehicle_ahead(predictions, trajectory_last.lane, non_ego_ahead)) {
        float non_ego_expect_s = non_ego_ahead.s;
	float dist = non_ego_expect_s - trajectory_last.s;
	cost += 1/(1 + exp(-(CAR_LENGTH - dist)/CAR_LENGTH));	
    }

    Vehicle non_ego_behind;
    if (ego.get_vehicle_behind(predictions, trajectory_last.lane, non_ego_behind)) {
        float non_ego_expect_s = non_ego_behind.s + non_ego_behind.vs;
	float dist = trajectory_last.s - non_ego_expect_s;
	cost += 1/(1 + exp(-(CAR_LENGTH - dist)/CAR_LENGTH));	
    } 
    return cost;
}

float s_over_acceleration_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data) {
    /*
    Cost reflects the s direction speed variation between the end and the start of the trajectory
    */
    float cost = 0.0;
    Vehicle trajectory_last = trajectory[1]; 
    Vehicle ego = vehicle;
    cost += fabs(trajectory_last.vs - ego.vs);
    return cost;
}

float d_over_acceleration_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data) {
    /*
    Cost reflects the d direction speed variation between the end and the start of the trajectory
    */
    float cost = 0.0;
    Vehicle trajectory_last = trajectory[1]; 
    Vehicle ego = vehicle;
    cost += fabs(trajectory_last.vd - ego.vd);
    return cost;
}
float lane_speed(const map<int, vector<Vehicle>> & predictions, int lane) {
    /*
    Average all non ego vehicles' speed, so to get the speed limit for a lane.
    */
    double average_speed = -1.0;
    double sum = 0.0;
    int count = 0;
    for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
        int key = it->first;
        Vehicle vehicle = it->second[0];
        if (vehicle.lane == lane && key != -1) {
            //return vehicle.v;
            //return vehicle.vs;
	    sum += vehicle.vs;
	    count++;
        }
    }
    
    if (count > 0) {
      average_speed = sum/count;
    }
    //Found no vehicle in the lane
    return average_speed;
}

float calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory) {
    /*
    Sum weighted cost functions to get total cost for trajectory.
    */
    map<string, float> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
    float cost = 0.0;

    //Add additional cost functions here.
    vector< function<float(const Vehicle & , const vector<Vehicle> &, const map<int, vector<Vehicle>> &, map<string, float> &)>> cf_list = {goal_distance_cost, inefficiency_cost, collision_cost, s_over_acceleration_cost, d_over_acceleration_cost};
    vector<float> weight_list = {REACH_GOAL, EFFICIENCY, COLLISION, S_OVER_ACCE, D_OVER_ACCE};
    
    for (int i = 0; i < cf_list.size(); i++) {
        float new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions, trajectory_data);
        cost += new_cost;
    }

    return cost;

}

map<string, float> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
    /*
    Generate helper data to use in cost functions:
    indended_lane: the current lane +/- 1 if vehicle is planning or executing a lane change.
    final_lane: the lane of the vehicle at the end of the trajectory.
    distance_to_goal: the distance of the end of the trajectory to the virtual goal.

    Note that indended_lane and final_lane are both included to help differentiate between planning and executing
    a lane change in the cost functions.
    */
    map<string, float> trajectory_data;
    Vehicle trajectory_last = trajectory[1];
    float intended_lane;

    intended_lane = trajectory_last.goal_lane;

    float distance_to_goal = vehicle.goal_s - trajectory_last.s;
    float final_lane = trajectory_last.lane;
    trajectory_data["intended_lane"] = intended_lane;
    trajectory_data["final_lane"] = final_lane;
    trajectory_data["distance_to_goal"] = distance_to_goal;
    return trajectory_data;
}

