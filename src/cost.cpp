#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>

const float REACH_GOAL = 0.3;
const float EFFICIENCY = 0.1;
const float COLLISION = 0.6;

float goal_distance_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data) {
    /*
    Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory.
    Cost of being out of goal lane also becomes larger as vehicle approaches goal distance.
    */
    float cost;
    float distance = data["distance_to_goal"];
    if (distance > 0) {
        //cost = 1 - 2*exp(-(1/distance));
	cost = 1 - exp(-(distance/20.0));
    } else {
        cost = 1;
    }
    cout << "GOAL cost is " << cost << ", distance is " << distance <<endl;
    return cost;
}

float inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data) {
    /*
    Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed. 
    */

    float proposed_speed_intended = lane_speed(predictions, data["intended_lane"]);
    if (proposed_speed_intended < 0) {
	cout << "No other car found, vehicle.target_speed is " << vehicle.target_speed << endl;
        proposed_speed_intended = vehicle.target_speed;
    }

    float proposed_speed_final = lane_speed(predictions, data["final_lane"]);
    if (proposed_speed_final < 0) {
        proposed_speed_final = vehicle.target_speed;
    }
    
    float cost = (2.0*vehicle.target_speed - proposed_speed_intended - proposed_speed_final)/vehicle.target_speed;
    cout << "EFFICIENCY cost is " << cost << ", proposed_speed_intended is " << proposed_speed_intended << ", proposed_speed_final is " << proposed_speed_final << endl;
    return cost;
}

float collision_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data) {
    float cost = 0.0;
    for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
        int key = it->first;
        Vehicle non_ego = it->second[0];
        if (vehicle.lane == non_ego.lane && key != -1) {
            float dist = fabs(vehicle.s - non_ego.s);
	    /*
            if (dist < 5.0) {
	        cost += 1.0;
	    }*/
            cost += 1 - exp(-dist/5.0);
        }
    }
    cout << "COLLISION cost is " << cost << endl; 
    return cost;
}

float lane_speed(const map<int, vector<Vehicle>> & predictions, int lane) {
    /*
    Average all non ego vehicles' speed, so to get the speed limit for a lane,
    we can just find one vehicle in that lane.
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
    vector< function<float(const Vehicle & , const vector<Vehicle> &, const map<int, vector<Vehicle>> &, map<string, float> &)>> cf_list = {goal_distance_cost, inefficiency_cost, collision_cost};
    vector<float> weight_list = {REACH_GOAL, EFFICIENCY, COLLISION};
    
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
    distance_to_goal: the distance of the vehicle to the goal.

    Note that indended_lane and final_lane are both included to help differentiate between planning and executing
    a lane change in the cost functions.
    */
    map<string, float> trajectory_data;
    Vehicle trajectory_last = trajectory[1];
    float intended_lane;

/*
    if (trajectory_last.state.compare("PLCL") == 0) {
        intended_lane = trajectory_last.lane + 1;	
    } else if (trajectory_last.state.compare("PLCR") == 0) {
        intended_lane = trajectory_last.lane - 1;
    } else {
        intended_lane = trajectory_last.lane;
    }
*/
    intended_lane = trajectory_last.goal_lane;

    float distance_to_goal = vehicle.goal_s - trajectory_last.s;
    float final_lane = trajectory_last.lane;
    trajectory_data["intended_lane"] = intended_lane;
    trajectory_data["final_lane"] = final_lane;
    trajectory_data["distance_to_goal"] = distance_to_goal;
    return trajectory_data;
}

