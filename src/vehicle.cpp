#include <algorithm>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"

#define LANE_WIDTH 4.0 
#define SAFE_DISTANCE 30.0
#define CAR_LENGTH 5.0
#define TIME_INTERVAL 1.0
#define SPEED_LIMIT 20.0 //unit:m/s
#define ACCELERATION_LIMIT 0.9 //unit:m/s/s
#define LC_D_SPEED 2.0 //unit:m/si
#define MPS_TO_KMPH 3.6
/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(){}

Vehicle::Vehicle(double s, double d, double v_s, double v_d, double a_s, double a_d, string state, int target_lane) {

    this->s = s;
    this->d = d;
    this->vs = v_s;
    this->vd = v_d;
    this->as = a_s;
    this->ad = a_d;
    this->state = state;

    this -> max_acceleration = ACCELERATION_LIMIT;
    this->lane = (int)d/LANE_WIDTH;
    this->goal_lane = target_lane;
    this->goal_s = this->s + SPEED_LIMIT*TIME_INTERVAL*2.0; // It's vitual goal
    this->target_speed = SPEED_LIMIT;
    this->preferred_buffer = this->vs*MPS_TO_KMPH/2.0;

}

Vehicle::~Vehicle() {}


vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions) {
    /*
    Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
    to the next state.

    INPUT: A predictions map. This is a map of vehicle id keys with predicted
        vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
        the vehicle at the current timestep and one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

    */
    vector<string> states = successor_states();
    float cost;
    vector<float> costs;
    vector<string> final_states;
    vector<vector<Vehicle>> final_trajectories;

    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
        vector<vector<Vehicle>> trajectorys = generate_trajectory(*it, predictions);
        if (trajectorys.size() > 0) {
            for (int i = 0; i < trajectorys.size(); ++i) {
                vector<Vehicle> trajectory = trajectorys[i];
                cost = calculate_cost(*this, predictions, trajectory);
                costs.push_back(cost);
                final_trajectories.push_back(trajectory);
            }
        }
    }

    vector<Vehicle> best_traj;
    if (final_trajectories.size() > 0) {
        vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
        int best_idx = distance(begin(costs), best_cost);
        best_traj = final_trajectories[best_idx];
    }
    return best_traj;
}

vector<string> Vehicle::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<string> states; 
    states.push_back("KL");
    string state = this->state;
    if(state.compare("KL") == 0) {
        states.push_back("PLCL");
        states.push_back("PLCR");
    } else if (state.compare("PLCL") == 0) {
        states.push_back("PLCL");
	if (this->vs > 10.0) {
            states.push_back("LCL");
        }
    } else if (state.compare("PLCR") == 0) {
         states.push_back("PLCR");
         if (this->vs > 10.0) {
            states.push_back("LCR");
         }
    } else if (state.compare("LCL") == 0) {
        states.push_back("LCL");
    } else if (state.compare("LCR") == 0) {
	states.push_back("LCR");
    }
    
    return states;
}

vector<vector<Vehicle>> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */

    vector<vector<Vehicle>> trajectory;
    if (state.compare("KL") == 0) {
        vector<vector<Vehicle>> options = keep_lane_trajectory(state, predictions);
        if (options.size() > 0) {
            for (int i = 0; i < options.size(); ++i) {
                trajectory.push_back(options[i]);
            }
        }
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
	vector<vector<Vehicle>> options = lane_change_trajectory(state, predictions);
        if (options.size() > 0) {
            for (int i = 0; i < options.size(); ++i) {
                trajectory.push_back(options[i]);
            }
        }
    } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        vector<vector<Vehicle>> options = prep_lane_change_trajectory(state, predictions);
        if (options.size() > 0) {
            for (int i = 0; i < options.size(); ++i) {
                trajectory.push_back(options[i]);
            }
        }
    }
    return trajectory;
}

vector<double> Vehicle::action(map<int, vector<Vehicle>> predictions, int lane, double interval, int choice) {
    /*
    Calculate the future position and speed based on the chosen acceleration.
    The acceleration options are between +/- max_acceleration.
    */
    double new_as = 0.0;
    new_as = this->max_acceleration - choice*(2*this->max_acceleration/(this->actions.size() - 1));
    double new_vs = min(this->vs + new_as*interval, this->target_speed);
    double new_s = this->s + new_vs*interval;
    return{new_s, new_vs, new_as};
}

vector<vector<Vehicle>> Vehicle::keep_lane_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a keep lane trajectory.
    */
    vector<vector<Vehicle>> trajectory;
    Vehicle vehicle_ahead;
    bool car_ahead = false;
    // Judge whether a car is detected ahead within the buffer distance or not
    if (get_vehicle_ahead(predictions, this->lane, vehicle_ahead)) {
        if (vehicle_ahead.s - this->s < this->preferred_buffer) {
	    car_ahead = true;
        }
    }
    int choice = 0;
    // If a car is detected ahead within the buffer range, only slowdown trajectories will be generated
    if (car_ahead) {
        choice = 3;
    }

    for(choice; choice < this->actions.size(); ++choice) {
        vector<Vehicle> option;
        vector<double> kinematics = action(predictions, this->lane, TIME_INTERVAL, choice);
        double new_s = kinematics[0];
        double new_vs = kinematics[1];
        double new_as = kinematics[2];
        if (new_s > this->s) {
            option.push_back(Vehicle(this->s, this->d, this->vs, this->vd, this->as, this->ad, state, this->goal_lane));
            double new_d = this->lane*LANE_WIDTH + LANE_WIDTH/2.0;
            option.push_back(Vehicle(new_s, new_d, new_vs, this->vd, new_as, this->ad, state, this->lane));
            trajectory.push_back(option);
	} 
    }

    return trajectory;
}

vector<vector<Vehicle>> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a trajectory preparing for a lane change.
    */
    double new_s;
    double new_vs;
    double new_as;
    Vehicle vehicle_behind;
    int new_lane = this->lane + lane_direction[state];

    vector<vector<Vehicle>> trajectory;
    if (new_lane < 0 || new_lane >= this->lanes_available) {
      return trajectory;
    }

    Vehicle vehicle_ahead;
    bool car_ahead = false;
    // Judge whether a car is detected ahead within the buffer distance or not
    if (get_vehicle_ahead(predictions, this->lane, vehicle_ahead)) {
        if (vehicle_ahead.s - this->s < this->preferred_buffer) {
	    car_ahead = true;
        }
    }
    int choice = 0;
    // If a car is detected ahead within the buffer range, only slowdown trajectories will be generated
    if (car_ahead) {
        choice = 3;
    }

    for(choice; choice < this->actions.size(); ++choice) {
        vector<Vehicle> option;
        vector<double> kinematics = action(predictions, this->lane, TIME_INTERVAL, choice);
        double new_s = kinematics[0];
        double new_vs = kinematics[1];
        double new_as = kinematics[2];
        if (new_s > this->s) {
            option.push_back(Vehicle(this->s, this->d, this->vs, this->vd, this->as, this->ad, state, this->goal_lane));
            double new_d = this->lane*LANE_WIDTH + LANE_WIDTH/2.0;
            option.push_back(Vehicle(new_s, new_d, new_vs, this->vd, new_as, this->ad, state, new_lane));
            trajectory.push_back(option);
	} 
    }

    return trajectory;
}

vector<vector<Vehicle>> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a lane change trajectory.
    */
    int new_lane = this->lane + lane_direction[state];
    if (new_lane != this->goal_lane) { //This might be during lane-changing
      new_lane = this->goal_lane;
    }

    vector<vector<Vehicle>> trajectory;
    if (new_lane < 0 || new_lane >= this->lanes_available) {
      return trajectory;
    }

    Vehicle next_lane_vehicle;
    //Check if a lane change is possible (check if another vehicle occupies that spot).
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        next_lane_vehicle = it->second[0];
        if (next_lane_vehicle.s < (this->s + CAR_LENGTH*2) && next_lane_vehicle.s > (this->s - CAR_LENGTH) && next_lane_vehicle.lane == new_lane) {
            //If lane change is not possible, return empty trajectory.
            return trajectory;
        }
    }

    Vehicle vehicle_ahead;
    bool car_ahead = false;
    if (get_vehicle_ahead(predictions, new_lane, vehicle_ahead)) {
	if (vehicle_ahead.s - this->s < this->preferred_buffer) {
	    car_ahead = true;
        }
    }
    int choice = 0;
    if (car_ahead) {
        choice = 3;
    }

    for(choice; choice < this->actions.size(); ++choice) {
        vector<double> kinematics = action(predictions, new_lane, TIME_INTERVAL, choice);
        double new_s = kinematics[0];
        double new_vs = kinematics[1];
        double new_as = kinematics[2];
        if (new_s > this->s) {
            // Three different d destination position options
	    for (int i = 1; i < 4; ++i) {
		vector<Vehicle> option;
                option.push_back(Vehicle(this->s, this->d, this->vs, this->vd, this->as, this->ad, state, this->goal_lane));
                double new_ad = 0.0;
	        double new_d = new_lane*LANE_WIDTH + i*LANE_WIDTH/4;
		double new_vd = new_d - this->d;
                option.push_back(Vehicle(new_s, new_d, new_vs, new_vd, new_as, new_ad, state, new_lane));
                trajectory.push_back(option);
            }
	} 
    }
    
    return trajectory;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    double max_s = -1.0;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
        if (temp_vehicle.lane == lane && temp_vehicle.s < this->s && temp_vehicle.s > max_s) {
            max_s = temp_vehicle.s;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    double min_s = 0.0;
    bool first = true;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
	if (temp_vehicle.lane == lane && temp_vehicle.s > this->s) {
	  if (first) {
	    min_s = temp_vehicle.s;
	    rVehicle = temp_vehicle;
	    found_vehicle = true;
            first = false;
	  } else {
	    if (temp_vehicle.s < min_s) {
              min_s = temp_vehicle.s;
              rVehicle = temp_vehicle;
              found_vehicle = true;
	    }
	  }
        }
    }
    return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(double interval, int horizon) {
    /*
    Generates predictions for non-ego vehicles to be used
    in trajectory generation for the ego vehicle.
    */
    vector<Vehicle> predictions;
    for(int i = 0; i < horizon; i++) {
      vector<double> cur_pos = position_at(i*interval);
      predictions.push_back(Vehicle(cur_pos[0], cur_pos[1], this->vs, this->vd, this->as, this->ad, this->state, this->goal_lane));
  	}
    return predictions;

}

vector<double> Vehicle::position_at(double t) {
    /*
    Calculate the future position given time
    */
    vector<double> position(2, 0.0);
    position[0] = this->s + this->vs*t + this->as*t*t/2.0;
    position[1] = this->d + this->vd*t + this->ad*t*t/2.0;
    return position;
}

void Vehicle::configure(vector<double> road_data) {
    /*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle. 
    */
    target_speed = road_data[0];
    lanes_available = road_data[1];
    max_acceleration = ACCELERATION_LIMIT;
}

void Vehicle::update_state(double s, double d, double vs, double vd) {
    /*
    Update the state of the ego vehicle
    */
    this->s = s;
    this->d = d;
    this->vs = vs;
    this->vd = vd;
    this->lane = (int)d/LANE_WIDTH;
    this->goal_s = this->s + this->target_speed*TIME_INTERVAL*2.0; // It's vitual goal
    this->preferred_buffer = this->vs*MPS_TO_KMPH/2.0;
}

