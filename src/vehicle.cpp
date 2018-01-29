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
#define LC_D_SPEED 2.0 //unit:m/s
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
    //max_acceleration = -1;
    this -> max_acceleration = ACCELERATION_LIMIT;
    this->lane = (int)d/LANE_WIDTH;
    this->goal_lane = target_lane;
    this->goal_s = this->s + SPEED_LIMIT*TIME_INTERVAL*2.0; // It's vitual goal
    this->target_speed = SPEED_LIMIT;
    //this->LC_finished = true;
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
    cout << "now state is " << this->state << ", have " << states.size() << " successor states" << endl;
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
	        cout << "state = " << trajectory[trajectory.size() - 1].state << ", cost = " << cost << endl;
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
    //return final_trajectories[best_idx];
}

vector<string> Vehicle::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM 
    discussed in the course, with the exception that lane changes happen 
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<string> states;
    //if (this->LC_finished) {
        states.push_back("KL");
    //}
    string state = this->state;
    if(state.compare("KL") == 0) {
        states.push_back("PLCL");
        states.push_back("PLCR");
    } else if (state.compare("PLCL") == 0) {
        //if (lane != lanes_available - 1) {
        states.push_back("PLCL");
	if (this->vs > 10.0) {
            states.push_back("LCL");
        }
        //}
    } else if (state.compare("PLCR") == 0) {
        //if (lane != 0) {
         states.push_back("PLCR");
         if (this->vs > 10.0) {
            states.push_back("LCR");
         }
        //}
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
    cout << "generate_trajectory....state = " << state << endl;
    vector<vector<Vehicle>> trajectory;
    if (state.compare("CS") == 0) {
        //trajectory.push_back(constant_speed_trajectory());
    } else if (state.compare("KL") == 0) {
        //trajectory = keep_lane_trajectory(predictions);
        vector<vector<Vehicle>> options = keep_lane_trajectory(predictions);
        if (options.size() > 0) {
            for (int i = 0; i < options.size(); ++i) {
                trajectory.push_back(options[i]);
            }
        }
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        //trajectory = lane_change_trajectory(state, predictions);
	vector<vector<Vehicle>> options = lane_change_trajectory(state, predictions);
        if (options.size() > 0) {
            for (int i = 0; i < options.size(); ++i) {
                trajectory.push_back(options[i]);
            }
        }
    } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        //trajectory = prep_lane_change_trajectory(state, predictions);
        vector<vector<Vehicle>> options = prep_lane_change_trajectory(state, predictions);
        if (options.size() > 0) {
            for (int i = 0; i < options.size(); ++i) {
                trajectory.push_back(options[i]);
            }
        }
    }
    return trajectory;
}

vector<double> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane, double interval) {
    /* 
    Gets next timestep kinematics (position, velocity, acceleration) 
    for a given lane. Tries to choose the maximum velocity and acceleration, 
    given other vehicle positions and accel/velocity constraints.
    */
    //float max_velocity_accel_limit = this->max_acceleration + this->v;
    //cout << "entering gk..... vs = " << this->vs << endl;
    double max_velocity_accel_limit = this->max_acceleration*interval + this->vs;
    double new_s;
    double new_vs;
    double new_as;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;

    if (get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
	
	//cout << "vehicle_ahead.vs is " << vehicle_ahead.vs << endl;
        if (get_vehicle_behind(predictions, lane, vehicle_behind)) {
            //new_vs = vehicle_ahead.vs; //must travel at the speed of traffic, regardless of preferred buffer
	    new_vs = min(vehicle_ahead.vs, this->target_speed);
	    //cout << "gk_1: new_vs = " << new_vs << endl;
        } else {
	    /*
            double max_velocity_in_front = ((vehicle_ahead.s - this->s - this->preferred_buffer) + vehicle_ahead.vs*interval - 0.5*(this->as)*interval*interval)/interval;
            new_vs = min(min(max_velocity_in_front, max_velocity_accel_limit), this->target_speed);
	    cout << "gk_2: new _vs = " << new_vs << ", max_velocity_in_front = " << max_velocity_in_front << ", max_velocity_accel_limit = " << max_velocity_accel_limit << ", target_speed = " << this->target_speed << endl;*/
	    if (vehicle_ahead.s - this->s < 20.0) {
	      double max_velocity_in_front = ((vehicle_ahead.s - this->s - 20.0) + vehicle_ahead.vs*interval - 0.5*(this->as)*interval*interval)/interval;
	      new_vs = min(min(vehicle_ahead.vs, this->target_speed), max_velocity_in_front);
	      //cout << "gk_2-1: new_vs = " << new_vs << endl;
	    } else {
	      new_vs = min(max_velocity_accel_limit, this->target_speed);
	      //cout << "gk_2-2: new_vs = " << new_vs << endl;
	    }
        }
    } else {
        new_vs = min(max_velocity_accel_limit, this->target_speed);
        //cout << "gk_3: new _vs = " << new_vs << ", max_velocity_accel_limit = " << max_velocity_accel_limit << ", target_speed = " << this->target_speed << endl;
    }
    
    new_as = (new_vs - this->vs)/interval; //Equation: (v_1 - v_0)/t = acceleration
    //if (new_as < 0) {
    //  new_as = 0.0;
    //}
    //cout << "new_as = " << new_as << endl;
    //new_s = this->s + new_vs*interval + new_as*interval*interval/2.0;
    new_s = this->s + new_vs*interval + new_as*interval*interval/2.0;
    return{new_s, new_vs, new_as};
    
}

vector<double> Vehicle::action(map<int, vector<Vehicle>> predictions, int lane, double interval, int choice) {
    double new_as = 0.0;
    
    //if (action.compare("SPEEDUP")) {
    if(choice == 0) {
        new_as = this->max_acceleration;
    //} else if (action.compare("SPEEDDOWN")) {
    } else if (choice == 2) {
        new_as = -this->max_acceleration;
    }

    double new_vs = min(this->vs + new_as*interval, this->target_speed);
    //double new_s = this->s + new_vs*interval + new_as*interval*interval/2.0;
    double new_s = this->s + new_vs*interval;

    cout << "new_vs is " << new_vs << ", new_as is " << new_as << endl;
    return{new_s, new_vs, new_as};
}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
    /*
    Generate a constant speed trajectory.
    */
/*
    float next_pos = position_at(1);
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state), 
                                  Vehicle(this->lane, next_pos, this->v, 0, this->state)};
    return trajectory;
*/
    vector<Vehicle> trajectory;
    return trajectory;
}

vector<vector<Vehicle>> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions) {
    /*
    Generate a keep lane trajectory.
    */
    //vector<Vehicle> trajectory = {Vehicle(this->s, this->d, this->vs, this->vd, this->as, this->ad, state, this->goal_lane)};
    vector<vector<Vehicle>> trajectory;
    Vehicle vehicle_ahead;
    bool car_ahead = false;
    if (get_vehicle_ahead(predictions, this->lane, vehicle_ahead)) {
        if (vehicle_ahead.s - this->s < SAFE_DISTANCE) {
	    car_ahead = true;
        }
    }
    int choice = 0;
    if (car_ahead) {
        choice = 2;
    }

    for(choice; choice < this->actions.size(); ++choice) {
        vector<Vehicle> option;
        cout << "this->actions[choice] is " << this->actions[choice] << endl;
        vector<double> kinematics = action(predictions, this->lane, TIME_INTERVAL, choice);
        double new_s = kinematics[0];
        double new_vs = kinematics[1];
        double new_as = kinematics[2];
        cout << "new_s is " << new_s << ", this->s is " << this->s << endl;
        if (new_s > this->s) {
            option.push_back(Vehicle(this->s, this->d, this->vs, this->vd, this->as, this->ad, state, this->goal_lane));
            double new_d = this->lane*LANE_WIDTH + LANE_WIDTH/2.0;
            option.push_back(Vehicle(new_s, new_d, new_vs, this->vd, new_as, this->ad, "KL", this->lane));
            trajectory.push_back(option);
	} 
    }

    return trajectory;

    /*
    vector<double> kinematics = get_kinematics(predictions, this->lane, TIME_INTERVAL);
    double new_s = kinematics[0];
    double new_vs = kinematics[1];
    double new_as = kinematics[2];
    if (new_s > this->s + 1.0) {
      trajectory.push_back(Vehicle(this->s, this->d, this->vs, this->vd, this->as, this->ad, state, this->goal_lane));
      double new_d = this->lane*LANE_WIDTH + LANE_WIDTH/2.0;
      //cout << "lane is " << this->lane << endl;
      //cout << "new_s is " << new_s << ", new_d is " <<new_d << endl;
      trajectory.push_back(Vehicle(new_s, new_d, new_vs, this->vd, new_as, this->ad, "KL", this->lane));
    }
    return trajectory;*/
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
    //this->goal_lane = new_lane;

    vector<vector<Vehicle>> trajectory;
    if (new_lane < 0 || new_lane > 2) {
      return trajectory;
    }

    Vehicle vehicle_ahead;
    bool car_ahead = false;
    if (get_vehicle_ahead(predictions, this->lane, vehicle_ahead)) {
        if (vehicle_ahead.s - this->s < SAFE_DISTANCE) {
	    car_ahead = true;
        }
    }
    int choice = 0;
    if (car_ahead) {
        choice = 2;
    }

    for(choice; choice < this->actions.size(); ++choice) {
        vector<Vehicle> option;
        //cout << "this->actions[choice] is " << this->actions[choice] << endl;
        vector<double> kinematics = action(predictions, this->lane, TIME_INTERVAL, choice);
        double new_s = kinematics[0];
        double new_vs = kinematics[1];
        double new_as = kinematics[2];
        //cout << "new_s is " << new_s << ", this->s is " << this->s << endl;
        if (new_s > this->s) {
            option.push_back(Vehicle(this->s, this->d, this->vs, this->vd, this->as, this->ad, state, this->goal_lane));
            double new_d = this->lane*LANE_WIDTH + LANE_WIDTH/2.0;
            option.push_back(Vehicle(new_s, new_d, new_vs, this->vd, new_as, this->ad, state, new_lane));
            trajectory.push_back(option);
	} 
    }

    /*
    trajectory.push_back(Vehicle(this->s, this->d, this->vs, this->vd, this->as, this->ad, state, this->goal_lane));

    vector<double> curr_lane_new_kinematics = get_kinematics(predictions, this->lane, TIME_INTERVAL);

    if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
        //Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_vs = curr_lane_new_kinematics[1];
        new_as = curr_lane_new_kinematics[2];
        
    } else {
        vector<double> best_kinematics;
        vector<double> next_lane_new_kinematics = get_kinematics(predictions, new_lane, TIME_INTERVAL);
        //Choose kinematics with lowest velocity.
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
            best_kinematics = next_lane_new_kinematics;
        } else {
            best_kinematics = curr_lane_new_kinematics;
        }
        new_s = best_kinematics[0];
        new_vs = best_kinematics[1];
        new_as = best_kinematics[2];
    }

    //trajectory.push_back(Vehicle(this->lane, new_s, new_v, new_a, state));
    double new_d = this->lane*LANE_WIDTH + LANE_WIDTH/2.0;
    trajectory.push_back(Vehicle(new_s, new_d, new_vs, this->vd, new_as, this->ad, state, new_lane));
    */
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
    if (new_lane < 0 || new_lane > 2) {
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
        if (vehicle_ahead.s - this->s < SAFE_DISTANCE) {
	    car_ahead = true;
        }
    }
    int choice = 0;
    if (car_ahead) {
        choice = 2;
    }

    for(choice; choice < this->actions.size(); ++choice) {
        vector<Vehicle> option;
        //cout << "this->actions[choice] is " << this->actions[choice] << endl;
        vector<double> kinematics = action(predictions, new_lane, TIME_INTERVAL, choice);
        double new_s = kinematics[0];
        double new_vs = kinematics[1];
        double new_as = kinematics[2];
        //cout << "new_s is " << new_s << ", this->s is " << this->s << endl;
        if (new_s > this->s) {
            option.push_back(Vehicle(this->s, this->d, this->vs, this->vd, this->as, this->ad, state, this->goal_lane));
            double new_ad = 0.0;
	    //double new_vd = LC_D_SPEED*lane_direction[state];
	    //double new_d = this->d + new_vd*TIME_INTERVAL;
	    double new_vd = 0.0;
	    double new_d = new_lane*LANE_WIDTH + LANE_WIDTH/2;
            option.push_back(Vehicle(new_s, new_d, new_vs, new_vd, new_as, new_ad, state, new_lane));
            trajectory.push_back(option);
	} 
    }
    
    /*
    trajectory.push_back(Vehicle(this->s, this->d, this->vs, this->vd, this->as, this->ad, state, this->goal_lane));
    vector<double> kinematics = get_kinematics(predictions, new_lane, TIME_INTERVAL);
    
    double new_ad = 0.0;
    double new_vd = LC_D_SPEED*lane_direction[state];
    double new_d = this->d + new_vd*TIME_INTERVAL;
    trajectory.push_back(Vehicle(kinematics[0], new_d, kinematics[1], new_vd, kinematics[2], new_ad, state, new_lane));
    */
    return trajectory;
}

void Vehicle::increment(int dt = 1) {
    double span = dt*TIME_INTERVAL;
    vector<double> position = position_at(span);
    this->s = position[0];
    this->d = position[1];
}

vector<double> Vehicle::position_at(double t) {
    vector<double> position(2, 0.0);
    //return this->s + this->v*t + this->a*t*t/2.0;
    position[0] = this->s + this->vs*t + this->as*t*t/2.0;
    position[1] = this->d + this->vd*t + this->ad*t*t/2.0;
    return position;
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
        //if (temp_vehicle.lane == this->lane && temp_vehicle.s > this->s && temp_vehicle.s < min_s) {
	if (temp_vehicle.lane == lane && temp_vehicle.s > this->s) {
	  cout << "lane " << lane <<" ahead v is " << temp_vehicle.vs << ", s is " << temp_vehicle.s << endl;
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
      //float next_s = position_at(i*interval);
      vector<double> cur_pos = position_at(i*interval);
      //float next_v = 0;
      //if (i < horizon-1) {
        //next_v = position_at((i+1)*interval) - s;
        //next_v = (position_at((i+1)*interval) - next_s)/interval; 
      //}
      //predictions.push_back(Vehicle(this->lane, next_s, next_v, 0));
      predictions.push_back(Vehicle(cur_pos[0], cur_pos[1], this->vs, this->vd, this->as, this->ad, this->state, this->goal_lane));
  	}
    return predictions;

}

void Vehicle::realize_next_state(vector<Vehicle> trajectory) {
    /*
    Sets state and kinematics for ego vehicle using the last state of the trajectory.
    */
/*
    Vehicle next_state = trajectory[1];
    this->state = next_state.state;
    this->lane = next_state.lane;
    this->s = next_state.s;
    this->v = next_state.v;
    this->a = next_state.a;
*/
}

void Vehicle::configure(vector<int> road_data) {
    /*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle. 
    */
    target_speed = road_data[0];
    lanes_available = road_data[1];
    goal_s = road_data[2];
    goal_lane = road_data[3];
    max_acceleration = road_data[4];
}

void Vehicle::update_state(double s, double d, double vs, double vd) {
    this->s = s;
    this->d = d;
    this->vs = vs;
    this->vd = vd;
    this->lane = (int)d/LANE_WIDTH;
    this->goal_s = this->s + SPEED_LIMIT*TIME_INTERVAL*2.0; // It's vitual goal
    this -> max_acceleration = ACCELERATION_LIMIT;
    this->target_speed = SPEED_LIMIT;
    /*
    if (this->lane = this->goal_lane) {
        this->LC_finished = true;
    } else {
	this->LC_finished = false;
    }*/
}

