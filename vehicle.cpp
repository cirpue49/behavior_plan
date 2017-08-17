//
//  vehicle.cpp
//  planar
//
//  Created by 村上候助 on 2017/08/09.
//  Copyright © 2017 村上候助. All rights reserved.
//

#include <iostream>
#include "vehicle.hpp"
#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>

/**
 * Initializes Vehicle
 */
Vehicle::Vehicle(int lane, int s, int v, int a) {
    
    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    state = "CS";
    max_acceleration = -1;
    
}

Vehicle::~Vehicle() {}

// TODO - Implement this method.


void Vehicle::update_state(map<int,vector < vector<int> > > predictions) {
    /*
     Updates the "state" of the vehicle by assigning one of the
     following values to 'self.state':
     
     "KL" - Keep Lane
     - The vehicle will attempt to drive its target speed, unless there is
     traffic in front of it, in which case it will slow down.
     
     "LCL" or "LCR" - Lane Change Left / Right
     - The vehicle will IMMEDIATELY change lanes and then follow longitudinal
     behavior for the "KL" state in the new lane.
     
     "PLCL" or "PLCR" - Prepare for Lane Change Left / Right
     - The vehicle will find the nearest vehicle in the adjacent lane which is
     BEHIND itself and will adjust speed to try to get behind that vehicle.
     
     INPUTS
     - predictions
     A dictionary. The keys are ids of other vehicles and the values are arrays
     where each entry corresponds to the vehicle's predicted location at the
     corresponding timestep. The FIRST element in the array gives the vehicle's
     current position. Example (showing a car with id 3 moving at 2 m/s):
     
     {
     3 : [
     {"s" : 4, "lane": 0},
     {"s" : 6, "lane": 0},
     {"s" : 8, "lane": 0},
     {"s" : 10, "lane": 0},
     ]
     }
     
     */
    cout << state << endl;
    vector<string> states{"KL", "LCL", "LCR"};
    if (lane == 0){states.erase(states.begin()+1);}
    else if(lane == 3){states.erase(states.begin()+2);}
    
    vector<float> costs;
    vector<Vehicle> trajectory;
    for(int i = 0; i < states.size(); i++){
        vector<Vehicle> trajectory = get_trajectory_for_state(states[i], predictions);
        float cost = calculate_cost(trajectory, predictions);
    }
    
    state = "KL"; // this is an example of how you change state.
    
    
}


vector<Vehicle> Vehicle::get_trajectory_for_state(string possible_state, map<int,vector < vector<int> > > predictions){
    Vehicle vehicle_instance = Vehicle(this->lane,this->s,this->v, this->a);
    vehicle_instance.state = this->state;
    //SPEED_LIMIT = 10 from main.cpp
    vehicle_instance.target_speed = 10;
    //MAX_ACCEL = 2 from main.cpp
    vehicle_instance.max_acceleration = 2;
    vector<Vehicle> trajectory{vehicle_instance};
    
    //change staten
    vehicle_instance.state = possible_state;
    //update acceleration
    vehicle_instance.realize_state(predictions);
    //one step forward
    int dt = 1;
    vehicle_instance.increment(dt);
    
    trajectory.push_back(vehicle_instance);
    return trajectory;
}

float Vehicle::calculate_cost(vector<Vehicle> trajectory, map<int,vector < vector<int> > > predictions){
    float cost = __FLT_MAX__;
    trajectory_data t_d = get_helper_data(trajectory, predictions);
    return cost;
}

//sub function for calculate_cost
trajectory_data Vehicle::get_helper_data(vector<Vehicle> trajectory, map<int,vector < vector<int> > > predictions ){
    trajectory_data t;
    
    Vehicle first = trajectory[0];
    Vehicle second = trajectory[1];
    int end_distance_to_goal = this->goal_s - second.s;
    int end_lanes_from_goal = abs(this->goal_lane - second.lane);
    float dt = trajectory.size();
    float avg_speed = (second.s - first.s)/dt;
    
    vector<double> accels;
    float closest_approach = 999999;
    
    int first_lane = first.lane;
    map<int,vector < vector<int> > > filtered_predictions  = filter_predictions_by_lane(predictions, first_lane);
    
    //trajectory at time = 1
    int s = second.s;
    int a = second.a;
    
    //iterate filtered object
    map<int, vector<vector<int> > >::iterator it = filtered_predictions.begin();
    while(it != filtered_predictions.end())
    {
        int s_now = it->second[1][1];
        int s_previous = it->second[0][1];
        bool vehicle_collision = check_collision( *this, s_previous, s_now);
        if(vehicle_collision){
            t.collides = vehicle_collision;
        }
        int dist = abs(s_now - s);
        if (dist  < closest_approach){
            closest_approach = dist;
        }
        it++;
    }
    t.proposed_lane = first_lane;
    t.avg_speed = avg_speed;
    t.max_accerelation = a;
    t.rms_accerelation = a*a;
    t.closest_approach = closest_approach;
    t.end_distance_to_goal = end_distance_to_goal;
    t.end_lanes_from_goal = end_lanes_from_goal;
    return t;
}

//called from get_helper_data function
map<int,vector < vector<int> > > Vehicle::filter_predictions_by_lane(map<int,vector<vector<int>>> predictions, int lane){
    map<int, vector<vector<int> > >::iterator it = predictions.begin();
    map<int,vector<vector<int> > > filtered;
    while(it != predictions.end())
    {
        int v_id = it->first;
        vector<vector<int> > v = it->second;
        if((v[0][0] == lane) && (v_id != -1))
        {
            filtered[v_id] = v;
        }
        it++;
    }
    return filtered;
}


//implementation python check collision function
//called from get_helper_data function
bool Vehicle::check_collision(Vehicle target, int s_previous, int s_now){
    int s = target.s;
    int v = target.v;
    int v_target = s_now - s_previous;
    if(s_previous < s){
        if(s_now >= s){
            return true;
        }
        else{
            return false;
        }
    }
    if (s_previous > s){
        if(s_now <= s){
            return true;
        }
        else{
            return false;
        }
    }
    if (s_previous == s){
        if( v_target  > v){
            return false;
        }
        else{
            return true;
        }
    }
    //never reach here
    cout << "Need to debug Vehicle::check_collision function" << endl;
    return false;
}


void Vehicle::configure(vector<int> road_data) {
    /*
     Called by simulator before simulation begins. Sets various
     parameters which will impact the ego vehicle.
     */
    target_speed = road_data[0];    lanes_available = road_data[1];
    max_acceleration = road_data[2];
    goal_lane = road_data[3];
    goal_s = road_data[4];
}

string Vehicle::display() {
    
    ostringstream oss;
    
    oss << "s:    " << this->s << "\n";
    oss << "lane: " << this->lane << "\n";
    oss << "v:    " << this->v << "\n";
    oss << "a:    " << this->a << "\n";
    
    return oss.str();
}

void Vehicle::increment(int dt = 1) {
    
    this->s += this->v * dt;
    this->v += this->a * dt;
}

vector<int> Vehicle::state_at(int t) {
    
    /*
     Predicts state of vehicle in t seconds (assuming constant acceleration)
     */
    int s = this->s + this->v * t + this->a * t * t / 2;
    int v = this->v + this->a * t;
    return {this->lane, s, v, this->a};
}


bool Vehicle::collides_with(Vehicle other, int at_time) {
    
    /*
     Simple collision detection.
     */
    vector<int> check1 = state_at(at_time);
    vector<int> check2 = other.state_at(at_time);
    return (check1[0] == check2[0]) && (abs(check1[1]-check2[1]) <= L);
}

Vehicle::collider Vehicle::will_collide_with(Vehicle other, int timesteps) {
    
    Vehicle::collider collider_temp;
    collider_temp.collision = false;
    collider_temp.time = -1;
    
    for (int t = 0; t < timesteps+1; t++)
    {
        if( collides_with(other, t) )
        {
            collider_temp.collision = true;
            collider_temp.time = t;
            return collider_temp;
        }
    }
    
    return collider_temp;
}

void Vehicle::realize_state(map<int,vector < vector<int> > > predictions) {
    
    /*
     Given a state, realize it by adjusting acceleration and lane.
     Note - lane changes happen instantaneously.
     */
    string state = this->state;
    if(state.compare("CS") == 0)
    {
        realize_constant_speed();
    }
    else if(state.compare("KL") == 0)
    {
        realize_keep_lane(predictions);
    }
    else if(state.compare("LCL") == 0)
    {
        realize_lane_change(predictions, "L");
    }
    else if(state.compare("LCR") == 0)
    {
        realize_lane_change(predictions, "R");
    }
    else if(state.compare("PLCL") == 0)
    {
        realize_prep_lane_change(predictions, "L");
    }
    else if(state.compare("PLCR") == 0)
    {
        realize_prep_lane_change(predictions, "R");
    }
    
}

void Vehicle::realize_constant_speed() {
    a = 0;
}

int Vehicle::_max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s) {
    
    int delta_v_til_target = target_speed - v;
    int max_acc = min(max_acceleration, delta_v_til_target);
    
    map<int, vector<vector<int> > >::iterator it = predictions.begin();
    vector<vector<vector<int> > > in_front;
    while(it != predictions.end())
    {
        
        int v_id = it->first;
        
        vector<vector<int> > v = it->second;
        
        if((v[0][0] == lane) && (v[0][1] > s))
        {
            in_front.push_back(v);
            
        }
        it++;
    }
    
    if(in_front.size() > 0)
    {
        int min_s = 1000;
        vector<vector<int>> leading = {};
        for(int i = 0; i < in_front.size(); i++)
        {
            if((in_front[i][0][1]-s) < min_s)
            {
                min_s = (in_front[i][0][1]-s);
                leading = in_front[i];
            }
        }
        
        int next_pos = leading[1][1];
        int my_next = s + this->v;
        int separation_next = next_pos - my_next;
        int available_room = separation_next - preferred_buffer;
        max_acc = min(max_acc, available_room);
    }
    
    return max_acc;
    
}

void Vehicle::realize_keep_lane(map<int,vector< vector<int> > > predictions) {
    this->a = _max_accel_for_lane(predictions, this->lane, this->s);
}

void Vehicle::realize_lane_change(map<int,vector< vector<int> > > predictions, string direction) {
    int delta = -1;
    if (direction.compare("L") == 0)
    {
        delta = 1;
    }
    this->lane += delta;
    int lane = this->lane;
    int s = this->s;
    this->a = _max_accel_for_lane(predictions, lane, s);
}

void Vehicle::realize_prep_lane_change(map<int,vector<vector<int> > > predictions, string direction) {
    int delta = -1;
    if (direction.compare("L") == 0)
    {
        delta = 1;
    }
    int lane = this->lane + delta;
    
    map<int, vector<vector<int> > >::iterator it = predictions.begin();
    vector<vector<vector<int> > > at_behind;
    while(it != predictions.end())
    {
        int v_id = it->first;
        vector<vector<int> > v = it->second;
        
        if((v[0][0] == lane) && (v[0][1] <= this->s))
        {
            at_behind.push_back(v);
            
        }
        it++;
    }
    if(at_behind.size() > 0)
    {
        
        int max_s = -1000;
        vector<vector<int> > nearest_behind = {};
        for(int i = 0; i < at_behind.size(); i++)
        {
            if((at_behind[i][0][1]) > max_s)
            {
                max_s = at_behind[i][0][1];
                nearest_behind = at_behind[i];
            }
        }
        int target_vel = nearest_behind[1][1] - nearest_behind[0][1];
        int delta_v = this->v - target_vel;
        int delta_s = this->s - nearest_behind[0][1];
        if(delta_v != 0)
        {
            
            int time = -2 * delta_s/delta_v;
            int a;
            if (time == 0)
            {
                a = this->a;
            }
            else
            {
                a = delta_v/time;
            }
            if(a > this->max_acceleration)
            {
                a = this->max_acceleration;
            }
            if(a < -this->max_acceleration)
            {
                a = -this->max_acceleration;
            }
            this->a = a;
        }
        else
        {
            int my_min_acc = max(-this->max_acceleration,-delta_s);
            this->a = my_min_acc;
        }
        
    }
    
}

vector<vector<int> > Vehicle::generate_predictions(int horizon = 10) {
    
    vector<vector<int> > predictions;
    for( int i = 0; i < horizon; i++)
    {
        vector<int> check1 = state_at(i);
        vector<int> lane_s = {check1[0], check1[1]};
        predictions.push_back(lane_s);
    }
    return predictions;
    
}

