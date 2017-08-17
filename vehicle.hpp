//
//  vehicle.hpp
//  planar
//
//  Created by 村上候助 on 2017/08/09.
//  Copyright © 2017 村上候助. All rights reserved.
//
#ifndef VEHICLE_H
#define VEHICLE_H

#include <stdio.h>

#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <map>
#include <string>
#include <iterator>

using namespace std;

struct trajectory_data{
    int proposed_lane;
    float avg_speed;
    float max_accerelation;
    float rms_accerelation;
    int closest_approach;
    int end_distance_to_goal;
    int end_lanes_from_goal;
    bool collides;
};


class Vehicle {
public:
    
    struct collider{
        
        bool collision ; // is there a collision?
        int  time; // time collision happens
        
    };
    
    int L = 1;
    
    int preferred_buffer = 6; // impacts "keep lane" behavior.
    
    int lane;
    
    int s;
    
    int v;
    
    int a;
    
    int target_speed;
    
    int lanes_available;
    
    int max_acceleration;
    
    int goal_lane;
    
    int goal_s;
    
    string state;
    
    /**
     * Constructor
     */
    Vehicle(int lane, int s, int v, int a);
    
    /**
     * Destructor
     */
    virtual ~Vehicle();
    
    void update_state(map<int, vector <vector<int> > > predictions);
    
    float calculate_cost(vector<Vehicle> trajectory, map<int,vector < vector<int> > > predictions);
    
    trajectory_data get_helper_data(vector<Vehicle> trajectory, map<int,vector < vector<int> > > predictions );
    
    map<int,vector < vector<int> > > filter_predictions_by_lane(map<int,vector < vector<int> >> predictions, int lane);
    
    bool check_collision(Vehicle target, int s_previous, int s_now);
    
    void configure(vector<int> road_data);
    
    string display();
    
    void increment(int dt);
    
    vector<int> state_at(int t);
    
    vector<Vehicle> get_trajectory_for_state(string state, map<int,vector < vector<int> > > predictions);
    
    bool collides_with(Vehicle other, int at_time);
    
    collider will_collide_with(Vehicle other, int timesteps);
    
    void realize_state(map<int, vector < vector<int> > > predictions);
    
    void realize_constant_speed();
    
    int _max_accel_for_lane(map<int,vector<vector<int> > > predictions, int lane, int s);
    
    void realize_keep_lane(map<int, vector< vector<int> > > predictions);
    
    void realize_lane_change(map<int,vector< vector<int> > > predictions, string direction);
    
    void realize_prep_lane_change(map<int,vector< vector<int> > > predictions, string direction);
    
    vector<vector<int> > generate_predictions(int horizon);
    
};
#endif

