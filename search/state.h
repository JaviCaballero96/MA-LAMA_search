/*********************************************************************
 * Author: Malte Helmert (helmert@informatik.uni-freiburg.de)
 * (C) Copyright 2003-2004 Malte Helmert
 * Modified by: Silvia Richter (silvia.richter@nicta.com.au),
 *              Matthias Westphal (westpham@informatik.uni-freiburg.de)             
 * (C) Copyright 2008 NICTA and Matthias Westphal
 *
 * This file is part of LAMA.
 *
 * LAMA is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 3
 * of the license, or (at your option) any later version.
 *
 * LAMA is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 *********************************************************************/

#ifndef STATE_H
#define STATE_H

#include <iostream>
#include <vector>
#include <ext/hash_set>
#include "landmarks_types.h"
#include <string>

using namespace std;
using namespace __gnu_cxx;

class Operator;
class PrePost;
class LandmarkNode;

typedef struct{
	int var;
	int val;
	int block_type;
	float time_freed;
	float time_set;
	string non_temporal_action_name;
} blocked_var;

typedef struct{
	float time_start;
	float time_end;
	string non_temporal_action_name;
	vector<PrePost*> functional_costs;
} runn_action;

class State {
    friend void read_everything(istream &in, bool generate_landmarks, bool reasonable_orders, bool read_init_state, bool read_runtime_constraints);
    vector<int> vars; // values for vars
    hash_set<const LandmarkNode *, hash_pointer> reached_lms;
    int reached_lms_cost;

    float g_value; // min. cost of reaching this state from the initial state
    float g_time_value;
    float g_current_time_value;
    void set_landmarks_for_initial_state();
    void update_reached_lms(const Operator &op);
    bool landmark_is_leaf(const LandmarkNode& node, 
			  const hash_set<const LandmarkNode*, hash_pointer>& reached) const;
    bool check_lost_landmark_children_needed_again(const LandmarkNode& node) const;
    
public:
    int applied_actions;
    vector<blocked_var> blocked_vars;
    vector<runn_action> running_actions;
    vector<float> numeric_vars_val;
    vector<string> applied_actions_vec;
    vector<pair<int, int> > timed_goals_obtained;
    State(istream &in);
    // State(const State &origin);
    State(const State &predecessor, const Operator &op);
    int &operator[](int index) {
	return vars[index];
    }
    int operator[](int index) const {
	return vars[index];
    }
    void set_var_value(int index, int value) {
    	vars[index] = value;
    }
    void dump() const;
    bool operator<(const State &other) const;

    float get_g_value() const {return g_value;}
    float get_g_time_value() const {return g_time_value;}
    float get_g_current_time_value() const {return g_current_time_value;}
    void  set_g_time_value(float time) {g_time_value = time;}
    void  set_g_current_time_value(float time) {g_current_time_value = time;}
    void change_ancestor(const State &new_predecessor, const Operator &new_op);
    vector<int> get_vars_state(){return vars;};
    vector<float> get_num_vars_state(){return numeric_vars_val;};

    int check_partial_plan(hash_set<const LandmarkNode*, hash_pointer>& reached) const;
    int get_needed_landmarks(hash_set<const LandmarkNode*, hash_pointer>& needed) const;
    template <typename T>
    T calculate_runtime_efect(string s_effect) const;
};

float get_new_time_window(Operator op, State* curr, float op_duration, vector<pair<int, float>* > ex_const_vector, int value);

#endif
