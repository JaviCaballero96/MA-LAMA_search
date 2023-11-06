/*********************************************************************
 * Author: Malte Helmert (helmert@informatik.uni-freiburg.de)
 * (C) Copyright 2003-2004 Malte Helmert
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

#ifndef SEARCH_ENGINE_H
#define SEARCH_ENGINE_H

#include <vector>
#include "state.h"

class Operator;

class SearchEngine {
public:
    typedef std::vector<const Operator *> Plan;
private:
    bool solved;
    Plan plan;
    float plan_cost;
    vector<float> plan_tamporal_info;
    vector<float> plan_duration_info;
    vector<float> plan_cost_info;
    vector<int> vars_end_state;
    vector<float> num_vars_end_state;
    vector<vector<blocked_var> > blocked_vars_info;
protected:
    enum {FAILED, SOLVED, IN_PROGRESS};
    virtual void initialize() {}
    virtual int step() = 0;

    void set_plan(const Plan &plan);
public:
    SearchEngine();
    virtual ~SearchEngine();
    virtual float statistics() const;
    bool found_solution() const;
    const Plan &get_plan() const;
    void search();
    float get_plan_cost(){return plan_cost;};
    void set_plan_cost(float cost){plan_cost = cost;};
    void set_plan_temporal_info(vector<float> plan_temp){plan_tamporal_info = plan_temp;};
    void set_plan_duration_info(vector<float> plan_temp){plan_duration_info = plan_temp;};
    vector<float>  get_plan_duration_info(){return plan_duration_info;};
    vector<float>  get_plan_temporal_info(){return plan_tamporal_info;};
    void set_plan_cost_info(vector<float> plan_temp){plan_cost_info = plan_temp;};
    void set_vars_end_state(vector<int> vars_state){vars_end_state = vars_state;}
    void set_blocked_vars_info(vector<vector<blocked_var> > blocked_vars) {blocked_vars_info = blocked_vars;}
    void set_num_vars_end_state(vector<float> num_vars_state){num_vars_end_state = num_vars_state;}
	vector<float> get_plan_cost_info(){return plan_cost_info;};
    vector<int> get_end_state(){return vars_end_state;};
    vector<float> get_num_end_state(){return num_vars_end_state;};
    vector<vector<blocked_var> > get_blocked_vars_info(){return blocked_vars_info;};
};

#endif
