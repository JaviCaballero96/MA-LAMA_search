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

#include "state.h"

#include "axioms.h"
#include "globals.h"
#include "operator.h"
#include "landmarks_graph.h"
#include "float.h"
#include "ff_heuristic.h"
#include "exprtk.cc"

#include <algorithm>
#include <iostream>
#include <cassert>
#include<sstream>
#include <algorithm>
using namespace std;

State::State(istream &in) {
    check_magic(in, "begin_state");
    for(int i = 0; i < g_variable_domain.size(); i++) {
    	int var;
    	float var_val;
    	in >> var;
    	if (var == -1){
    		vars.push_back(var);
    		in >> var_val;
    		numeric_vars_val.push_back(var_val);
    	}else{
    		vars.push_back(var);
    		numeric_vars_val.push_back(FLT_MAX);
    	}
    }
    check_magic(in, "end_state");

    g_default_axiom_values = vars;
    applied_actions = 0;
    g_value = 0;
    g_current_time_value = 0;
    g_time_value = 0;
    reached_lms_cost = 0;
}

void State::update_reached_lms(const Operator &op) {
    if(g_lgraph == NULL)
	return;
    for(int j = 0; j < op.get_pre_post().size(); j++) {
	const PrePost &pre_post = op.get_pre_post()[j];
	// Test whether this effect got applied (it may have been conditional)
	if((*this)[pre_post.var] == pre_post.post) {
	    const LandmarkNode* node_p = 
		g_lgraph->landmark_reached(make_pair(pre_post.var, pre_post.post));
	    if(node_p != 0) {
		if(reached_lms.find(node_p) == reached_lms.end()) {
		    //cout << endl << "New LM reached! +++++++ ";
		    //g_lgraph->dump_node(node_p);
		    // Only add leaves of landmark graph to reached
		    const LandmarkNode& node = *node_p;
		    if(landmark_is_leaf(node, reached_lms)) { 
			//cout << "inserting new LM into reached." << endl;
			reached_lms.insert(node_p);
			reached_lms_cost += node_p->min_cost;
		    } 
		    //else
		    //cout << "not inserting into reached, has parents" << endl;
		}
	    }
	}
    }
    const set<LandmarkNode*>& nodes = g_lgraph->get_nodes();
    set<LandmarkNode*>::const_iterator it = nodes.begin();
    for(; it != nodes.end(); ++it) {
	const LandmarkNode* node = *it;
	for(int i = 0; i < node->vars.size(); i++) {
	    if((*this)[node->vars[i]] == node->vals[i]) {
		if(reached_lms.find(node) == reached_lms.end()) {
		    // cout << "New LM reached by axioms: "; g_lgraph->dump_node(node);
		    if(landmark_is_leaf(*node, reached_lms)) { 
			// cout << "inserting new LM into reached. (2)" << endl;
			reached_lms.insert(node);  
			reached_lms_cost += node->min_cost;
		    } 
		    // else
			// cout << "not inserting into reached, has parents" << endl;
		}
	    }
	}
    }
}

void State::change_ancestor(const State &new_predecessor, const Operator &new_op) {
    reached_lms = new_predecessor.reached_lms; // Can this be a problem?
    reached_lms_cost = new_predecessor.reached_lms_cost;
    update_reached_lms(new_op);
    if(g_length_metric)
    	g_value = new_predecessor.get_g_value() + 2;
    else
    	g_value = new_predecessor.get_g_value() + new_op.get_cost();
    if (g_use_metric) // if using action costs, all costs have been increased by 1
		g_value = g_value - 1;

    float op_duration = 0;
    float op_end_time = 0;
    float op_start_time = new_predecessor.g_current_time_value;
    if(!is_temporal)
    {
    	op_duration = 0.01;
    	g_time_value = 0.01;
    	g_current_time_value = new_predecessor.g_current_time_value + 0.01;
    }else {

		// Get action duration
		if(new_op.get_name().find("_start") != string::npos){
			// Get the duration calculating the costfrom the current state
			vector<PrePost>::const_iterator it_pp = new_op.get_pre_post().begin();
			for(; it_pp != new_op.get_pre_post().end(); ++it_pp) {
				PrePost pp = *it_pp;
				if(g_variable_name[pp.var] == total_time_var)
				{
					if(pp.have_runtime_cost_effect)
					{
						op_duration = new_predecessor.calculate_runtime_efect<float>(pp.runtime_cost_effect);
					} else{
						op_duration = pp.f_cost;
					}

					break;
				}
			}
		}else{
			// Get the duration from the running action
			vector<runn_action>::const_iterator it_ra_const = new_predecessor.running_actions.begin();
			for(; (it_ra_const != new_predecessor.running_actions.end()) && (op_duration == 0); it_ra_const++){
				if((*it_ra_const).non_temporal_action_name == new_op.get_non_temporal_action_name()){
					vector<PrePost*>::const_iterator it_fc = (*it_ra_const).functional_costs.begin();
					for(; (it_fc != (*it_ra_const).functional_costs.end()) && (op_duration == 0); ++it_fc) {
						PrePost* pp = *it_fc;
						if(g_variable_name[pp->var] == total_time_var)
						{
							op_duration = pp->f_cost;
						}
					}
				}
			}
		}

		vector<runn_action>::const_iterator it_ra_const = new_predecessor.running_actions.begin();
		for(; it_ra_const != new_predecessor.running_actions.end(); it_ra_const++)
		{
			this->running_actions.push_back(*it_ra_const);
		}

		g_time_value = op_duration;
		if(new_op.get_name().find("_start") != string::npos){
			running_actions.push_back(*(new(runn_action)));
			running_actions.back().non_temporal_action_name = new_op.get_non_temporal_action_name();
			running_actions.back().time_start = new_predecessor.g_current_time_value;
			running_actions.back().time_end = new_predecessor.g_current_time_value + op_duration;

			for(int i = 0; i < new_op.get_pre_post().size(); i++) {
				PrePost* pre_post = new PrePost(new_op.get_pre_post()[i].var,
						new_op.get_pre_post()[i].pre, new_op.get_pre_post()[i].post,
						new_op.get_pre_post()[i].f_cost,
						new_op.get_pre_post()[i].cond);
				pre_post->have_runtime_cost_effect = new_op.get_pre_post()[i].have_runtime_cost_effect;
				if(pre_post->have_runtime_cost_effect)
					pre_post->runtime_cost_effect = new_op.get_pre_post()[i].runtime_cost_effect;
				else
					pre_post->runtime_cost_effect = "";

				if((pre_post->pre == -2) || (pre_post->pre == -3) || (pre_post->pre == -4))
				{
					if(pre_post->have_runtime_cost_effect)
					{
						pre_post->f_cost = new_predecessor.calculate_runtime_efect<float>(pre_post->runtime_cost_effect);
						running_actions.back().functional_costs.push_back(pre_post);
					} else
						running_actions.back().functional_costs.push_back(pre_post);
				}
			}
			op_end_time = new_predecessor.g_current_time_value + 0.01;
			g_current_time_value = new_predecessor.g_current_time_value + 0.01;

		} else {
			vector<runn_action>::iterator it_ra = this->running_actions.begin();
			for(; it_ra != this->running_actions.end();)
			{
				if((*it_ra).non_temporal_action_name == new_op.get_non_temporal_action_name())
				{
					op_end_time = (*it_ra).time_end;
					it_ra = this->running_actions.erase(it_ra);
					break;
				}else{
					it_ra++;
				}
			}
			g_current_time_value = op_end_time;
		}

		// Copy locked variables
		vector<blocked_var>::const_iterator it_bv = new_predecessor.blocked_vars.begin();
		for(; it_bv != new_predecessor.blocked_vars.end(); it_bv++)
		{
			this->blocked_vars.push_back(*it_bv);
		}

		// Now update the locked variables, the operation is different for start and end actions
		if(new_op.get_name().find("_start") != string::npos)
		{
			// Add blocks to variables
			vector<PrePost>::const_iterator it_pb = new_op.get_pre_block().begin();
			for(; it_pb != new_op.get_pre_block().end(); it_pb++ )
			{
				blocked_var* new_block = new(blocked_var);
				new_block->var = it_pb-> var;
				new_block->val = it_pb->post;
				new_block->block_type = it_pb->pre;
				new_block->time_set = new_predecessor.g_current_time_value;
				new_block->time_freed = new_predecessor.g_current_time_value + op_duration;
				new_block->non_temporal_action_name = new_op.get_non_temporal_action_name();

				blocked_vars.push_back(*new_block);
			}
		}
		else
		{
			// Remove blocks to variables
			vector<blocked_var>::iterator it_pb = this->blocked_vars.begin();
			for(; it_pb != this->blocked_vars.end();)
			{
				if(it_pb->non_temporal_action_name == new_op.get_non_temporal_action_name())
				{
					it_pb = this->blocked_vars.erase(it_pb);
				} else
					it_pb++;
			}
		}
    }

	// Copy locked variables
	vector<float>::const_iterator it_f = new_predecessor.numeric_vars_val.begin();
	for(; it_f != new_predecessor.numeric_vars_val.end(); it_f++)
	{
		this->numeric_vars_val.push_back(*it_f);
	}
    // Only truly update numeric values if the action is an end action
    for(int i = 0; (i < new_op.get_pre_post().size()) && ((new_op.get_name().find("_end") != string::npos) || (is_temporal)) ; i++) {
		const PrePost &pre_post = new_op.get_pre_post()[i];
		if(pre_post.does_fire(new_predecessor)){
			switch(pre_post.pre){
			case -2:{
				vars[pre_post.var] = pre_post.post;
				float cal_cost = 0;
				if (!pre_post.have_runtime_cost_effect){
					numeric_vars_val[pre_post.var] = new_predecessor.numeric_vars_val[pre_post.var] + pre_post.f_cost;
				    cal_cost = pre_post.f_cost;
				}
				else{
					cal_cost = new_predecessor.calculate_runtime_efect<float>(pre_post.runtime_cost_effect);
					numeric_vars_val[pre_post.var] = new_predecessor.numeric_vars_val[pre_post.var] + cal_cost;
				}

				break;
			}
			case -3:
				vars[pre_post.var] = pre_post.post;
				if (!pre_post.have_runtime_cost_effect)
					numeric_vars_val[pre_post.var] = new_predecessor.numeric_vars_val[pre_post.var] - pre_post.f_cost;
				else{
					float cal_cost = new_predecessor.calculate_runtime_efect<float>(pre_post.runtime_cost_effect);
					numeric_vars_val[pre_post.var] = new_predecessor.numeric_vars_val[pre_post.var] - cal_cost;
				}
				break;

			case -4:
				vars[pre_post.var] = pre_post.post;
				if (!pre_post.have_runtime_cost_effect)
					numeric_vars_val[pre_post.var] = pre_post.f_cost;
				else{
					float cal_cost = new_predecessor.calculate_runtime_efect<float>(pre_post.runtime_cost_effect);
					numeric_vars_val[pre_post.var] = cal_cost;
				}
				break;

			case -5:
			case -6:
				break;
			}
		}
	}

    applied_actions = new_predecessor.applied_actions + 1;
    applied_actions_vec = new_predecessor.applied_actions_vec;
    applied_actions_vec.push_back(new_op.get_name());
}

/* State::State(const State &origin)
: vars(origin.vars), numeric_vars_val(origin.numeric_vars_val),
  reached_lms(origin.reached_lms), reached_lms_cost(origin.reached_lms_cost){
	g_time_value = origin.g_time_value;
	g_current_time_value = origin.g_current_time_value;
	g_value = origin.get_g_value();

	applied_actions = origin.applied_actions;
	for(int i = 0; i < origin.applied_actions_vec.size(); i++)
	{
		applied_actions_vec.push_back(origin.applied_actions_vec[i]);
	}

	running_actions = origin.running_actions;
	for(int i = 0; i < origin.running_actions.size(); i++)
	{
		running_actions.push_back(origin.running_actions[i]);
	}

	blocked_vars = origin.blocked_vars;
	for(int i = 0; i < origin.blocked_vars.size(); i++)
	{
		blocked_vars.push_back(origin.blocked_vars[i]);
	}

	just_added_external_const = origin.just_added_external_const;
	for(int i = 0; i < origin.just_added_external_const.size(); i++)
	{
		just_added_external_const.push_back(origin.just_added_external_const[i]);
	}
} */

State::State(const State &predecessor, const Operator &op)
    : vars(predecessor.vars), numeric_vars_val(predecessor.numeric_vars_val),
	  reached_lms(predecessor.reached_lms), reached_lms_cost(predecessor.reached_lms_cost) {
    assert(!op.is_axiom());

	float op_duration = 0;
	float op_end_time = 0;
	float op_start_time = predecessor.g_current_time_value;
    if(is_temporal){
		// Get action duration
		if(op.get_name().find("_start") != string::npos){
			// Get the duration calculating the costfrom the current state
			vector<PrePost>::const_iterator it_pp = op.get_pre_post().begin();
			for(; it_pp != op.get_pre_post().end(); ++it_pp) {
				PrePost pp = *it_pp;
				if(g_variable_name[pp.var] == total_time_var)
				{
					if(pp.have_runtime_cost_effect)
					{
						op_duration = predecessor.calculate_runtime_efect<float>(pp.runtime_cost_effect);
					} else{
						op_duration = pp.f_cost;
					}

					break;
				}
			}
		}else{
			// Get the duration from the running action
			vector<runn_action>::const_iterator it_ra_const = predecessor.running_actions.begin();
			for(; (it_ra_const != predecessor.running_actions.end()) && (op_duration == 0); it_ra_const++){
				if((*it_ra_const).non_temporal_action_name == op.get_non_temporal_action_name()){
					vector<PrePost*>::const_iterator it_fc = (*it_ra_const).functional_costs.begin();
					for(; (it_fc != (*it_ra_const).functional_costs.end()) && (op_duration == 0); ++it_fc) {
						PrePost* pp = *it_fc;
						if(g_variable_name[pp->var] == total_time_var)
						{
							op_duration = pp->f_cost;
						}
					}
				}
			}
		}

		vector<runn_action>::const_iterator it_ra_const = predecessor.running_actions.begin();
		for(; it_ra_const != predecessor.running_actions.end(); it_ra_const++)
		{
			this->running_actions.push_back(*it_ra_const);
		}

		g_time_value = op_duration;
		if(op.get_name().find("_start") != string::npos){
			running_actions.push_back(*(new(runn_action)));
			running_actions.back().non_temporal_action_name = op.get_non_temporal_action_name();
			running_actions.back().time_start = predecessor.g_current_time_value;
			running_actions.back().time_end = predecessor.g_current_time_value + op_duration;

			for(int i = 0; i < op.get_pre_post().size(); i++) {
				PrePost* pre_post = new PrePost(op.get_pre_post()[i].var,
						op.get_pre_post()[i].pre, op.get_pre_post()[i].post,
						op.get_pre_post()[i].f_cost,
						op.get_pre_post()[i].cond);

				pre_post->have_runtime_cost_effect = op.get_pre_post()[i].have_runtime_cost_effect;
				if(pre_post->have_runtime_cost_effect)
					pre_post->runtime_cost_effect = op.get_pre_post()[i].runtime_cost_effect;
				else
					pre_post->runtime_cost_effect = "";

				if((pre_post->pre == -2) || (pre_post->pre == -3) || (pre_post->pre == -4))
				{
					if(pre_post->have_runtime_cost_effect)
					{
						pre_post->f_cost = predecessor.calculate_runtime_efect<float>(pre_post->runtime_cost_effect);
						running_actions.back().functional_costs.push_back(pre_post);
					} else
						running_actions.back().functional_costs.push_back(pre_post);
				}
			}
			op_end_time = predecessor.g_current_time_value + 0.01;
			g_current_time_value = predecessor.g_current_time_value + 0.01;

		} else {
			vector<runn_action>::iterator it_ra = this->running_actions.begin();
			for(; it_ra != this->running_actions.end();)
			{
				if((*it_ra).non_temporal_action_name == op.get_non_temporal_action_name())
				{
					op_end_time = (*it_ra).time_end;
					it_ra = this->running_actions.erase(it_ra);
					break;
				}else{
					it_ra++;
				}
			}
			g_current_time_value = op_end_time;
		}

		// Copy locked variables
		vector<blocked_var>::const_iterator it_bv = predecessor.blocked_vars.begin();
		for(; it_bv != predecessor.blocked_vars.end(); it_bv++)
		{
			this->blocked_vars.push_back(*it_bv);
		}

		// Now update the locked variables, the operation is different for start and end actions
		if(op.get_name().find("_start") != string::npos)
		{
			// Add blocks to variables
			vector<PrePost>::const_iterator it_pb = op.get_pre_block().begin();
			for(; it_pb != op.get_pre_block().end(); it_pb++ )
			{
				blocked_var* new_block = new(blocked_var);
				new_block->var = it_pb-> var;
				new_block->val = it_pb->post;
				new_block->block_type = it_pb->pre;
				new_block->time_set = predecessor.g_current_time_value;
				new_block->time_freed = predecessor.g_current_time_value + op_duration;
				new_block->non_temporal_action_name = op.get_non_temporal_action_name();

				blocked_vars.push_back(*new_block);
			}
		}
		else
		{
			// Remove blocks to variables
			vector<blocked_var>::iterator it_pb = this->blocked_vars.begin();
			for(; it_pb != this->blocked_vars.end();)
			{
				if(it_pb->non_temporal_action_name == op.get_non_temporal_action_name())
				{
					it_pb = this->blocked_vars.erase(it_pb);
				} else
					it_pb++;
			}
		}
    } else
    {
    	op_end_time = predecessor.g_current_time_value + 0.01;
    	g_time_value = 0.01;
    	g_current_time_value = predecessor.g_current_time_value + 0.01;
    }

	/* for(int k = 0; k < external_blocked_vars.size(); k++)
	{
		if(external_blocked_vars[k]->val_pre == -2)
			continue;

		if((external_blocked_vars[k]->time_set >= predecessor.g_current_time_value) &&
				(external_blocked_vars[k]->time_set < op_end_time))
		{

			bool action_changes_constraint = false;
			vector<PrePost>::const_iterator it_pp = op.get_pre_post().begin();
			for(; it_pp != op.get_pre_post().end(); ++it_pp) {
				PrePost pp = *it_pp;
				if(pp.var == external_blocked_vars[k]->var)
				{
					action_changes_constraint = true;
				}
			}
			if(!action_changes_constraint)
			{
				const std::vector<Prevail> prevail;
				PrePost* pre_post = new PrePost(external_blocked_vars[k]->var, external_blocked_vars[k]->val_pre,
						external_blocked_vars[k]->val_pos, 0, prevail);
				if(pre_post->does_fire(predecessor))
				{
					vars[pre_post->var] = pre_post->post;
					// cout << "external var: " << vars[pre_post->var] << " to " << pre_post->post << endl;
				}
				else
					cout << "something bad happened" << endl;
				delete pre_post;
			}
		}
	}*/

    // Update values affected by operator.
    for(int i = 0; i < op.get_pre_post().size(); i++) {
		const PrePost &pre_post = op.get_pre_post()[i];
		if(pre_post.does_fire(predecessor))
			vars[pre_post.var] = pre_post.post;
    }

    // Only truly update numeric values if the action is an end action
    for(int i = 0; (i < op.get_pre_post().size()) && ((op.get_name().find("_end") != string::npos) || (is_temporal)) ; i++) {
		const PrePost &pre_post = op.get_pre_post()[i];
		if(pre_post.does_fire(predecessor)){
			switch(pre_post.pre){
			case -2:{
				vars[pre_post.var] = pre_post.post;
				float cal_cost = 0;
				if (!pre_post.have_runtime_cost_effect){
					numeric_vars_val[pre_post.var] = numeric_vars_val[pre_post.var] + pre_post.f_cost;
				    cal_cost = pre_post.f_cost;
				}
				else{
					cal_cost = this->calculate_runtime_efect<float>(pre_post.runtime_cost_effect);
					numeric_vars_val[pre_post.var] = numeric_vars_val[pre_post.var] + cal_cost;
				}

				break;
			}
			case -3:
				vars[pre_post.var] = pre_post.post;
				if (!pre_post.have_runtime_cost_effect)
					numeric_vars_val[pre_post.var] = numeric_vars_val[pre_post.var] - pre_post.f_cost;
				else{
					float cal_cost = this->calculate_runtime_efect<float>(pre_post.runtime_cost_effect);
					numeric_vars_val[pre_post.var] = numeric_vars_val[pre_post.var] - cal_cost;
				}
				break;

			case -4:
				vars[pre_post.var] = pre_post.post;
				if (!pre_post.have_runtime_cost_effect)
					numeric_vars_val[pre_post.var] = pre_post.f_cost;
				else{
					float cal_cost = this->calculate_runtime_efect<float>(pre_post.runtime_cost_effect);
					numeric_vars_val[pre_post.var] = cal_cost;
				}
				break;

			case -5:
			case -6:
				break;

			default:
				vars[pre_post.var] = pre_post.post;
			}
		}
	}
    g_axiom_evaluator->evaluate(*this);
    // Update set of reached landmarks.
    update_reached_lms(op);
    // Update g_value
    if(g_length_metric)
    	g_value = predecessor.get_g_value() + 2;
    else {
    	if (!op.get_have_runtime_cost())
        	g_value = predecessor.get_g_value() + op.get_cost();
    	else {
    		g_value = predecessor.get_g_value() + this->calculate_runtime_efect<float>(op.get_runtime_cost()) + 1;
    	}
    }

    applied_actions = predecessor.applied_actions + 1;
    applied_actions_vec = predecessor.applied_actions_vec;
    applied_actions_vec.push_back(op.get_name());

    if (g_use_metric) // if using action costs, all costs have been increased by 1
    	g_value = g_value - 1;
}

void State::dump() const {
    for(int i = 0; i < vars.size(); i++)
	cout << "  " << g_variable_name[i] << ": " << vars[i] << endl;
}

bool State::operator<(const State &other) const {
    return lexicographical_compare(vars.begin(), vars.end(),
				   other.vars.begin(), other.vars.end());
}

void State::set_landmarks_for_initial_state() {
    hash_set<const LandmarkNode*, hash_pointer> initial_state_landmarks;
    if(g_lgraph == NULL) {
	// landmarks not used, set empty
	reached_lms = initial_state_landmarks;
	return;
    }
    for(int i = 0; i < g_variable_domain.size(); i++) {
        const pair<int, int> a = make_pair(i, (*g_initial_state)[i]);
        if(g_lgraph->simple_landmark_exists(a)) {
            LandmarkNode& node = g_lgraph->get_simple_lm_node(a);
            if(node.parents.size() == 0) {
                initial_state_landmarks.insert(&node);
                reached_lms_cost += node.min_cost;
            }
        }
	else{
	    set<pair<int, int> > a_set;
	    a_set.insert(a);
	    if (g_lgraph->disj_landmark_exists(a_set)) {
		LandmarkNode& node = g_lgraph->get_disj_lm_node(a);
		if(node.parents.size() == 0) {
		    initial_state_landmarks.insert(&node);
		    reached_lms_cost += node.min_cost;
		}
	    }
	}
    } 
    cout << initial_state_landmarks.size() << " initial landmarks, " 
	 << g_goal.size() << " goal landmarks" << endl; 
    reached_lms = initial_state_landmarks;
}

bool State::landmark_is_leaf(const LandmarkNode& node, 
			     const hash_set<const LandmarkNode*, hash_pointer>& reached) const {
//Note: this is the same as !check_node_orders_disobeyed
    const hash_map<LandmarkNode*, edge_type, hash_pointer >& parents 
        = node.parents;
    /*
      cout << "in is_leaf, reached is ----- " << endl;
      hash_set<const LandmarkNode*, hash_pointer>::const_iterator it;
      for(it = reached.begin(); it != reached.end(); ++it) {
      cout << *it << " ";
      lgraph.dump_node(*it);
      }
      cout << "---------" << endl;
    */
    for(hash_map<LandmarkNode*, edge_type, hash_pointer >::const_iterator parent_it = 
            parents.begin(); parent_it != parents.end(); parent_it++) {
        LandmarkNode* parent_p = parent_it->first;

        if(true) // Note: no condition on edge type here

            if(reached.find(parent_p) == reached.end()){

                //cout << "parent is not in reached: "; 
                //cout << parent_p << " ";
                //lgraph.dump_node(parent_p);
		return false;
            }
    }
    //cout << "all parents are in reached" << endl;
    return true;
}

bool State::check_lost_landmark_children_needed_again(const LandmarkNode& node) const {
    const hash_set<const LandmarkNode*, hash_pointer>& reached = this->reached_lms;
    const hash_map<LandmarkNode*, edge_type, hash_pointer >& children 
        = node.children;
  
    for(hash_map<LandmarkNode*, edge_type, hash_pointer >::const_iterator child_it = 
            children.begin(); child_it != children.end(); child_it++) {
        LandmarkNode* child_p = child_it->first;
        if(child_it->second == gn) // Note: condition on edge type here!
            if(reached.find(child_p) == reached.end()){
                return true;
            }
    }
    return false;
}

int State::get_needed_landmarks(hash_set<const LandmarkNode*, hash_pointer>& needed) const {
    // Calculate landmarks that will be needed again and their cost
    const hash_set<const LandmarkNode*, hash_pointer>& reached = this->reached_lms;
    int needed_lm_cost = 0;
    for(hash_set<const LandmarkNode*, hash_pointer>::const_iterator it 
            = reached.begin(); it != reached.end(); it++)
        if(!(*it)->is_true_in_state(*this)) {
            if((*it)->is_goal()) {
		needed.insert(*it);
		needed_lm_cost += (*it)->min_cost;
	    }
	    else {
	        const LandmarkNode& node = **it;
		if(check_lost_landmark_children_needed_again(node)) {
		    needed.insert(&node);
		    needed_lm_cost += node.min_cost;
		    //cout << "needed again is "; g_lgraph->dump_node(&node);
		}
	    }
	}
    return needed_lm_cost;
}


int State::check_partial_plan(hash_set<const LandmarkNode*, hash_pointer>& reached) const {
    // Return reached landmarks and their cost
    reached = reached_lms;
    return reached_lms_cost;
}

template <typename T>
T State::calculate_runtime_efect(string s_effect) const {

	// First get current value of runtime numerical variables
	string s_eff_aux = s_effect;
	while(s_effect.find(":") != string::npos){
		string var = "";
		int i_var;
		float var_value;
		s_eff_aux = s_eff_aux.substr(s_eff_aux.find(":") + 1, s_eff_aux.length() - 1);
		var = s_eff_aux.substr(0, s_eff_aux.find(":"));
		s_eff_aux = s_eff_aux.substr(s_eff_aux.find(":") + 1, s_eff_aux.length() - 1);
		stringstream strm(var);
		strm >> i_var;
		var_value = numeric_vars_val[i_var];
   		std::ostringstream strm_var;
   		strm_var << var_value;

		string from = ":" + var + ":";
		string to = strm_var.str();
	    size_t start_pos = 0;
	    while((start_pos = s_effect.find(from, start_pos)) != std::string::npos) {
	    	s_effect.replace(start_pos, from.length(), to);
	        start_pos += to.length(); // Handles case where 'to' is a substring of 'from'
	    }
	}

    // now solve the operation
	typedef exprtk::symbol_table<T> symbol_table_t;
	typedef exprtk::expression<T>   expression_t;
	typedef exprtk::parser<T>       parser_t;
	T result = 0;
	symbol_table_t symbol_table;
	symbol_table.add_constants();
	expression_t expression;
	expression.register_symbol_table(symbol_table);
	parser_t parser;
	parser.compile(s_effect,expression);
	result = expression.value();
	return result;
}
