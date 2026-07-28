/*********************************************************************
 * Author: Malte Helmert (helmert@informatik.uni-freiburg.de)
 * (C) Copyright 2003-2004 Malte Helmert
 * Modified by: Silvia Richter (silvia.richter@nicta.com.au)
 * (C) Copyright 2008 NICTA
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

#include "best_first_search.h"

#include "globals.h"
#include "heuristic.h"
#include "successor_generator.h"
#include "operator.h"
#include "ff_heuristic.h"
#include "landmarks_count_heuristic.h"

#include <cassert>
#include <algorithm>
using namespace std;

OpenListInfo::OpenListInfo(Heuristic *heur, bool only_pref) {
    heuristic = heur;
    only_preferred_operators = only_pref;
    priority = 0;
}

OpenListEntry::OpenListEntry(const State *_parent, const Operator *_op, float _parent_heur) {
    parent = _parent;
    op = _op;
    parent_heur = _parent_heur;
}

BestFirstSearchEngine::BestFirstSearchEngine()
    : current_state(*g_initial_state) {
    generated_states = 0;
    current_predecessor = 0;
    current_operator = 0;
}

BestFirstSearchEngine::~BestFirstSearchEngine() {
}

void BestFirstSearchEngine::add_heuristic(Heuristic *heuristic,
					  bool use_estimates,
					  bool use_preferred_operators) {
    assert(use_estimates || use_preferred_operators);
    heuristics.push_back(heuristic);
    best_heuristic_values.push_back(-1);
    if(use_estimates) {
	open_lists.push_back(OpenListInfo(heuristic, false));
	open_lists.push_back(OpenListInfo(heuristic, true));
    }
    if(use_preferred_operators)
	preferred_operator_heuristics.push_back(heuristic);
}

void BestFirstSearchEngine::initialize() {
    cout << "Conducting best first search." << endl;
    assert(!open_lists.empty());
}

float BestFirstSearchEngine::statistics() const {
    cout << "Expanded " << closed_list.size() << " state(s)." << endl;
    cout << "Generated " << generated_states << " state(s)." << endl;
    return closed_list.size();
}

int BestFirstSearchEngine::step() {
    // Invariants:
    // - current_state is the next state for which we want to compute the heuristic.
    // - current_predecessor is a permanent pointer to the predecessor of that state.
    // - current_operator is the operator which leads to current_state from predecessor.
  
    if(!closed_list.contains(current_state)) {  
	const State *parent_ptr = closed_list.insert(
	    current_state, current_predecessor, current_operator);

	if(g_lm_heur != NULL)
	    g_lm_heur->set_recompute_heuristic(current_state);
	if(g_ff_heur != NULL)
	    g_ff_heur->set_recompute_heuristic(); 
        for(int i = 0; i < heuristics.size(); i++) 
            heuristics[i]->evaluate(current_state);   
 	
	if(!is_dead_end()) {
	    if(check_goal())
		return SOLVED;
	    if(check_progress()) {
		report_progress();
		reward_progress();
	    }
	    generate_successors(parent_ptr);
	}
    }
    return fetch_next_state();
}

// Merge a candidate plan's own shared-variable transitions with the
// ones already relayed from earlier-solved agents, and check that the
// whole chronological sequence is self-consistent: every transition's
// precondition must match the value the immediately preceding
// transition left the variable at. Checking only against the relayed
// timeline's value "at the time" an action fires (as the ordinary
// validity checks do) cannot see this: an agent solved first commits
// to a transition with no knowledge of anyone else, and a later agent
// can independently cause an earlier transition on the same variable
// that quietly invalidates it, without either agent's own, local view
// ever showing a conflict.
static bool transition_time_order(
		const pair<float, pair<int, int> > &a,
		const pair<float, pair<int, int> > &b) {
	if(a.first != b.first)
		return a.first < b.first;
	// Ties are broken the same way the relayed timeline itself is
	// built: a release (no precondition of its own) before an acquire.
	bool a_is_release = (a.second.first == -1);
	bool b_is_release = (b.second.first == -1);
	if(a_is_release != b_is_release)
		return a_is_release;
	return false;
}

bool plan_respects_shared_var_transitions(
		const vector<const Operator *> &plan, const vector<State> &states_plan) {
	for(int k = 0; k < g_shared_vars_timed_values.size(); k++)
	{
		int local_var = g_shared_vars_timed_values[k]->first;

		// This plan's own transitions on this shared variable.
		vector<pair<float, pair<int, int> > > merged;
		for(int i = 0; i < plan.size(); i++)
		{
			const Operator *op = plan[i];
			float t = states_plan[i + 1].get_g_current_time_value();

			bool touched = false;
			for(vector<PrePost>::const_iterator it_pp = op->get_pre_post().begin();
					it_pp != op->get_pre_post().end(); ++it_pp)
			{
				if(it_pp->var == local_var) {
					merged.push_back(make_pair(t, make_pair(it_pp->pre, it_pp->post)));
					touched = true;
					break;
				}
			}
			if(touched)
				continue;
			for(vector<Prevail>::const_iterator it_pv = op->get_prevail().begin();
					it_pv != op->get_prevail().end(); ++it_pv)
			{
				if(it_pv->var == local_var) {
					merged.push_back(make_pair(t, make_pair(it_pv->prev, it_pv->prev)));
					break;
				}
			}
		}

		if(merged.empty())
			continue;

		// The relayed transitions from earlier-solved agents (skip the
		// "-1 at time 0" placeholder used only to seed the timeline).
		vector<pair<int, float>* > &ext_transitions = *(g_shared_vars_timed_values[k]->second);
		vector<int> &ext_pre_values = *(g_shared_vars_pre_values[k]);
		for(int j = 1; j < ext_transitions.size(); j++)
			merged.push_back(make_pair(ext_transitions[j]->second,
					make_pair(ext_pre_values[j], ext_transitions[j]->first)));

		sort(merged.begin(), merged.end(), transition_time_order);

		int last_val = -1;
		for(int i = 0; i < merged.size(); i++)
		{
			int pre = merged[i].second.first;
			int post = merged[i].second.second;
			if((pre != -1) && (last_val != -1) && (pre != last_val))
				return false;
			last_val = post;
		}
	}
	return true;
}

bool BestFirstSearchEngine::is_dead_end() {
    // If a reliable heuristic reports a dead end, we trust it.
    // Otherwise, all heuristics must agree on dead-end-ness.
    int dead_end_counter = 0;
    for(int i = 0; i < heuristics.size(); i++) {
	if(heuristics[i]->is_dead_end()) {
	    if(heuristics[i]->dead_ends_are_reliable())
		return true;
	    else
		dead_end_counter++;
	}
    }
    return dead_end_counter == heuristics.size();
}

bool BestFirstSearchEngine::check_goal() {
    // Any heuristic reports 0 if this is a goal state, so we can
    // pick an arbitrary one.
    Heuristic *heur = open_lists[0].heuristic;
    if(!heur->is_dead_end() && heur->get_heuristic() == 0) {
	// We actually need this silly !heur->is_dead_end() check because
	// this state *might* be considered a non-dead end by the
	// overall search even though heur considers it a dead end
	// (e.g. if heur is the CG heuristic, but the FF heuristic is
	// also computed and doesn't consider this state a dead end.
	// If heur considers the state a dead end, it cannot be a goal
	// state (heur will not be *that* stupid). We may not call
	// get_heuristic() in such cases because it will barf.

	// If (and only if) using action costs the heuristic might report 0
	// even though the goal is not reached - check.
	if(g_use_metric)
	    for(int i = 0; i < g_goal.size(); i++)
		if(current_state[g_goal[i].first] != g_goal[i].second)
		    return false;

	Plan plan;
	vector<State> states_plan = closed_list.trace_path(current_state, plan);

	if(!plan_respects_shared_var_transitions(plan, states_plan))
	    return false;

	cout << "Solution found!" << endl;
	vector<float> plan_temporal_info;
	for(int i = 1; i < states_plan.size(); i++)
	{
		plan_temporal_info.push_back(states_plan[i].get_g_current_time_value());
	}
	vector<float> plan_duration_info;
	for(int i = 1; i < states_plan.size(); i++)
	{
		plan_duration_info.push_back(states_plan[i].get_g_time_value());
	}
	vector<float> plan_cost_info;
	for(int i = 1; i < states_plan.size(); i++)
	{
		plan_cost_info.push_back(states_plan[i].get_g_value());
	}
	vector<vector<blocked_var> > plan_blocked_vars_info;
	for(int i = 1; i < states_plan.size(); i++)
	{
		plan_blocked_vars_info.push_back(states_plan[i].blocked_vars);
	}
	set_plan_cost(current_state.get_g_value());
	set_plan_temporal_info(plan_temporal_info);
	set_plan_duration_info(plan_duration_info);
	set_plan_cost_info(plan_cost_info);
	set_vars_end_state(states_plan[states_plan.size() - 1].get_vars_state());
	set_num_vars_end_state(states_plan[states_plan.size() - 1].get_num_vars_state());
	set_blocked_vars_info(plan_blocked_vars_info);

	set_plan(plan);
	return true;
    } else {
	return false;
    }
}

bool BestFirstSearchEngine::check_progress() {
    bool progress = false;
    for(int i = 0; i < heuristics.size(); i++) {
	if(heuristics[i]->is_dead_end())
	    continue;
	int h = heuristics[i]->get_heuristic();
	int &best_h = best_heuristic_values[i];
	if(best_h == -1 || h < best_h) {
	    best_h = h;
	    progress = true;
	}
    }
    return progress;
}

void BestFirstSearchEngine::report_progress() {
    cout << "Best heuristic value: ";
    for(int i = 0; i < heuristics.size(); i++) {
	cout << best_heuristic_values[i];
	if(i != heuristics.size() - 1)
	    cout << "/";
    }
    cout << " [expanded " << closed_list.size() << " state(s)]" << endl;
}

void BestFirstSearchEngine::reward_progress() {
    // Boost the "preferred operator" open lists somewhat whenever
    // progress is made. This used to be used in multi-heuristic mode
    // only, but it is also useful in single-heuristic mode, at least
    // in Schedule.
    //
    // TODO: Test the impact of this, and find a better way of rewarding
    // successful exploration. For example, reward only the open queue
    // from which the good state was extracted and/or the open queues
    // for the heuristic for which a new best value was found.

    for(int i = 0; i < open_lists.size(); i++)
	if(open_lists[i].only_preferred_operators)
	    open_lists[i].priority -= 1000;
}  

void BestFirstSearchEngine::generate_successors(const State *parent_ptr) {
    vector<const Operator *> all_operators;
    g_successor_generator->generate_applicable_ops(current_state, all_operators);
    check_functional_validity(current_state, all_operators);
    if(is_temporal){
		check_var_locks_validity(current_state, all_operators);
		check_temporal_soundness_validity(current_state, all_operators);
		check_temporal_goals_validity(current_state, all_operators);
    }
    check_external_locks_validity(current_state, all_operators);

    vector<const Operator *> preferred_operators;
    for(int i = 0; i < preferred_operator_heuristics.size(); i++) {
	Heuristic *heur = preferred_operator_heuristics[i];
	if(!heur->is_dead_end())
	    heur->get_preferred_operators(preferred_operators);
    }
    check_functional_validity(current_state, preferred_operators);
    if(is_temporal){
		check_var_locks_validity(current_state, preferred_operators);
		check_temporal_soundness_validity(current_state, preferred_operators);
		check_temporal_goals_validity(current_state, preferred_operators);
    }
    check_external_locks_validity(current_state, preferred_operators);

    if(parent_ptr->running_actions.size() > 1)
    {
    	vector<const Operator *>::iterator it = all_operators.begin();
    	for(; it != all_operators.end();) {

    		if((*it)->get_name().find("_start") == string::npos) {
    			it = all_operators.erase(it);
    		}else
    			it++;
    	}

    	it = preferred_operators.begin();
		for(; it != preferred_operators.end();) {

			if((*it)->get_name().find("_start") == string::npos) {
				it = preferred_operators.erase(it);
			}else
				it++;
		}
    }

    for(int i = 0; i < open_lists.size(); i++) {
	Heuristic *heur = open_lists[i].heuristic;
	if(!heur->is_dead_end()) {
	    int h = heur->get_heuristic();
	    OpenList<OpenListEntry> &open = open_lists[i].open;
	    vector<const Operator *> &ops =
		open_lists[i].only_preferred_operators ?
		preferred_operators : all_operators;
	    for(int j = 0; j < ops.size(); j++) {
		// Tie braker criterium ensures breadth-first search on plateaus
		// (will be equal to depth of node if no action costs are used,
		// and cost of node otherwise)
		float tie_braker = parent_ptr->get_g_value() + ops[j]->get_cost();
		open.insert(make_pair(h, tie_braker), 
			    OpenListEntry(parent_ptr, ops[j], h));
	    }
	}
    }
    generated_states += all_operators.size();
}

int BestFirstSearchEngine::fetch_next_state() {
    OpenListInfo *open_info = select_open_queue();
    if(!open_info) {
	cout << "Completely explored state space -- no solution!" << endl;
	return FAILED;
    }

    OpenListEntry next = open_info->open.remove_min();
    open_info->priority++;

    current_predecessor = next.parent;
    current_operator = next.op;
    try{
    	current_state = State(*current_predecessor, *current_operator);
    }
    catch(...)
    {
    	cout << "This state will not generate successors" << endl;
    	current_state = State(*current_predecessor, *current_operator);
    }

    return IN_PROGRESS;
}

OpenListInfo *BestFirstSearchEngine::select_open_queue() {
    OpenListInfo *best = 0;
    for(int i = 0; i < open_lists.size(); i++)
	if(!open_lists[i].open.empty() &&
	   (best == 0 || open_lists[i].priority < best->priority))
	    best = &open_lists[i];
    return best;
}
