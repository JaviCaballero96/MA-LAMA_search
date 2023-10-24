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

#include "globals.h"

#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
using namespace std;

#include "axioms.h"
#include "domain_transition_graph.h"
#include "operator.h"
#include "state.h"
#include "successor_generator.h"
#include "landmarks_graph.h"
#include "landmarks_graph_rpg_sasp.h"
#include "ff_heuristic.h"

void check_magic(istream &in, string magic) {
    string word;
    in >> word;
    if(word != magic) {
        cout << "Failed to match magic word '" << magic << "'." << endl;
        cout << "Got '" << word << "'." << endl;
        exit(1);
    }
}

void read_metric(istream &in) {
  string metric_aux = "";

  check_magic(in, "begin_metric");
  g_use_metric = true;
  g_length_metric = false;
  in >> g_op_metric;

  if (g_op_metric == "end")
	  g_length_metric = true;

  while(g_op_metric != "end")
  {
	  in >> metric_aux;
	  if (metric_aux == "end")
		  break;
	  g_n_metric.push_back(atoi(metric_aux.c_str()));
  }
  check_magic(in, "end_metric");
}

void read_variables(istream &in) {
    check_magic(in, "begin_variables");
    int count;
    in >> count;
    for(int i = 0; i < count; i++) {
        string name;
        in >> name;
        g_variable_name.push_back(name);
        int range;
        in >> range;
        g_variable_domain.push_back(range);
        int layer;
        in >> layer;
        g_axiom_layers.push_back(layer);
        int isTotalTime;
        in >> isTotalTime;
        if(isTotalTime)
        	total_time_var = name;
    }
    check_magic(in, "end_variables");
}

void read_shared(istream &in) {
    check_magic(in, "begin_shared");
    int count;
    in >> count;
    for(int i = 0; i < count; i++) {
        int var;
        string name;
        in >> name >> var;
        g_shared_vars.push_back(make_pair(name, var));
    }
    check_magic(in, "end_shared");
}

void read_goal(istream &in) {
    check_magic(in, "begin_goal");
    int count;
    in >> count;
    for(int i = 0; i < count; i++) {
        int var, val;
        in >> var >> val;
        g_goal.push_back(make_pair(var, val));
    }
    check_magic(in, "end_goal");
}

void dump_goal() {
    cout << "Goal Conditions:" << endl;
    for(int i = 0; i < g_goal.size(); i++)
        cout << "  " << g_variable_name[g_goal[i].first] << ": "
             << g_goal[i].second << endl;
}

void read_operators(istream &in) {
    int count;
    is_temporal = false;
    in >> count;
    for(int i = 0; i < count; i++)
        g_operators.push_back(Operator(in, false));
}

void read_axioms(istream &in) {
    int count;
    in >> count;
    for(int i = 0; i < count; i++)
        g_axioms.push_back(Operator(in, true));

    g_axiom_evaluator = new AxiomEvaluator;
    g_axiom_evaluator->evaluate(*g_initial_state);
}

void build_landmarks_graph(bool reasonable_orders) {
	/* The operators are iondexed by its propositions as preconditions and effects */
    g_lgraph = new LandmarksGraphNew();
    g_lgraph->read_external_inconsistencies();
    if(reasonable_orders) {
	g_lgraph->use_reasonable_orders();
    }
    g_lgraph->generate();
    cout << "Generated " << g_lgraph->number_of_landmarks() << " landmarks, of which "
	 << g_lgraph->number_of_disj_landmarks() << " are disjunctive" << endl
	 << "          " << g_lgraph->number_of_edges() << " edges\n";
    //g_lgraph->dump();
}

void read_everything(istream &in, bool generate_landmarks, bool reasonable_orders, bool read_init_state, bool read_runtime_constraints) {
    read_metric(in);
    read_variables(in);
    g_initial_state = new State(in);
    if(read_init_state)
    {
    	read_ext_init_state();
    }
    read_shared(in);
    if(read_runtime_constraints)
    {
    	read_runtime_contraints();
    }
    read_goal(in);
    read_operators(in);
    read_axioms(in);
    check_magic(in, "begin_SG");
    g_successor_generator = read_successor_generator(in);
    check_magic(in, "end_SG");
    DomainTransitionGraph::read_all(in);
    if(generate_landmarks){
	if(!g_ff_heur)
	    g_ff_heur = new FFHeuristic;
	build_landmarks_graph(reasonable_orders);
    }
    g_initial_state->set_landmarks_for_initial_state();
}

void dump_everything() {
    cout << "Use metric? " << g_use_metric << endl;
    cout << "Variables (" << g_variable_name.size() << "):" << endl;
    for(int i = 0; i < g_variable_name.size(); i++)
        cout << "  " << g_variable_name[i]
             << " (range " << g_variable_domain[i] << ")" << endl;
    cout << "Initial State:" << endl;
    g_initial_state->dump();
    dump_goal();
    cout << "Successor Generator:" << endl;
    g_successor_generator->dump();
    for(int i = 0; i < g_variable_domain.size(); i++)
        g_transition_graphs[i]->dump();
}

void read_runtime_contraints()
{
	std::fstream in;
	in.open ("current_constraints", std::fstream::in);
	check_magic(in, "begin_constraints");
	while(true)
	{
		float time_init = 0;
		float act_duration = 0;
		string s_aux = "";
		in >> time_init;
		if(time_init == -1)
		{
			break;
		}
		in >> act_duration;

		ext_constraint* bv = new(ext_constraint);
		bv->in_current_agent = false;
		bv->time_set = time_init;
		bv->duration = act_duration;
		bool remaining_constraints = true;

		while(remaining_constraints)
		{
			in >> s_aux;
			if(s_aux.find("(") != string::npos){
				s_aux = s_aux.substr(s_aux.find("(") + 1, s_aux.length());
			}

			if(s_aux == "1")
			{
				int i_aux = 0;
				in >> i_aux;
				bool var_found = false;
			    for(int z = 0; z < g_shared_vars.size(); z++)
			    {
			    	int shared_number_1 = atoi((g_shared_vars[z].first.substr(3, g_shared_vars[z].first.length())).c_str());
			    	int shared_number_2 = g_shared_vars[z].second;

			    	if(shared_number_1 == i_aux)
			    	{
			    		var_found = true;
			    		bv->in_current_agent = true;
			    		i_aux = shared_number_2;
			    		break;
			    	}
			    }
			    /* if(!var_found)
				{
			    	in >> i_aux;
			    	in >> s_aux;
					if(s_aux.find(")") != string::npos) {
						s_aux = s_aux.substr(0, s_aux.find(")"));
						bv->val_pos = std::atof(s_aux.c_str());
						remaining_constraints = false;
					} else{
						bv->val_pos = std::atof(s_aux.c_str());
						in >> s_aux;
						assert(s_aux == "|");
					}
					continue;
				} */
				bv->var = i_aux;
				in >> i_aux;
				bv->val_pre = i_aux;
				in >> s_aux;
				if(s_aux.find(")") != string::npos) {
					s_aux = s_aux.substr(0, s_aux.find(")"));
					bv->val_pos = std::atof(s_aux.c_str());
					remaining_constraints = false;
				} else{
					bv->val_pos = std::atof(s_aux.c_str());
					in >> s_aux;
					assert(s_aux == "|");
				}

			} else
			{
				int i_aux = 0;
				in >> i_aux;
				bool var_found = false;
			    for(int z = 0; z < g_shared_vars.size(); z++)
			    {
			    	int shared_number_1 = atoi((g_shared_vars[z].first.substr(3, g_shared_vars[z].first.length())).c_str());
			    	int shared_number_2 = g_shared_vars[z].second;

			    	if(shared_number_1 == i_aux)
			    	{
			    		var_found = true;
			    		bv->in_current_agent  = true;
			    		i_aux = shared_number_2;
			    		break;
			    	}
			    }
			    /* if(!var_found)
				{
			    	in >> s_aux;
					if(s_aux.find(")") != string::npos) {
						s_aux = s_aux.substr(0, s_aux.find(")"));
						bv->val_pos = std::atof(s_aux.c_str());
						remaining_constraints = false;
					} else{
						bv->val_pos = std::atof(s_aux.c_str());
						in >> s_aux;
						assert(s_aux == "|");
					}
					continue;
				} */
				bv->var = i_aux;
				bv->val_pre = -2;
				in >> s_aux;
				if(s_aux.find(")") != string::npos) {
					s_aux = s_aux.substr(0, s_aux.find(")"));
					bv->val_pos = std::atof(s_aux.c_str());
					remaining_constraints = false;
				} else{
					bv->val_pos = std::atof(s_aux.c_str());
					in >> s_aux;
					assert(s_aux == "|");
				}
			}
			external_blocked_vars.push_back(bv);
		}
	}
	check_magic(in, "end_constraints");

	process_shared_vars_values();
}

void process_shared_vars_values()
{
	for(int i = 0; i < g_shared_vars.size(); i ++)
	{
		pair<int, vector<pair<int, float>* >* >* shared_var_pair_list = new pair<int, vector<pair<int, float>* >* >();

		shared_var_pair_list->first = g_shared_vars[i].second;

		vector<pair<int, float>* > *var_timed_values = new vector<pair<int, float>* >();

		pair<int, float> *first_value = new pair<int, float>();
		first_value->first = int(-1);
		first_value->second = float(0.00);
		var_timed_values->push_back(first_value);

		int min_index = -1;
		float curr_time_value = 0;
		bool min_found = false;
		int last_added_val = -2;
		do{
			min_found = false;
			float min_time = 999999;
			for(int j = 0; j < external_blocked_vars.size(); j++)
			{
				if((external_blocked_vars[j]->in_current_agent) && (external_blocked_vars[j]->var == g_shared_vars[i].second))
				{
					if((min_index == -1) || ((curr_time_value < external_blocked_vars[j]->time_set) && (min_time > external_blocked_vars[j]->time_set))){
						min_index = j;
						min_found = true;
						min_time = external_blocked_vars[j]->time_set;
					}
				}
			}

			if((min_index != -1) && min_found)
			{
				if(last_added_val != external_blocked_vars[min_index]->val_pos)
				{
					last_added_val = external_blocked_vars[min_index]->val_pos;
					pair<int, float> *timed_value = new pair<int, float>();
					timed_value->first = external_blocked_vars[min_index]->val_pos;
					timed_value->second = external_blocked_vars[min_index]->time_set;

					var_timed_values->push_back(timed_value);
					curr_time_value = external_blocked_vars[min_index]->time_set;
				}else{
					curr_time_value = external_blocked_vars[min_index]->time_set;
				}
			}

		} while(min_found);


		shared_var_pair_list->second = var_timed_values;
		g_shared_vars_timed_values.push_back(shared_var_pair_list);
	}


	for(int k = 0; k < g_shared_vars_timed_values.size(); k++)
	{
		cout << "States for shared var: " <<  g_shared_vars_timed_values[k]->first << endl;
		for(int j = 0; j < g_shared_vars_timed_values[k]->second->size(); j++)
		{
			cout << "--- " << (*(g_shared_vars_timed_values[k]->second))[j]->first <<
					" // " << (*(g_shared_vars_timed_values[k]->second))[j]->second << endl;
		}
	}
}

void read_ext_init_state()
{
	std::fstream in;
	int var_list_size;
	string slash = "";
	in.open ("end_state", std::fstream::in);
	check_magic(in, "begin_state");
	in >> var_list_size;
	for(int i = 0; i < var_list_size; i++)
	{
		string var_name = "";
		int val = 0;
		in >> var_name;
		in >> slash;
		in >> val;
		for(int j = 0; j < g_variable_name.size(); j++)
		{
			if(var_name == g_variable_name[j])
			{
				g_initial_state->set_var_value(j, val);
				break;
			}
		}
	}
	check_magic(in, "end_state");
	check_magic(in, "begin_num_state");
	in >> var_list_size;
	for(int i = 0; i < var_list_size; i++)
	{
		string var_name = "";
		string aux = "";
		in >> var_name;
		in >> aux;
		if(aux == "-")
		{
			float val = 0;
			in >> val;
			for(int j = 0; j < g_variable_name.size(); j++)
			{
				if(var_name == g_variable_name[j])
				{
					g_initial_state->numeric_vars_val[j] = val;
					break;
				}
			}
		}
	}
	check_magic(in, "end_num_state");
}

bool g_use_metric;
bool g_length_metric;
string g_op_metric;
vector <int> g_n_metric;
vector<string> g_variable_name;
vector<int> g_variable_domain;
vector<int> g_axiom_layers;
string total_time_var;
vector<int> g_default_axiom_values;
State *g_initial_state;
vector<ext_constraint*> external_blocked_vars;
vector<pair<int, int> > g_goal;
vector<pair<string, int> > g_shared_vars;
vector<pair<int, vector<pair<int, float>* >* >* > g_shared_vars_timed_values;
vector<Operator> g_operators;
vector<Operator> g_axioms;
AxiomEvaluator *g_axiom_evaluator;
SuccessorGenerator *g_successor_generator;
vector<DomainTransitionGraph *> g_transition_graphs;
CausalGraph *g_causal_graph;
Cache *g_cache;
int g_cache_hits = 0, g_cache_misses = 0;

FFHeuristic *g_ff_heur;
LandmarksCountHeuristic *g_lm_heur;
LandmarksGraph *g_lgraph;
