/*********************************************************************
 * Authors: Matthias Westphal (westpham@informatik.uni-freiburg.de),
 *          Silvia Richter (silvia.richter@nicta.com.au)
 * (C) Copyright 2008 Matthias Westphal and NICTA
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

#include <vector>
#include <list>
#include <utility>
#include <cassert>
#include <map>
#include <set>
#include <ext/hash_map>
#include <fstream>
#include <sstream>
#include <climits>
#include <unistd.h>
#define GetCurrentDir getcwd

#include "landmarks_graph.h"
#include "operator.h"
#include "state.h"
#include "globals.h"
#include "ff_heuristic.h"

using namespace std;


static inline bool _operator_eff_includes_non_conditional(const Operator& o, 
							  const LandmarkNode* lmp) {
/* Test whether the landmark is achieved by the operator unconditionally. 
   A disjunctive landmarks is achieved if one of its disjuncts is achieved. */
    assert(lmp != NULL);
    const vector<PrePost>& prepost = o.get_pre_post();
    for (unsigned j = 0; j < prepost.size(); j++) {
        for(unsigned int k = 0; k < lmp->vars.size(); k++) {
            if(prepost[j].var == lmp->vars[k] && prepost[j].post == lmp->vals[k])
                if(prepost[j].cond.empty())
                    return true;
        }
    }
    return false;
}


LandmarksGraph::LandmarksGraph() : landmarks_count(0),
				   use_external_inconsistencies(false),
				   reasonable_orders(false) {
    generate_operators_lookups();
}

bool LandmarksGraph::simple_landmark_exists(const pair<int, int>& lm) const {
    hash_map<pair<int, int>, LandmarkNode*, hash_int_pair>::const_iterator it = simple_lms_to_nodes.find(lm);
    assert(it == simple_lms_to_nodes.end() || !it->second->disjunctive);
    return it != simple_lms_to_nodes.end();
}

bool LandmarksGraph::landmark_exists(const pair<int, int>& lm) const {
    set<pair<int, int> > lm_set;
    lm_set.insert(lm);
    return simple_landmark_exists(lm) || disj_landmark_exists(lm_set);
}

bool LandmarksGraph::disj_landmark_exists(const set<pair<int, int> >& lm) const {
    // Test whether ONE of the facts in lm is present in some disj. LM
    for(set<pair<int, int> >::const_iterator it = lm.begin(); it != lm.end(); ++ it) {
        const pair<int, int>& prop = *it;
        hash_map<pair<int, int>, LandmarkNode*, hash_int_pair>::const_iterator it2 = disj_lms_to_nodes.find(prop);
        if(it2 != disj_lms_to_nodes.end())
            return true;
    }
    return false;
}

bool LandmarksGraph::exact_same_disj_landmark_exists(const set<pair<int, int> >& lm) const {
    // Test whether a disj. LM exists which consists EXACTLY of those facts in lm
    LandmarkNode* lmn = NULL;
    for(set<pair<int, int> >::const_iterator it = lm.begin(); it != lm.end(); ++ it) {
        const pair<int, int>& prop = *it;
        hash_map<pair<int, int>, LandmarkNode*, hash_int_pair>::const_iterator it2 
	    = disj_lms_to_nodes.find(prop);
	if(it2 == disj_lms_to_nodes.end())
	    return false;
        else {
	    if(lmn != NULL && lmn != it2->second) {
		return false;
	    }
	    else if(lmn == NULL)
		lmn = it2->second;
	}
    }
    return true;
}

const LandmarkNode* LandmarksGraph::landmark_reached(const pair<int, int>& prop) const {
/* Return pointer to landmark node that corresponds to the given fact, or 0 if no such
   landmark exists. (TODO: rename to "get_landmark()") 
*/
    const LandmarkNode* node_p = 0;
    hash_map<pair<int, int>, LandmarkNode*, hash_int_pair>::const_iterator it = simple_lms_to_nodes.find(prop);
    if(it != simple_lms_to_nodes.end())
        node_p = it->second;
    else {
        hash_map<pair<int, int>, LandmarkNode*, hash_int_pair>::const_iterator it2 = disj_lms_to_nodes.find(prop);
        if(it2 != disj_lms_to_nodes.end())
            node_p = it2->second;
    }
    return node_p;
}

LandmarkNode& LandmarksGraph::landmark_add_simple(const pair<int, int>& lm) {
    assert(!landmark_exists(lm));
    vector<int> vars;
    vector<int> vals;
    vars.push_back(lm.first);
    vals.push_back(lm.second);
    LandmarkNode* new_node_p = new LandmarkNode(vars, vals, false);
    nodes.insert(new_node_p);
    simple_lms_to_nodes.insert(make_pair(lm, new_node_p));
    landmarks_count++;
    return *new_node_p;
}

LandmarkNode& LandmarksGraph::landmark_add_disjunctive(const set<pair<int, int> >& lm) {
    vector<int> vars;
    vector<int> vals;
    for(set<pair<int, int> >::iterator it = lm.begin(); it != lm.end(); ++it) {
        vars.push_back(it->first);
        vals.push_back(it->second);
	assert(!landmark_exists(make_pair(it->first, it->second)));
    }
    LandmarkNode* new_node_p = new LandmarkNode(vars, vals, true);
    new_node_p->disjunctive = true;
    nodes.insert(new_node_p);
    for(set<pair<int, int> >::iterator it = lm.begin(); it != lm.end(); ++it) {
        disj_lms_to_nodes.insert(make_pair(*it, new_node_p));
    }
    landmarks_count++;
    return *new_node_p;
}

void LandmarksGraph::generate() {
    //cout << "generating landmarks" << endl;
    generate_landmarks();
    if(reasonable_orders) {
        cout << "approx. reasonable orders" << endl;
        approximate_reasonable_orders(false);
        cout << "approx. obedient reasonable orders" << endl;
        approximate_reasonable_orders(true);
    }
    mk_acyclic_graph();
    landmarks_cost = calculate_lms_cost();
}


void LandmarksGraph::read_external_inconsistencies() {
/* Read inconsistencies that were found by translator from separate file. Note: this
   is somewhat cumbersome, but avoids substantial changes to translator and predecessor 
   output structure. Translator finds groups of facts such that exactly one of those facts
   is true at any point in time. Hence, all facts within a group are mutually exclusive.
 */
    cout << "Reading invariants from file..." << endl;
    ifstream myfile ("all.groups");
    if(myfile.is_open()) {
        ifstream &in = myfile;
        check_magic(in, "begin_groups");
        int no_groups;
        in >> no_groups;
        // Number of variables is unknown, as preprocessing may throw out variables,
        // whereas our input comes directly from the translator. Thus we build an index
	// that maps for each variable from the number used by the translator (in var name) 
	// to the number used in the preprocessor output for that variable.
        hash_map<int, int> variable_index;
        for(int i = 0; i < g_variable_name.size(); i++) {
            string number = g_variable_name[i].substr(3);
            int number2 = atoi(number.c_str());
            variable_index.insert(make_pair(number2, i));
        }
        for(int j = 0; j < g_variable_name.size(); j++) {
            inconsistent_facts.push_back(vector<set<pair<int, int> > >());
            for(int k = 0; k < g_variable_domain[j]; k++)
                inconsistent_facts[j].push_back(set<pair<int, int> >());
        }

        for(int i = 0; i < no_groups; i++) {
            check_magic(in, "group");
            int no_facts;
            in >> no_facts;
            vector<pair <int, int> > invariant_group;
            for(int j = 0; j < no_facts; j++) {
                int var, val, no_args;
                in >> var >> val;
                string predicate, endline, aux;
                in >> predicate;
                if ((predicate == "Decrease") ||
                		(predicate == "Increase") ||
						(predicate == "Assign") ||
						(predicate == "GreaterThan") ||
						(predicate == "LessThan")) {
                	in >> aux;
                	while (aux != ">") {
                		predicate = predicate + " " + aux;
                    	in >> aux;
                	}
                }
				in >> no_args;
				vector<string> args;
				for(int k = 0; k < no_args; k++) {
					string arg;
					in >> arg;
					args.push_back(arg);
				}

                getline(in, endline);
		// Variable may not be in index if it has been discarded by preprocessor
                if(variable_index.find(var) != variable_index.end()) {
                    pair<int, int> var_val_pair = make_pair(variable_index.find(var)->second, val);
                    invariant_group.push_back(var_val_pair);
		    // Save fact with predicate name (needed for disj. LMs / 1-step lookahead)
                    Pddl_proposition prop;
                    prop.predicate = predicate;
                    prop.arguments = args;
                    pddl_propositions.insert(make_pair(var_val_pair, prop));
                    if(pddl_proposition_indeces.find(predicate) == pddl_proposition_indeces.end()) {
                        pddl_proposition_indeces.insert(make_pair(predicate, pddl_proposition_indeces.size()));
                    }
                }
            }
            for(int j = 0; j < invariant_group.size(); j++) {
                for(int k = 0; k < invariant_group.size(); k++) {
                    if(j == k)
                        continue;
                    inconsistent_facts[invariant_group[j].first][invariant_group[j].second].insert
                        (make_pair(invariant_group[k].first, invariant_group[k].second));
                }
            }
        }
        check_magic(in, "end_groups");
        myfile.close();
	use_external_inconsistencies = true;
/*
        if( remove( "all.groups" ) != 0 )
            perror( "Error deleting file" );
        else
            cout << "File successfully deleted" << endl;
*/
        cout << "done" << endl;
    }
    else {
    	char buff[200]; //create string buffer to hold path
    	GetCurrentDir( buff, 200 );
    	string current_working_dir(buff);
        cout << "Unable to open invariants file! in " << current_working_dir << "/all.groups" << endl;
        exit(1);
    }
}

bool LandmarksGraph::relaxed_task_solvable(vector<vector<int> >& lvl_var, 
					   vector<hash_map<pair<int, int>, int, 
					   hash_int_pair> >& lvl_op, 
					   bool level_out,
					   const LandmarkNode* exclude,
					   bool compute_lvl_op) const {
/* Test whether the relaxed planning task is solvable without achieving the propositions in
   "exclude" (do not apply operators that would add a proposition from "exclude").
   As a side effect, collect in lvl_var and lvl_op the earliest possible point in time
   when a proposition / operator can be achieved / become applicable in the relaxed task. 
 */

    // Initialize lvl_op and lvl_var to INT_MAX
    if(compute_lvl_op) {
	lvl_op.resize(g_operators.size() + g_axioms.size());
	for(int i = 0; i < g_operators.size() + g_axioms.size(); i++) {
	    const Operator& op = get_operator_for_lookup_index(i);
	    lvl_op[i] = hash_map<pair<int, int>, int, hash_int_pair>();
	    const vector<PrePost>& prepost = op.get_pre_post();
	    for(unsigned j = 0; j < prepost.size(); j++) 
		lvl_op[i].insert(make_pair(make_pair(prepost[j].var, prepost[j].post), INT_MAX));
	}
    }
    lvl_var.resize(g_variable_name.size());
    for (unsigned var = 0; var < g_variable_name.size(); var++) {
        lvl_var[var].resize(g_variable_domain[var], INT_MAX); 
    }
    // Extract propositions from "exclude"
    hash_set<const Operator *, hash_operator_ptr> exclude_ops;
    vector<pair<int, int> > exclude_props;
    if(exclude != NULL) {
	for(int op = 0; op < g_operators.size(); op++) {
	    if(_operator_eff_includes_non_conditional(g_operators[op], exclude))
		exclude_ops.insert(&g_operators[op]);
	}
	for(int i = 0; i < exclude->vars.size(); i++)
	    exclude_props.push_back(make_pair(exclude->vars[i], exclude->vals[i]));
    }
    // Do relaxed exploration in ff_heuristic class
    g_ff_heur->compute_reachability_with_excludes(lvl_var, lvl_op, level_out, exclude_props, exclude_ops,
						  compute_lvl_op);
    
    // Test whether all goal propositions have a level of less than INT_MAX
    for(int i = 0; i < g_goal.size(); i++)
		if(lvl_var[g_goal[i].first][g_goal[i].second] == INT_MAX) {
			return false;
		}
    return true;
}

void LandmarksGraph::generate_operators_lookups() {
/* Build datastructures for efficient landmark computation. Map propositions
   to the operators that achieve them or have them as preconditions */

	/* operators_pre_lookup: [var] [value] -> indexes of operators precond
	 * operators_eff_lookup: [var] [value] -> indexes of operators effects
	 * empty_pre_operators: ops with no pre of eff
	 * */
  
    operators_pre_lookup.resize(g_variable_domain.size());
    operators_eff_lookup.resize(g_variable_domain.size());
    for (unsigned i = 0; i < g_variable_domain.size(); i++) {
        operators_pre_lookup[i].resize(g_variable_domain[i]);
        operators_eff_lookup[i].resize(g_variable_domain[i]);
    }
    for (unsigned i = 0; i < g_operators.size() + g_axioms.size(); i++) {
	const Operator& op = get_operator_for_lookup_index(i);

        const vector<PrePost>& prepost = op.get_pre_post();
        bool no_pre = true;
        for (unsigned j = 0; j < prepost.size(); j++) {
            if((prepost[j].pre != -1) && (prepost[j].pre != -2) &&
            		(prepost[j].pre != -3) && (prepost[j].pre != -4)
					&& (prepost[j].pre != -5)  && (prepost[j].pre != -6)) {
                no_pre = false;
                operators_pre_lookup[prepost[j].var][prepost[j].pre].push_back(i);
            }
            operators_eff_lookup[prepost[j].var][prepost[j].post].push_back(i);
        }
        const vector<Prevail>& prevail = op.get_prevail();
        for(unsigned j = 0; j < prevail.size(); j++) {
                no_pre = false;
                operators_pre_lookup[prevail[j].var][prevail[j].prev].push_back(i);
	}
        if(no_pre)
            empty_pre_operators.push_back(i);
    }
}

bool LandmarksGraph::effect_always_happens(const vector<PrePost>& prepost, 
					   set<pair<int, int> >& eff) const {
/* Test whether the condition of a conditional effect is trivial, i.e. always true.
   We test for the simple case that the same effect proposition is triggered by
   a set of conditions of which one will always be true. This is e.g. the case in
   Schedule, where the effect 
   (forall (?oldpaint - colour)
      (when (painted ?x ?oldpaint)
         (not (painted ?x ?oldpaint))))
   is translated by the translator to: if oldpaint == blue, then not painted ?x, and if
   oldpaint == red, then not painted ?x etc.
   If conditional effects are found that are always true, they are returned in "eff".
 */
    // Go through all effects of operator (as given by prepost) and collect: 
    // - all variables that are set to some value in a conditional effect (effect_vars)
    // - variables that can be set to more than one value in a cond. effect (nogood_effect_vars)
    // - a mapping from cond. effect propositions to all the conditions that they appear with 
    set<int> effect_vars;
    set<int> nogood_effect_vars;
    map<int, pair<int, vector<pair<int, int> > > > effect_conditions;
    for(unsigned i = 0; i < prepost.size(); i++) {
        if(prepost[i].cond.empty() || 
           nogood_effect_vars.find(prepost[i].var) != nogood_effect_vars.end()) {
            // Var has no condition or can take on different values, skipping 
            continue;
        }
        if(effect_vars.find(prepost[i].var) != effect_vars.end()) {
            // We have seen this effect var before
            assert(effect_conditions.find(prepost[i].var) != effect_conditions.end());
            int old_eff = effect_conditions.find(prepost[i].var)->second.first;
            if(old_eff != prepost[i].post) {
                // Was different effect
                nogood_effect_vars.insert(prepost[i].var);
                continue;
            }
        }
        else{
            // We have not seen this effect var before
            effect_vars.insert(prepost[i].var);     
        }
        if(effect_conditions.find(prepost[i].var) != effect_conditions.end() &&
           effect_conditions.find(prepost[i].var)->second.first == prepost[i].post) {
            // We have seen this effect before, adding conditions
            for(int k = 0; k < prepost[i].cond.size(); k++) {
		vector<pair<int, int> >& vec = effect_conditions.find
                    (prepost[i].var)->second.second;
                vec.push_back(make_pair(prepost[i].cond[k].var, prepost[i].cond[k].prev ));
            }		    
        }
        else {
            // We have not seen this effect before, making new effect entry 
            vector<pair<int, int> >& vec = effect_conditions.insert
               (make_pair(prepost[i].var, make_pair(prepost[i].post, 
                           vector<pair<int, int> >()))).first->second.second;
            for(int k = 0; k < prepost[i].cond.size(); k++) {
                vec.push_back(make_pair(prepost[i].cond[k].var, prepost[i].cond[k].prev ));
            }
        }	       		
    }

    // For all those effect propositions whose variables do not take on different values...
    map<int, pair<int, vector<pair<int, int> > > >::iterator it = effect_conditions.begin();
    for(; it != effect_conditions.end(); ++it) {
        if(nogood_effect_vars.find(it->first) != nogood_effect_vars.end()) {
            continue;
        }
	// ...go through all the conditions that the effect has, and map condition 
	// variables to the set of values they take on (in unique_conds)
        map<int, set<int> > unique_conds;
        vector<pair<int, int> >& conds = it->second.second;
        for(unsigned int j = 0; j < conds.size(); j++) {
            if(unique_conds.find(conds[j].first) != unique_conds.end()) {
                unique_conds.find(conds[j].first)->second.insert(conds[j].second);
            }
            else {
                set<int>& the_set = unique_conds.insert
                    (make_pair(conds[j].first, set<int>())).first->second;
                the_set.insert(conds[j].second);
            }
        }
	// Check for each condition variable whether the number of values it takes on is
	// equal to the domain of that variable...
	pair<int, int> effect = make_pair(it->first, it->second.first);
	bool is_always_reached = true;
        map<int, set<int> >::iterator it2 = unique_conds.begin();
        for(; it2 != unique_conds.end(); ++it2) {
	    bool is_surely_reached_by_var = false;
            if(it2->second.size() == g_variable_domain[it2->first]) {
		is_surely_reached_by_var = true;
            }
	    // ...or else if the condition variable is the same as the effect variable,
	    // check whether the condition variable takes on all other values except the
	    // effect value
            else if (it2->first == it->first &&
                     it2->second.size() == g_variable_domain[it2->first] - 1) {
                // Number of different values is correct, now ensure that the effect value
		// was the one missing
                it2->second.insert(it->second.first);
                if(it2->second.size() == g_variable_domain[it2->first]) {
		    is_surely_reached_by_var = true;
                }
            }
	    // If one of the condition variables does not fulfill the criteria, the effect
	    // is not certain to happen
	    if(!is_surely_reached_by_var)
		is_always_reached = false;
        }
	if(is_always_reached)
	    eff.insert(effect);
    }
    return eff.empty();
}

inline bool LandmarksGraph::inconsistent(const pair<int,int>& a, 
					 const pair<int,int>& b) const {
    assert(a.first != b.first || a.second != b.second);
    if(a.first == b.first && a.second != b.second)
        return true;
    if(use_external_inconsistencies &&
       inconsistent_facts[a.first][a.second].find(b) != inconsistent_facts[a.first][a.second].end()) 
        return true;
    return false;
}

bool LandmarksGraph::interferes(const LandmarkNode* node_a, const LandmarkNode* node_b) const {
/* Facts a and b interfere (i.e., achieving b before a would mean having to delete b 
   and re-achieve it in order to achieve a) if one of the following condition holds:
   1. a and b are inconsistent
   2. All actions that add a also add e, and e and b are inconsistent
   3. There is a greedy necessary predecessor x of a, and x and b are inconsistent
   This is the definition of Hoffmann et al. except that they have one more condition: 
   "all actions that add a delete b". However, in our case (SAS+ formalism), this condition
   is the same as 2.
*/
    assert(!node_a->disjunctive && ! node_b->disjunctive);
    pair<int, int> a = make_pair(node_a->vars[0], node_a->vals[0]);
    pair<int, int> b = make_pair(node_b->vars[0], node_b->vals[0]);
    assert(a.first != b.first || a.second != b.second);

    // 1. a, b inconsistent
    if(inconsistent(a,b))
        return true;

    // 2. Shared effect e in all operators reaching a, and e, b inconsistent
    hash_map<int, int> shared_eff;
    bool init = true;
    const vector<int>& ops = get_operators_including_eff(a);
    // Intersect operators that achieve a one by one
    for(unsigned i = 0; i < ops.size(); i++) {
	const Operator& op = get_operator_for_lookup_index(ops[i]);
	// If no shared effect among previous operators, break
	if(!init && shared_eff.empty()) 
	    break;
	// Else, insert effects of this operator into set "next_eff" if
	// it is an unconditional effect or a conditional effect that is sure to 
	// happen. (Such "trivial" conditions can arise due to our translator,
        // e.g. in Schedule. There, the same effect is conditioned on a disjunction
	// of conditions of which one will always be true. We test for a simple kind
	// of these trivial conditions here.)
	const vector<PrePost>& prepost = op.get_pre_post();
	set<pair<int, int> > trivially_conditioned_effects;
	bool testing_for_trivial_conditions = true;
	bool trivial_conditioned_effects_found = false;
	if(testing_for_trivial_conditions)	     
	    trivial_conditioned_effects_found = effect_always_happens
		(prepost, trivially_conditioned_effects);
	hash_map<int, int> next_eff;
	for(unsigned i = 0; i < prepost.size(); i++) {
	    if(prepost[i].cond.empty() && prepost[i].var != a.first) {
		next_eff.insert(make_pair(prepost[i].var, prepost[i].post));
	    }
	    else if(testing_for_trivial_conditions && trivial_conditioned_effects_found &&
		    trivially_conditioned_effects.find(make_pair(prepost[i].var, prepost[i].post)) !=
		    trivially_conditioned_effects.end()) 
		next_eff.insert(make_pair(prepost[i].var, prepost[i].post));
	}
	// Intersect effects of this operator with those of previous operators  
	if(init)
	    swap(shared_eff, next_eff);
	else {
	    hash_map<int, int> result;
	    for(hash_map<int, int>::iterator it1 = shared_eff.begin(); it1 != shared_eff.end(); it1++) {
		hash_map<int, int>::iterator it2 = next_eff.find(it1->first);
		if(it2 != next_eff.end() && it2->second == it1->second)
		    result.insert(*it1);
	    }
	    swap(shared_eff, result);
	}
	init = false;
    }
    // Test whether one of the shared effects is inconsistent with b
    for(hash_map<int, int>::iterator it = shared_eff.begin(); 
	it != shared_eff.end(); it++)
	if(make_pair(it->first, it->second) != a && 
	   make_pair(it->first, it->second) != b && inconsistent(*it, b)) 
	    return true;

    // 3. Exists LM x, inconsistent x, b and x->_gn a
    const LandmarkNode& node = *node_a;
    for(hash_map<LandmarkNode*, edge_type, hash_pointer >::const_iterator it = 
            node.parents.begin(); it != node.parents.end(); it++) {    
        edge_type edge = it->second;
        pair<int, int> parent_prop = make_pair(it->first->vars[0],it->first->vals[0]);  
        if((edge == n || edge == gn) && parent_prop != b && inconsistent(parent_prop, b))
            return true;
    }
    // No inconsistency found
    return false;
}

inline static bool _in_goal(const pair<int,int>& l) {
    for(vector<pair<int, int> >::const_iterator start = g_goal.begin(); 
        start != g_goal.end(); start++)
        if(*start == l) // l \subseteq goal
            return true;

    return false;
}

void LandmarksGraph::approximate_reasonable_orders(bool obedient_orders) {
/* Approximate reasonable and obedient reasonable orders according to Hoffmann et al. If flag
   "obedient_orders" is true, we calculate obedient reasonable orders, otherwise reasonable orders. 

   If node_p is in goal, then any node2_p which interferes with node_p can be reasonably ordered 
   before node_p. Otherwise, if node_p is greedy necessary predecessor of node2, and there is another
   predecessor "parent" of node2, then parent and all predecessors of parent can be ordered reasonably 
   before node_p if they interfere with node_p.
 */
    for(set<LandmarkNode*>::iterator it = nodes.begin(); it != nodes.end(); it++) {
        LandmarkNode* node_p = *it;
        if(node_p->disjunctive)
            continue;
        pair<int, int> node_prop = make_pair(node_p->vars[0], node_p->vals[0]);

        if(!obedient_orders &&_in_goal(node_prop)) {
            for(set<LandmarkNode*>::iterator it2 = nodes.begin(); it2 != nodes.end(); it2++) {
                LandmarkNode* node2_p = *it2;
                if(node2_p->disjunctive)
                    continue;
                pair<int, int> node2_prop = make_pair(node2_p->vars[0], node2_p->vals[0]);
                if(node_prop != node2_prop && interferes(node2_p, node_p)) {
		    edge_add(*node2_p, *node_p, r);
                }
            }
        }
        else if(!node_p->is_true_in_state(*g_initial_state)) {
            // Collect candidates for reasonable orders in "interesting nodes". 
            // Use hash set to filter duplicates.
            hash_set<LandmarkNode*, hash_pointer> interesting_nodes(g_variable_name.size());
            for(hash_map<LandmarkNode*, edge_type, hash_pointer >::iterator 
                    it = node_p->children.begin(); it != node_p->children.end(); it++) {
                if(it->second == gn) { // found node2: node_p ->_gn node2
                    LandmarkNode& node2 = *(it->first);
                    for(hash_map<LandmarkNode*, edge_type, hash_pointer>::iterator 
                            it2 = node2.parents.begin(); it2 != node2.parents.end(); it2++) { // find parent
                        edge_type& edge = it2->second;
                        LandmarkNode& parent = *(it2->first);
			if(parent.disjunctive)
                            continue;
                        if( (edge == gn || edge == n || edge == ln || (obedient_orders && edge == r)) && 
                            &parent != node_p) { // find predecessors or parent and collect in "interesting nodes" 
                            interesting_nodes.insert(&parent);
                            collect_ancestors(interesting_nodes, parent, obedient_orders);
                        }
                    }
                }
            }
	    // Insert reasonable orders between those members of "interesting nodes" that interfere 
	    // with node_p. 
            for(hash_set<LandmarkNode*, hash_pointer>::iterator 
                    it3 = interesting_nodes.begin(); it3 != interesting_nodes.end(); it3++) {
                if((*it3)->disjunctive)
                    continue;
                pair<int, int> it_prop = make_pair((*it3)->vars[0], (*it3)->vals[0]);
                if(it_prop != node_prop && interferes(*it3, node_p)) {
		    if(!obedient_orders)
			edge_add(**it3, *node_p, r);
		    else
			edge_add(**it3, *node_p, o_r);
		}
            }
        }

    }
}

void LandmarksGraph::collect_ancestors(hash_set<LandmarkNode*, hash_pointer>& result, 
				       LandmarkNode& node, bool use_reasonable) {
    /* Returns all ancestors in the landmark graph of landmark node "start" */

    // There could be cycles if use_reasonable == true
    list<LandmarkNode*> open_nodes;
    hash_set<LandmarkNode*, hash_pointer > closed_nodes;
    for(hash_map<LandmarkNode*, edge_type, hash_pointer >::iterator it 
            = node.parents.begin(); it != node.parents.end(); it++) {
        edge_type& edge = it->second;
        LandmarkNode& parent = *(it->first);
        if(edge == gn || edge == n || edge == ln || 
           (use_reasonable && edge == r))
            if(closed_nodes.find(&parent) == closed_nodes.end()) {
                open_nodes.push_back(&parent);
                closed_nodes.insert(&parent);
                result.insert(&parent);
            }
    }
    while(!open_nodes.empty()) {
        LandmarkNode& node2 = *(open_nodes.front());
        for(hash_map<LandmarkNode*, edge_type, hash_pointer >::iterator it 
                = node2.parents.begin(); it != node2.parents.end(); it++) {
            edge_type& edge = it->second;
            LandmarkNode& parent = *(it->first);
            if(edge == gn || edge == n || edge == ln || 
               (use_reasonable && edge == r)) {
                if (closed_nodes.find(&parent) == closed_nodes.end()) {
                    open_nodes.push_back(&parent);
                    closed_nodes.insert(&parent);
                    result.insert(&parent);
                }
            }
        }
        open_nodes.pop_front();
    }
}

void LandmarksGraph::print_proposition(const pair<int,int> &proposition) const {
  hash_map<pair<int, int>, Pddl_proposition, hash_int_pair>::const_iterator it = 
    pddl_propositions.find(proposition);
  if(it != pddl_propositions.end()) {
    cout << it->second.to_string();
  }
  else {
    cout << "Name unknown";
  }
  cout << " (" << g_variable_name[proposition.first] << "(" << proposition.first << ")"
       << "->" << proposition.second << ")";
}

void LandmarksGraph::dump_node(const LandmarkNode* node_p) const {
    if(node_p->disjunctive) 
        cout << "{";
    for(unsigned int i = 0; i < node_p->vars.size(); i++ ) {
        pair<int, int> node_prop = make_pair(node_p->vars[i], node_p->vals[i]);
        hash_map<pair<int, int>, Pddl_proposition, hash_int_pair>::const_iterator it = 
            pddl_propositions.find(node_prop);
        if(it != pddl_propositions.end()) {
            cout << it->second.to_string() << " ("
                 << g_variable_name[node_prop.first] << "(" << node_prop.first << ")"
                 << "->" << node_prop.second << ")";
        }
        else {
            cout << g_variable_name[node_prop.first] 
                 << " (" << node_prop.first << ") "  
                 << "->" << node_prop.second;
        }
        if(i < node_p->vars.size() - 1)
            cout << ", ";
    }
    if(node_p->disjunctive) 
        cout << "}"; 
    if(g_use_metric)
	cout << " min_cost: " << node_p->min_cost;
    cout << endl;
}

void LandmarksGraph::dump() const {
    for(set<LandmarkNode*>::const_iterator it 
            = nodes.begin(); it != nodes.end(); it++) {
        LandmarkNode* node_p = *it;
        dump_node(node_p);
        for(hash_map<LandmarkNode*, edge_type, hash_pointer >::const_iterator parent_it 
                = node_p->parents.begin(); parent_it != node_p->parents.end(); parent_it++) {
            const edge_type& edge = parent_it->second;
            const LandmarkNode* parent_p = parent_it->first;
            cout << "\t\t<-_";
            switch(edge) {
            case n:  cout << "n   "; break;
            case r:  cout << "r   "; break;
            case gn: cout << "gn  "; break;
            case ln: cout << "ln  "; break;
            case o_r:cout << "o_r "; break;
            }
            dump_node(parent_p);
        }
        for(hash_map<LandmarkNode*, edge_type, hash_pointer >::const_iterator child_it 
                = node_p->children.begin(); child_it != node_p->children.end(); child_it++) {
            const edge_type& edge = child_it->second;
            const LandmarkNode* child_p = child_it->first;
            cout << "\t\t->_";
            switch(edge) {
            case n:  cout << "n   "; break;
            case r:  cout << "r   "; break;
            case gn: cout << "gn  "; break;
            case ln: cout << "ln  "; break;
            case o_r:cout << "o_r "; break;
            }
            dump_node(child_p);
        }
    }    
}

int LandmarksGraph::number_of_edges() const {
    int total = 0;
    for(set<LandmarkNode*>::const_iterator it 
            = nodes.begin(); it != nodes.end(); it++)
        total += (*it)->children.size();
    return total;
}

void LandmarksGraph::edge_add(LandmarkNode& from, 
			      LandmarkNode& to, edge_type type) {
/* Adds an edge in the landmarks graph if there is no contradicting edge (simple measure to
   reduce cycles. If the edge is already present, the stronger edge type wins.
*/
    assert(&from != &to);
    assert(from.parents.find(&to) == from.parents.end() || type == r || type == o_r);
    assert(to.children.find(&from) == to.children.end() || type == r || type == o_r);

    if(type == r || type == o_r) // simple cycle test
        if(from.parents.find(&to) != from.parents.end()) { // Edge in opposite direction exists
            //cout << "edge in opposite direction exists" << endl;
            if(from.parents.find(&to)->second > type) // Stronger order present, return
                return;
            // Edge in opposite direction is weaker, delete
            from.parents.erase(&to);
            to.children.erase(&from);
        }
    // If edge already exists, remove if weaker
    if(from.children.find(&to) != from.children.end() && 
       from.children.find(&to)->second < type) {
        from.children.erase(&to);
        assert(to.parents.find(&from) != to.parents.end());
        to.parents.erase(&from);

        assert(to.parents.find(&from) == to.parents.end());
        assert(from.children.find(&to) == from.children.end());
    }
    // If edge does not exist (or has just been removed), insert
    if(from.children.find(&to) == from.children.end()) {
        assert(to.parents.find(&from) == to.parents.end());
        from.children.insert(make_pair(&to, type));
        to.parents.insert(make_pair(&from, type));
        //cout << "added parent with address " << &from << endl; 
    }
    assert(from.children.find(&to) != from.children.end());
    assert(to.parents.find(&from) != to.parents.end());
}


void LandmarksGraph::mk_acyclic_graph() {
    hash_set<LandmarkNode*, hash_pointer> acyclic_node_set(number_of_landmarks());
    int removed_edges = 0;
    for(set<LandmarkNode*>::iterator it = 
            nodes.begin(); it != nodes.end(); it++) {
        LandmarkNode& lmn = **it;
	if(acyclic_node_set.find(&lmn) == acyclic_node_set.end())
	    removed_edges += loop_acyclic_graph(lmn, acyclic_node_set);
    }
    assert(acyclic_node_set.size() == number_of_landmarks());
    cout << "Removed " << removed_edges << " reasonable or obedient reasonable orders\n";
}

bool LandmarksGraph::remove_first_weakest_cycle_edge(LandmarkNode* cur, 
                                                     list<pair<LandmarkNode*, edge_type> >& path,
                                                     list<pair<LandmarkNode*, edge_type> >::iterator it) { 

    LandmarkNode* parent_p = 0;
    LandmarkNode* child_p = 0;
    for(list<pair<LandmarkNode*, edge_type> >::iterator it2 = it ; it2 != path.end() ; it2++) {
        edge_type edge = it2->second;
        if(edge == o_r || edge == r) {
            parent_p = it2->first;
            if(*it2 == path.back()) {
                child_p = cur;
                break;
            }
            else {
                list<pair<LandmarkNode*, edge_type> >::iterator child_it = it2;
                child_it++;
                child_p = child_it->first;
            }
            if(edge == o_r) 
                break;
            // else no break since o_r order could still appear in list
        }
    }
    assert(parent_p != 0 && child_p != 0);
    assert(parent_p->children.find(child_p) != parent_p->children.end());
    assert(child_p->parents.find(parent_p) != child_p->parents.end());
    parent_p->children.erase(child_p);
    child_p->parents.erase(parent_p);
    return true;  
}

int LandmarksGraph::loop_acyclic_graph(LandmarkNode& lmn, hash_set<LandmarkNode*, 
				       hash_pointer>& acyclic_node_set) {
    assert(acyclic_node_set.find(&lmn) == acyclic_node_set.end());
    int nr_removed = 0;
    list<pair<LandmarkNode*, edge_type> > path;
    hash_set<LandmarkNode*, hash_pointer> visited = hash_set<LandmarkNode*, 
        hash_pointer>(number_of_landmarks());
    LandmarkNode* cur = &lmn;
    while(true) {
	assert(acyclic_node_set.find(cur) == acyclic_node_set.end());
	if(visited.find(cur) != visited.end()) { // cycle
            // find other occurence of cur node in path
            list<pair<LandmarkNode*, edge_type> >::iterator it;
            for(it = path.begin(); it != path.end(); it++) {
                if(it->first == cur)
                    break;
            }
            assert(it != path.end() && it->first == cur);
            // remove edge from graph
	    remove_first_weakest_cycle_edge(cur, path, it);
	    //assert(removed);
	    nr_removed++;
            
            path.clear();
            cur = &lmn;
            visited.clear();
            continue;
	}
	visited.insert(cur);
	bool empty = true;
	for(hash_map<LandmarkNode*, edge_type, hash_pointer >::const_iterator it 
                = cur->children.begin(); it != cur->children.end(); it++) {
            edge_type edge = it->second;
            LandmarkNode* child_p = it->first;
	    if(acyclic_node_set.find(child_p) == acyclic_node_set.end()) {
		path.push_back(make_pair(cur, edge));
		cur = child_p;
		empty = false;
		break;
	    }
	}
	if(!empty) 
	    continue;
	
	// backtrack
	visited.erase(cur);
	acyclic_node_set.insert(cur);
	if(!path.empty()) {
	    cur = path.back().first;
	    path.pop_back();
	    visited.erase(cur);
	}
	else
	    break;
    }
    assert(acyclic_node_set.find(&lmn) != acyclic_node_set.end());
    return nr_removed;
}
int LandmarksGraph::calculate_lms_cost() const {
    int result = 0;
    for(set<LandmarkNode*>::const_iterator it = nodes.begin();
	it != nodes.end(); it++)
	result += (*it)->min_cost;

    return result;
}
