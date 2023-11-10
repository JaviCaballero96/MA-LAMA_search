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

#include "globals.h"
#include "operator.h"
#include "state.h"
#include "successor_generator.h"

#include <cstdlib>
#include <iostream>
#include <vector>
#include <string>

using namespace std;

class SuccessorGeneratorSwitch : public SuccessorGenerator {
    int switch_var;
    SuccessorGenerator *immediate_ops;
    vector<SuccessorGenerator *> generator_for_value;
    SuccessorGenerator *default_generator;
public:
    SuccessorGeneratorSwitch(istream &in);
    virtual void generate_applicable_ops(const State &curr,
					 vector<const Operator *> &ops);
    virtual void _dump(string indent);
};

class SuccessorGeneratorGenerate : public SuccessorGenerator {
    vector<const Operator *> op;
public:
    SuccessorGeneratorGenerate(istream &in);
    virtual void generate_applicable_ops(const State &curr,
					 vector<const Operator *> &ops);
    virtual void _dump(string indent);
};

extern bool use_hard_temporal_constraints;

SuccessorGeneratorSwitch::SuccessorGeneratorSwitch(istream &in) {
    in >> switch_var;
    immediate_ops = read_successor_generator(in);
    for(int i = 0; i < g_variable_domain[switch_var]; i++)
	generator_for_value.push_back(read_successor_generator(in));
    default_generator = read_successor_generator(in);
}

void SuccessorGeneratorSwitch::generate_applicable_ops(
    const State &curr, vector<const Operator *> &ops) {
    immediate_ops->generate_applicable_ops(curr, ops);
    if(curr[switch_var] != -1)
    	generator_for_value[curr[switch_var]]->generate_applicable_ops(curr, ops);
    default_generator->generate_applicable_ops(curr, ops);
    /*check_functional_validity(curr, ops);
    check_var_locks_validity(curr, ops);
    check_temporal_soundness_validity(curr, ops);*/
}

void check_functional_validity(
		const State &curr, vector<const Operator *> &ops) {
	vector<const Operator *>::iterator it = ops.begin();

	for(; it != ops.end();) {
		const Operator * op = *it;

		// we have to update the numeric values up to the point to which the action has been executed
		// create an auxiliary numeric values vector
		State* aux_state = new State(curr);
		for(int j = 0; j < curr.numeric_vars_val.size(); j++)
		{
			aux_state->numeric_vars_val[j] = curr.numeric_vars_val[j];
		}

		// apply the numeric effects that are running, rules are:
		// -- increase and decrease effects happen during all the time the action executes
		// -- assign effects happen abruptly at the end of the action
		// These rules aim to imitate real world behaviors

		vector<runn_action>::const_iterator it_ra = curr.running_actions.begin();
		for(; (is_temporal) && (it_ra != curr.running_actions.end()); ++it_ra) {

			// calculate how much time has passed
			float action_duration = (*it_ra).time_end - (*it_ra).time_start;
			float time_passed = curr.get_g_current_time_value() - (*it_ra).time_start;
			float peroneage_completed = (time_passed / action_duration);

			vector<PrePost*>::const_iterator it_fc = (*it_ra).functional_costs.begin();
			for(; it_fc != (*it_ra).functional_costs.end(); ++it_fc) {

				// calculate new values taking into account the time that has passed
				PrePost pre_post = *(*it_fc);
				switch(pre_post.pre){
					case -2:{
						if (!pre_post.have_runtime_cost_effect){
							aux_state->numeric_vars_val[pre_post.var] = aux_state->numeric_vars_val[pre_post.var] + (pre_post.f_cost * peroneage_completed);
						}
						else{
							aux_state->numeric_vars_val[pre_post.var] = aux_state->numeric_vars_val[pre_post.var] +
									(aux_state->calculate_runtime_efect<float>(pre_post.runtime_cost_effect) * peroneage_completed);
						}

						break;
					}
					case -3:
						if (!pre_post.have_runtime_cost_effect)
							aux_state->numeric_vars_val[pre_post.var] = aux_state->numeric_vars_val[pre_post.var] - (pre_post.f_cost * peroneage_completed);
						else{
							aux_state->numeric_vars_val[pre_post.var] = aux_state->numeric_vars_val[pre_post.var] -
									(aux_state->calculate_runtime_efect<float>(pre_post.runtime_cost_effect) * peroneage_completed);
						}
						break;

					case -4:
					case -5:
					case -6:
						break;

					default:
						break;
					}

			}
		}




		bool op_valid = true;
		vector<PrePost>::const_iterator it_pp = op->get_pre_post().begin();
		for(; it_pp != op->get_pre_post().end(); ++it_pp) {
			PrePost pp = *it_pp;
			if(pp.pre == -5)
			{
				if (aux_state->numeric_vars_val[pp.var] < pp.f_cost) {
					op_valid = false;
					break;
				}
			}else if(pp.pre == -6) {
				if (aux_state->numeric_vars_val[pp.var] > pp.f_cost) {
					op_valid = false;
					break;
				}
			}

			if(!op_valid)
				break;
		}

		delete aux_state ;

		if (!op_valid){
			it = ops.erase(it);
		}else
			it++;
	}
}

void check_external_locks_validity(const State &curr, vector<const Operator *> &ops) {
	vector<const Operator *>::iterator it = ops.begin();
	for(; it != ops.end();) {
		const Operator * op = *it;
		bool op_valid = true;

		// Get action duration, temporal duration, not snap.
		// start --- end
		float op_end_time = curr.get_g_current_time_value() + 0.01;
		float op_duration = 0;
		if(op->get_name().find("_end") != string::npos)
		{
			vector<runn_action>::const_iterator it_ra = curr.running_actions.begin();
			for(; it_ra != curr.running_actions.end();)
			{
				if((*it_ra).non_temporal_action_name == op->get_non_temporal_action_name())
				{
					op_end_time = (*it_ra).time_end;
				}else{
					it_ra++;
				}

				break;
			}
		}else if(op->get_name().find("_start") != string::npos)
		{
			// Get the duration calculating the costfrom the current state
			vector<PrePost>::const_iterator it_pp = op->get_pre_post().begin();
			for(; it_pp != op->get_pre_post().end(); ++it_pp) {
				PrePost pp = *it_pp;
				if(g_variable_name[pp.var] == total_time_var)
				{
					if(pp.have_runtime_cost_effect)
					{
						op_duration = curr.calculate_runtime_efect<float>(pp.runtime_cost_effect);
					} else {
						op_duration = pp.f_cost;
					}
					break;
				}
			}
		}

		/* bool action_changes_constraint = false;
		for(int k = 0; k < external_blocked_vars.size(); k++)
		{
			if((external_blocked_vars[k]->time_set >= curr.get_g_current_time_value()) &&
					(external_blocked_vars[k]->time_set < op_end_time))
			{
				vector<PrePost>::const_iterator it_pp = op->get_pre_post().begin();
				for(; it_pp != op->get_pre_post().end(); ++it_pp)
				{

					PrePost pp = *it_pp;
					if(pp.var == external_blocked_vars[k]->var)
					{
						if((op->get_name().find("_end") != string::npos) && (curr.running_actions.size() != 1))
						{
							op_valid = false;
							break;
						} else if ((op->get_name().find("_start") != string::npos) && (curr.running_actions.size() != 0)){
							op_valid = false;
							break;
						}
						action_changes_constraint = true;
					}
				}
			}
		} */

		for(int k = 0; (k < g_shared_vars_timed_values.size()) && (op_valid); k++)
		{
			vector<PrePost>::const_iterator it_pp = op->get_pre_post().begin();
			for(; it_pp != op->get_pre_post().end(); ++it_pp)
			{
				PrePost pp = *it_pp;
				if(pp.var == g_shared_vars_timed_values[k]->first){
					// Search constraint value at that time
					bool tested = false;
					for(int j = 0; j < (g_shared_vars_timed_values[k]->second->size() - 1); j++)
					{
						if((op_end_time > (*(g_shared_vars_timed_values[k]->second))[j]->second) &&
								(op_end_time <= (*(g_shared_vars_timed_values[k]->second))[j + 1]->second))
						{
							tested = true;
							if(((*(g_shared_vars_timed_values[k]->second))[j]->first != pp.pre) &&
									(pp.pre != -1) &&
									((*(g_shared_vars_timed_values[k]->second))[j]->first != -1)
							  )
							{
								/* if((op->get_name().find("_end") != string::npos) && (curr.running_actions.size() != 1))
								{
									op_valid = false;
									break;
								} else */
								if(!use_hard_temporal_constraints)
								{
									op_valid = false;
									break;
								}
								if(is_temporal) {
									if ((op->get_name().find("_start") != string::npos) && (curr.running_actions.size() != 0)){
										op_valid = false;
										break;
									} else if (op->get_name().find("_end") != string::npos){
										op_valid = false;
										break;
									}
								}else {
									op_valid = false;
									break;
								}
							} else if(op_duration > (((*(g_shared_vars_timed_values[k]->second))[j + 1]->second) - (curr.get_g_current_time_value())))
							{
								/* if((op->get_name().find("_end") != string::npos) && (curr.running_actions.size() != 1))
								{
									op_valid = false;
									break;
								} else */
								if(!use_hard_temporal_constraints)
								{
									op_valid = false;
									break;
								}
								if(is_temporal) {
									if ((op->get_name().find("_start") != string::npos) && (curr.running_actions.size() != 0)){
										op_valid = false;
										break;
									} else if (op->get_name().find("_end") != string::npos){
										op_valid = false;
										break;
									}
								}
							}
						}
					}

					if(!tested)
					{
						if(((*(g_shared_vars_timed_values[k]->second))[g_shared_vars_timed_values[k]->second->size() - 1]->first != pp.pre) &&
								(pp.pre != -1) &&
								((*(g_shared_vars_timed_values[k]->second))[g_shared_vars_timed_values[k]->second->size() - 1]->first != -1)
						  )
						{
							op_valid = false;
							break;
						}
					}
				}
			}
		}


		if (!op_valid){
			it = ops.erase(it);

		}else
			it++;
	}

	return;
}

void check_var_locks_validity(
		const State &curr, vector<const Operator *> &ops) {
	vector<const Operator *>::iterator it = ops.begin();
	for(; it != ops.end();) {
		const Operator * op = *it;

		// Get action duration, temporal duration, not snap.
		// start --- end
		float op_duration = 0;
		if(op->get_name().find("_start") != string::npos)
		{
			vector<PrePost>::const_iterator it_pp = op->get_pre_post().begin();
			for(; it_pp != op->get_pre_post().end(); ++it_pp) {
				PrePost pp = *it_pp;
				if(g_variable_name[pp.var] == total_time_var)
				{
					if(pp.have_runtime_cost_effect)
					{
						op_duration = curr.get_g_current_time_value() + curr.calculate_runtime_efect<float>(pp.runtime_cost_effect);
					} else{
						op_duration = curr.get_g_current_time_value() + pp.f_cost;
					}

					break;
				}
			}
		}


		bool op_valid = true;
		vector<PrePost>::const_iterator it_pp = op->get_pre_post().begin();
		for(; it_pp != op->get_pre_post().end(); ++it_pp) {
			PrePost pp = *it_pp;
			for(int k = 0; k < curr.blocked_vars.size(); k++)
			{
				if(curr.blocked_vars[k].var == pp.var)
				{
					if(curr.blocked_vars[k].block_type == -7)
					{
						// check if the block was set for this end action
						if((curr.blocked_vars[k].non_temporal_action_name == op->get_non_temporal_action_name()))
							continue;

						// If the block is over all and the value is different, the action is not valid
						// the time the variable is unblock is not relevant in this case
						op_valid = false;
						break;

					} else{
						// check if the block was set for this end action
						if((curr.blocked_vars[k].non_temporal_action_name == op->get_non_temporal_action_name()))
							continue;
						// If the block is at the end of the action, then the time at which the variable is freed has to be considered
						if(curr.blocked_vars[k].time_freed > op_duration){
							op_valid = false;
							break;
						}
					}
				}
			}

			if(!op_valid)
				break;
		}

		if (!op_valid){
			it = ops.erase(it);

		}else
			it++;
	}
}

void check_temporal_soundness_validity(
		const State &curr, vector<const Operator *> &ops) {
	// We have to assure that we do not apply actions that will happen later in time than others already running
	vector<const Operator *>::iterator it = ops.begin();
	for(; it != ops.end();) {
		const Operator * op = *it;
		bool op_valid = true;
		if(op->get_name().find("_end") != string::npos)
		{
			// Check that the action ending is the one that must happen first
			const runn_action* min_action_ending = NULL;
			float min_time = -1;
			vector<runn_action>::const_iterator it_ra_const = curr.running_actions.begin();
			for(; (it_ra_const != curr.running_actions.end()); it_ra_const++)
			{
				if((min_time == -1) || (min_time > (*it_ra_const).time_end))
				{
					min_time = (*it_ra_const).time_end;
					min_action_ending = &(*it_ra_const);
				}
			}

			if((min_action_ending != NULL) && (min_action_ending->non_temporal_action_name != op->get_non_temporal_action_name()))
			{
				op_valid = false;
			}
		}

		if (!op_valid){
			it = ops.erase(it);

		}else
			it++;
	}
}

void SuccessorGeneratorSwitch::_dump(string indent) {
    cout << indent << "switch on " << g_variable_name[switch_var] << endl;
    cout << indent << "immediately:" << endl;
    immediate_ops->_dump(indent + "  ");
    for(int i = 0; i < g_variable_domain[switch_var]; i++) {
	cout << indent << "case " << i << ":" << endl;
	generator_for_value[i]->_dump(indent + "  ");
    }
    cout << indent << "always:" << endl;
    default_generator->_dump(indent + "  ");
}

void SuccessorGeneratorGenerate::generate_applicable_ops(const State &,
							 vector<const Operator *> &ops) {
    ops.insert(ops.end(), op.begin(), op.end());
}

SuccessorGeneratorGenerate::SuccessorGeneratorGenerate(istream &in) {
    int count;
    in >> count;
    for(int i = 0; i < count; i++) {
	int op_index;
	in >> op_index;
	op.push_back(&g_operators[op_index]);
    }
}

void SuccessorGeneratorGenerate::_dump(string indent) {
    for(int i = 0; i < op.size(); i++) {
	cout << indent;
	op[i]->dump();
    }
}

SuccessorGenerator *read_successor_generator(istream &in) {
    string type;
    in >> type;
    if(type == "switch") {
	return new SuccessorGeneratorSwitch(in);
    } else if(type == "check") {
	return new SuccessorGeneratorGenerate(in);
    }
    cout << "Illegal successor generator statement!" << endl;
    cout << "Expected 'switch' or 'check', got '" << type << "'." << endl;
    exit(1);
}

