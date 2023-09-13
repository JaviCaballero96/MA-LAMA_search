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

#ifndef OPERATOR_H
#define OPERATOR_H

#include <cassert>
#include <iostream>
#include <string>
#include <vector>

#include "globals.h"
#include "state.h"

class Variable;

struct Prevail {
    int var;
    int prev;
    Prevail(std::istream &in);
    Prevail(int v, int p) : var(v), prev(p) {}

    bool is_applicable(const State &state) const {
	assert(var >= 0 && var < g_variable_name.size());
	assert(prev >= 0 && prev < g_variable_domain[var]);
	return state[var] == prev;
    }

    void dump() const;
};

struct PrePost {
    int var;
    int pre, post;
    float f_cost;
    bool have_runtime_cost_effect;
    string runtime_cost_effect;
    bool is_conditional_effect;
    std::vector<Prevail> cond;
    PrePost() {} // Needed for axiom file-reading constructor, unfortunately.
    PrePost(std::istream &in);
    PrePost(int v, int pr, int po, float f_co, const std::vector<Prevail> &co)
	: var(v), pre(pr), post(po), f_cost(f_co), cond(co) {}

    bool is_applicable(const State &state) const {
	assert(var >= 0 && var < g_variable_name.size());
	assert(pre == -1 || pre == -2 || pre == -3 || pre == -4  || pre == -5 || pre == -6 ||
			(pre >= 0 && pre < g_variable_domain[var]));
	return pre == -1 || state[var] == pre;
    }

    bool does_fire(const State &state) const {
	for(int i = 0; i < cond.size(); i++)
	    if(!cond[i].is_applicable(state))
		return false;
	return true;
    }

    void dump() const;
};

class Operator {
    bool is_an_axiom;
    std::vector<Prevail> prevail;      // var, val
    std::vector<PrePost> pre_post;     // var, old-val, new-val, effect conditions
    std::string name;
    float cost;
    bool have_runtime_cost;
    string runtime_cost;
public:
    Operator(std::istream &in, bool is_axiom);
    void dump() const;
    std::string get_name() const {return name;}

    bool is_axiom() const {return is_an_axiom;}

    const std::vector<Prevail> &get_prevail() const {return prevail;}
    const std::vector<PrePost> &get_pre_post() const {return pre_post;}

    bool is_applicable(const State &state) const {
	for(int i = 0; i < prevail.size(); i++)
	    if(!prevail[i].is_applicable(state))
		return false;
	for(int i = 0; i < pre_post.size(); i++)
	    if(!pre_post[i].is_applicable(state))
		return false;
	return true;
    }
    bool get_have_runtime_cost() const {return have_runtime_cost;};
    string get_runtime_cost() const {return runtime_cost;};
    float get_cost() const {return cost;};

};

#endif
