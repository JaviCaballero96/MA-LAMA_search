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

#include "globals.h"
#include "operator.h"

#include <iostream>
#include <sstream>
#include <string>
using namespace std;

bool is_temporal = false;

Prevail::Prevail(istream &in) {
    in >> var >> prev;
}

PrePost::PrePost(istream &in) {
	/* effects */
    int condCount;
    string s_aux = "";
    in >> condCount;
    for(int i = 0; i < condCount; i++)
        cond.push_back(Prevail(in));
    in >> var >> pre >> post;
    if ((pre == -2) || (pre == -3) || (pre == -4)  || (pre == -5)  || (pre == -6))
    {
    	in >> s_aux;
    	if (s_aux.find('(') == std::string::npos)
    	{
            istringstream buffer(s_aux);
            buffer >> f_cost;
            have_runtime_cost_effect = false;
            runtime_cost_effect = "";
    	}else
    	{
    		f_cost = 0;
    		have_runtime_cost_effect = true;
    		runtime_cost_effect = s_aux;
    	}
    }

}

Operator::Operator(istream &in, bool axiom) {
    is_an_axiom = axiom;
    if(!is_an_axiom) {
        check_magic(in, "begin_operator");
        in >> ws;
        getline(in, name);
        if((!is_temporal) && (name.find("_start") != string::npos) || (name.find("_end") != string::npos))
        	is_temporal = true;
        int count;
        in >> count;
        for(int i = 0; i < count; i++)
            prevail.push_back(Prevail(in));
        in >> count;
        for(int i = 0; i < count; i++)
            pre_post.push_back(PrePost(in));
        in >> count;
        for(int i = 0; i < count; i++)
            pre_block.push_back(PrePost(in));
	float op_cost;
	in >> op_cost;
	// Note: increase cost of all actions by 1 to deal with
	// 0-cost actions (needed for bounded search).

	//TODO float +1 no se pa que es esto
	// cost = op_cost;
	cost = op_cost + 1;

	string s_aux = "";
	in >> s_aux;
	if(s_aux == "runtime")
	{
		have_runtime_cost = true;
		in >> runtime_cost;
	}else
	{
		have_runtime_cost = false;
		runtime_cost = "";
		in >> s_aux;
	}


    check_magic(in, "end_operator");
    } else {
        name = "<axiom>";
	cost = 0;
        check_magic(in, "begin_rule");
        pre_post.push_back(PrePost(in));
        check_magic(in, "end_rule");
    }

    if(name.find("_start") != string::npos)
    {
    	non_temporal_name = name.substr(0, name.find("_start")) + name.substr(name.find("_start") + 6, name.length());
    }else {
    	non_temporal_name = name.substr(0, name.find("_end")) + name.substr(name.find("_end") + 4, name.length());
    }
}

void Prevail::dump() const {
    cout << g_variable_name[var] << ": " << prev;
}

void PrePost::dump() const {
    cout << g_variable_name[var] << ": " << pre << " => " << post;
    if(!cond.empty()) {
        cout << " if";
        for(int i = 0; i < cond.size(); i++) {
            cout << " ";
            cond[i].dump();
        }
    }
}

void Operator::dump() const {
    cout << name << ":";
    for(int i = 0; i < prevail.size(); i++) {
        cout << " [";
        prevail[i].dump();
        cout << "]";
    }
    for(int i = 0; i < pre_post.size(); i++) {
        cout << " [";
        pre_post[i].dump();
        cout << "]";
    }
    cout << endl;
}
