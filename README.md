
## --- If you want to use MA-LAMA, please refer to the pddl2-SAS-translate repository ---

This project composes the phase THREE of the MA-LAMA planner, it is only meant to be downloaded separately for developement purposes.
More precisely, this module comprises the temporally contrained agent local searches, it is launched one time for each agent in all Search Phases.

To launch, it takes as an input the output.sas file(s) from the preprocess module and the all.groups from the translate module:

./search (arguments) <output_preproagent[n_agnet]>

arguments:
  - w: use wa_star search.
  - l: use landmarks heuristic.
  - f: use ff heuristic.
  - F: use preferred operators.
  - i: launch iterative searches.
  - s: read initial state from the <end.state> file in the root directory.
  - c: read runtime temporal constrainst from <current_constraints> file in the root directory.
  - h: use hard temporal contraints (this makes the search find more solutions sacrificing makespan optimization).

Generates in the root directory:
  - end.state: contains the state of the shared variables for the most optimized found solution.
  - current_constraints: contains the temporal constraints that the most optimized found solution sets to the shared variables.
  - agnet_[n_agnet].[n_found_solution]: one file for each solution foiund, written in the snap task paradigm.

    output_preproagent[n_agnet]: one for each agent, contains the processed metric, variables, shared variables, initial state, goals, operators, and causal graph.

