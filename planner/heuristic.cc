/*
 * Copyright 2015 Ciara Kamahele-Sanfratello
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <cassert>
#include <iostream>
#include <memory>

#include "heuristic.h"
#include "uc_search.h"

using namespace std;

// no heuristic (always 0)
float HZero::Cost(const State &initial_state, const State &goal_state, const vector<const Operator*> &operators, const Environment& env) const {
  return 0.f;
}

// computationally cheap, underestimating heuristic
float HMax::Cost(const State &initial_state, const State &goal_state, const vector<const Operator*> &operators, const Environment& env) const {
  unique_ptr<State> new_state(new State(initial_state));
  if (goal_state.SatisfiedBy(new_state.get())) {
    return 0.f;
  }

  float base_cost = -1.f;
  for (const Operator* o : operators) {
    if (base_cost < 0.f) {
      base_cost = o->GetBaseCost();
    } else if (o->GetBaseCost() < base_cost) {
      base_cost = o->GetBaseCost();
    }
  }

  int depth = 0;

  do {
    vector<unique_ptr<Action>> actions;
    for (const Operator* o : operators) {
      o->ApplicableActions(*(new_state.get()), env, &actions); 
    }
    for (unique_ptr<Action> &a : actions) {
      a->AddSuccessor(new_state.get());
    }

    depth++;
  } while (!goal_state.SatisfiedBy(new_state.get()));

  return depth * base_cost;
}

// computationally cheap, usually overestimating heuristic
float HHSP::Cost(const State &initial_state, const State &goal_state, const vector<const Operator*> &operators, const Environment& env) const {
  unique_ptr<State> new_state(new State(initial_state));
  if (goal_state.SatisfiedBy(new_state.get())) {
    return 0.f;
  }

  float base_cost = -1.f;
  for (const Operator* o : operators) {
    if (base_cost < 0.f) {
      base_cost = o->GetBaseCost();
    } else if (o->GetBaseCost() < base_cost) {
      base_cost = o->GetBaseCost();
    }
  }

  float cost = 0.f;
  int prev_satisfied = goal_state.NumSatisfiedBy(&initial_state);
  int depth = 0;

  do {
    vector<unique_ptr<Action>> actions;
    for (const Operator* o : operators) {
      o->ApplicableActions(*(new_state.get()), env, &actions); 
    }
    for (unique_ptr<Action> &a : actions) {
      a->AddSuccessor(new_state.get());
    }

    depth++;
    int num_satisfied = goal_state.NumSatisfiedBy(new_state.get());
    cost += (num_satisfied - prev_satisfied) * base_cost * depth;
    prev_satisfied = num_satisfied;
  } while (!goal_state.SatisfiedBy(new_state.get()));

  return cost;
}

// computationally expensive, underestimating heuristic
float HFF::Cost(const State &initial_state, const State &goal_state, const vector<const Operator*> &operators, const Environment& env) const {
  unique_ptr<State> new_state(new State(initial_state));
  float cost = 0.f;
  unique_ptr<State> goal_state_ptr(new State(goal_state));
  const vector<const State*> goal_set = {goal_state_ptr.get()};
  UCSearch(move(new_state), goal_set, operators, HMax(), env, nullptr, nullptr, &cost, true, false, 0.f, 0.f);
  return cost;
}
