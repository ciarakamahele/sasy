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

#ifndef UC_SEARCH_H
#define UC_SEARCH_H

#include <vector>

#include "heuristic.h"
#include "operator.h"
#include "support.h"

class SearchNode {
 public:
  //TODO: make state and action const unique ptrs
  // SearchNode takes ownership of state and action, but not of parent.
  // The lifetime of parent and action must exceed the lifetime of the SearchNode
  SearchNode(std::unique_ptr<const State> state, const SearchNode *parent, std::unique_ptr<const Action> action, float heuristic_cost, int count, float weight);

  struct PathPair {
    PathPair(const State &state, const Action &action) : state(state), action(action) {}
    const State state;
    const Action action;
  };

  void GetPath(std::vector<PathPair> *path) const;

  float GetWeight() const;

  float GetCost() const;

  float GetParentActionCost() const;

  float GetHeuristicCost() const;

  float GetHMax(std::vector<const State*> &goal_set, const std::vector<const Operator*> &operators, const Environment& env);

  float GetWeightedCost() const;

  void GetCosts(std::vector<float> *costs) const;
  
  bool InPath(const State &state) const;

  std::string GetString() const;

  const State* GetState() const;

  int GetCount() const;

  void Actions(const std::vector<const Operator*> &operators, const Environment& env, std::vector<std::unique_ptr<Action>> *actions) const;

  std::unique_ptr<State> CreateSuccessor(const Action &action, bool add_only) const;

 private:
  const std::unique_ptr<const State> state_;
  const SearchNode *parent_;
  const std::unique_ptr<const Action> action_;
  float parent_cost_;
  float action_cost_;
  float heuristic_cost_;
  float hmax_;
  int count_;
  float weight_;
};

inline std::ostream& operator<<(std::ostream &os, const SearchNode &search_node) {
  os << search_node.GetString();
  return os;
}

// min priority queue
struct CompareSearchNodePtr {
  bool operator()(const SearchNode *lhs, const SearchNode *rhs) {
    return lhs->GetWeightedCost() > rhs->GetWeightedCost();
  }
};

float HeuristicCost(const Heuristic &h, const State &initial_state, const std::vector<const State*> &goal_set, const std::vector<const Operator*> &operators, const Environment& env);


//TODO: change state and action to be const unique ptrs
bool Search(std::unique_ptr<const State> initial_state, const std::vector<const State*> &goal_set, const std::vector<const Operator*> &operators, const Environment &env, const Heuristic &h, bool verbose);

bool Search(std::unique_ptr<const State> initial_state, const std::vector<const State*> &goal_set, const std::vector<const Operator*> &operators, const Environment &env, const Heuristic &h, bool verbose, float epsilon, float weight);

//TODO: change state and action to be const unique ptrs
// this function will take ownership of initial_state
bool UCSearch(std::unique_ptr<const State> initial_state, const std::vector<const State*> &goal_set, const std::vector<const Operator*> &operators, const Heuristic &h, const Environment &env, std::vector<SearchNode::PathPair> *path, std::vector<float> *costs, float *cost, bool add_only, bool verbose, float epsilon, float weight);

#endif  // UC_SEARCH_H
