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

#include <algorithm>
#include <chrono>
#include <iostream>
#include <queue>
#include <sstream>
#include <unordered_map>

#include "string_registry.h"
#include "uc_search.h"

using namespace std;

SearchNode::SearchNode(std::unique_ptr<const State> state, const SearchNode *parent, std::unique_ptr<const Action> action, float heuristic_cost, int count, float weight) : state_(move(state)), parent_(parent), action_(move(action)), heuristic_cost_(heuristic_cost), hmax_(-1.f), count_(count), weight_(weight) {
    action_cost_ = (action_ == nullptr) ? 0.f : action_->GetCost();
    parent_cost_ = (parent_ == nullptr) ?  0.f : parent->GetParentActionCost();
}

// trace back through parent pointers to construct path taken
void SearchNode::GetPath(vector<PathPair> *path) const {
  // takes a copy of state and action because they are created inside of Search
  // and otherwise would not exist anymore
  path->emplace_back(*state_, *action_);
  if (parent_ != nullptr) {
    parent_->GetPath(path);
  }
}

float SearchNode::GetWeight() const {
  return weight_;
}

float SearchNode::GetCost() const {
  return parent_cost_ + action_cost_ + heuristic_cost_;
}

float SearchNode::GetParentActionCost() const {
  return parent_cost_ + action_cost_;
}

float SearchNode::GetHeuristicCost() const {
  return heuristic_cost_;
}

float SearchNode::GetWeightedCost() const {
    return (parent_cost_ + action_cost_) * weight_ + heuristic_cost_ * (1.f - weight_);
}


float SearchNode::GetHMax(vector<const State*> &goal_set, const vector<const Operator*> &operators, const Environment& env) {
  if (hmax_ == -1.f) {
    hmax_ = HeuristicCost(HMax(), *(this->GetState()), goal_set, operators, env);
  }
  return hmax_;
}

// trace back through parent pointers to find costs at each step
void SearchNode::GetCosts(vector<float> *costs) const {
  costs->push_back(GetCost());
  if (parent_ != nullptr) {
    parent_->GetCosts(costs);
  }
}

bool SearchNode::InPath(const State &state) const {
  if (state == *state_) {
    return true;
  }
  if (parent_ != nullptr) {
    return parent_->InPath(state);
  }
  return false;
}

string SearchNode::GetString() const {
  ostringstream ss;
  ss << "SearchNode{state:";
  ss << *state_;
  ss << ",\naction:";
  ss << ((action_ == nullptr) ? "nullptr" : action_->GetString());
  ss << ", cost:";
  ss << GetCost();
  ss << ", parent/action cost:";
  ss << GetParentActionCost();
  ss << ", heuristic cost:";
  ss << GetHeuristicCost();
  ss << ", count:";
  ss << GetCount();
  ss << "}";
  return ss.str();
}

const State* SearchNode::GetState() const {
  return state_.get();
}

int SearchNode::GetCount() const {
  return count_;
}

void SearchNode::Actions(const vector<const Operator*> &operators, const Environment& env, vector<unique_ptr<Action>> *actions) const {
  for (const Operator* o : operators) {
    o->ApplicableActions(*state_, env, actions); 
  }
}

unique_ptr<State> SearchNode::CreateSuccessor(const Action &action, bool add_only) const {
  unique_ptr<State> new_state(new State(*state_));
  if (add_only) {
    action.AddSuccessor(new_state.get());
  } else {
    action.Successor(new_state.get());
  }
  return new_state;
}

float HeuristicCost(const Heuristic &h, const State &initial_state, const vector<const State*> &goal_set, const vector<const Operator*> &operators, const Environment& env) {
  float cost = h.Cost(initial_state, *goal_set[0], operators, env);
  for (int i = 1; i < goal_set.size(); i++) {
    float cmp_cost = h.Cost(initial_state, *goal_set[i], operators, env);
    if (cmp_cost < cost) {
      cost = cmp_cost;
    }
  }
  return cost;
}

bool Search(unique_ptr<const State> start_state, const vector<const State*> &goal_set, const vector<const Operator*> &operators, const Environment &env, const Heuristic &h, bool verbose) {
  return Search(move(start_state), goal_set, operators, env, h, verbose, 0.f, 0.f);
}

bool Search(unique_ptr<const State> start_state, const vector<const State*> &goal_set, const vector<const Operator*> &operators, const Environment &env, const Heuristic &h, bool verbose, float epsilon, float weight) {
  vector<SearchNode::PathPair> path;
  vector<float> costs;

  State start_state_copy = *start_state;
    
  const chrono::steady_clock::time_point time_start = chrono::steady_clock::now();
  bool search_result = UCSearch(move(start_state), goal_set, operators, h, env, &path, &costs, nullptr, false, verbose, epsilon, weight);
  const chrono::steady_clock::time_point time_end = chrono::steady_clock::now();
  int ms = chrono::duration_cast<chrono::milliseconds>(time_end - time_start).count();

  for (int i = path.size() - 1; i >= 0; --i) {
    cout << path[i].action.GetString() << endl << endl;
    path[i].action.Successor(&start_state_copy);
    cout << start_state_copy.GetString() << endl << endl;
  }

  for (int i = path.size() - 1; i >= 0; --i) {
    cout << path[i].action.GetPlanString() << endl;
    path[i].action.Successor(&start_state_copy);
  }

  cout << "search finished " << ((search_result) ? "successfully" : "unsuccessfully") << " after " << ms / 1000.f << " seconds" << endl;

  return search_result;
}

bool UCSearch(unique_ptr<const State> initial_state, const vector<const State*> &goal_set, const vector<const Operator*> &operators, const Heuristic &h, const Environment &env, vector<SearchNode::PathPair> *path, vector<float> *costs, float *cost, bool add_only, bool verbose, float epsilon, float weight) {
  const int kNoAction = StringRegistry::Get()->GetInt("no_action");

  // search nodes that have been created and put in the agenda
  // (consists of unique ptrs to SearchNodes holding unique ptrs to States)
  vector<unique_ptr<SearchNode>> visited;

  // search nodes waiting to be expanded
  // (consists of SearchNode pointers into visited list)
  priority_queue<const SearchNode*, vector<const SearchNode*>, CompareSearchNodePtr> agenda;

  // states that have been expanded and their children put in the agenda
  // (consists of State pointers into visited list)
  StateSet expanded;

  // track the order and number of nodes that have been pushed onto the agenda 
  int count_visited = 0;
  int count_expanded = 0;
  int count_prev_expanded = 0;
  //bool checked = false;

  float initial_heuristic_cost = HeuristicCost(h, *initial_state, goal_set, operators, env);
  visited.emplace_back(new SearchNode(move(initial_state), nullptr, std::unique_ptr<const Action>(new Action(kNoAction, 0.f, {}, {}, {})), initial_heuristic_cost, ++count_visited, weight));
  agenda.push(visited.back().get());

  while (!agenda.empty()) {
    const SearchNode *node = agenda.top();
    agenda.pop();

    if (epsilon > 0.f) {
      float base_cost = node->GetParentActionCost();
      float min_cost_to_goal = HeuristicCost(HMax(), *node->GetState(), goal_set, operators, env);

      vector<const SearchNode*> considered_nodes;

      // find node with lowest estimated cost to goal, with cost so far
      // within epsilon from minimum cost so far
      while (!agenda.empty() && agenda.top()->GetParentActionCost() <= (base_cost + epsilon)) {
        //cout << "cost is: " << agenda.top()->GetCost() << ", base cost is: " << base_cost << endl;

        float cost_to_goal = HeuristicCost(HMax(), *(agenda.top()->GetState()), goal_set, operators, env);
        if (cost_to_goal < min_cost_to_goal) {
          considered_nodes.push_back(node);
          node = agenda.top();
        } else {
          considered_nodes.push_back(agenda.top());
        }
        agenda.pop();
      }
      // replace nodes in agenda
      for (const SearchNode *node : considered_nodes) {
        agenda.push(node);
      }
    }

    if (verbose) {cout << "\n" << endl;}

    // check if state was previously expanded, otherwise mark as expanded
    StateSet::const_iterator iter = expanded.find(node->GetState());
    if (iter != expanded.end()) {
        count_prev_expanded++;
        if (verbose) {cout << "previously expanded: " << *node << endl;}
    // expand state
    } else {
      if (verbose) {cout << "expanding node " << *node << endl;}
      expanded.insert(node->GetState());
      count_expanded++;

      // check if state satisfies goal

      for (const State* goal_state : goal_set) {
        if (goal_state->SatisfiedBy(node->GetState())) {
          // done with search!
          // write out path and cost
          if (add_only) {
            *cost = node->GetCost();
          } else {
            cout << "found goal state! " << count_expanded << " nodes expanded, " << count_visited << " nodes visited, " << count_prev_expanded << " nodes skipped, solution cost: " << node->GetCost() << endl;
            cout << "satisfies goal state " << *goal_state << endl;
            node->GetPath(path);
            node->GetCosts(costs);
          }
          return true;
        }
      }

      // get applicable actions
      vector<unique_ptr<Action>> actions;
      node->Actions(operators, env, &actions);

      // add any children to agenda
      if (actions.empty()) {
        if (verbose) {cout << "no applicable actions" << endl;}
      } else {
        if (verbose) {cout << "predicted cost: " << node->GetCost() << ", " << actions.size() << " applicable actions" << endl << endl;}
        for (unique_ptr<Action> &a : actions) {
          unique_ptr<State> new_state = node->CreateSuccessor(*a, add_only);

          float heuristic_cost = HeuristicCost(h, *new_state, goal_set, operators, env);
          visited.emplace_back(new SearchNode(move(new_state), node, move(a), heuristic_cost, ++count_visited, weight));
          agenda.push(visited.back().get());

          if (verbose) {cout << "queued child " << *(visited.back()) << endl << endl;}
        }
      }
      if (!verbose && (count_expanded % 100) == 0) {cout << count_expanded << " nodes expanded, " << count_visited << " nodes visited, " << count_prev_expanded << " nodes skipped\nexpanding node #" << node->GetCount() << " cost:" << node->GetCost() << " " << *(node->GetState()) << endl << endl;}
    }
  }

  // search failed
  return false;
}
