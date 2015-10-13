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

#include <cmath>
#include <iostream>

#include "string_registry.h"
#include "support.h"

using namespace std;

// Fluent

Fluent::Fluent(int predicate, const vector<int> &args, int value) : Fluent(predicate, args, value, 0.f) {}

// maximum probability for a Fluent is 1.0
Fluent::Fluent(int predicate, const vector<int> &args, int value, float prob) : predicate_(predicate), args_(args), value_(value), prob_(prob > 1.f ? 1.f : prob) {
  // compute hash of predicate, value, and args
  size_t result = 0;
  HashCombine(predicate, &result);
  HashCombine(value, &result);
  for (const int arg : args) {
      HashCombine(arg, &result);
  }
  hash_ex_prob_ = result;

  // compute hash of predicate, value, args, and prob
  HashCombine(this->RoundProb(prob), &result);
  hash_ = result;
};

string Fluent::GetString() const {
  ostringstream ss;
  ss << "Fluent{" << StringRegistry::Get()->GetString(predicate_) << ", ";
  ss << "args:<" << Stringer(args_, ", ") << ">, value:" << value_ << ", prob: " << prob_ << "}";
  return ss.str();
}

int Fluent::GetPredicate() const {
  return predicate_;
}

const std::vector<int>& Fluent::GetArgs() const {
  return args_;
}

int Fluent::GetValue() const {
  return value_;
}

float Fluent::GetProb() const {
  return prob_;
}

float Fluent::RoundProb(float prob) const {
  //return prob;
  //return floor(floor(std::log2(prob) * 100.f) / 16.f);
  return floor(prob * 50.f);

}

size_t Fluent::Hash() const {
  return hash_;
}

size_t Fluent::HashExcludingProb() const {
  return hash_ex_prob_;
}

// State

State::State(const vector<Fluent> &fluents) {
  hash_ = 0;
  for (const Fluent &fluent : fluents) {
    Add(fluent);
  }
};

size_t State::Hash() const {
  return hash_;
};

string State::GetString() const {
  std::ostringstream ss;
  ss << "State{fluents:<" << Stringer(fluents_, ",\n") << ">}";
  return ss.str();
}

void State::Add(const Fluent &f) {
  FluentExcludingProbSet::const_iterator iter = fluents_ex_prob_.find(f);
  // fluent does not already exist
  if (iter == fluents_ex_prob_.end()) {
    fluents_.insert(f);
    hash_ ^= f.Hash();
    fluents_ex_prob_.insert(f);
  // fluent already exists and current probability is more strict
  } else if (iter->GetProb() < f.GetProb()) {
    Remove(*iter);
    Add(f);
  }
}

void State::Remove(const Fluent &f) {
  fluents_.erase(f);
  hash_ ^= f.Hash();
  fluents_ex_prob_.erase(f);
}

bool State::SatisfiedBy(const State *state) const {
  for (const Fluent &fluent : fluents_) {
    if (fluent.GetProb() > state->GetProb(fluent)) {
      return false;
    }
  }
  return true;
}

// counts the number of fluents in this state satisfied by the other state
int State::NumSatisfiedBy(const State *other) const {
  int num_satisfied = 0;
  for (const Fluent &fluent : fluents_) {
    if (fluent.GetProb() <= other->GetProb(fluent)) {
      num_satisfied += 1;
    }
  }
  return num_satisfied;
}

float State::GetProb(const Fluent &f) const {
  FluentExcludingProbSet::const_iterator iter = fluents_ex_prob_.find(f);
  return (iter == fluents_ex_prob_.end()) ? 0.f : iter->GetProb();
}

bool State::ApproximatelyEquals(const State *state) const {
  for (const Fluent &fluent : fluents_) {
    if (fluent.RoundProb(fluent.GetProb()) != fluent.RoundProb(state->GetProb(fluent))) {
      return false;
    }
  }
  return true;
}

// Action

Action::Action(int name, float cost, const std::vector<Fluent> &add_list,
               const std::vector<Fluent> &delete_list, const std::vector<int> &info) :
               name_(name), cost_(cost), add_list_(add_list), delete_list_(delete_list), info_(info) {}

std::string Action::GetPlanString() const {
  stringstream ss;
  ss << StringRegistry::Get()->GetString(name_);
  for (int i = 0; i < info_.size(); ++i) {
    ss << "_" << info_[i];
  }
  return ss.str();
}

int Action::GetName() const {
  return name_;
}

float Action::GetCost() const {
  return cost_;
}

const vector<int>& Action::GetInfo() const {
  return info_;
}

string Action::GetString() const {
  std::ostringstream ss;
  ss << "Action{" << StringRegistry::Get()->GetString(name_) << ",\n";
  ss << "       cost: " << cost_ << ",\n";
  ss << "       add_list:<" << Stringer(add_list_, ", ") << ">,\n";
  ss << "       delete_list:<" << Stringer(delete_list_, ", ") << ">}";
  return ss.str();
}


void Action::Successor(State *state) const {
  // delete fluents before adding new fluents in case add and delete list overlap
  for (const Fluent &f : delete_list_) {
    state->Remove(f);
  }
  for (const Fluent &f : add_list_) {
    state->Add(f);
  }
}

void Action::AddSuccessor(State *state) const {
  // only add fluents
  for (const Fluent &f : add_list_) {
    state->Add(f);
  }
}
