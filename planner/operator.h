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

#ifndef OPERATOR_H
#define OPERATOR_H

#include <memory>
#include <vector>

#include <cassert>
#include <cmath>

#include "support.h"

class Environment;

class Operator {
 public:
  Operator(int name);

  Operator(int name, float prob, float obs);

  Operator(int name, float prob, float obs, bool log_cost);

  Operator(int name, float prob, float obs, float base_cost, float cost_multiplier, bool log_cost);

  virtual ~Operator() {}

  std::string GetString() const;

  //TODO: inline? if called often
  int GetName() const;

  //TODO: inline?
  float GetProb() const;

  //TODO: inline?
  float GetObs() const;

  //TODO: inline?
  float GetBaseCost() const;

  virtual void ApplicableActions(const State &state, const Environment &env, std::vector<std::unique_ptr<Action>> *actions) const = 0;

 protected:
  float Cost(float p) const {
    assert (p > 0.f);
    return base_cost_ + cost_multiplier_ * (log_cost_ ? -std::log2(p) : (1.f / p));
  }

  int name_;
  float prob_; // actuator accuracy
  float obs_; // sensor accuracy
  float base_cost_; // base action cost; 1 by default
  float cost_multiplier_; // multiplier for determinized part of cost; 1 by default
  float log_cost_; // calculate costs as -log2(p), otherwise 1 / p; false by default
};

inline std::ostream& operator<<(std::ostream &os, const Operator &op) {
  os << op.GetString();
  return os;
}

#endif  // OPERATOR_H
