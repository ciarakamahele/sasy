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

#include <sstream>

#include "operator.h"
#include "string_registry.h"

using namespace std;

// Operator

Operator::Operator(int name) : Operator(name, 1.f, 1.f, 1.f, 1.f, true) {}

Operator::Operator(int name, float prob, float obs) : Operator(name, prob, obs, 1.f, 1.f, false) {}

Operator::Operator(int name, float prob, float obs, bool log_cost) : Operator(name, prob, obs, 1.f, 1.f, log_cost) {}

Operator::Operator(int name, float prob, float obs, float base_cost, float cost_multiplier, bool log_cost) : name_(name), prob_(prob), obs_(obs), base_cost_(base_cost), cost_multiplier_(cost_multiplier), log_cost_(log_cost) {}

int Operator::GetName() const {
  return name_;
}

float Operator::GetProb() const {
  return prob_;
}

float Operator::GetObs() const {
  return obs_;
}

float Operator::GetBaseCost() const {
  return base_cost_;
}

string Operator::GetString() const {
  std::ostringstream ss;
  ss << "Operator{" << StringRegistry::Get()->GetString(name_) << ", ";
  ss << "prob: " << prob_ << ", ";
  ss << "obs: " << obs_;
  return ss.str();
}
