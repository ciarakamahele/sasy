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

#ifndef KITCHEN_OPERATOR_H
#define KITCHEN_OPERATOR_H

#include "kitchen/environment.h"
#include <operator.h>
#include "support.h"

namespace kitchen {

class Operator : public ::Operator {
 public:
  Operator(int name, float prob, float obs) : ::Operator(name, prob, obs) {}

  Operator(int name, float prob, float obs, bool log_cost) : ::Operator(name, prob, obs, log_cost) {}

  Operator(int name, float prob, float obs, float base_cost, float cost_multiplier, bool log_cost) : ::Operator(name, prob, obs, base_cost, cost_multiplier, log_cost) {}

  void ApplicableActions(const State &state, const ::Environment &env, std::vector<std::unique_ptr<Action>> *actions) const override {
    const Environment &kitchen_env = static_cast<const Environment&>(env);
    ApplicableActions(state, kitchen_env, actions);
  }

  virtual void ApplicableActions(const State &state, const Environment &env, std::vector<std::unique_ptr<Action>> *actions) const = 0;
};

class MoveOperator : public Operator {
 public:
  MoveOperator(int name, float prob, float base_cost, bool log_cost);

  void ApplicableActions(const State &state, const Environment &env, std::vector<std::unique_ptr<Action>> *actions) const override;
};

class PickOperator : public Operator {
 public:
  PickOperator(int name, float prob, float base_cost, bool log_cost);

  void ApplicableActions(const State &state, const Environment &env, std::vector<std::unique_ptr<Action>> *actions) const override;
};

class PlaceOperator : public Operator {
 public:
  PlaceOperator(int name, float prob, float base_cost, bool log_cost);

  void ApplicableActions(const State &state, const Environment &env, std::vector<std::unique_ptr<Action>> *actions) const override;
};

class CookOperator : public Operator {
 public:
  CookOperator(int name, float prob, float obs);

  void ApplicableActions(const State &state, const Environment &env, std::vector<std::unique_ptr<Action>> *actions) const override;
};

class LookRobotOperator : public Operator {
 public:
  LookRobotOperator(int name, float prob, float base_cost, float cost_multiplier, bool log_cost);

  void ApplicableActions(const State &state, const Environment &env, std::vector<std::unique_ptr<Action>> *actions) const override;
};

class LookHandOperator : public Operator {
 public:
  LookHandOperator(int name, float prob, float base_cost, float cost_multiplier, bool log_cost);

  void ApplicableActions(const State &state, const Environment &env, std::vector<std::unique_ptr<Action>> *actions) const override;
};

class LookObjOperator : public Operator {
 public:
  LookObjOperator(int name, float prob, float base_cost, float cost_multiplier, bool log_cost);

  void ApplicableActions(const State &state, const Environment &env, std::vector<std::unique_ptr<Action>> *actions) const override;
};

} // namespace kitchen

#endif  // KITCHEN_OPERATOR_H
