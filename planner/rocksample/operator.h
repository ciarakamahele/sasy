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

#ifndef ROCKSAMPLE_OPERATOR_H
#define ROCKSAMPLE_OPERATOR_H

#include <operator.h>
#include "rocksample/environment.h"
#include "string_registry.h"
#include "support.h"

namespace rocksample {

class Operator : public ::Operator {
 public:
  Operator(int name, float prob, float obs) : ::Operator(name, prob, obs) {}

  Operator(int name, float prob, float obs, bool log_cost) : ::Operator(name, prob, obs, log_cost) {}

  Operator(int name, float prob, float obs, float base_cost, float cost_multiplier, bool log_cost) : ::Operator(name, prob, obs, base_cost, cost_multiplier, log_cost) {}

  void ApplicableActions(const State &state, const ::Environment &env, std::vector<std::unique_ptr<Action>> *actions) const override {
    const Environment &rocksample_env = static_cast<const Environment&>(env);
    ApplicableActions(state, rocksample_env, actions);
  }

  virtual void ApplicableActions(const State &state, const Environment &env, std::vector<std::unique_ptr<Action>> *actions) const = 0;

 protected:
  void Terminate(float terminate_p, float steps, std::vector<std::unique_ptr<Action>> *actions) const {
    if (terminate_p > 0.f) {
      const int kTerminate = StringRegistry::Get()->GetInt("terminate");
      const int kTerminated = StringRegistry::Get()->GetInt("terminated");
      const int kSteps = StringRegistry::Get()->GetInt("steps");

      const std::vector<Fluent> preconditions{};
      const std::vector<Fluent> add_list{Fluent(kTerminated, {}, 0, 1.f),
                                         Fluent(kSteps, {}, 0, steps + 1)};
      const std::vector<Fluent> delete_list{Fluent(kSteps, {}, 0, steps)};
      actions->emplace_back(new Action(kTerminate, Cost(terminate_p), add_list, delete_list, {}));
    }
  }
};

class NorthOperator : public Operator {
 public:
  NorthOperator(int name, float prob, float cost_multiplier, bool log_cost);

  void ApplicableActions(const State &state, const Environment &env, std::vector<std::unique_ptr<Action>> *actions) const override;
};

class SouthOperator : public Operator {
 public:
  SouthOperator(int name, float prob, float cost_multiplier, bool log_cost);

  void ApplicableActions(const State &state, const Environment &env, std::vector<std::unique_ptr<Action>> *actions) const override;
};

class EastOperator : public Operator {
 public:
  EastOperator(int name, float prob, float cost_multiplier, bool log_cost);

  void ApplicableActions(const State &state, const Environment &env, std::vector<std::unique_ptr<Action>> *actions) const override;
};

class WestOperator : public Operator {
 public:
  WestOperator(int name, float prob, float cost_multiplier, bool log_cost);

  void ApplicableActions(const State &state, const Environment &env, std::vector<std::unique_ptr<Action>> *actions) const override;
};

class SampleOperator : public Operator {
 public:
  SampleOperator(int name, float prob, float cost_multiplier, bool log_cost);

  void ApplicableActions(const State &state, const Environment &env, std::vector<std::unique_ptr<Action>> *actions) const override;
};

class CheckOperator : public Operator {
 public:
  CheckOperator(int name, float prob, float cost_multiplier, bool log_cost);

  void ApplicableActions(const State &state, const Environment &env, std::vector<std::unique_ptr<Action>> *actions) const override;
};

class NoOperator : public Operator {
 public:
  NoOperator(int name, float prob, float cost_multiplier, bool log_cost);

  void ApplicableActions(const State &state, const Environment &env, std::vector<std::unique_ptr<Action>> *actions) const override;
};

} // namespace rocksample

#endif  // ROCKSAMPLE_OPERATOR_H
