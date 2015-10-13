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

#ifndef GRIPPER_OPERATOR_H
#define GRIPPER_OPERATOR_H

#include <operator.h>
#include "gripper/environment.h"
#include "string_registry.h"
#include "support.h"

namespace gripper {

class Operator : public ::Operator {
 public:
  Operator(int name) : ::Operator(name) {}

  void ApplicableActions(const State &state, const ::Environment &env, std::vector<std::unique_ptr<Action>> *actions) const override {
    const Environment &gripper_env = static_cast<const Environment&>(env);
    ApplicableActions(state, gripper_env, actions);
  }

  virtual void ApplicableActions(const State &state, const Environment &env, std::vector<std::unique_ptr<Action>> *actions) const = 0;

};

class MoveOperator : public Operator {
 public:
  MoveOperator(int name);

  void ApplicableActions(const State &state, const Environment &env, std::vector<std::unique_ptr<Action>> *actions) const override;
};

class PickOperator : public Operator {
 public:
  PickOperator(int name);

  void ApplicableActions(const State &state, const Environment &env, std::vector<std::unique_ptr<Action>> *actions) const override;
};

class PlaceOperator : public Operator {
 public:
  PlaceOperator(int name);

  void ApplicableActions(const State &state, const Environment &env, std::vector<std::unique_ptr<Action>> *actions) const override;
};

} // namespace gripper

#endif  // GRIPPER_OPERATOR_H
