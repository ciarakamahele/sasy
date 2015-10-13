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

#ifndef HEURISTIC_H
#define HEURISTIC_H

#include "support.h"
#include "operator.h"

class Heuristic {
 public:
  virtual ~Heuristic() {}
  virtual float Cost(const State &initial_state, const State &goal_state, const std::vector<const Operator*> &operators, const Environment& env) const = 0;
};

// no heuristic (always 0)
class HZero : public Heuristic {
 public:
  float Cost(const State &initial_state, const State &goal_state, const std::vector<const Operator*> &operators, const Environment& env) const override;
  
};

// computationally cheap, underestimating heuristic
class HMax : public Heuristic {
 public:
  float Cost(const State &initial_state, const State &goal_state, const std::vector<const Operator*> &operators, const Environment& env) const override;
};

// computationally cheap, usually overestimating heuristic
class HHSP : public Heuristic {
 public:
  float Cost(const State &initial_state, const State &goal_state, const std::vector<const Operator*> &operators, const Environment& env) const override;
};

// computationally expensive, underestimating heuristic
class HFF : public Heuristic {
 public:
  float Cost(const State &initial_state, const State &goal_state, const std::vector<const Operator*> &operators, const Environment& env) const override;
};

#endif  // HEURISTIC_H
