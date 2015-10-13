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
#include <cmath>
#include <iostream>
#include <sstream>

#include "operator.h"
#include "string_registry.h"

namespace rocksample {

using namespace std;

// NorthOperator

NorthOperator::NorthOperator(int name, float prob, float cost_multiplier, bool log_cost) : Operator(name, prob, 1.f, 0.f, cost_multiplier, log_cost) {}

void NorthOperator::ApplicableActions(const State &state, const Environment &env, vector<unique_ptr<Action>> *actions) const {
  const int kNorth = StringRegistry::Get()->GetInt("north");
  const int kY = StringRegistry::Get()->GetInt("y");
  const int kSteps = StringRegistry::Get()->GetInt("steps");
  const int kTerminated = StringRegistry::Get()->GetInt("terminated");

  if (state.GetProb(Fluent(kTerminated, {}, 0)) == 0.f) {
    // not already at the top of the map
    if (state.GetProb(Fluent(kY, {}, 0)) == 0.f) {
      int robot_y = -1;
      // find current robot y
      for (int y = 1; y < env.GetNumLocs(); y++) {
        if (state.GetProb(Fluent(kY, {}, y)) == 1.f) {
          robot_y = y;
          break;
        }
      }
      assert(robot_y >= 0);

      if (prob_ > 0.f) {
        const vector<Fluent> preconditions{};
        const vector<Fluent> add_list{Fluent(kY, {}, robot_y - 1, 1.f),
                                      Fluent(kSteps, {}, 0, state.GetProb(Fluent(kSteps, {}, 0)) + 1)};
        const vector<Fluent> delete_list{Fluent(kY, {}, robot_y, 1.f),
                                         Fluent(kSteps, {}, 0, state.GetProb(Fluent(kSteps, {}, 0)))};
        actions->emplace_back(new Action(kNorth, 10.f + Cost(prob_), add_list, delete_list, {}));
      }
      Terminate(1.f - prob_, state.GetProb(Fluent(kSteps, {}, 0)), actions);
    }
  }
}

// SouthOperator

SouthOperator::SouthOperator(int name, float prob, float cost_multiplier, bool log_cost) : Operator(name, prob, 1.f, 0.f, cost_multiplier, log_cost) {}

void SouthOperator::ApplicableActions(const State &state, const Environment &env, vector<unique_ptr<Action>> *actions) const {
  const int kSouth = StringRegistry::Get()->GetInt("south");
  const int kY = StringRegistry::Get()->GetInt("y");
  const int kSteps = StringRegistry::Get()->GetInt("steps");
  const int kTerminated = StringRegistry::Get()->GetInt("terminated");

  if (state.GetProb(Fluent(kTerminated, {}, 0)) == 0.f) {
    // not already at the bottom of the map
    if (state.GetProb(Fluent(kY, {}, env.GetNumLocs() - 1)) == 0.f) {
      int robot_y = -1;
      // find current robot y
      for (int y = 0; y < env.GetNumLocs() - 1; y++) {
        if (state.GetProb(Fluent(kY, {}, y)) == 1.f) {
          robot_y = y;
          break;
        }
      }
      assert(robot_y >= 0);

      if (prob_ > 0.f) {
        const vector<Fluent> preconditions{};
        const vector<Fluent> add_list{Fluent(kY, {}, robot_y + 1, 1.f),
                                      Fluent(kSteps, {}, 0, state.GetProb(Fluent(kSteps, {}, 0)) + 1)};
        const vector<Fluent> delete_list{Fluent(kY, {}, robot_y, 1.f),
                                         Fluent(kSteps, {}, 0, state.GetProb(Fluent(kSteps, {}, 0)))};
        actions->emplace_back(new Action(kSouth, 10.f + Cost(prob_), add_list, delete_list, {}));
      }
      Terminate(1.f - prob_, state.GetProb(Fluent(kSteps, {}, 0)), actions);
    }
  }
}

// EastOperator

EastOperator::EastOperator(int name, float prob, float cost_multiplier, bool log_cost) : Operator(name, prob, 1.f, 0.f, cost_multiplier, log_cost) {}

void EastOperator::ApplicableActions(const State &state, const Environment &env, vector<unique_ptr<Action>> *actions) const {
  const int kEast = StringRegistry::Get()->GetInt("east");
  const int kX = StringRegistry::Get()->GetInt("x");
  const int kSteps = StringRegistry::Get()->GetInt("steps");
  const int kTerminated = StringRegistry::Get()->GetInt("terminated");

  if (state.GetProb(Fluent(kTerminated, {}, 0)) == 0.f) {
    // not already one spot past the right of the map
    if (state.GetProb(Fluent(kX, {}, env.GetNumLocs())) == 0.f) {
      int robot_x = -1;
      // find current robot x
      for (int x = 0; x < env.GetNumLocs(); x++) {
        if (state.GetProb(Fluent(kX, {}, x)) == 1.f) {
          robot_x = x;
          break;
        }
      }
      assert(robot_x >= 0);

      if (prob_ > 0.f) {
        // moving off east edge of map
        if ((robot_x + 1) == env.GetNumLocs()) {
          const vector<Fluent> preconditions{};
          const vector<Fluent> add_list{Fluent(kX, {}, robot_x + 1, 1.f),
                                        Fluent(kSteps, {}, 0, state.GetProb(Fluent(kSteps, {}, 0)) + 1),
                                        Fluent(kTerminated, {}, 0, 1.f)};
          const vector<Fluent> delete_list{Fluent(kX, {}, robot_x, 1.f),
                                           Fluent(kSteps, {}, 0, state.GetProb(Fluent(kSteps, {}, 0))),
                                           Fluent(kTerminated, {}, 0, 0.f)};
          actions->emplace_back(new Action(kEast, Cost(prob_), add_list, delete_list, {}));
        } else {
          const vector<Fluent> preconditions{};
          const vector<Fluent> add_list{Fluent(kX, {}, robot_x + 1, 1.f),
                                        Fluent(kSteps, {}, 0, state.GetProb(Fluent(kSteps, {}, 0)) + 1)};
          const vector<Fluent> delete_list{Fluent(kX, {}, robot_x, 1.f),
                                           Fluent(kSteps, {}, 0, state.GetProb(Fluent(kSteps, {}, 0)))};
          actions->emplace_back(new Action(kEast, 10.f + Cost(prob_), add_list, delete_list, {}));

          Terminate(1.f - prob_, state.GetProb(Fluent(kSteps, {}, 0)), actions);
        }
      }
    }
  }
}

// WestOperator

WestOperator::WestOperator(int name, float prob, float cost_multiplier, bool log_cost) : Operator(name, prob, 1.f, 0.f, cost_multiplier, log_cost) {}

void WestOperator::ApplicableActions(const State &state, const Environment &env, vector<unique_ptr<Action>> *actions) const {
  const int kWest = StringRegistry::Get()->GetInt("west");
  const int kX = StringRegistry::Get()->GetInt("x");
  const int kSteps = StringRegistry::Get()->GetInt("steps");
  const int kTerminated = StringRegistry::Get()->GetInt("terminated");

  if (state.GetProb(Fluent(kTerminated, {}, 0)) == 0.f) {
    // not already at the left of the map
    if (state.GetProb(Fluent(kX, {}, 0)) == 0.f) {
      int robot_x = -1;
      // find current robot x
      for (int x = 1; x < env.GetNumLocs() + 1; x++) {
        if (state.GetProb(Fluent(kX, {}, x)) == 1.f) {
          robot_x = x;
          break;
        }
      }
      assert(robot_x >= 0);


      if (prob_ > 0.f) {
        const vector<Fluent> preconditions{};
        const vector<Fluent> add_list{Fluent(kX, {}, robot_x - 1, 1.f),
                                      Fluent(kSteps, {}, 0, state.GetProb(Fluent(kSteps, {}, 0)) + 1)};
        const vector<Fluent> delete_list{Fluent(kX, {}, robot_x, 1.f),
                                         Fluent(kSteps, {}, 0, state.GetProb(Fluent(kSteps, {}, 0)))};
        actions->emplace_back(new Action(kWest, 10.f + Cost(prob_), add_list, delete_list, {}));
      }
      Terminate(1.f - prob_, state.GetProb(Fluent(kSteps, {}, 0)), actions);
    }
  }
}

// SampleOperator

SampleOperator::SampleOperator(int name, float prob, float cost_multiplier, bool log_cost) : Operator(name, prob, 1.f, 0.f, cost_multiplier, log_cost) {}

void SampleOperator::ApplicableActions(const State &state, const Environment &env, vector<unique_ptr<Action>> *actions) const {
  const int kSample = StringRegistry::Get()->GetInt("sample");
  const int kSampled = StringRegistry::Get()->GetInt("sampled");
  const int kX = StringRegistry::Get()->GetInt("x");
  const int kY = StringRegistry::Get()->GetInt("y");
  const int kBRockGood = StringRegistry::Get()->GetInt("rock_good");
  const int kSteps = StringRegistry::Get()->GetInt("steps");
  const int kTerminated = StringRegistry::Get()->GetInt("terminated");

  if (state.GetProb(Fluent(kTerminated, {}, 0)) == 0.f) {
    int robot_x = -1;
    int robot_y = -1;
    // find current robot x
    for (int x = 0; x < env.GetNumLocs() + 1; x++) {
      if (state.GetProb(Fluent(kX, {}, x)) == 1.f) {
        robot_x = x;
        break;
      }
    }
    // find current robot y
    for (int y = 0; y < env.GetNumLocs() + 1; y++) {
      if (state.GetProb(Fluent(kY, {}, y)) == 1.f) {
        robot_y = y;
        break;
      }
    }
    assert(robot_x >= 0 && robot_y >= 0);

    if (prob_ > 0.f) {
      }
    
    if (prob_ > 0.f) {
      int rock = env.GetRock(robot_x, robot_y);
      // there is a rock at robot location we haven't already sampled
      if (rock != -1 && state.GetProb(Fluent(kSampled, {}, rock)) == 0.f) {
        const vector<Fluent> add_list{Fluent(kSampled, {}, rock, 1.f),
                                      Fluent(kSteps, {}, 0, state.GetProb(Fluent(kSteps, {}, 0)) + 1)};
        const vector<Fluent> delete_list{Fluent(kSampled, {}, rock, 0.f),
                                         Fluent(kSteps, {}, 0, state.GetProb(Fluent(kSteps, {}, 0)))};
        float sample_cost = 20.f * (1.f - state.GetProb(Fluent(kBRockGood, {}, rock)));
        actions->emplace_back(new Action(kSample, sample_cost + Cost(prob_), add_list, delete_list, {}));
      }
    }
    Terminate(1.f - prob_, state.GetProb(Fluent(kSteps, {}, 0)), actions);
  }
}

// CheckOperator

CheckOperator::CheckOperator(int name, float prob, float cost_multiplier, bool log_cost) : Operator(name, prob, 1.f, 0.f, cost_multiplier, log_cost) {}

void CheckOperator::ApplicableActions(const State &state, const Environment &env, vector<unique_ptr<Action>> *actions) const {
  const int kCheck = StringRegistry::Get()->GetInt("check");
  const int kX = StringRegistry::Get()->GetInt("x");
  const int kY = StringRegistry::Get()->GetInt("y");
  const int kBRockGood = StringRegistry::Get()->GetInt("rock_good");
  const int kSampled = StringRegistry::Get()->GetInt("sampled");
  const int kSteps = StringRegistry::Get()->GetInt("steps");
  const int kTerminated = StringRegistry::Get()->GetInt("terminated");

  if (state.GetProb(Fluent(kTerminated, {}, 0)) == 0.f) {
    int robot_x = -1;
    int robot_y = -1;
    // find current robot x
    for (int x = 0; x < env.GetNumLocs() + 1; x++) {
      if (state.GetProb(Fluent(kX, {}, x)) == 1.f) {
        robot_x = x;
        break;
      }
    }
    // find current robot y
    for (int y = 0; y < env.GetNumLocs() + 1; y++) {
      if (state.GetProb(Fluent(kY, {}, y)) == 1.f) {
        robot_y = y;
        break;
      }
    }
    assert(robot_x >= 0 && robot_y >= 0);

    if (prob_ > 0.f) {
      for (int rock = 0; rock < env.GetNumRocks(); ++rock) {
        // only check rocks we have not already sampled
        if (state.GetProb(Fluent(kSampled, {}, rock)) == 0.f) {
          int rock_x = env.GetRockX(rock);
          int rock_y = env.GetRockY(rock);
          float d = std::sqrt(std::pow(robot_x - rock_x, 2) + std::pow(robot_y - rock_y, 2));
          float efficiency = std::exp(-d);
          float sensor_accuracy = 0.5f + 0.5f * efficiency;
          float rock_good_p = state.GetProb(Fluent(kBRockGood, {}, rock));
          assert(!isnan(rock_good_p));

          // P(obs rock is good) = P(rock is good) * P(sensor is right) + (1 - P(rock is good)) * (1 - P(sensor is right))
          float obs_rock_good_p = rock_good_p * sensor_accuracy + (1.f - rock_good_p) * (1.f - sensor_accuracy);

          // P(obs rock is bad) = P(rock is good) * (1 - P(sensor is right)) + (1 - P(rock is good)) * P(sensor is right)
          float obs_rock_bad_p = rock_good_p * (1.f - sensor_accuracy) + (1.f - rock_good_p) * sensor_accuracy;

          if (obs_rock_good_p > 0.f) {
            // P(rock is good | obs rock is good)
            float rock_good_obs_good = (rock_good_p * sensor_accuracy) / obs_rock_good_p;
            assert(!isnan(rock_good_obs_good));

            // observe rock is good
            const vector<Fluent> add_list_good{Fluent(kBRockGood, {}, rock, rock_good_obs_good),
                                               Fluent(kSteps, {}, 0, state.GetProb(Fluent(kSteps, {}, 0)) + 1)};
            const vector<Fluent> delete_list_good{Fluent(kBRockGood, {}, rock, rock_good_p),
                                                  Fluent(kSteps, {}, 0, state.GetProb(Fluent(kSteps, {}, 0)))};
            actions->emplace_back(new Action(kCheck, 10.f + Cost(obs_rock_good_p * prob_) , add_list_good, delete_list_good, {rock}));
          }

          if (obs_rock_bad_p > 0.f) {
            // P(rock is good | obs rock is bad)
            float rock_good_obs_bad = (rock_good_p * (1.f - sensor_accuracy)) / obs_rock_bad_p;
            assert(!isnan(rock_good_obs_bad));

            // observe rock is bad
            const vector<Fluent> add_list_bad{Fluent(kBRockGood, {}, rock, rock_good_obs_bad),
                                              Fluent(kSteps, {}, 0, state.GetProb(Fluent(kSteps, {}, 0)) + 1)};
            const vector<Fluent> delete_list_bad{Fluent(kBRockGood, {}, rock, rock_good_p),
                                                 Fluent(kSteps, {}, 0, state.GetProb(Fluent(kSteps, {}, 0)))};
            actions->emplace_back(new Action(kCheck, 10.f + Cost(obs_rock_bad_p * prob_) , add_list_bad, delete_list_bad, {rock}));
          }
        }
      }
    }
    Terminate(1.f - prob_, state.GetProb(Fluent(kSteps, {}, 0)), actions);
  }
}

// NoOperator

NoOperator::NoOperator(int name, float prob, float cost_multiplier, bool log_cost) : Operator(name, prob, 1.f, 10.f, cost_multiplier, log_cost) {}

void NoOperator::ApplicableActions(const State &state, const Environment &env, vector<unique_ptr<Action>> *actions) const {
  const int kNoop = StringRegistry::Get()->GetInt("noop");
  const int kSteps = StringRegistry::Get()->GetInt("steps");
  const int kTerminated = StringRegistry::Get()->GetInt("terminated");

  if (state.GetProb(Fluent(kTerminated, {}, 0)) > 0.f) {
    const vector<Fluent> add_list{Fluent(kSteps, {}, 0, state.GetProb(Fluent(kSteps, {}, 0)) + 1)};
    const vector<Fluent> delete_list{Fluent(kSteps, {}, 0, state.GetProb(Fluent(kSteps, {}, 0)))};
    actions->emplace_back(new Action(kNoop, Cost(prob_), add_list, delete_list, {}));
  }
}

} // namespace rocksample
