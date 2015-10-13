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
#include <chrono>
#include <iostream>
#include <memory>
#include <vector>

#include "environment.h"
#include "heuristic.h"
#include "rocksample/context.h"
#include "rocksample/operator.h"
#include "string_registry.h"
#include "support.h"
#include "uc_search.h"

namespace rocksample {

using namespace std;

bool RockSample(bool verbose, bool input_file, float discount) {
  // operator names
  const int kNorth = StringRegistry::Get()->GetInt("north");
  const int kSouth = StringRegistry::Get()->GetInt("south");
  const int kEast = StringRegistry::Get()->GetInt("east");
  const int kWest = StringRegistry::Get()->GetInt("west");
  const int kSample = StringRegistry::Get()->GetInt("sample");
  const int kCheck = StringRegistry::Get()->GetInt("check");
  const int kNoop = StringRegistry::Get()->GetInt("noop");

  // predicates
  const int kX = StringRegistry::Get()->GetInt("x");
  const int kY = StringRegistry::Get()->GetInt("y");
  const int kSampled = StringRegistry::Get()->GetInt("sampled");
  const int kBRockGood = StringRegistry::Get()->GetInt("rock_good");
  const int kTerminated = StringRegistry::Get()->GetInt("terminated");
  const int kSteps = StringRegistry::Get()->GetInt("steps");

  int num_locs;
  int num_rocks;
  vector<int> rock_locs;
  int robot_x;
  int robot_y;
  vector<float> rock_qualities;
  vector<float> rock_sampled;

  // read domain description from redirected input file
  if (input_file) {
    // environment
    if (!(cin >> num_locs)) {
      cout << "Invalid domain description file format" << endl;
      return -1;
    }

    if (!(cin >> num_rocks)) {
      cout << "Invalid domain description file format" << endl;
      return -1;
    }

    for (int rock = 0; rock < num_rocks; ++rock) {
      int rock_x;
      if (!(cin >> rock_x)) {
        cout << "Invalid domain description file format" << endl;
        return -1;
      }
      rock_locs.push_back(rock_x);

      int rock_y;
      if (!(cin >> rock_y)) {
        cout << "Invalid domain description file format" << endl;
        return -1;
      }
      rock_locs.push_back(rock_y);
    }

    for (int rock = 0; rock < num_rocks; ++rock) {
      float rock_q;
      if (!(cin >> rock_q)) {
        cout << "Invalid domain description file format" << endl;
        return -1;
      }
      rock_qualities.push_back(rock_q);
    }

    for (int rock = 0; rock < num_rocks; ++rock) {
      float rock_s;
      if (!(cin >> rock_s)) {
        cout << "Invalid domain description file format" << endl;
        return -1;
      }
      rock_sampled.push_back(rock_s);
    }

    // start position
    if (!(cin >> robot_x)) {
      cout << "Invalid domain description file format" << endl;
      return -1;
    }

    if (!(cin >> robot_y)) {
      cout << "Invalid domain description file format" << endl;
      return -1;
    }
  } else {
    // realy tiny

    /*
    // environment
    num_locs = 1;
    num_rocks = 1;
    rock_locs.push_back(0);
    rock_locs.push_back(0);

    rock_qualities.push_back(0.5f);

    rock_sampled.push_back(0.f);

    // start position
    robot_x = 0;
    robot_y = 0;
    */

    // tiny

    // environment
    num_locs = 2;
    num_rocks = 2;
    rock_locs.push_back(0);
    rock_locs.push_back(0);
    rock_locs.push_back(1);
    rock_locs.push_back(1);

    rock_qualities.push_back(0.5f);
    rock_qualities.push_back(0.5f);

    rock_sampled.push_back(0.f);
    rock_sampled.push_back(0.f);

    // start position
    robot_x = 0;
    robot_y = 1;
  }

  assert((rock_locs.size() / 2) == num_rocks);
  assert((rock_locs.size() % 2) == 0);
  assert(robot_x >= 0 && robot_x < num_locs);
  assert(robot_y >= 0 && robot_y < num_locs);

  Environment env(num_locs, num_rocks, rock_locs);
  // generate start state
  unique_ptr<State> start_state(new State({Fluent(kX, {}, robot_x, 1.f),
                                           Fluent(kY, {}, robot_y, 1.f)}));
  for (int rock = 0; rock < num_rocks; rock++) {
    start_state -> Add(Fluent(kSampled, {}, rock, rock_sampled[rock]));
    // initially, rocks have a 50% chance of being good
    start_state -> Add(Fluent(kBRockGood, {}, rock, rock_qualities[rock]));
  }
  start_state -> Add(Fluent(kSteps, {}, 0, 0.f));

  // goal state (terminated and off the right edge of the map after expected num steps)
  unique_ptr<State> goal_state(new State({Fluent(kTerminated, {}, 0, 1.f),
                                          Fluent(kX, {}, num_locs, 1.f),
                                          Fluent(kSteps, {}, 0, 1.f/(1.f - discount))}));
  const vector<const State*> goal_set = {goal_state.get()};
  
  bool log_cost = true;
  float move_cost_multiplier = 1.f;
  float sample_cost_multiplier = 1.f;
  float check_cost_multiplier = 1.f;
  // all action probs are 1/(1-y) and all action obs are 1
  unique_ptr<Operator> north_op(new NorthOperator(kNorth, discount, move_cost_multiplier, log_cost));
  unique_ptr<Operator> south_op(new SouthOperator(kSouth, discount, move_cost_multiplier, log_cost));
  unique_ptr<Operator> east_op(new EastOperator(kEast, discount, move_cost_multiplier, log_cost));
  unique_ptr<Operator> west_op(new WestOperator(kWest, discount, move_cost_multiplier, log_cost));
  unique_ptr<Operator> sample_op(new SampleOperator(kSample, discount, sample_cost_multiplier, log_cost));
  unique_ptr<Operator> check_op(new CheckOperator(kCheck, discount, check_cost_multiplier, log_cost));
  unique_ptr<Operator> no_op(new NoOperator(kNoop, 1.f, 1.f, log_cost));
  const vector<const ::Operator*> operators = {north_op.get(), south_op.get(), east_op.get(), west_op.get(), sample_op.get(), check_op.get(), no_op.get()};

  return Search(move(start_state), goal_set, operators, env, HZero(), verbose);
}

} // namespace rocksample
