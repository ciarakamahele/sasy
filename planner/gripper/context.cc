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
#include "gripper/context.h"
#include "gripper/operator.h"
#include "string_registry.h"
#include "support.h"
#include "uc_search.h"

namespace gripper {

using namespace std;

bool Gripper(bool verbose, bool input_file, float discount) {
  // operator names
  const int kMove = StringRegistry::Get()->GetInt("move");
  const int kPick = StringRegistry::Get()->GetInt("pick");
  const int kPlace = StringRegistry::Get()->GetInt("place");

  // predicates
  const int kAtRobby = StringRegistry::Get()->GetInt("at_robby");
  const int kAt = StringRegistry::Get()->GetInt("at");
  const int kFree = StringRegistry::Get()->GetInt("free");
  const int kCarry = StringRegistry::Get()->GetInt("carry");

  int num_rooms = 2;
  int num_balls = 1;
  int num_grippers = 1;
  int room = 0;
  vector<int> ball_locs = {0};
  vector<int> gripper_holding {-1};

  assert(ball_locs.size() == num_balls);
  assert(gripper_holding.size() == num_grippers);
  assert(room >= 0 && room < num_rooms);

  Environment env(num_rooms, num_balls, num_grippers);
  // generate start state
  unique_ptr<State> start_state(new State({Fluent(kAtRobby, {}, room, 1.f)}));

  for (int ball = 0; ball < num_balls; ball++) {
    if (ball_locs[ball] >= 0) {
      start_state -> Add(Fluent(kAt, {ball}, ball_locs[ball], 1.f));
    }
  }

  for (int gripper = 0; gripper < num_grippers; gripper++) {
    int ball = gripper_holding[gripper];
    if (ball >= 0) {
      // make sure ball and robot are in the same room
      assert(ball_locs[ball] == room);
      start_state -> Add(Fluent(kCarry, {gripper}, ball, 1.f));
    } else {
      start_state -> Add(Fluent(kFree, {}, gripper, 1.f));
    }
  }

  // goal state
  unique_ptr<State> goal_state(new State({Fluent(kAt, {0}, 1, 1.f)}));
  const vector<const State*> goal_set = {goal_state.get()};
  
  
  unique_ptr<Operator> move_op(new MoveOperator(kMove));
  unique_ptr<Operator> pick_op(new PickOperator(kPick));
  unique_ptr<Operator> place_op(new PlaceOperator(kPlace));
  const vector<const ::Operator*> operators = {move_op.get(), pick_op.get(), place_op.get()};

  return Search(move(start_state), goal_set, operators, env, HZero(), verbose);
}

} // namespace gripper
