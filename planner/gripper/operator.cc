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

namespace gripper {

using namespace std;

// MoveOperator

MoveOperator::MoveOperator(int name) : Operator(name) {}

void MoveOperator::ApplicableActions(const State &state, const Environment &env, vector<unique_ptr<Action>> *actions) const {
  const int kMove = StringRegistry::Get()->GetInt("move");
  const int kAtRobby = StringRegistry::Get()->GetInt("at_robby");

  for (int from_room = 0; from_room < env.GetNumRooms(); from_room++) {
    // robot is in from_room
    if (state.GetProb(Fluent(kAtRobby, {}, from_room)) == 1.f) {
      for (int to_room = 0; to_room < env.GetNumRooms(); to_room++) {
        if (to_room != from_room) {
          const vector<Fluent> preconditions{};
          const vector<Fluent> add_list{Fluent(kAtRobby, {}, to_room, 1.f)};
          const vector<Fluent> delete_list{Fluent(kAtRobby, {}, from_room, 1.f)};
          actions->emplace_back(new Action(kMove, 1.f, add_list, delete_list, {from_room, to_room}));
        }
      }
    break;
    }
  }
}

// PickOperator

PickOperator::PickOperator(int name) : Operator(name) {}

void PickOperator::ApplicableActions(const State &state, const Environment &env, vector<unique_ptr<Action>> *actions) const {
  const int kPick = StringRegistry::Get()->GetInt("pick");
  const int kAtRobby = StringRegistry::Get()->GetInt("at_robby");
  const int kCarry = StringRegistry::Get()->GetInt("carry");
  const int kAt = StringRegistry::Get()->GetInt("at");
  const int kFree = StringRegistry::Get()->GetInt("free");

  for (int room = 0; room < env.GetNumRooms(); room++) {
    // robot is in room
    if (state.GetProb(Fluent(kAtRobby, {}, room)) == 1.f) {
      for (int ball = 0; ball < env.GetNumBalls(); ball++) {
        // ball is in room
        if (state.GetProb(Fluent(kAt, {ball}, room)) == 1.f) {
          for (int gripper = 0; gripper < env.GetNumGrippers(); gripper++) {
            // gripper is free
            if (state.GetProb(Fluent(kFree, {}, gripper)) == 1.f) {
              const vector<Fluent> preconditions{};
              const vector<Fluent> add_list{Fluent(kCarry, {gripper}, ball, 1.f)};
              const vector<Fluent> delete_list{Fluent(kAt, {ball}, room, 1.f),
                                               Fluent(kFree, {}, gripper, 1.f)};
              actions->emplace_back(new Action(kPick, 1.f, add_list, delete_list, {ball, gripper}));
            }
          }
        }
      }
    break;
    }
  }
}

// PlaceOperator

PlaceOperator::PlaceOperator(int name) : Operator(name) {}

void PlaceOperator::ApplicableActions(const State &state, const Environment &env, vector<unique_ptr<Action>> *actions) const {
  const int kPlace = StringRegistry::Get()->GetInt("place");
  const int kAtRobby = StringRegistry::Get()->GetInt("at_robby");
  const int kAt = StringRegistry::Get()->GetInt("at");
  const int kFree = StringRegistry::Get()->GetInt("free");
  const int kCarry = StringRegistry::Get()->GetInt("carry");

  for (int room = 0; room < env.GetNumRooms(); room++) {
    // robot is in room
    if (state.GetProb(Fluent(kAtRobby, {}, room)) == 1.f) {
      for (int gripper = 0; gripper < env.GetNumGrippers(); gripper++) {
        for (int ball = 0; ball < env.GetNumBalls(); ball++) {
          // robot is holding ball
          if (state.GetProb(Fluent(kCarry, {gripper}, ball)) == 1.f) {
            const vector<Fluent> preconditions{};
            const vector<Fluent> add_list{Fluent(kAt, {ball}, room, 1.f),
                                             Fluent(kFree, {}, gripper, 1.f)};
            const vector<Fluent> delete_list{Fluent(kCarry, {gripper}, ball, 1.f)};
            actions->emplace_back(new Action(kPlace, 1.f, add_list, delete_list, {ball, gripper}));
          }
        }
      }
      break;
    }
  }
}

} // namespace gripper
