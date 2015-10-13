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

#include "environment.h"

namespace gripper {

using namespace std;

// Environment

Environment::Environment(int num_rooms, int num_balls, int num_grippers) : num_rooms_(num_rooms), num_balls_(num_balls), num_grippers_(num_grippers) {}

int Environment::GetNumRooms() const {
  return num_rooms_;
}

int Environment::GetNumBalls() const {
  return num_balls_;
}

int Environment::GetNumGrippers() const {
  return num_grippers_;
}

} // namespace gripper
