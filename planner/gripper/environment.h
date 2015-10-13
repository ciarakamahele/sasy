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

#ifndef GRIPPER_ENVIRONMENT_H
#define GRIPPER_ENVIRONMENT_H

#include <vector>

#include <environment.h>

namespace gripper {

class Environment : public ::Environment {
 public:
  Environment(int num_rooms, int num_balls, int num_grippers);

  int GetNumRooms() const;

  int GetNumBalls() const;

  int GetNumGrippers() const;

 private:
  int num_rooms_;
  int num_balls_;
  int num_grippers_;
};

} // namespace gripper

#endif  // GRIPPER_ENVIRONMENT_H
