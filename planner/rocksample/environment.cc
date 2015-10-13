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

namespace rocksample {

using namespace std;

// Environment

Environment::Environment(int num_locs, int num_rocks, const vector<int> &rock_locs) : num_locs_(num_locs), num_rocks_(num_rocks), rock_locs_(rock_locs) {}

int Environment::GetNumLocs() const {
  return num_locs_;
}

int Environment::GetNumRocks() const {
  return num_rocks_;
}

const std::vector<int>& Environment::GetRockLocs() const {
  return rock_locs_;
}

int Environment::GetRockX(int rock) const {
  return rock_locs_[rock * 2];
}

int Environment::GetRockY(int rock) const {
  return rock_locs_[(rock * 2) + 1];
}

// returns rock number if there is a rock at this location, otherwise -1
int Environment::GetRock(int x, int y) const {
  for (int rock = 0; rock < num_rocks_; ++rock) {
    if ((rock_locs_[rock * 2] == x) && (rock_locs_[(rock * 2) + 1] == y)) {
      return rock;
    }
  }
  return -1;
}

} // namespace rocksample
