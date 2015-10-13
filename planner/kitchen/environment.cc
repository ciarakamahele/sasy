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

namespace kitchen {

using namespace std;

// Environment

Environment::Environment(int num_locs, int num_objs, const vector<int> &stove_locs) : num_locs_(num_locs), num_objs_(num_objs), stove_locs_(stove_locs) {}

int Environment::GetNumLocs() const {
  return num_locs_;
}

int Environment::GetNumObjs() const {
  return num_objs_;
}

const std::vector<int>& Environment::GetStoveLocs() const {
  return stove_locs_;
}

} // namespace kitchen
