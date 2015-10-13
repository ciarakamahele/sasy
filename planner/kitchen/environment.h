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

#ifndef KITCHEN_ENVIRONMENT_H
#define KITCHEN_ENVIRONMENT_H

#include <vector>

#include <environment.h>

namespace kitchen {

class Environment : public ::Environment {
 public:
  Environment(int num_locs, int num_objs, const std::vector<int> &stove_locs);

  int GetNumLocs() const;

  int GetNumObjs() const;

  const std::vector<int>& GetStoveLocs() const;
 
 private:
  const int num_locs_;
  const int num_objs_;
  const std::vector<int> stove_locs_;
};

} // namespace kitchen

#endif  // KITCHEN_ENVIRONMENT_H

