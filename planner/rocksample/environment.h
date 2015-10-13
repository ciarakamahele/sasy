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

#ifndef ROCKSAMPLE_ENVIRONMENT_H
#define ROCKSAMPLE_ENVIRONMENT_H

#include <vector>

#include <environment.h>

namespace rocksample {

class Environment : public ::Environment {
 public:
  Environment(int num_locs, int num_rocks, const std::vector<int> &rock_locs);

  int GetNumLocs() const;

  int GetNumRocks() const;

  const std::vector<int>& GetRockLocs() const;

  int GetRockX(int rock) const;

  int GetRockY(int rock) const;

  int GetRock(int x, int y) const;

 private:
  const int num_locs_;
  const int num_rocks_;
  const std::vector<int> rock_locs_;
};

} // namespace rocksample

#endif  // ROCKSAMPLE_ENVIRONMENT_H
