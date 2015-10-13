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

#include <iostream>

#include <gflags/gflags.h>

#include "kitchen/context.h"
#include "rocksample/context.h"
#include "gripper/context.h"
#include "string_registry.h"

DEFINE_string(problem, "", "One of the following problem kinds: kitchen, rocksample, gripper");
DEFINE_double(discount, 0.95f, "The discounting factor to use");
DEFINE_bool(verbose, false, "Make the search verbose");
DEFINE_bool(file, false, "Specify domain using an input file");
DEFINE_double(weight, 0.f, "Specify weight (0.0 = greediest)");
DEFINE_double(epsilon, 0.f, "Specify epsilon");

using namespace std;

int main(int argc, char **argv) {
  google::ParseCommandLineFlags(&argc, &argv, true);

  StringRegistry::Init();

  int result = 1; // Default to error.

  if (FLAGS_problem == "kitchen") {
    result = kitchen::Kitchen(FLAGS_verbose, FLAGS_file, static_cast<float>(FLAGS_weight), static_cast<float>(FLAGS_epsilon));
  } else if (FLAGS_problem == "rocksample") {
    result = rocksample::RockSample(FLAGS_verbose, FLAGS_file, static_cast<float>(FLAGS_discount));
  } else if (FLAGS_problem == "gripper") {
    result = gripper::Gripper(FLAGS_verbose, FLAGS_file, static_cast<float>(FLAGS_discount));
  } else if (FLAGS_problem.empty()) {
    cerr << "Need to specify the 'problem' parameter!" << endl;
  } else {
    cerr << "Unknown problem kind '" << FLAGS_problem << "'!" << endl;
  }

  return result;
}
