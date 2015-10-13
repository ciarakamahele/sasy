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
#include "kitchen/context.h"
#include "kitchen/operator.h"
#include "string_registry.h"
#include "support.h"
#include "uc_search.h"

namespace kitchen {

using namespace std;

bool Kitchen(bool verbose, bool input_file, float weight, float epsilon) {
  // operator names
  const int kMove= StringRegistry::Get()->GetInt("move");
  const int kPick = StringRegistry::Get()->GetInt("pick");
  const int kPlace = StringRegistry::Get()->GetInt("place");
  const int kLookRobot = StringRegistry::Get()->GetInt("look_robot");
  const int kLookHand = StringRegistry::Get()->GetInt("look_hand");
  const int kLookObj = StringRegistry::Get()->GetInt("look_obj");

  // predicates
  const int kBConf = StringRegistry::Get()->GetInt("conf");
  const int kBHeld = StringRegistry::Get()->GetInt("held");
  const int kBObjLoc = StringRegistry::Get()->GetInt("obj_loc");
  const int kBFree = StringRegistry::Get()->GetInt("free");

  int num_locs;
  int num_objs;
  vector<int> stove_locs;

  float move_prob;
  float pick_prob; 
  float place_prob;
  float look_prob; 

  // start state
  vector<float> robot_loc;
  vector<float> held_obj;
  vector<vector<float>> obj_locs;
  vector<float> free_locs;

  // goal state
  vector<float> robot_goal;
  vector<float> held_goal;
  vector<vector<float>> obj_loc_goals;
  vector<float> free_loc_goals;

  // read domain description from redirected input file
  if (input_file) {
    // environment
    cout << "1" << endl;
    if (!(cin >> num_locs)) {
      cout << "Invalid domain description file format" << endl;
      return -1;
    }
    cout << "2" << endl;

    if (!(cin >> num_objs)) {
      cout << "Invalid domain description file format" << endl;
      return -1;
    }

    if (!(cin >> move_prob)) {
      cout << "Invalid domain description file format" << endl;
      return -1;
    }

    if (!(cin >> pick_prob)) {
      cout << "Invalid domain description file format" << endl;
      return -1;
    }

    if (!(cin >> place_prob)) {
      cout << "Invalid domain description file format" << endl;
      return -1;
    }

    if (!(cin >> look_prob)) {
      cout << "Invalid domain description file format" << endl;
      return -1;
    }

    // start state

    // robot_loc
    for (int loc = 0; loc < num_locs; ++loc) {
      float p;
      if (!(cin >> p)) {
        cout << "Invalid domain description file format" << endl;
        return -1;
      }
      robot_loc.push_back(p);
    }
    // held_obj
    for (int obj = 0; obj < (num_objs + 1); ++obj) {
      float p;
      if (!(cin >> p)) {
        cout << "Invalid domain description file format" << endl;
        return -1;
      }
      held_obj.push_back(p);
    }

    // obj_locs
    for (int obj = 0; obj < num_objs; ++obj) {
      vector<float> obj_loc;
      for (int loc = 0; loc < num_locs; ++loc) {
        float p;
        if (!(cin >> p)) {
          cout << "Invalid domain description file format" << endl;
          return -1;
        }
        obj_loc.push_back(p);
      }
      obj_locs.push_back(obj_loc);
    }

    // free_locs
    for (int loc = 0; loc < num_locs; ++loc) {
      float p;
      if (!(cin >> p)) {
        cout << "Invalid domain description file format" << endl;
        return -1;
      }
      free_locs.push_back(p);
    }

    // goal state

    // robot_goal
    for (int loc = 0; loc < num_locs; ++loc) {
      float p;
      if (!(cin >> p)) {
        cout << "Invalid domain description file format" << endl;
        return -1;
      }
      robot_goal.push_back(p);
    }

    // held_goal
    for (int obj = 0; obj < (num_objs + 1); ++obj) {
      float p;
      if (!(cin >> p)) {
        cout << "Invalid domain description file format" << endl;
        return -1;
      }
      held_goal.push_back(p);
    }

    // obj_loc_goals
    for (int obj = 0; obj < num_objs; ++obj) {
      vector<float> obj_loc;
      for (int loc = 0; loc < num_locs; ++loc) {
        float p;
        if (!(cin >> p)) {
          cout << "Invalid domain description file format" << endl;
          return -1;
        }
        obj_loc.push_back(p);
      }
      obj_loc_goals.push_back(obj_loc);
    }

    // free_loc_goals
    for (int loc = 0; loc < num_locs; ++loc) {
      float p;
      if (!(cin >> p)) {
        cout << "Invalid domain description file format" << endl;
        return -1;
      }
      free_loc_goals.push_back(p);
    }
  } else {
    /*
    // environment
    num_locs = 20;
    num_objs = 2;

    move_prob = 0.9f;
    pick_prob = 0.9f;
    place_prob = 0.9f;
    look_prob = 0.9f;

    // start state
    robot_loc.push_back(1.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);

    held_obj.push_back(0.f);
    held_obj.push_back(0.f);
    held_obj.push_back(1.f);

    obj_locs.push_back({1.f,0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f});
    obj_locs.push_back({0.f,0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f});

    free_locs.push_back(0.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(0.f);

    // goal state
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.9f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);

    held_goal.push_back(0.f);
    held_goal.push_back(0.f);
    held_goal.push_back(0.9f);

    obj_loc_goals.push_back({0.f, 0.f, 0.9f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f});
    obj_loc_goals.push_back({0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.9f});

    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.f);
    */

    /*
    // environment
    num_locs = 10;
    num_objs = 1;

    move_prob = 0.9f;
    pick_prob = 0.9f;
    place_prob = 0.9f;
    look_prob = 0.9f;

    // start state
    robot_loc.push_back(1.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);

    held_obj.push_back(0.f);
    held_obj.push_back(1.f);

    obj_locs.push_back({1.f,0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f});

    free_locs.push_back(0.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);

    // goal state
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.9f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);

    held_goal.push_back(0.f);
    held_goal.push_back(0.9f);

    obj_loc_goals.push_back({0.f, 0.f, 0.f, 0.9f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f});

    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    */

      /*
    // environment
    num_locs = 4;
    num_objs = 2;

    move_prob = 0.9f;
    pick_prob = 0.9f;
    place_prob = 0.9f;
    look_prob = 0.9f;

    // start state
    //robot_loc.push_back(0.25f);
    //robot_loc.push_back(0.25f);
    //robot_loc.push_back(0.25f);

    //robot_loc.push_back(0.1f);
    robot_loc.push_back(1.0f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);

    held_obj.push_back(0.0f);
    held_obj.push_back(0.0f);
    held_obj.push_back(1.0f);

    obj_locs.push_back({1.f, 0.f, 0.f, 0.f});
    obj_locs.push_back({0.f, 0.f, 1.f, 0.f});
    //obj_locs.push_back({0.6f, 0.2f, 0.2f});

    //free_locs.push_back(0.4f);
    //free_locs.push_back(0.8f);
    //free_locs.push_back(0.8f);
    free_locs.push_back(0.f);
    free_locs.push_back(1.f);
    free_locs.push_back(0.f);
    free_locs.push_back(1.f);

    // goal state
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.9f);
    //robot_goal.push_back(0.f);

    held_goal.push_back(0.f);
    held_goal.push_back(0.f);
    held_goal.push_back(0.9f);

    //obj_loc_goals.push_back({0.f, 0.f, 0.9f, 0.f});
    obj_loc_goals.push_back({0.f, 0.9f, 0.f, 0.f});
    obj_loc_goals.push_back({0.f, 0.f, 0.f, 0.9f});

    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.f);
    //free_loc_goals.push_back(0.f);
    */

    /*
    // environment
    num_locs = 8;
    num_objs = 2;

    move_prob = 0.9f;
    pick_prob = 0.9f;
    place_prob = 0.9f;
    look_prob = 0.9f;

    // start state
    //robot_loc.push_back(0.25f);
    //robot_loc.push_back(0.25f);
    //robot_loc.push_back(0.25f);

    //robot_loc.push_back(0.1f);
    robot_loc.push_back(1.0f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);

    held_obj.push_back(0.0f);
    held_obj.push_back(0.0f);
    held_obj.push_back(1.0f);

    obj_locs.push_back({1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f});
    obj_locs.push_back({0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f});
    //obj_locs.push_back({0.6f, 0.2f, 0.2f});

    //free_locs.push_back(0.4f);
    //free_locs.push_back(0.8f);
    //free_locs.push_back(0.8f);
    free_locs.push_back(0.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(0.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);

    // goal state
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.9f);
    //robot_goal.push_back(0.f);

    held_goal.push_back(0.f);
    held_goal.push_back(0.f);
    held_goal.push_back(0.9f);

    //obj_loc_goals.push_back({0.f, 0.f, 0.9f, 0.f});
    obj_loc_goals.push_back({0.f, 0.f, 0.f, 0.9f, 0.f, 0.f, 0.f, 0.f});
    obj_loc_goals.push_back({0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.9f});

    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.f);
    //free_loc_goals.push_back(0.f);
    */

    // environment
    num_locs = 4;
    num_objs = 1;

    move_prob = 0.9f;
    pick_prob = 0.9f;
    place_prob = 0.9f;
    look_prob = 0.9f;

    // start state
    //robot_loc.push_back(0.25f);
    //robot_loc.push_back(0.25f);
    //robot_loc.push_back(0.25f);

    //robot_loc.push_back(0.1f);
    robot_loc.push_back(1.0f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);

    held_obj.push_back(0.0f);
    held_obj.push_back(1.0f);

    obj_locs.push_back({1.f, 0.f, 0.f, 0.f});
    //obj_locs.push_back({0.6f, 0.2f, 0.2f});

    //free_locs.push_back(0.4f);
    //free_locs.push_back(0.8f);
    //free_locs.push_back(0.8f);
    free_locs.push_back(0.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);

    // goal state
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.9f);
    //robot_goal.push_back(0.f);

    held_goal.push_back(0.f);
    held_goal.push_back(0.9f);

    //obj_loc_goals.push_back({0.f, 0.f, 0.9f, 0.f});
    obj_loc_goals.push_back({0.f, 0.f, 0.f, 0.9f});

    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.f);
    //free_loc_goals.push_back(0.f);


    /*
    // environment
    num_locs = 3;
    num_objs = 1;

    move_prob = 0.9f;
    pick_prob = 0.9f;
    place_prob = 0.9f;
    look_prob = 0.9f;

    // start state
    //robot_loc.push_back(0.25f);
    //robot_loc.push_back(0.25f);
    //robot_loc.push_back(0.25f);

    robot_loc.push_back(1.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    //robot_loc.push_back(0.f);

    held_obj.push_back(0.f);
    held_obj.push_back(1.f);

    //obj_locs.push_back({1.f, 0.f, 0.f, 0.f});
    obj_locs.push_back({1.f, 0.f, 0.f});

    free_locs.push_back(0.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    //free_locs.push_back(1.f);

    // goal state
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.9f);
    robot_goal.push_back(0.f);
    //robot_goal.push_back(0.f);

    held_goal.push_back(0.f);
    held_goal.push_back(0.9f);

    //obj_loc_goals.push_back({0.f, 0.f, 0.9f, 0.f});
    obj_loc_goals.push_back({0.f, 0.f, 0.9f});

    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.f);
    //free_loc_goals.push_back(0.f);
    */

      /*
    // environment
    num_locs = 4;
    num_objs = 2;

    move_prob = 0.9f;
    pick_prob = 0.9f;
    place_prob = 0.9f;
    look_prob = 0.9f;

    // start state
    robot_loc.push_back(0.1f);
    robot_loc.push_back(0.9f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);

    held_obj.push_back(0.9f);
    held_obj.push_back(0.f);
    held_obj.push_back(0.1f);

    obj_locs.push_back({0.4f, 0.6f, 0.f, 0.f});
    obj_locs.push_back({0.f, 0.f, 1.f, 0.f});

    free_locs.push_back(0.6f);
    free_locs.push_back(0.4f);
    free_locs.push_back(0.f);
    free_locs.push_back(1.f);

    // goal state
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.9f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);

    held_goal.push_back(0.f);
    held_goal.push_back(0.f);
    held_goal.push_back(0.9f);

    obj_loc_goals.push_back({0.f, 0.9f, 0.f, 0.f});
    obj_loc_goals.push_back({0.f, 0.f, 0.f, 0.9f});

    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.f);
    */
/*
    // environment
    num_locs = 4;
    num_objs = 2;

    move_prob = 0.9f;
    pick_prob = 0.9f;
    place_prob = 0.9f;
    look_prob = 0.9f;

    // start state
    robot_loc.push_back(1.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);

    held_obj.push_back(0.f);
    held_obj.push_back(0.f);
    held_obj.push_back(1.f);

    obj_locs.push_back({1.f, 0.f, 0.f, 0.f});
    obj_locs.push_back({0.f, 0.f, 1.f, 0.f});

    free_locs.push_back(0.f);
    free_locs.push_back(1.f);
    free_locs.push_back(0.f);
    free_locs.push_back(1.f);

    // goal state
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.9f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);

    held_goal.push_back(0.f);
    held_goal.push_back(0.f);
    held_goal.push_back(0.9f);

    obj_loc_goals.push_back({0.f, 0.9f, 0.f, 0.f});
    obj_loc_goals.push_back({0.f, 0.f, 0.f, 0.9f});

    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.f);
   */

     
    /*
    // environment
    num_locs = 8;
    num_objs = 2;

    move_prob = 0.9f;
    pick_prob = 0.9f;
    place_prob = 0.9f;
    look_prob = 0.9f;

    // start state
    robot_loc.push_back(1.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);

    held_obj.push_back(0.f);
    held_obj.push_back(0.f);
    held_obj.push_back(1.f);

    obj_locs.push_back({1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f});
    obj_locs.push_back({0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f});

    free_locs.push_back(0.f);
    free_locs.push_back(0.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);

    // goal state
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.9f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);

    held_goal.push_back(0.f);
    held_goal.push_back(0.f);
    held_goal.push_back(0.9f);

    obj_loc_goals.push_back({0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.9f, 0.f});
    obj_loc_goals.push_back({0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.9f});

    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.9f);
    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.f);
    */

    /*
    // environment
    num_locs = 12;
    num_objs = 3;

    move_prob = 1.f;
    pick_prob = 1.f;
    place_prob = 1.f;
    look_prob = 1.f;

    // start state
    robot_loc.push_back(1.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);
    robot_loc.push_back(0.f);

    held_obj.push_back(0.f);
    held_obj.push_back(0.f);
    held_obj.push_back(0.f);
    held_obj.push_back(1.f);

    obj_locs.push_back({1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f});
    obj_locs.push_back({0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f});
    obj_locs.push_back({1.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 1.f, 0.f, 0.f, 0.f});

    free_locs.push_back(0.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(0.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(0.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);
    free_locs.push_back(1.f);

    // goal state
    robot_goal.push_back(0.6f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);
    robot_goal.push_back(0.f);

    held_goal.push_back(0.f);
    held_goal.push_back(0.f);
    held_goal.push_back(0.f);
    held_goal.push_back(0.6f);

    obj_loc_goals.push_back({0.f, 0.6f, 0.f, 0.0f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f});
    obj_loc_goals.push_back({0.f, 0.f, 0.f, 0.f, 0.f, 0.6f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f});
    obj_loc_goals.push_back({0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.f, 0.6f, 0.f, 0.f});

    free_loc_goals.push_back(0.6f);
    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.6f);
    free_loc_goals.push_back(0.6f);
    free_loc_goals.push_back(0.6f);
    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.6f);
    free_loc_goals.push_back(0.6f);
    free_loc_goals.push_back(0.6f);
    free_loc_goals.push_back(0.f);
    free_loc_goals.push_back(0.6f);
    free_loc_goals.push_back(0.6f);
    */
  }

  cout << "num_objs: " << num_objs << ", num_locs: " << num_locs << endl;
  cout << "robot_loc: {";
  for (float i : robot_loc) {
    cout << i << " ";
  }
  cout << "}" << endl;
  cout << "held_obj: {";
  for (float i : held_obj) {
    cout << i << " ";
  }
  cout << "}" << endl;
  cout << "obj_locs:";
  for (int o = 0; o < num_objs; o++) {
    cout << "{";
    for (float i : obj_locs[o]) {
      cout << i << " ";
    }
    cout << "}" << endl;
  }
  cout << "free_locs: {";
  for (float i : free_locs) {
    cout << i << " ";
  }
  cout << "}" << endl;

  cout << "robot_goal: {";
  for (float i : robot_goal) {
    cout << i << " ";
  }
  cout << "}" << endl;
  cout << "held_goal: {";
  for (float i : held_goal) {
    cout << i << " ";
  }
  cout << "}" << endl;
  cout << "obj_loc_goals:";
  for (int o = 0; o < num_objs; o++) {
    cout << "{";
    for (float i : obj_loc_goals[o]) {
      cout << i << " ";
    }
    cout << "}" << endl;
  }
  cout << "free_loc_goals: {";
  for (float i : free_loc_goals) {
    cout << i << " ";
  }
  cout << "}" << endl;


  assert(num_objs == obj_locs.size());
  assert(num_objs <= num_locs);
  assert(robot_loc.size() == num_locs);

  assert(robot_goal.size() == num_locs);
  assert(held_obj.size() == (num_objs + 1));
  assert(held_goal.size() == (num_objs + 1));
  assert(obj_locs.size() == num_objs);
  assert(obj_locs.size() == 0 || obj_locs[0].size() == num_locs);
  assert(obj_loc_goals.size() == num_objs);
  assert(obj_loc_goals.size() == 0 || obj_loc_goals[0].size() == num_locs);
  assert(free_locs.size() == num_locs);
  assert(free_loc_goals.size() == num_locs);
  Environment env(num_locs, num_objs, stove_locs);

  vector<Fluent> start_fluents;
  // generate start state
  for (int loc = 0; loc < num_locs; loc++) {
    if (robot_loc[loc]) {start_fluents.push_back(Fluent(kBConf, {}, loc, robot_loc[loc]));}
    if (free_locs[loc]) {start_fluents.push_back(Fluent(kBFree, {}, loc, free_locs[loc]));}
    for (int obj = 0; obj < obj_locs.size(); ++obj) {
      if (obj_locs[obj][loc]) {start_fluents.push_back(Fluent(kBObjLoc, {obj}, loc, obj_locs[obj][loc]));}
    }
  }

  for (int obj = 0; obj < num_objs; ++obj) {
    if (held_obj[obj]) {start_fluents.push_back(Fluent(kBHeld, {}, obj, held_obj[obj]));}
  }
  if (held_obj[num_objs]) {start_fluents.push_back(Fluent(kBHeld, {}, -1, held_obj[num_objs]));}

  unique_ptr<State> start_state(new State(start_fluents));

  // generate goal state
  vector<Fluent> goal_fluents;
  for (int loc = 0; loc < num_locs; loc++) {
    if (robot_goal[loc]) {goal_fluents.push_back(Fluent(kBConf, {}, loc, robot_goal[loc]));}
    if (free_loc_goals[loc]) {goal_fluents.push_back(Fluent(kBFree, {}, loc, free_loc_goals[loc]));}
    for (int obj = 0; obj < obj_locs.size(); ++obj) {
      if (obj_loc_goals[obj][loc]) {goal_fluents.push_back(Fluent(kBObjLoc, {obj}, loc, obj_loc_goals[obj][loc]));}
    }
  }

  for (int obj = 0; obj < num_objs; ++obj) {
    if (held_goal[obj]) {goal_fluents.push_back(Fluent(kBHeld, {}, obj, held_goal[obj]));}
  }
  if (held_goal[num_objs]) {goal_fluents.push_back(Fluent(kBHeld, {}, -1, held_goal[num_objs]));}

  unique_ptr<State> goal_state(new State(goal_fluents));
  const vector<const State*> goal_set = {goal_state.get()};

  bool log_cost = true;
  float move_base_cost = 1.f;
  float pick_base_cost = 1.f;
  float place_base_cost = 1.f;
  float look_base_cost = 1.f;
  float look_cost_multiplier = 2.f;

  unique_ptr<Operator> move_op(new MoveOperator(kMove, move_prob, move_base_cost, log_cost));
  unique_ptr<Operator> pick_op(new PickOperator(kPick, pick_prob, pick_base_cost, log_cost));
  unique_ptr<Operator> place_op(new PlaceOperator(kPlace, place_prob, place_base_cost, log_cost));
  unique_ptr<Operator> look_robot_op(new LookRobotOperator(kLookRobot, look_prob, look_base_cost, look_cost_multiplier, log_cost));
  unique_ptr<Operator> look_hand_op(new LookHandOperator(kLookHand, look_prob, look_base_cost, look_cost_multiplier, log_cost));
  unique_ptr<Operator> look_obj_op(new LookObjOperator(kLookObj, look_prob, look_base_cost, look_cost_multiplier, log_cost));

  const vector<const ::Operator*> operators = {move_op.get(), pick_op.get(), place_op.get(), look_robot_op.get(), look_hand_op.get(), look_obj_op.get()};


  //cout << *start_state.get() << endl;
  //cout << goal_state << endl;
  return Search(move(start_state), goal_set, operators, env, HHSP(), verbose, epsilon, weight);
}

} // namespace kitchen
