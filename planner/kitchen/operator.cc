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

#include <sstream>
#include <iostream>
#include <cmath>

#include "string_registry.h"
#include "operator.h"

//TODO: implement state.ValuesOf(kBConf, {}) instead of iterating through all combinations

namespace kitchen {

using namespace std;

// MoveOperator

MoveOperator::MoveOperator(int name, float prob, float base_cost, bool log_cost) : Operator(name, prob, 0.f, base_cost, 0.f, log_cost) {}

void MoveOperator::ApplicableActions(const State &state, const Environment &env, vector<unique_ptr<Action>> *actions) const {
  const int kMove = StringRegistry::Get()->GetInt("move");
  const int kBConf = StringRegistry::Get()->GetInt("conf");
  const int kBHeld = StringRegistry::Get()->GetInt("held");
  const int kBFree = StringRegistry::Get()->GetInt("free");
  const int kBObjLoc = StringRegistry::Get()->GetInt("obj_loc");

  for (int start_loc = 0; start_loc < env.GetNumLocs(); ++start_loc) {
    float startrlp  = state.GetProb(Fluent(kBConf, {}, start_loc));
    // in start loc
    if (startrlp > 0.f) {
      vector<int> end_locs;
      // move left
      int new_loc = start_loc - 1;
      if (new_loc >= 0) {end_locs.push_back(new_loc);}
      // move right
      new_loc = start_loc + 1;
      if (new_loc < env.GetNumLocs()) {end_locs.push_back(new_loc);}
      float hnp  = state.GetProb(Fluent(kBHeld, {}, -1));
      // move to end loc
      for (int end_loc : end_locs) {
        float freep  = state.GetProb(Fluent(kBFree, {}, end_loc));
        // robot moves if it's holding nothing or
        // if it's holding something and endloc is free
        float movep = startrlp * (hnp + (1.f - hnp) * freep) * prob_;
        if (movep > 0.f) {
          float endrlp  = state.GetProb(Fluent(kBConf, {}, end_loc));

          const vector<Fluent> preconditions{};
          vector<Fluent> add_list{Fluent(kBConf, {}, start_loc, startrlp - movep),
                                  Fluent(kBConf, {}, end_loc, endrlp + movep)};
          vector<Fluent> delete_list{Fluent(kBConf, {}, start_loc, startrlp),
                                     Fluent(kBConf, {}, end_loc, endrlp)};

          float startfreep = state.GetProb(Fluent(kBFree, {}, start_loc));
          float endfreep = state.GetProb(Fluent(kBFree, {}, end_loc));
          delete_list.push_back(Fluent(kBFree, {}, start_loc, startfreep));
          delete_list.push_back(Fluent(kBFree, {}, end_loc, endfreep));
          for (int obj = 0; obj < env.GetNumObjs(); ++obj) {
            float hp  = state.GetProb(Fluent(kBHeld, {}, obj));
            float startolp  = state.GetProb(Fluent(kBObjLoc, {obj}, start_loc));
            float endolp  = state.GetProb(Fluent(kBObjLoc, {obj}, end_loc));
            float objmovep = startolp * hp * freep * prob_;
            if (objmovep > 0.f) {
              add_list.push_back(Fluent(kBObjLoc, {obj}, start_loc, startolp - objmovep));
              add_list.push_back(Fluent(kBObjLoc, {obj}, end_loc, endolp + objmovep));
              delete_list.push_back(Fluent(kBObjLoc, {obj}, start_loc, startolp));
              delete_list.push_back(Fluent(kBObjLoc, {obj}, end_loc, endolp));
              startfreep += objmovep;
              endfreep -= objmovep;
            }
          }
          add_list.push_back(Fluent(kBFree, {}, start_loc, startfreep));
          add_list.push_back(Fluent(kBFree, {}, end_loc, endfreep));

          actions->emplace_back(new Action(kMove, Cost(1.f), add_list, delete_list, {start_loc, end_loc}));
        }
      }
    }
  }
}

// PickOperator

PickOperator::PickOperator(int name, float prob, float base_cost, bool log_cost) : Operator(name, prob, 0.f, base_cost, 0.f, log_cost) {}

void PickOperator::ApplicableActions(const State &state, const Environment &env, vector<unique_ptr<Action>> *actions) const {
  const int kPick = StringRegistry::Get()->GetInt("pick");
  const int kBConf = StringRegistry::Get()->GetInt("conf");
  const int kBObjLoc = StringRegistry::Get()->GetInt("obj_loc");
  const int kBHeld = StringRegistry::Get()->GetInt("held");

  float hnp  = state.GetProb(Fluent(kBHeld, {}, -1));
  if (hnp > 0.f) {
    for (int loc = 0; loc < env.GetNumLocs(); ++loc) {
      float rlp  = state.GetProb(Fluent(kBConf, {}, loc));
      if (rlp > 0.f) {
        for (int obj = 0; obj < env.GetNumObjs(); ++obj) {
          float olp  = state.GetProb(Fluent(kBObjLoc, {obj}, loc));
          if (olp > 0.f) {
            float hop  = state.GetProb(Fluent(kBHeld, {}, obj));
            float pickp = rlp * olp * hnp * prob_;

            vector<Fluent> preconditions{};
            vector<Fluent> add_list{Fluent(kBHeld, {}, -1, hnp - pickp),
                                    Fluent(kBHeld, {}, obj, hop + pickp)};
            vector<Fluent> delete_list{Fluent(kBHeld, {}, -1, hnp),
                                       Fluent(kBHeld, {}, obj, hop)};

            actions->emplace_back(new Action(kPick, Cost(1.f), add_list, delete_list, {obj, loc}));
          }
        }
      }
    }
  }
}

// PlaceOperator

PlaceOperator::PlaceOperator(int name, float prob, float base_cost, bool log_cost) : Operator(name, prob, 0.f, base_cost, 0.f, log_cost) {}

void PlaceOperator::ApplicableActions(const State &state, const Environment &env, vector<unique_ptr<Action>> *actions) const {
  const int kPlace = StringRegistry::Get()->GetInt("place");
  const int kBConf = StringRegistry::Get()->GetInt("conf");
  const int kBObjLoc = StringRegistry::Get()->GetInt("obj_loc");
  const int kBHeld = StringRegistry::Get()->GetInt("held");

  for (int loc = 0; loc < env.GetNumLocs(); ++loc) {
    float rlp  = state.GetProb(Fluent(kBConf, {}, loc));
    if (rlp > 0.f) {
      for (int obj = 0; obj < env.GetNumObjs(); ++obj) {
        float hop  = state.GetProb(Fluent(kBHeld, {}, obj));
        float olp  = state.GetProb(Fluent(kBObjLoc, {obj}, loc));
        float placep = rlp * olp * hop * prob_;
        if (placep > 0.f) {
          float hnp  = state.GetProb(Fluent(kBHeld, {}, -1));

          vector<Fluent> preconditions{};
          vector<Fluent> add_list{Fluent(kBHeld, {}, obj, hop - placep),
                                  Fluent(kBHeld, {}, -1, hnp + placep)};
          vector<Fluent> delete_list{Fluent(kBHeld, {}, obj, hop),
                                     Fluent(kBHeld, {}, -1, hnp)};

          actions->emplace_back(new Action(kPlace, Cost(1.f), add_list, delete_list, {obj, loc}));
        }
      }
    }
  }
}

// CookOperator

CookOperator::CookOperator(int name, float prob, float obs) : Operator(name, prob, obs) {}

void CookOperator::ApplicableActions(const State &state, const Environment &env, vector<unique_ptr<Action>> *actions) const {
  const int kCook = StringRegistry::Get()->GetInt("cook");
  const int kBObjLoc = StringRegistry::Get()->GetInt("obj_loc");
  const int kBHeld = StringRegistry::Get()->GetInt("held");
  const int kBCooked = StringRegistry::Get()->GetInt("cooked");

  float hnp  = state.GetProb(Fluent(kBHeld, {}, -1));
  if (hnp > 0.f) {
    for (int stove_loc : env.GetStoveLocs()) {
      for (int obj = 0; obj < env.GetNumObjs(); ++obj) {
        float olp  = state.GetProb(Fluent(kBObjLoc, {obj}, stove_loc));
        if (olp > 0.f) {
          float startcp  = state.GetProb(Fluent(kBCooked, {}, obj));
          float endcp  = startcp + (1.f - startcp) * olp * hnp * prob_;

          vector<Fluent> preconditions{};
          vector<Fluent> add_list{Fluent(kBCooked, {}, obj, endcp)};
          vector<Fluent> delete_list{Fluent(kBCooked, {}, obj, startcp)};

          actions->emplace_back(new Action(kCook, Cost(1.f), add_list, delete_list, {}));
        }
      }
    }
  }
}

// LookRobotOperator

LookRobotOperator::LookRobotOperator(int name, float prob, float base_cost, float cost_multiplier, bool log_cost) : Operator(name, prob, 0.f, base_cost, cost_multiplier, log_cost) {}

void LookRobotOperator::ApplicableActions(const State &state, const Environment &env, vector<unique_ptr<Action>> *actions) const {
  const int kLookRobot = StringRegistry::Get()->GetInt("look_robot");
  const int kBConf = StringRegistry::Get()->GetInt("conf");

  for (int loc = 0; loc < env.GetNumLocs(); ++loc) {
    float start_p  = state.GetProb(Fluent(kBConf, {}, loc));
    if (start_p > 0.f) {
      // P(obs = Robot)
      // = P(obs = Robot | loc = l) * P(loc = l) + 
      //   P(obs = Robot | ^(loc = l)) * P(^(loc = l)) + 
      float obs_p = prob_ * start_p + (1.f - prob_) * (1.f - start_p);
      float success_p = prob_ / obs_p;
      float fail_p = (1.f - prob_) / obs_p;

      vector<Fluent> preconditions{};
      vector<Fluent> add_list{Fluent(kBConf, {}, loc, start_p * success_p)};
      vector<Fluent> delete_list{Fluent(kBConf, {}, loc, start_p)};

      for (int o_loc = 0; o_loc < env.GetNumLocs(); ++o_loc) {
        if (o_loc != loc) {
          add_list.push_back(Fluent(kBConf, {}, o_loc, state.GetProb(Fluent(kBConf, {}, o_loc)) * fail_p));
          delete_list.push_back(Fluent(kBConf, {}, o_loc, state.GetProb(Fluent(kBConf, {}, o_loc))));
        }
      }

      if (obs_p > 0.f) {
        actions->emplace_back(new Action(kLookRobot, Cost(obs_p), add_list, delete_list, {loc}));
      }
    }
  }
}

// LookHandOperator

LookHandOperator::LookHandOperator(int name, float prob, float base_cost, float cost_multiplier, bool log_cost) : Operator(name, prob, 0.f, base_cost, cost_multiplier, log_cost) {}

void LookHandOperator::ApplicableActions(const State &state, const Environment &env, vector<unique_ptr<Action>> *actions) const {
  const int kLookHand = StringRegistry::Get()->GetInt("look_hand");
  const int kBHeld = StringRegistry::Get()->GetInt("held");

  for (int obj = -1; obj < env.GetNumObjs(); ++obj) {
    float start_p  = state.GetProb(Fluent(kBHeld, {}, obj));
    if (start_p > 0.f) {
      // P(obs = o)
      // = P(obs = o | holding = o) * P(holding = o) + 
      //   P(obs = o | ^(holding = o)) * P(^(holding = o)) + 
      float obs_p = prob_ * start_p + (1.f - prob_) * (1.f - start_p);
      float success_p = prob_ / obs_p;
      float fail_p = (1.f - prob_) / obs_p;

      vector<Fluent> preconditions{};
      vector<Fluent> add_list{Fluent(kBHeld, {}, obj, start_p * success_p)};
      vector<Fluent> delete_list{Fluent(kBHeld, {}, obj, start_p)};

      for (int o_obj = -1; o_obj < env.GetNumObjs(); ++o_obj) {
        if (o_obj != obj) {
          add_list.push_back(Fluent(kBHeld, {}, o_obj, state.GetProb(Fluent(kBHeld, {}, o_obj)) * fail_p));
          delete_list.push_back(Fluent(kBHeld, {}, o_obj, state.GetProb(Fluent(kBHeld, {}, o_obj))));
        }
      }

      if (obs_p > 0.f) {
        actions->emplace_back(new Action(kLookHand, Cost(obs_p), add_list, delete_list, {obj}));
      }
    }
  }
}

// LookObjOperator

LookObjOperator::LookObjOperator(int name, float prob, float base_cost, float cost_multiplier, bool log_cost) : Operator(name, prob, 0.f, base_cost, cost_multiplier, log_cost) {}

void LookObjOperator::ApplicableActions(const State &state, const Environment &env, vector<unique_ptr<Action>> *actions) const {
  const int kLookObj = StringRegistry::Get()->GetInt("look_obj");
  const int kBObjLoc = StringRegistry::Get()->GetInt("obj_loc");
  const int kBFree = StringRegistry::Get()->GetInt("free");

  // look for obj in location
  for (int obj = 0; obj < env.GetNumObjs(); ++obj) {
    for (int loc = 0; loc < env.GetNumLocs(); ++loc) {
      float start_p  = state.GetProb(Fluent(kBObjLoc, {obj}, loc));
      //if (start_p > (1.f / env.GetNumObjs())) {
      //if (start_p > 0.f) {
      if (start_p > 0.1f) {
        // P(obs = o)
        // = P(obs = o | oloc = l) * P(oloc = l) + 
        //   P(obs = o | ^(oloc = l)) * P(^(oloc = l)) + 
        float obs_p = prob_ * start_p + (1 - prob_) * (1 - start_p);
        float success_p = prob_ / obs_p;
        float fail_p = (1.f - prob_) / obs_p;


        vector<Fluent> preconditions{};
        vector<Fluent> add_list{Fluent(kBObjLoc, {obj}, loc, start_p * success_p)};
        vector<Fluent> delete_list{Fluent(kBObjLoc, {obj}, loc, start_p)};

        // adjust probabilities of other objects
        for (int o_obj = 0; o_obj < env.GetNumObjs(); ++o_obj) {
          if (o_obj != obj) {
            add_list.push_back(Fluent(kBObjLoc, {o_obj}, loc, state.GetProb(Fluent(kBObjLoc, {o_obj}, loc)) * fail_p));
            delete_list.push_back(Fluent(kBObjLoc, {o_obj}, loc, state.GetProb(Fluent(kBObjLoc, {o_obj}, loc))));
          }
        }

        if (obs_p > 0.f) {
          actions->emplace_back(new Action(kLookObj, Cost(obs_p), add_list, delete_list, {obj, loc}));
        }
      }
    }
  }
  // look for nothing in location
  for (int loc = 0; loc < env.GetNumLocs(); ++loc) {
    float start_p  = state.GetProb(Fluent(kBFree, {}, loc));
    //if (start_p > 0.1f) {
    //if (start_p > (1.f / env.GetNumObjs())) {
    if (start_p > 0.f) {
      // P(obs = o)
      // = P(obs = o | oloc = l) * P(oloc = l) + 
      //   P(obs = o | ^(oloc = l)) * P(^(oloc = l)) + 
      float obs_p = prob_ * start_p + (1.f - prob_) * (1.f - start_p);
      float success_p = prob_ / obs_p;
      float fail_p = (1.f - prob_) / obs_p;

      vector<Fluent> preconditions{};
      vector<Fluent> add_list{Fluent(kBFree, {}, loc, start_p * success_p)};
      vector<Fluent> delete_list{Fluent(kBFree, {}, loc, start_p)};

      // adjust probabilities of objects
      for (int obj = 0; obj < env.GetNumObjs(); ++obj) {
        add_list.push_back(Fluent(kBObjLoc, {obj}, loc, state.GetProb(Fluent(kBObjLoc, {obj}, loc)) * fail_p));
        delete_list.push_back(Fluent(kBObjLoc, {obj}, loc, state.GetProb(Fluent(kBObjLoc, {obj}, loc))));
      }

      if (obs_p > 0.f) {
        actions->emplace_back(new Action(kLookObj, Cost(obs_p), add_list, delete_list, {-1, loc}));
      }
    }
  }
}

} // namespace kitchen
