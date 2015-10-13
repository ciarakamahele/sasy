# Copyright 2015 Ciara Kamahele-Sanfratello
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math
from Filter import Filter

class KitchenFilter(Filter):
    def __init__(self):
        pass

    def update_belief(self, belief_state, action, obs):
        new_belief_state = belief_state.copy()
        # move
        if action.action == 'am':
            fromLoc, toLoc = action.args
            hnp = belief_state.heldObj[-1]
            freep = belief_state.freeLocs[toLoc]
            startrlp = belief_state.robotLoc[fromLoc]
            endrlp = belief_state.robotLoc[toLoc]
            # move if we are in toLoc, holding nothing or holding something, and fromLoc is free
            movep = startrlp * (hnp + (1 - hnp) * freep) * belief_state.move_prob;
            new_belief_state.robotLoc[fromLoc] = startrlp - movep
            new_belief_state.robotLoc[toLoc] = endrlp + movep

            startfreep = belief_state.freeLocs[fromLoc]
            endfreep = belief_state.freeLocs[toLoc]
            # object moves if robot moves, it is held, and fromLoc is free
            for obj in range(belief_state.numObjs):
                hp = belief_state.heldObj[obj]
                startolp = belief_state.objLocs[obj][fromLoc]
                endolp = belief_state.objLocs[obj][toLoc]
                objmovep = startolp * hp * freep * belief_state.move_prob
                new_belief_state.objLocs[obj][fromLoc] = startolp - objmovep
                new_belief_state.objLocs[obj][toLoc] = objmovep + endolp
                startfreep += objmovep
                endfreep -= objmovep
            new_belief_state.freeLocs[fromLoc] = startfreep
            new_belief_state.freeLocs[toLoc] = endfreep
        # pick
        if action.action == 'api':
            obj, loc = action.args
            hnp = belief_state.heldObj[-1]
            rlp = belief_state.robotLoc[loc]
            olp = belief_state.objLocs[obj][loc]
            # pick if robot and object are in location and holding nothing
            pickp = rlp * olp * hnp * belief_state.pick_prob
            new_belief_state.heldObj[-1] = hnp - pickp
            new_belief_state.heldObj[obj] = belief_state.heldObj[obj] + pickp
        # place
        if action.action == 'apl':
            obj, loc = action.args
            hp = belief_state.heldObj[obj]
            rlp = belief_state.robotLoc[loc]
            olp = belief_state.objLocs[obj][loc]
            # place if robot and object are in location and holding object
            placep = rlp * olp * hp * belief_state.pick_prob
            new_belief_state.heldObj[-1] = belief_state.heldObj[-1] + placep
            new_belief_state.heldObj[obj] = hp - placep
        # lookobj
        if action.action == 'alo':
            obj, loc = action.args
            # looking for an empty location
            if obj == -1:
                p = belief_state.freeLocs[loc]
            # looking for an object
            else:
                p = belief_state.objLocs[obj][loc]
            # observed object
            if obs == 'oobj':
                obs_p = belief_state.look_obs * p + (1.0 - belief_state.look_obs) * (1.0 - p)
                success_p = belief_state.look_obs / obs_p
                fail_p = (1.0 - belief_state.look_obs) / obs_p
            # didn't observe object
            else:
                obs_p = (1.0 - belief_state.look_obs) * p + belief_state.look_obs * (1.0 - p)
                success_p = (1.0 - belief_state.look_obs) / obs_p
                fail_p = belief_state.look_obs / obs_p
            # looking for an empty location
            if obj == -1:
                new_belief_state.freeLocs[loc] = belief_state.freeLocs[loc] * sucess_p
                for o in range(belief_state.numObjs):
                    new_belief_state.objLocs[o][loc] = belief_state.objLocs[o][loc] * fail_p
                new_belief_state.normalize_objLocs()
                new_belief_state.recompute_free_locs()
            # looking for an object
            else:
                for o in range(belief_state.numObjs):
                    new_belief_state.objLocs[o][loc] = belief_state.objLocs[o][loc] * fail_p
                new_belief_state.objLocs[obj][loc] = belief_state.objLocs[obj][loc] * success_p
                new_belief_state.normalize_objLocs()
                new_belief_state.recompute_free_locs()
        # lookrobot
        if action.action == 'alr':
            loc = action.args[0]
            p = belief_state.robotLoc[loc]
            # observed robot
            if obs == 'orobot':
                obs_p = belief_state.look_obs * p + (1.0 - belief_state.look_obs) * (1.0 - p)
                success_p = belief_state.look_obs / obs_p
                fail_p = (1.0 - belief_state.look_obs) / obs_p
            # didn't observe object
            else:
                obs_p =(1.0 - belief_state.look_obs) * p + belief_state.look_obs * (1.0 - p)
                success_p = (1.0 - belief_state.look_obs) / obs_p
                fail_p = belief_state.look_obs / obs_p
            new_belief_state.robotLoc = [x * fail_p for x in belief_state.robotLoc]
            new_belief_state.robotLoc[loc] = belief_state.robotLoc[loc] * success_p
        # lookhand
        if action.action == 'alh':
            obj = action.args[0]
            p = belief_state.heldObj[obj]
            # observed object
            if obs == 'oobj':
                obs_p = belief_state.look_obs * p + (1.0 - belief_state.look_obs) * (1.0 - p)
                success_p = belief_state.look_obs / obs_p
                fail_p = (1.0 - belief_state.look_obs) / obs_p
            # didn't observe object
            else:
                obs_p = (1.0 - belief_state.look_obs) * p + belief_state.look_obs * (1.0 - p)
                success_p = (1.0 - belief_state.look_obs) / obs_p
                fail_p = belief_state.look_obs / obs_p
            new_belief_state.heldObj = [x * fail_p for x in belief_state.heldObj]
            new_belief_state.heldObj[obj] = success_p * belief_state.heldObj[obj]
        return new_belief_state
