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

import math, random
from Primitives import *

class KitchenState(State):
    def __init__(self, numObjs, numLocs, stoveLoc,
                 move_info, pick_info, place_info, cook_info, look_info,
                 robotLoc, heldObj, objLocs, freeLocs,
                 round_precision=3, world=False, epsilon=0.1):
        self.numObjs = numObjs
        self.numLocs = numLocs
        self.stoveLoc = stoveLoc

        self.move_info = move_info
        self.pick_info = pick_info
        self.place_info = place_info
        self.cook_info = cook_info
        self.look_info = look_info

        self.move_prob, self.bump_obs, self.move_cost = move_info
        self.pick_prob, self.drop_obs, self.pick_cost = pick_info
        self.place_prob, self.stick_obs, self.place_cost = place_info
        self.cook_prob, self.raw_obs, self.cook_cost = cook_info
        self.look_obs, self.look_cost = look_info

        # robot and object locations are represented as [p_loc0, p_loc1, p_loc2, ...]
        self.robotLoc = robotLoc[:]
        # heldObj is represented as [p_obj0, p_obj1, p_obj2, ..., p_nothing]
        self.heldObj = heldObj[:]
        # objLocs is represented as [obj0loc[], obj1loc[], obj2loc[] ...]
        self.objLocs = [objLoc[:] for objLoc in objLocs]
        # freeLocs is represented as [p_loc0_free, p_loc1_free, p_loc2_free, ...]
        self.freeLocs = freeLocs[:]

        self.round_precision = round_precision
        self.epsilon = epsilon
        self.world = world
        self.assert_valid()


    # return robot location if definite else None
    def robot_loc(self):
        for loc in range(self.numLocs):
            if self.robotLoc[loc] == 1:
                return loc
        return None

    # set robot location if definite
    def set_robot_loc(self, loc):
        self.robotLoc[self.robot_loc()] = 0
        self.robotLoc[loc] = 1

    # return held object if definite else None
    def held_obj(self):
        for obj in range(self.numObjs + 1):
            if self.heldObj[obj] == 1:
                return obj
        return None

    # set held object if definite
    def set_held_obj(self, obj):
        self.heldObj[self.held_obj()] = 0
        self.heldObj[obj] = 1

    # return object location if definite else None
    def obj_loc(self, obj):
        for loc in range(self.numLocs):
            if self.objLocs[obj][loc] == 1:
                return loc
        return None

    # return True if location is free
    def free_loc(self, loc):
        return self.freeLocs[loc] == 1

    # set object location if definite
    def set_obj_loc(self, obj, loc):
        self.objLocs[obj][self.obj_loc(obj)] = 0
        self.objLocs[obj][loc] = 1

    # recompute free loc probabilities
    def recompute_free_locs(self):
        for l in range(self.numLocs):
            s = 0.0
            for o in range(self.numObjs):
                s += self.objLocs[o][l]
            self.freeLocs[l] = 1.0 - s

    def normalize_robotLoc(self):
        self.robotLoc = [x / sum(self.robotLoc) for x in self.robotLoc]


    def normalize_objLoc(self, o):
        self.objLocs[o] = [x / sum(self.objLocs[o]) for x in self.objLocs[o]]

    def normalize_objLocs(self):
        for o in range(self.numObjs):
            self.normalize_objLoc(o)

    def normalize_heldObj(self):
        self.heldObj = [x / sum(self.heldObj) for x in self.heldObj]

    def assert_valid(self):
        if self.world:
            return True

        # robot bloc <= 1
        if sum(self.robotLoc) > 1.0:
            # normalize
            if sum(self.robotLoc) <= (1.0 + self.epsilon):
                self.normalize_robotLoc()
            else:
                assert(False)

        # obj blocs <= 1
        for i, o in enumerate(self.objLocs):
            if sum(o) > 1.0:
                # normalize
                if sum(o) <= (1.0 + self.epsilon):
                    self.normalize_objLoc(i)
                else:
                    assert(False)

        # bholds + bholdnothings <= 1
        if sum(self.heldObj) > 1.0:
            # normalize
            if sum(self.heldObj) <= (1.0 + self.epsilon):
                self.normalize_heldObj()
            else:
                assert(False)

        self.recompute_free_locs()

    def assert_complete(self):
        if self.world:
            return True

        # robot bloc == 1
        assert(sum(self.robotLoc) == 1.0)
        # obj blocs == 1
        assert(all([round(sum(o), self.round_precision) == 1.0 for o in self.objLocs]))
        # bholds + bholdnothings == 1
        assert(round(sum(self.heldObj), self.round_precision) == 1.0)
        # loc blocs + bfrees == 1
        for l in range(self.numLocs):
            s = self.freeLocs[l]
            for o in range(self.numObjs):
                s += self.objLocs[o][l]
            assert(round(s, self.round_precision) == 1.0)

    def copy(self):
        return KitchenState(self.numObjs, self.numLocs, self.stoveLoc,
                            self.move_info, self.pick_info, self.place_info, self.cook_info, self.look_info,
                            self.robotLoc, 
                            self.heldObj, 
                            self.objLocs,
                            self.freeLocs)

    def equals(self, goal_state):
        if self.numObjs != goal_state.numObjs or self.numLocs != goal_state.numLocs:
            return False
        for l in range(self.numLocs):
            if round(self.robotLoc[l], self.round_precision) != round(goal_state.robotLoc[l], self.round_precision):
                return False
        for o in range(self.numObjs):
            for l in range(self.numLocs):
                if round(self.objLocs[o][l], self.round_precision) != round(goal_state.objLocs[o][l], self.round_precision):
                    return False
        for o in range(self.numObjs + 1):
            if round(self.heldObj[o], self.round_precision) != round(goal_state.heldObj[o], self.round_precision):
                return False
        for l in range(self.numLocs):
            if round(self.freeLocs[l], self.round_precision) != round(goal_state.freeLocs[l], self.round_precision):
                return False
        return True

    def satisfies(self, goal_state):
        if self.numObjs != goal_state.numObjs or self.numLocs != goal_state.numLocs:
            return False
        for l in range(self.numLocs):
            if round(self.robotLoc[l], self.round_precision) < round(goal_state.robotLoc[l], self.round_precision):
                return False
        for o in range(self.numObjs):
            for l in range(self.numLocs):
                if round(self.objLocs[o][l], self.round_precision) < round(goal_state.objLocs[o][l], self.round_precision):
                    return False
        for o in range(self.numObjs + 1):
            if round(self.heldObj[o], self.round_precision) < round(goal_state.heldObj[o], self.round_precision):
                return False
        for l in range(self.numLocs):
            if round(self.freeLocs[l], self.round_precision) < round(goal_state.freeLocs[l], self.round_precision):
                return False
        return True        

    def attributes(self):
        return (self.numObjs, self.numLocs, self.stoveLoc, self.move_info, self.pick_info, self.place_info, self.cook_info, self.look_info)

    def attributes_str(self):
        return 'numObjs: %d, numLocs: %d,\nmove: %s,\npick: %s,\nplace: %s\nlook: %s' %\
               (self.numObjs, self.numLocs,
                str(self.move_info),
                str(self.pick_info),
                str(self.place_info),
                str(self.look_info))

    def __str__(self):
        r = 'State: %d locs, %d objs\n' % (self.numLocs, self.numObjs)
        r += 'loc: [' 
        for p in self.robotLoc:
            r += '%.3f ' % p
        r = r[:len(r) - 1] + ']\n'

        r += 'held: [' 
        for p in self.heldObj[:len(self.heldObj) - 1]:
            r += '%.3f ' % p
        r = r[:len(r) - 1] + '] %.3f\n' % self.heldObj[-1]

        for i, o in enumerate(self.objLocs):
            r += 'object %d: [' % i
            for p in o: 
                r += '%.3f ' % p
            r = r[:len(r) - 1]
            r += ']\n'
        r =  r[:len(r) - 1] + '\n'

        r += 'free: [' 
        for p in self.freeLocs:
            r += '%.3f ' % p
        r = r[:len(r) - 1] + ']\n'

        return r

class KitchenAction(Action):
    # move: fromLoc, toLoc
    # pick: obj, loc
    # place: obj, loc
    # lookobj: obj, loc
    # lookrobot: loc
    # lookhand: obj
    # reachgoal
    def __init__(self, action, args=[]):
        self.action = action
        self.args = args

    def __str__(self):
        return self.action + ' ' + str(self.args)

class KitchenWorld(World):
    def __init__(self):
        self.state = None

    # generate initial world based off of belief_state, leaving belief_state unchanged
    def generate_new_world(self, belief_state):
        self.state = belief_state.copy()

        # robotLoc
        p = random.uniform(0.0, 1.0)
        total = 0.0
        for loc in range(belief_state.numLocs):
            total += belief_state.robotLoc[loc]
            if p <= total:
                self.state.robotLoc = [0] * belief_state.numLocs
                self.state.robotLoc[loc] = 1
                break

        # heldObj
        p = random.uniform(0.0, 1.0)
        total = 0.0
        for obj in range(belief_state.numObjs + 1):
            total += belief_state.heldObj[loc]
            if p <= total:
                self.state.heldObj = [0] * (belief_state.numObjs + 1)
                self.state.heldObj[obj] = 1
                break

        # objLocs
        for obj in range(belief_state.numObjs):
            p = random.uniform(0.0, 1.0)
            total = 0.0
            for loc in range(belief_state.numLocs):
                total += belief_state.objLocs[obj][loc]
                if p <= total:
                    self.state.objLocs[obj] = [0] * belief_state.numLocs
                    self.state.objLocs[obj][loc] = 1
                    break

        # freeLocs
        self.state.freeLocs = [1] * belief_state.numLocs
        for obj in range(belief_state.numObjs):
            self.state.freeLocs[self.state.obj_loc(obj)] = 0

    def execute_action(self, action):
        if action.action == 'am':
            p = random.uniform(0.0, 1.0)
            actuator_success = False
            # actuator succeeds
            if p <= self.state.move_prob:
                fromLoc, toLoc = action.args
                # in fromLoc
                if self.state.robot_loc() == fromLoc:
                    # holding nothing
                    if self.state.held_obj() == self.state.numObjs:
                        # move robot
                        self.state.set_robot_loc(toLoc)
                        actuator_success = True
                    # holding something and toLoc is free
                    elif self.state.freeLocs[toLoc] == 1:
                        # free old location
                        self.state.freeLocs[fromLoc] = 1
                        # move robot
                        self.state.set_robot_loc(toLoc)
                        # move held object
                        self.state.set_obj_loc(self.state.held_obj(), toLoc)
                        # occupy new location
                        self.state.freeLocs[toLoc] = 0
                        actuator_success = True
            return ('oaction', self.state.move_cost, actuator_success)
        elif action.action == 'api':
            p = random.uniform(0.0, 1.0)
            actuator_success = False
            # actuator succeeds
            if p <= self.state.pick_prob:
                obj, loc = action.args
                # not holding anything and robot and object are in the location
                if self.state.held_obj() == self.state.numObjs and\
                   self.state.obj_loc(obj) == loc and\
                   self.state.robot_loc() == loc:
                    # pick up object
                    self.state.set_held_obj(obj)
                    actuator_success = True
            return ('oaction', self.state.pick_cost, actuator_success)
        elif action.action == 'apl':
            p = random.uniform(0.0, 1.0)
            actuator_success = False
            # actuator succeeds
            if p <= self.state.place_prob:
                obj, loc = action.args
                # holding the object
                if self.state.held_obj() == obj:
                    # place up object
                    self.state.set_held_obj(self.state.numObjs)
                    actuator_success = True
            return ('oaction', self.state.place_cost, actuator_success)
        elif action.action == 'alo':
            obj, loc = action.args
            p = random.uniform(0.0, 1.0)
            # sensor succeeds
            if p <= self.state.look_obs:
                if obj == -1:
                    return ('oobj' if self.state.free_loc(loc) else 'onone', self.state.look_cost, True)
                else:
                    return ('oobj' if not self.state.free_loc(loc) else 'onone', self.state.look_cost, True)
            # sensor fails
            else:
                if obj == -1:
                    return ('onone' if self.state.obj_loc(obj) != loc else 'oobj', self.state.look_cost, False)
                else:
                    return ('onone' if self.state.obj_loc(obj) == loc else 'oobj', self.state.look_cost, False)
        elif action.action == 'alr':
            loc = action.args[0]
            p = random.uniform(0.0, 1.0)
            # sensor succeeds
            if p <= self.state.look_obs:
                return ('orobot' if self.state.robot_loc() == loc else 'onone', self.state.look_cost, True)
            # sensor fails
            else:
                return ('onone' if self.state.robot_loc() == loc else 'orobot', self.state.look_cost, False)
        elif action.action == 'alh':
            obj = action.args[0]
            p = random.uniform(0.0, 1.0)
            # sensor succeeds
            if p <= self.state.look_obs:
                if obj == -1:
                    return ('oobj' if self.state.held_obj() == self.state.numObjs else 'onone', self.state.look_cost, True)
                else:
                    return ('oobj' if self.state.held_obj() == obj else 'onone', self.state.look_cost, True)
            # sensor fails
            else:
                if obj == -1:
                    return ('onone' if self.state.held_obj() == self.state.numObjs else 'oobj', self.state.look_cost, False)
                else:
                    return ('onone' if self.state.held_obj() == obj else 'oobj', self.state.look_cost, False)
        else:
            # invalid action
            assert(False)
        
    def success(self, goal_state):
        return self.state.satisfies(goal_state)
