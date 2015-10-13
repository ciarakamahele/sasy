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


class RockState(State):
    def __init__(self, numLocs, numRocks, rockLocs, rockQuality, rockSampled, robotX, robotY):
        self.numLocs = numLocs
        self.numRocks = numRocks
        self.rockLocs = rockLocs

        # rockQuality is represented as [p_rock0good, p_rock1good, p_rock2good, ...]
        self.rockQuality = rockQuality
        # rockSampled is represented as [p_rock0sampled, p_rock1sampled, p_rock2sampled, ...]
        self.rockSampled = rockSampled

        self.robotX = robotX
        self.robotY = robotY
        self.assert_valid()

    def assert_valid(self):
        assert(self.robotX >= 0 and self.robotX < self.numLocs)
        assert(self.robotY >= 0 and self.robotY < self.numLocs)
        assert(len(self.rockLocs) == (self.numRocks * 2))
        assert(len(self.rockQuality) == self.numRocks)
        assert(len(self.rockSampled) == self.numRocks)

    def copy(self):
        return RockState(self.numLocs,
                         self.numRocks,
                         self.rockLocs[:],
                         self.rockQuality[:],
                         self.rockSampled[:],
                         self.robotX,
                         self.robotY)

    def equals(self, other_state):
        return self.numLocs == other_state.numLocs and \
               self.numRocks == other_state.numRocks and \
               self.rockLocs == other_state.rockLocs and \
               self.rockQuality == other_state.rockQuality and \
               self.rockSampled == other_state.rockSampled and \
               self.robotX == other_state.robotX and \
               self.robotY == other_state.robotY

    def __str__(self):
        r = 'State: %d locs, %d rocks\n' % (self.numLocs, self.numRocks)
        r = 'Robot x: %d, y: %d\n' % (self.robotX, self.robotY)
        r += 'sampled: [' 
        for p in self.rockSampled:
            r += '%.2f ' % p
        r = r[:len(r) - 1] + ']\n'
        r += 'quality: [' 
        for p in self.rockQuality:
            r += '%.2f ' % p
        r = r[:len(r) - 1] + ']\n'

        return r

class RockAction(Action):
    # north
    # south
    # east
    # west
    # check: rock
    # sample
    # reachgoal
    def __init__(self, action, args=[]):
        self.action = action
        self.args = args

    def __str__(self):
        return self.action + ' ' + str(self.args)

class RockWorld(World):
    def __init__(self):
        self.state = None

    # generate initial world
    def generate_new_world(self, belief_state):
        self.state = belief_state.copy()
        # no rocks have been sampled yet
        self.state.rockSampled = [0] * self.state.numRocks
        # each rock has a 50% chance of being good or bad
        for rock in range(self.state.numRocks):
            self.state.rockQuality[rock] = 0 if (random.uniform(0.0, 1.0) <= 0.5) else 1

    def execute_action(self, action):
        # move north
        if action.action == 'amn':
            self.state.robotY = max(0, self.state.robotY - 1)
            return ('onone', 0)
        # move south
        elif action.action == 'ams':
            self.state.robotY = min(self.state.numLocs - 1, self.state.robotY + 1)
            return ('onone', 0)
        # move west
        elif action.action == 'amw':
            self.state.robotX = max(0, self.state.robotX - 1)
            return ('onone', 0)
        # move east
        elif action.action == 'ame':
            # robot may move one space off of east side of map
            self.state.robotX = min(self.state.numLocs, self.state.robotX + 1)
            if self.state.robotX == self.state.numLocs:
                return ('onone', -10)
            else:
                return ('onone', 0)
        # sample
        elif action.action == 'as':
            sampled_rock = -1
            # check for a rock to sample
            for i in range(0, len(self.state.rockLocs), 2):
                if self.state.robotX == self.state.rockLocs[i] and self.state.robotY == self.state.rockLocs[i + 1]:
                    sampled_rock = i / 2
                    break
            # there is a rock to sample
            if sampled_rock != -1:
                # sample rock
                self.state.rockSampled[sampled_rock] = 1
                # rock becomes bad after being sampled
                if self.state.rockQuality[sampled_rock] == 1:
                    self.state.rockQuality[sampled_rock] = 0
                    return ('onone', -10)
                else:
                    return ('onone', 10)
            else:
                return ('onone', 0)
        # check
        elif action.action == 'ac':
            rock = action.args[0]
            rock_x, rock_y = self.state.rockLocs[rock * 2], self.state.rockLocs[(rock * 2) + 1]
            d = math.sqrt((self.state.robotX - rock_x) ** 2 + (self.state.robotY - rock_y) ** 2)
            efficiency = math.exp(-d)
            p = 0.5 + 0.5 * efficiency

            # correctly observe quality of rock
            if (random.uniform(0.0, 1.0) <= p):
                return ('ogood', 0) if self.state.rockQuality[rock] == 1 else ('obad', 0)
            # incorrectly observe quality of rock
            else:
                return ('obad', 0) if self.state.rockQuality[rock] == 1 else ('ogood', 0)
        else:
            # invalid action
            assert(False)

    def success(self, goal_state):
        # successful if there are no more good rocks to sample and robot has moved
        # off the right edge of the map
        return not any([x == 1 for x in self.state.rockQuality]) \
               and self.state.robotX == self.state.numLocs
