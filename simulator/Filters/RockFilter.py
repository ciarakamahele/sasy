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

class RockFilter(Filter):
    def __init__(self):
        pass

    def update_belief(self, belief_state, action, obs):
        new_belief_state = belief_state.copy()
        # check
        if action.action == 'ac':
            checkRock = action.args[0]
            rock_x, rock_y = belief_state.rockLocs[checkRock * 2], belief_state.rockLocs[(checkRock * 2) + 1]
            d = math.sqrt((belief_state.robotX - rock_x) ** 2 + (belief_state.robotY - rock_y) ** 2)
            efficiency = math.exp(-d)
            p = 0.5 + 0.5 * efficiency

            #            observe good        observe bad
            # rock good  p / 2               (1 - p) / 2
            # rock bad   (1 - p) / 2         p / 2

            # if we observe good, there is a p chance the rock was good, and 1 - p chance the rock was bad
            # P(observe good | rock good)*P(rock good) / P(observe good)
            # if the rock is good, there's a p chance we observe good and a 1-p chance we observe bad
            # current rock good belief = current rock good belief * p /2 / 0.5
            # current rock bad belief = current rock bad belief * (1 - p) /2 / 0.5

            # if we observe bad, there is a p chance the rock was bad, and 1 - p chance the rock was good
            # P(observe good | rock good)*P(rock good) / P(observe good)
            # if the rock is bad, there's a p chance we observe bad and a 1-p chance we observe good

            # checked rock is good
            if obs == 'ogood':
                if p == 1:
                    new_belief_state.rockQuality[checkRock] = 1
                else:
                    new_rock_good = belief_state.rockQuality[checkRock] * p
                    new_rock_bad = (1 - belief_state.rockQuality[checkRock]) * (1 - p)
                    new_belief_state.rockQuality[checkRock] = new_rock_good / (new_rock_good + new_rock_bad)

            # checked rock is bad
            else:   
                if p == 1:
                    new_belief_state.rockQuality[checkRock] = 0
                else:
                    new_rock_good = belief_state.rockQuality[checkRock] * (1 - p)
                    new_rock_bad = (1 - belief_state.rockQuality[checkRock]) * p
                    new_belief_state.rockQuality[checkRock] = new_rock_good / (new_rock_good + new_rock_bad)
        # move north
        elif action.action == 'amn':
            new_belief_state.robotY = max(0, new_belief_state.robotY - 1)
        # move south
        elif action.action == 'ams':
            new_belief_state.robotY = min(new_belief_state.numLocs - 1, new_belief_state.robotY + 1)
        # move west
        elif action.action == 'amw':
            new_belief_state.robotX = max(0, new_belief_state.robotX - 1)
        # move east
        elif action.action == 'ame':
            # robot may move one space off of east side of map
            new_belief_state.robotX = min(new_belief_state.numLocs, new_belief_state.robotX + 1)
        # sample
        elif action.action == 'as':
            sampled_rock = -1
            # check for a rock to sample
            for i in range(0, len(belief_state.rockLocs), 2):
                if belief_state.robotX == belief_state.rockLocs[i] and belief_state.robotY == belief_state.rockLocs[i + 1]:
                    sampled_rock = i / 2
                    break
            # there is a rock to sample
            if sampled_rock != -1:
                # sample rock
                new_belief_state.rockSampled[sampled_rock] = 1
                # rock becomes bad after being sampled
                new_belief_state.rockQuality[sampled_rock] = 0
        else:
            # invalid action
            assert(False)




        return new_belief_state
