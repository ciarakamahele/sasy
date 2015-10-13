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

from Simulator import *
from Primitives.RockPrimitives import RockState, RockWorld
from Planners.RockTestPlanner import RockTestPlanner
from Planners.RockPlanner import RockPlanner
from Filters.RockFilter import RockFilter

discount = 0.9
numLocs = 2
numRocks = 2
rockLocs = [0, 0, 1, 1]
robotX, robotY = 0, 1

'''
numLocs = 7
numRocks = 8
rockLocs = [2, 0, 0, 1, 3, 1, 6, 3, 2, 4, 3, 4, 5, 5, 1, 6]
robotX, robotY = 0, 3
'''

rockQuality = [0.5] * numRocks
rockSampled = [0] * numRocks

initial_state = RockState(numLocs, numRocks, rockLocs, rockQuality, rockSampled, robotX, robotY)
goal_state = initial_state

operator_d =        {'amn': {'verbose': 'Moving north'},
                     'ams': {'verbose': 'Moving south'},
                     'ame': {'verbose': 'Moving east'},
                     'amw': {'verbose': 'Moving west'},
                     'as':  {'verbose': 'Sampling'},
                     'ac':  {'verbose': 'Checking rock %d'}}

sim = Simulator(operator_d, RockPlanner(discount),  RockFilter(), RockWorld())
sim.run_sim(initial_state, goal_state, verbose=True, slow=True)
