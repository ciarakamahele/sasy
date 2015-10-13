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

from Primitives.RockPrimitives import *
from Planners.RockTestPlanner import *

numLocs = 2
numRocks = 2
rockLocs = [0, 0, 1, 1]

robotX = 0
robotY = 1

rockSampled = [0, 0]
rockQuality = [0.5, 0.5]

initial_state = RockState(numLocs, numRocks, rockLocs, rockQuality, rockSampled, robotX, robotY)
goal_state = initial_state

p = RockTestPlanner()
print p.next_action(initial_state, goal_state, None)
print p.next_action(initial_state, goal_state, 'ogood')
print p.next_action(initial_state, goal_state, 'ogood')
print p.next_action(initial_state, goal_state, 'obad')
print p.next_action(initial_state, goal_state, 'ogood')
print p.next_action(initial_state, goal_state, 'obad')
print p.next_action(initial_state, goal_state, 'ogood')
print p.next_action(initial_state, goal_state, 'ogood')
print p.next_action(initial_state, goal_state, 'obad')
