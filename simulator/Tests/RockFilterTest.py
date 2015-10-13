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
from Filters.RockFilter import *

numLocs = 2
numRocks = 2
rockLocs = [0, 0, 1, 1]

rockQuality = [0.5, 0.1]
rockSampled = [0, 1]

robotX = 0
robotY = 0

f = RockFilter()
belief_state = RockState(numLocs, numRocks, rockLocs,
                         rockQuality, rockSampled, robotX, robotY)

new_belief_state = f.update_belief(belief_state, RockAction('ac', [0]), 'ogood')
assert(new_belief_state.equals(RockState(numLocs, numRocks, rockLocs,
                               [1.0, 0.1], rockSampled, robotX, robotY)))
new_belief_state = f.update_belief(new_belief_state, RockAction('ac', [0]), 'obad')
assert(new_belief_state.equals(RockState(numLocs, numRocks, rockLocs,
                               [0.0, 0.1], rockSampled, robotX, robotY)))
new_belief_state = f.update_belief(new_belief_state, RockAction('ac', [1]), 'ogood')
print new_belief_state
new_belief_state = f.update_belief(new_belief_state, RockAction('ac', [1]), 'ogood')
print new_belief_state
new_belief_state = f.update_belief(new_belief_state, RockAction('ac', [1]), 'obad')
print new_belief_state

print 'all tests pass!'


