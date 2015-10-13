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

from Primitives.KitchenPrimitives import *
from Planners.KitchenTestPlanner import *

numLocs = 3
numObjs = 2
stoveLoc = 0

move_info = (1, 1, 0)
pick_info = (1, 1, 0)
place_info = (1, 1, 0)
cook_info = (1, 1, 0)
look_info = (1, 0)

robotLoc = [1, 0, 0]
objLocs = [[1, 0, 0], [0, 0, 1]]
freeLocs = [0, 1, 0]
heldObj = [0, 0, 1]

world = KitchenWorld()
initial_state = KitchenState(numObjs, numLocs, stoveLoc,
                             move_info, pick_info, place_info, cook_info, look_info,
                             robotLoc, heldObj, objLocs, freeLocs)

robotLoc = [0, 1, 0]
objLocs = [[1, 0, 0], [0, 1, 0]]
freeLocs = [0, 0, 1]
heldObj = [0, 1, 0]

goal_state = KitchenState(numObjs, numLocs, stoveLoc,
                          move_info, pick_info, place_info, cook_info, look_info,
                          robotLoc, heldObj, objLocs, freeLocs)

p = KitchenTestPlanner()

print p.next_action(initial_state, goal_state, None)
print p.next_action(initial_state, goal_state, 'onone')
print p.next_action(initial_state, goal_state, 'onone')
print p.next_action(initial_state, goal_state, 'onone')
print p.next_action(initial_state, goal_state, 'onone')
print p.next_action(initial_state, goal_state, 'onone')
print p.next_action(initial_state, goal_state, 'onone')
print p.next_action(initial_state, goal_state, 'onone')

# move: fromLoc, toLoc
# pick: obj, loc
# place: obj, loc
# lookobj: obj, loc
# lookrobot: loc
# lookhandobj: obj
# reachgoal
