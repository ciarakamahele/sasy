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

# test generate_new_world()
world.generate_new_world(initial_state)
initial_world_state = world.state.copy()
#print initial_state
#print world.state
#print goal_state

# test copy(), equals()
assert(initial_state.equals(initial_state.copy()))
assert(goal_state.equals(goal_state.copy()))
assert(world.state.equals(initial_world_state.copy()))


# move: fromLoc, toLoc
# pick: obj, loc
# place: obj, loc
# lookobj: obj, loc
# lookrobot: loc
# lookhandobj: obj
# lookhandn
# reachgoal

# test execute_action(), success()
o, cost = world.execute_action(KitchenAction('am', [1, 2]))
assert(world.state.equals(initial_world_state))
assert(not world.success(goal_state))
assert(o == 'oaction')
o, cost = world.execute_action(KitchenAction('alh', [2]))
assert(o == 'oobj')
o, cost = world.execute_action(KitchenAction('api', [0, 0]))
assert(not world.state.equals(initial_world_state))
assert(not world.success(goal_state))
assert(o == 'oaction')
o, cost = world.execute_action(KitchenAction('alh', [2]))
assert(o == 'onone')
o, cost = world.execute_action(KitchenAction('alh', [1]))
assert(o == 'onone')
o, cost = world.execute_action(KitchenAction('alh', [0]))
assert(o == 'oobj')
o, cost = world.execute_action(KitchenAction('apl', [0, 0]))
assert(world.state.equals(initial_world_state))
assert(not world.success(goal_state))
assert(o == 'oaction')
o, cost = world.execute_action(KitchenAction('api', [0, 0]))
assert(not world.state.equals(initial_world_state))
assert(not world.success(goal_state))
assert(o == 'oaction')
o, cost = world.execute_action(KitchenAction('am', [0, 1]))
assert(not world.state.equals(initial_world_state))
assert(not world.success(goal_state))
assert(o == 'oaction')
o, cost = world.execute_action(KitchenAction('am', [0, 1]))
assert(not world.state.equals(initial_world_state))
assert(not world.success(goal_state))
assert(o == 'oaction')
o, cost = world.execute_action(KitchenAction('apl', [0, 1]))
assert(not world.state.equals(initial_world_state))
assert(not world.success(goal_state))
assert(o == 'oaction')
o, cost = world.execute_action(KitchenAction('api', [0, 1]))
assert(not world.state.equals(initial_world_state))
assert(not world.success(goal_state))
assert(o == 'oaction')
o, cost = world.execute_action(KitchenAction('alh', [0]))
assert(o == 'oobj')
o, cost = world.execute_action(KitchenAction('api', [0, 1]))
assert(not world.state.equals(initial_world_state))
assert(not world.success(goal_state))
assert(o == 'oaction')
o, cost = world.execute_action(KitchenAction('am', [1, 0]))
assert(not world.state.equals(initial_world_state))
assert(not world.success(goal_state))
assert(o == 'oaction')
o, cost = world.execute_action(KitchenAction('apl', [0, 0]))
assert(world.state.equals(initial_world_state))
assert(not world.success(goal_state))
assert(o == 'oaction')
o, cost = world.execute_action(KitchenAction('apl', [0, 0]))
assert(world.state.equals(initial_world_state))
assert(not world.success(goal_state))
assert(o == 'oaction')
o, cost = world.execute_action(KitchenAction('alo', [0, 0]))
assert(o == 'oobj')
o, cost = world.execute_action(KitchenAction('am', [0, 1]))
assert(not world.state.equals(initial_world_state))
assert(not world.success(goal_state))
assert(o == 'oaction')
o, cost = world.execute_action(KitchenAction('alr', [1]))
assert(o == 'orobot')
o, cost = world.execute_action(KitchenAction('api', [1, 1]))
assert(not world.state.equals(initial_world_state))
assert(not world.success(goal_state))
assert(o == 'oaction')
o, cost = world.execute_action(KitchenAction('alo', [1, 1]))
assert(o == 'onone')
o, cost = world.execute_action(KitchenAction('api', [1, 2]))
assert(not world.state.equals(initial_world_state))
assert(not world.success(goal_state))
assert(o == 'oaction')
o, cost = world.execute_action(KitchenAction('am', [1, 2]))
assert(not world.state.equals(initial_world_state))
assert(not world.success(goal_state))
assert(o == 'oaction')
o, cost = world.execute_action(KitchenAction('api', [1, 2]))
assert(not world.state.equals(initial_world_state))
assert(not world.success(goal_state))
assert(o == 'oaction')
o, cost = world.execute_action(KitchenAction('alo', [1, 2]))
assert(o == 'oobj')
o, cost = world.execute_action(KitchenAction('alo', [1, 1]))
assert(o == 'onone')
o, cost = world.execute_action(KitchenAction('am', [2, 1]))
assert(not world.state.equals(initial_world_state))
assert(world.success(goal_state))
assert(o == 'oaction')

print 'all tests pass!'
