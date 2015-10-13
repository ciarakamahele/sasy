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

numLocs = 2
numRocks = 2
rockLocs = [0, 0, 1, 1]

rockQuality = [0.5, 0.9]
rockSampled = [0, 1]

robotX = 0
robotY = 1

world = RockWorld()
test_state = RockState(numLocs, numRocks, rockLocs,
                          rockQuality, rockSampled, robotX, robotY)

# test generate_new_world()
world.generate_new_world(test_state)
initial_world_state = world.state.copy()
print initial_state
print world.state

# test copy(), equals()
assert(test_state.equals(test_state.copy()))
assert(world.state.equals(initial_world_state.copy()))

# test execute_action(), success()
world.execute_action(RockAction('ams'))
assert(world.state.equals(initial_world_state))
assert(not world.success())
world.execute_action(RockAction('amn'))
assert(not world.success())
assert(not world.state.equals(initial_world_state))
world.execute_action(RockAction('as'))
assert(not world.success())
world.execute_action(RockAction('ame'))
assert(not world.success())
world.execute_action(RockAction('ams'))
assert(not world.success())
world.execute_action(RockAction('as'))
assert(not world.success())
world.execute_action(RockAction('ame'))
assert(world.success())

print 'all tests pass!'
