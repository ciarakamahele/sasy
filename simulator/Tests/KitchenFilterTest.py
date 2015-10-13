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
from Filters.KitchenFilter import *

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

initial_belief_state = KitchenState(numObjs, numLocs, stoveLoc,
                                    move_info, pick_info, place_info, cook_info, look_info,
                                    robotLoc, heldObj, objLocs, freeLocs)

f = KitchenFilter()

new_belief_state = f.update_belief(initial_belief_state, KitchenAction('am', [0, 1]), 'onone')
new_belief_state = f.update_belief(new_belief_state, KitchenAction('am', [1, 0]), 'onone')
assert(new_belief_state.equals(initial_belief_state))
new_belief_state = f.update_belief(initial_belief_state, KitchenAction('api', [0, 0]), 'onone')
new_belief_state = f.update_belief(new_belief_state, KitchenAction('apl', [0, 0]), 'onone')
assert(new_belief_state.equals(initial_belief_state))

move_info = (0.9, 1, 0)
pick_info = (0.8, 1, 0)
place_info = (1, 1, 0)
cook_info = (1, 1, 0)
look_info = (1, 0)

robotLoc = [0.7, 0.2, 0.1]
objLocs = [[0, 0.9, 0.1], [0, 0.1, 0.9]]
freeLocs = [1, 0, 0]
heldObj = [0, 0, 1]

initial_belief_state = KitchenState(numObjs, numLocs, stoveLoc,
                                    move_info, pick_info, place_info, cook_info, look_info,
                                    robotLoc, heldObj, objLocs, freeLocs)

#print initial_belief_state
new_belief_state = f.update_belief(initial_belief_state, KitchenAction('am', [0, 1]), 'onone')
#print new_belief_state
new_belief_state = f.update_belief(new_belief_state, KitchenAction('am', [0, 1]), 'onone')
#print new_belief_state
new_belief_state = f.update_belief(new_belief_state, KitchenAction('am', [0, 1]), 'onone')
#print new_belief_state
new_belief_state = f.update_belief(new_belief_state, KitchenAction('api', [0, 1]), 'onone')
#print new_belief_state


move_info = (0.9, 1, 1)
pick_info = (0.9, 1, 1)
place_info = (0.9, 1, 1)
cook_info = (0.9, 1, 1)
look_info = (0.9, 1)

robotLoc =  [0.100, 0.900, 0.000]
heldObj = [0.022, 0.000, 0.978]
objLocs = [[0.190, 0.810, 0.000], [0.000, 0.000, 1.000]]
freeLocs = [0.810, 0.190, 0.000]

robotLoc =  [0.100, 0.900, 0.000]
heldObj = [0.076, 0.000, 0.924]
objLocs = [[0.190, 0.810, 0.000], [0.000, 0.000, 1.000]]
freeLocs = [0.810, 0.190, 0.000]

robotLoc =  [0.100, 0.900, 0.000]
heldObj = [0.310, 0.000, 0.690]
objLocs = [[0.190, 0.810, 0.000], [0.000, 0.000, 1.000]]
freeLocs = [0.810, 0.190, 0.000]

initial_belief_state = KitchenState(numObjs, numLocs, stoveLoc,
                                    move_info, pick_info, place_info, cook_info, look_info,
                                    robotLoc, heldObj, objLocs, freeLocs)

#print initial_belief_state
new_belief_state = f.update_belief(initial_belief_state, KitchenAction('alh', [-1]), 'obj')
#print new_belief_state
new_belief_state.assert_valid()


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
heldObj = [0.5, 0, 0.5]

initial_belief_state = KitchenState(numObjs, numLocs, stoveLoc,
                                    move_info, pick_info, place_info, cook_info, look_info,
                                    robotLoc, heldObj, objLocs, freeLocs)

f = KitchenFilter()

#print initial_belief_state
new_belief_state = f.update_belief(initial_belief_state, KitchenAction('alh', [-1]), 'onone')
#print new_belief_state
#new_belief_state = f.update_belief(new_belief_state, KitchenAction('am', [1, 0]), 'onone')

numLocs = 3
numObjs = 2
stoveLoc = 0

move_info = (1, 1, 0)
pick_info = (1, 1, 0)
place_info = (1, 1, 0)
cook_info = (1, 1, 0)
look_info = (1, 0)

robotLoc = [1, 0, 0]
objLocs = [[0.5, 0.5, 0], [0, 0, 1]]
freeLocs = [0.5, 0.5, 0]
heldObj = [0, 0, 1]

initial_belief_state = KitchenState(numObjs, numLocs, stoveLoc,
                                    move_info, pick_info, place_info, cook_info, look_info,
                                    robotLoc, heldObj, objLocs, freeLocs)

f = KitchenFilter()

print "initial"
print initial_belief_state
new_belief_state = f.update_belief(initial_belief_state, KitchenAction('alo', [-1, 1]), 'onone')
print "final"
print new_belief_state
#new_belief_state = f.update_belief(new_belief_state, KitchenAction('am', [1, 0]), 'onone')

print 'all tests pass!'


