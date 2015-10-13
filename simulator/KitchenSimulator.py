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
from Primitives.KitchenPrimitives import KitchenState, KitchenWorld
from Planners.KitchenTestPlanner import KitchenTestPlanner
from Planners.KitchenPlanner import KitchenPlanner
from Filters.KitchenFilter import KitchenFilter

move_info = (0.9, 1, 1)
pick_info = (0.9, 1, 1)
place_info = (0.9, 1, 1)
cook_info = (0.9, 1, 1)
look_info = (0.9, 1)

numLocs = 3
numObjs = 2
stoveLoc = 0

robotLoc = [1, 0, 0]
heldObj = [0, 0, 1]
objLocs = [[1, 0, 0], [0, 0, 1]]
freeLocs = [0, 1, 0]


numLocs = 4
numObjs = 1
stoveLoc = 0

robotLoc = [1, 0, 0]
heldObj = [0, 1]
objLocs = [[1, 0, 0, 0],
           [0, 0, 1, 0]]
freeLocs = [0, 1, 0, 1]

numLocs = 4 
numObjs = 1
stoveLoc = 0

robotLoc = [1, 0, 0, 0]
heldObj = [0, 1]
objLocs = [[1, 0, 0, 0]]
freeLocs = [0, 1, 1, 1]



world = KitchenWorld()
initial_state = KitchenState(numObjs, numLocs, stoveLoc,
                             move_info, pick_info, place_info, cook_info, look_info,
                             robotLoc, heldObj, objLocs, freeLocs)

freeLocs = [0.0, 0.0, 0.0]

robotLoc = [0.0, 0.0, 0.0, 0.9]
heldObj = [0, 0, 0.9]
objLocs = [[0, 0.9, 0, 0],
           [0, 0, 0, 0.9]]

robotLoc = [0, 0, 0, 0.9]
heldObj = [0, 0.9]
objLocs = [[0.0, 0.0, 0.0, 0.9]]
freeLocs = [0.0] * numLocs

goal_state = KitchenState(numObjs, numLocs, stoveLoc,
                          move_info, pick_info, place_info, cook_info, look_info,
                          robotLoc, heldObj, objLocs, freeLocs, world=True)

operator_d =        {'am': {'verbose': 'Moving from loc %d to loc %d'},
                     'api': {'verbose': 'Picking up obj %d from loc %d'},
                     'apl': {'verbose': 'Placing obj %d in loc %d'},
                     'alr': {'verbose': 'Looking for robot in loc %d'},
                     'alh':  {'verbose': 'Looking in hand for obj %d'},
                     'alo':  {'verbose': 'Looking for obj %d in loc %d'}}

sim = Simulator(operator_d, KitchenPlanner(),  KitchenFilter(), KitchenWorld())
sim.run_sim(initial_state, goal_state, verbose=False, slow=False, sims=10)
