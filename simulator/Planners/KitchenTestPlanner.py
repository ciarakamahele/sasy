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

from Planner import Planner
from Primitives.KitchenPrimitives import KitchenAction

class KitchenTestPlanner(Planner):
    def __init__(self):
        self.plan = None

    def next_action(self, initial_state, goal_state, prev_obs):
        if self.plan is None or len(self.plan) == 0 or prev_obs is None:
            self.plan = [KitchenAction('am', [0, 1]),
                         KitchenAction('am', [1, 2]),
                         KitchenAction('api', [0, 2]),
                         KitchenAction('am', [2, 1]),
                         KitchenAction('apl', [0, 1]),
                         KitchenAction('am', [1, 2])]
        return self.plan.pop(0)
