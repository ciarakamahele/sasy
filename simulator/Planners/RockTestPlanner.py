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
from Primitives.RockPrimitives import RockAction

class RockTestPlanner(Planner):
    def __init__(self, discount):
        self.plan = None
        self.discount = discount

    def next_action(self, initial_state, goal_state, prev_obs):
        if self.plan is None or len(self.plan) == 0:
            self.plan = [RockAction('amn'),
                        RockAction('ac', (0,)),
                        RockAction('as'),
                        RockAction('ame'),
                        RockAction('ams'),
                        RockAction('ac', (1,)),
                        RockAction('ame'),
                        RockAction('arg')]
        return self.plan.pop(0)
