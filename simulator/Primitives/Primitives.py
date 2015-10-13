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

# State, Action, and World are interfaces which should be defined for each specific problem type
import math

class State:
    def __init__(self):
        pass

    def assert_valid(self):
        pass

    def copy(self):
        pass

    def equals(self, other_state):
        pass

    def __str__(self):
        pass

class Action:
    def __init__(self, action, args=[]):
        pass

    def __str__(self):
        pass

class World:
    def __init__(self):
        pass

    def generate_new_world(self, belief_state):
        pass

    def execute_action(self, action):
        pass

    def success(self, goal_state):
        pass
