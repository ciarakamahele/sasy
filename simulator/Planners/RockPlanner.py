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
from Primitives.RockPrimitives import RockState, RockAction

import subprocess

class RockPlanner(Planner):
    def __init__(self, discount):
        self.path = '/Users/ciara/Dropbox/MIT/LIS/ciara-planner'
        self.plan = None
        self.discount = discount

    def next_action(self, initial_state, goal_state, prev_obs):
        # replan
        if self.plan is None or prev_obs is None or prev_obs == 'obad':
            print 'replanning'
            f = open(self.path + '/temp.txt', 'w')
            f.write('%d\n' % initial_state.numLocs)
            f.write('%d\n' % initial_state.numRocks)
            for x in initial_state.rockLocs:
                f.write('%d\n' % x)
            for x in initial_state.rockQuality:
                f.write('%.2f\n' % x)
            for x in initial_state.rockSampled:
                f.write('%d\n' % x)
            f.write('%d\n' % initial_state.robotX)
            f.write('%d\n' % initial_state.robotY)
            f.close()
            
            input_file = open('%s/temp.txt' % self.path)
            p = subprocess.Popen(['%s/main' % self.path,
                                  '--file=true',
                                  '--problem=rocksample',
                                  '--discount=%0.5f' % self.discount],
                                 stdin=input_file,
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            out, err = p.communicate()
            try:
                plan = out.split('\n')
                self.plan = []
                for i in range(len(plan)):
                    if plan[i] == 'north':
                        self.plan.append(RockAction('amn'))
                    elif plan[i] == 'south':
                        self.plan.append(RockAction('ams'))
                    elif plan[i] == 'east':
                        self.plan.append(RockAction('ame'))
                    elif plan[i] == 'west':
                        self.plan.append(RockAction('amw'))
                    elif plan[i] == 'sample':
                        self.plan.append(RockAction('as'))
                    elif plan[i][0:5] == 'check':
                        self.plan.append(RockAction('ac', (int(plan[i][6:]),)))
                self.plan.append(RockAction('arg'))
            except:
                print out
                raw_input()
        return self.plan.pop(0)
