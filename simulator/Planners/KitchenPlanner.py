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
from Primitives.KitchenPrimitives import KitchenState, KitchenAction

import subprocess, threading, time, Queue
            
class KitchenPlanner(Planner):
    def __init__(self):
        self.path = '/Users/ciara/Dropbox/Ciara/LIS/planner'
        self.plan = None

    def write_input_file(self, initial_state, goal_state):
        f = open(self.path + '/temp.txt', 'w')
        f.write('%d\n' % initial_state.numLocs)
        f.write('%d\n' % initial_state.numObjs)
        f.write('%.2f\n' % initial_state.move_prob)
        f.write('%.2f\n' % initial_state.pick_prob)
        f.write('%.2f\n' % initial_state.place_prob)
        f.write('%.2f\n' % initial_state.look_obs)
        for x in initial_state.robotLoc:
            f.write('%.2f\n' % x)
        for x in initial_state.heldObj:
            f.write('%.2f\n' % x)
        for locs in initial_state.objLocs:
            for x in locs:
                f.write('%.2f\n' % x)
        for x in initial_state.freeLocs:
            f.write('%.2f\n' % x)
        for x in goal_state.robotLoc:
            f.write('%.2f\n' % x)
        for x in goal_state.heldObj:
            f.write('%.2f\n' % x)
        for locs in goal_state.objLocs:
            for x in locs:
                f.write('%.2f\n' % x)
        for x in goal_state.freeLocs:
            f.write('%.2f\n' % x)
        f.close()

    def run_cmd(self, timeout, weight):
        q = Queue.Queue()
        def target(q):
            input_file = open('%s/temp.txt' % self.path)
            #print 'waiting to replan'
            #raw_input()
            #print 'replanning with w=%.2f' % weight
            self.process = subprocess.Popen(['%s/main' % self.path,
                                  '--file=true',
                                  '--problem=kitchen',
                                  '--weight=' + str(weight),
                                  '--epsilon=0.0'],
                                 stdin=input_file,
                                 stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            out, err = self.process.communicate()
            q.put(out)

        thread = threading.Thread(target=target, args=[q])
        start_time = time.time()
        thread.start()
        thread.join(timeout)

        if thread.is_alive():
            self.process.terminate()
            thread.join()
            print "killed"
            return False
        else:
            #print 'elapsed time: %.2f' % (time.time() - start_time)
            self.parse_plan(q.get())
            return True


    def parse_plan(self, out):
        try:
            plan = out.split('\n')
            self.plan = []
            for i in range(len(plan)):
                if plan[i][0:4] == 'move':
                    args = plan[i][5:].split('_')
                    self.plan.append(KitchenAction('am', [int(arg) for arg in args]))
                elif plan[i][0:4] == 'pick':
                    args = plan[i][5:].split('_')
                    self.plan.append(KitchenAction('api', [int(arg) for arg in args]))
                elif plan[i][0:5] == 'place':
                    args = plan[i][6:].split('_')
                    self.plan.append(KitchenAction('apl', [int(arg) for arg in args]))
                elif plan[i][0:10] == 'look_robot':
                    args = plan[i][11:].split('_')
                    self.plan.append(KitchenAction('alr', [int(arg) for arg in args]))
                elif plan[i][0:9] == 'look_hand':
                    args = plan[i][10:].split('_')
                    self.plan.append(KitchenAction('alh', [int(arg) for arg in args]))
                elif plan[i][0:8] == 'look_obj':
                    args = plan[i][9:].split('_')
                    self.plan.append(KitchenAction('alo', [int(arg) for arg in args]))
            self.plan.append(KitchenAction('arg'))
        except:
            print "parse_plan exception"
            print out
        #for a in self.plan:
            #print a

    def next_action(self, initial_state, goal_state, prev_obs):
        replanned = False
        # replan
        if self.plan is None or prev_obs is None or prev_obs == 'onone':
            replanned = True
            self.write_input_file(initial_state, goal_state)

            success = False

            #weight = 0.15
            #success = self.run_cmd(10, 0.15)
            #if not success:
                #weight = 0.0
                #success = self.run_cmd(600, 0.0)
            success = self.run_cmd(600, 0.35)
            if not success:
                print "could not make a plan in 10 minutes"
                assert(False)

            #while (success and weight < 0.35):
                #success = self.run_cmd(10, weight + 0.05)
                #weight += 0.05

            #print "done replanning"
        return (self.plan.pop(0), 1 if replanned else 0)

