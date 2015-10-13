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

# Simulator is a generic class that can simulate for every correclty implemented problem type
import os, random, math, time

class Simulator:
    def __init__(self, operator_d, planner, filter, world):
        self.operator_d = operator_d
        self.planner = planner
        self.filter = filter
        self.world = world
        self.belief_state = None

    def run_sim(self, initial_state, goal_state, discount=0.95, sims=10, steps=20, verbose=False, slow=False, very_slow=False):
        total_reward = 0.0
        total_time = 0
        successful_sims = 0
        discount_rewards = [0] * sims
        sim_rewards = [0] * sims
        planning_times = [0] * sims
        replans = [0] * sims

        for sim in range(sims):
            start_time = time.time()
            self.belief_state = initial_state.copy()
            self.world.generate_new_world(self.belief_state)

            if verbose:
                print 'Goal: %s\nInitial belief: %s\nInitial world: %s\n' \
                      % (goal_state, self.belief_state, self.world.state)

            sim_reward = 0.0
            discount_reward = 0.0
            planning_time = 0.0
            replan = 0
            step = 0
            obs = None
            for step in range(steps):
                start_time = time.time()
                action, replanned = self.planner.next_action(self.belief_state, goal_state, obs)
                replan += replanned
                planning_time += time.time() - start_time

                # stop if planner believes we have reached goal state
                if action.action == 'arg':
                    break

                if verbose:
                    print 'Step %d' % (step + 1)
                    print self.operator_d[action.action]['verbose'] % tuple(action.args)

                obs, cost, obs_success = self.world.execute_action(action)
                #print "actuator/sensor was %s" % ("successful" if obs_success else "unsucessful")

                discount_reward -= 1 * discount ** (step + 1)
                sim_reward -= 1
                #print "Cost: %d" % cost
                #print "Sim reward: %d" % sim_reward

                #print "Observed: %s" % obs

                self.belief_state = self.filter.update_belief(self.belief_state, action, obs)
                
                if verbose:
                    print 'New belief: %s\nNew world: %s\nGoal: %s' \
                          % (self.belief_state, self.world.state, goal_state)

                #if very_slow:
                    #raw_input()
                    #pass
                #else:
                    #print ''

            if self.world.success(goal_state):
                #print 'Successful :)'
                successful_sims += 1
                discount_reward += 100 * discount ** (step + 1)
                sim_reward += 100
            #else:
                #print 'Unsuccessful :('

            total_reward += sim_reward

            #print 'Simulation %d: discount reward %.2f, sim reward %.2f' \
                  #% (sim + 1, discount_reward, sim_reward)
            discount_rewards[sim] = discount_reward
            sim_rewards[sim] = sim_reward
            planning_times[sim] = planning_time
            replans[sim] = replan

            elapsed_time = time.time() - start_time
            total_time += elapsed_time
            #print 'time: %d, avg time: %d' \
                  #% (elapsed_time, total_time / (sim + 1))

            #if slow:
                #raw_input()
            #else:
                #print ''

        print 'Average reward over %d simulations: %.2f' % (sims, total_reward / sims)
        print 'Average time over %d simulations: %.2f' % (sims, total_time / sims)
        print '%d/%d simulations successful' % (successful_sims, sims)
        print 'sim_rewards = %s' % str(sim_rewards)
        print 'discount_rewards = %s' % str(discount_rewards)
        print 'planning_times = %s' % str(planning_times)
        print 'replans = %s' % str(replans)
