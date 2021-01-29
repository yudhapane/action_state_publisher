#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from task_planner.msg import Action

import argparse
import os, time, sys

import color
import planner
import pddlparser
import pddl
import dot_plan
import dot_ma_plan 
from json_ma_plan_iros21 import make_plan

def parse():
    usage = 'python3 main.py <PROBLEM> [-v N]'
    description = "this is an interface to run the planner on the solenoid domain."
    parser = argparse.ArgumentParser(usage=usage, description=description)

    parser.add_argument('problem', type=str, help='name of a PDDL problem file', 
        choices=("prob1_1", "prob2_1", "prob3_1", "prob4_1", "prob1_0", "prob2_0", "prob3_0", "prob4_0"))
    parser.add_argument("-v", "--verbose", default=0, type=int, choices=(0, 1, 2),
        help="increase output verbosity: 0 (minimal), 1 (high-level), 2 (classical planners outputs) (default=0)", )

    return parser.parse_args()


#############################
#############################

# def callback(action_msg, (problem, plan, state, goals, verbose)):
def callback(action_msg):

    global problem, plan, state, goals

    rospy.loginfo(rospy.get_caller_id() + 'I heard %s %s %s', action_msg.id, action_msg.succeed, action_msg.monitors)

    #############################
    ## simulate and execute the plan
    for level, step in plan.items():

        if step == 'GOAL':
            # goal state is achieved
            print(color.fg_voilet('@ GOAL'))
            sub_proc.unregister()
            rospy.signal_shutdown('finished')
            return

        else:
            # unfold step into a tuple of actoins and outcomes
            (actions, outcomes) = step

            for action in actions:
                ## print out some info
                print(color.fg_yellow(' + ') + str(action))
                state = state.apply(action)

                ## find an action matching the received action
                if '_'.join(action.sig) == action_msg.id:
                    if action_msg.succeed:
                        # apply action to the state and update the state
                        print(action_msg.succeed)
                    else:
                        # update the state with the action violence and make a re-plane

                        ## convert back the predicates frozenset to a list and update the state
                        ## i.e., remove ('arm_canreach', 'robot', 'object')
                        state_predicates = list(state.predicates)
                        # state_predicates.remove(('arm_canreach', action.sig[1:], action.sig[1:]))
                        # state_predicates.append(('blocked', action.sig[1:]))
                        state.predicates = frozenset(state_predicates)

                        print(action_msg.monitors)
                        print(color.fg_yellow('action failed'))
                    break

            # inner for-loop finished with no break
            else:
                continue
            # inner for-loop was broken, then break outer for-loop too
            break

    # ## check if goal is achieved 
    # if state.is_true(goals):
    #     sub_proc.unregister()
    #     rospy.signal_shutdown('finished')
    #     print(color.fg_yellow('@ GOAL'))
    #     return

    # for-loop was broken due to execution failure, then make a new plan and continue
    #############################
    ## create a new pddl problem file at /tmp/safe-planner
    problem_pddl = pddl.pddl(problem, state=state, goal=goals)

    ## set the default solenoid domain parameters
    ## call planner to make an initial policy given the domain, problem and planners
    (policy, plan, plan_json_file, actions_json_file) = \
        make_plan(domain = domain, \
                  problem = problem_pddl, \
                  planners = planners, \
                  agents = agents, \
                  temporal_actions = temporal_actions, \
                  rank=False, \
                  verbose=verbose)

    #############################
    ## call the execution engine giving the initial plan
    # plan_json_file actions_json_file

    return


#############################
#############################
#############################

if __name__ == '__main__':

    # change to the task planner directory
    os.chdir(os.path.split(__file__)[0])

    args = parse()

    global domain, problem, planners, agents, temporal_actions, verbose, plan, state, goals

    ## parse domain and create a domain object
    problem = pddlparser.PDDLParser.parse("benchmarks/multirob/solenoid/" + args.problem + ".pddl")

    ## keep track of the current state and the current goal to achieve
    ## in any geometric limitation, the goal is updated to recover from 
    ## the failure situation (we do not plan again to the original goal)
    state = problem.initial_state
    goals = problem.goals

    ## create a new pddl problem file at /tmp/safe-planner
    problem_pddl = pddl.pddl(problem, state=state, goal=goals)

    ## set the default solenoid domain parameters
    domain = "benchmarks/multirob/solenoid/domain.pddl"
    planners = ["optic-clp"]
    agents = ("left_arm", "right_arm")
    temporal_actions = ("avoid_collision", "admittance_control")
    verbose = args.verbose

    #############################
    ## set the default solenoid domain parameters
    ## call planner to make an initial policy given the domain, problem and planners
    # (policy, plan, plan_json_file, actions_json_file) = \
    #     make_plan(domain = "benchmarks/multirob/solenoid/domain.pddl", \
    #               problem = problem_pddl, \
    #               planners = ["optic-clp"], \
    #               agents = ("left_arm", "right_arm"), \
    #               temporal_actions = ("avoid_collision", "admittance_control"), \
    #               rank=False, \
    #               verbose=args.verbose)
    (policy, plan, plan_json_file, actions_json_file) = \
        make_plan(domain = domain, \
                  problem = problem_pddl, \
                  planners = planners, \
                  agents = agents, \
                  temporal_actions = temporal_actions, \
                  rank=False, \
                  verbose=verbose)

    #############################
    ## call the execution engine giving the initial plan
    # plan_json_file actions_json_file

    #############################
    ## listen and follow the execution engine 
    # progress_plan(policy, plan)

    rospy.init_node('tamp_interface', anonymous=True)

    global sub_proc
    sub_proc = rospy.Subscriber("chatter", Action, callback)
    # sub_proc = rospy.Subscriber("chatter", Action, callback, (problem, plan, state, goals, args.verbose))

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    print('shutdown')

    # exit()

    # #############################
    # ## while goal is not achieved 
    # while True:

    #     #############################
    #     ## wait for a message from the execution engine
    #     action_msg = rospy.wait_for_message("chatter", Action)
    #     # rospy.loginfo(rospy.get_caller_id() + 'I heard %s %s %s', action_msg.id, action_msg.succeed, action_msg.monitors)

    #     #############################
    #     ## simulate and execute the plan
    #     for level, step in plan.items():

    #         if step == 'GOAL':
    #             # goal state is achieved
    #             print(color.fg_voilet('@ GOAL'))
    #             break

    #         else:
    #             # unfold step into a tuple of actoins and outcomes
    #             (actions, outcomes) = step

    #             for action in actions:
    #                 ## print out some info
    #                 print(color.fg_yellow(' + ') + str(action))
    #                 state = state.apply(action)

    #                 ## find an action matching the received action
    #                 if '_'.join(action.sig) == action_msg.id:
    #                     if action_msg.succeed:
    #                         # apply action to the state and update the state
    #                         print(action_msg.succeed)
    #                     else:
    #                         # update the state with the action violence and make a re-plane

    #                         ## convert back the predicates frozenset to a list and update the state
    #                         ## i.e., remove ('arm_canreach', 'robot', 'object')
    #                         state_predicates = list(state.predicates)
    #                         # state_predicates.remove(('arm_canreach', action.sig[1:], action.sig[1:]))
    #                         # state_predicates.append(('blocked', action.sig[1:]))
    #                         state.predicates = frozenset(state_predicates)

    #                         print(action_msg.monitors)
    #                         print(color.fg_yellow('action failed'))
    #                         break

    #             # inner for-loop finished with no break
    #             else:
    #                 continue
    #             # inner for-loop was broken, then break outer for-loop too
    #             break

    #     ## check if goal is achieved 
    #     if state.is_true(problem.goals):
    #         break

    #     # for-loop was broken due to execution failure, then make a new plan and continue
    #     #############################
    #     ## create a new pddl problem file at /tmp/safe-planner
    #     problem_pddl = pddl.pddl(problem, state=state, goal=goals)

    #     ## set the default solenoid domain parameters
    #     ## call planner to make an initial policy given the domain, problem and planners
    #     (policy, plan, plan_json_file, actions_json_file) = \
    #         make_plan(domain = "benchmarks/multirob/solenoid/domain.pddl", \
    #                   problem = problem_pddl, \
    #                   planners = ["optic-clp"], \
    #                   agents = ("left_arm", "right_arm"), \
    #                   temporal_actions = ("avoid_collision", "admittance_control"), \
    #                   rank=False, \
    #                   verbose=args.verbose)

    # print('shutdown')
