#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from task_planner.msg import Plan
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
    parser.add_argument("-a", "--agents", nargs='+', type=str, default=["left_arm", "right_arm"], 
        help="a list of agents: e.g., -a left_arm right_arm")
    parser.add_argument("-c", "--planners", nargs='+', type=str, default=["ff"], 
        choices=os.listdir('planners'), metavar='PLNNER', 
        help="a list of classical planners: ff, fd, m, prob, optic-clp, lpg-td, lpg, vhpop (e.g. -c ff fd m) (default=[optic-clp])")
    parser.add_argument("-t", "--temporal_actions", nargs='+', type=str, default=["avoid_collision", "admittance_control"], 
        help="a list of temporal (durative) actions: e.g., -t avoid_collision admittance_control")
    parser.add_argument("-r", "--rank", help="to disable ranking the compiled classical planning domains \
        by higher probabilistic outcomes (default=True)", action="store_true", default=False)
    parser.add_argument("-v", "--verbose", default=0, type=int, choices=(0, 1, 2),
        help="increase output verbosity: 0 (minimal), 1 (high-level), 2 (classical planners outputs) (default=0)", )

    return parser.parse_args()


#############################
#############################

def send_json_plan(plan_json_file, actions_json_file):

    # a list keeping the ids of successfully executed actions
    global action_ids
    action_ids = []

    plan_json_file = os.path.splitext(os.path.splitext(plan_json_file)[0])[0]
    pub = rospy.Publisher('plan', String, queue_size=10, latch=True)
    rospy.init_node('tamp_interface', anonymous=True)
    pub.publish(plan_json_file)
    print(color.fg_yellow('\n - path to the json files: ') + '{}'.format(plan_json_file))

    # pub = rospy.Publisher('plan', Plan, queue_size=10)
    # rospy.init_node('tamp_interface', anonymous=True)
    # plan = Plan()
    # plan.plan_json_file = plan_json_file
    # plan.actions_json_file = actions_json_file
    # pub.publish(plan)
    # print(color.fg_yellow('\n - plan json files: ') + '{}\n\t\t    {}'.format(plan_json_file, actions_json_file))
    return


#############################
#############################

def action_execution_verification(action_msg):

    global args, domain, problem, state, plan, action_ids

    #############################
    ## simulate and execute the plan
    for level, step in plan.items():

        ## check if goal is achieved 
        if state.is_true(problem.goals) or step == 'GOAL':
            # goal state is achieved
            print(color.fg_voilet('@ GOAL'))
            sub_proc.unregister()
            rospy.signal_shutdown('finished')
            return

        else:
            # unfold step into a tuple of actions and outcomes
            (actions, outcomes) = step

            for action in actions:
                ## find an action matching the received action
                if '_'.join(action.sig) == action_msg.id:

                    # if action was already visited
                    if '_'.join(action.sig) in action_ids: return

                    # add action's id to the visited action_ids
                    action_ids.append('_'.join(action.sig))

                    # check if action succeeded
                    if action_msg.succeed:
                        ## print out succeeded action
                        print(color.fg_yellow(' + ') + str(action))
                        # apply action to the state and update the state
                        state = state.apply(action)
                    else:
                        ## print out failed action
                        print(color.fg_red(' - ') + str(action))
                        for monitor in action_msg.monitors:
                            print(color.fg_red('   ---- ') + '{} {}'.format(monitor.predicate, monitor.arguments[0]))

                        ## update the state with the action violence and make a re-plane
                        ## convert the predicates frozenset to a list and update the state
                        ## i.e., remove ('collision_free', 'left_arm')
                        state_predicates = list(state.predicates)

                        for monitor in action_msg.monitors:
                            if 'collision' in monitor.predicate:
                                if ('collision_free', monitor.arguments[0]) in state_predicates:
                                    state_predicates.remove(('collision_free', monitor.arguments[0]))
                                if not ('collision_detected', monitor.arguments[0]) in state_predicates:
                                    state_predicates.append(('collision_detected', monitor.arguments[0]))
                            elif 'admittance' in monitor.predicate:
                                if ('admittance_free', monitor.arguments[0]) in state_predicates:
                                    state_predicates.remove(('admittance_free', monitor.arguments[0]))
                                if not ('admittance_detected', monitor.arguments[0]) in state_predicates:
                                    state_predicates.append(('admittance_detected', monitor.arguments[0]))

                        ## convert back to frozenset
                        state.predicates = frozenset(state_predicates)

                        #############################
                        ## create a new pddl problem file at /tmp/safe-planner
                        problem_pddl = pddl.pddl(problem, state=state)

                        ## call planner to make an initial policy given the domain, problem and planners
                        (policy, plan, plan_json_file, actions_json_file) = \
                            make_plan(domain = domain, \
                                      problem = problem_pddl, \
                                      planners = args.planners, \
                                      agents = args.agents, \
                                      temporal_actions = args.temporal_actions, \
                                      rank=False, \
                                      verbose=args.verbose)

                        #############################
                        ## call the execution engine giving the initial plan
                        # plan_json_file actions_json_file
                        send_json_plan(plan_json_file, actions_json_file)

                    return


#############################
#############################
#############################

if __name__ == '__main__':

    # change to the task planner directory
    os.chdir(os.path.split(__file__)[0])

    global args, domain, problem, state, plan, action_ids

    args = parse()

    ## parse domain and create a domain object
    problem = pddlparser.PDDLParser.parse("benchmarks/multirob/solenoid/" + args.problem + ".pddl")

    ## keep track of the current state and the current goal to achieve
    ## in any geometric limitation, the goal is updated to recover from 
    ## the failure situation (we do not plan again to the original goal)
    state = problem.initial_state

    ## create a new pddl problem file at /tmp/safe-planner
    problem_pddl = pddl.pddl(problem, state=state)

    ## set the default solenoid domain 
    domain = "benchmarks/multirob/solenoid/domain.pddl"

    #############################
    ## set the default solenoid domain parameters
    ## call planner to make an initial policy given the domain, problem and planners
    (policy, plan, plan_json_file, actions_json_file) = \
        make_plan(domain = domain, \
                  problem = problem_pddl, \
                  planners = args.planners, \
                  agents = args.agents, \
                  temporal_actions = args.temporal_actions, \
                  rank=False, \
                  verbose=args.verbose)

    #############################
    ## call the execution engine giving the initial plan
    send_json_plan(plan_json_file, actions_json_file)

    #############################
    ## listen and follow the execution engine 
    # progress_plan(policy, plan)

    rospy.init_node('tamp_interface', anonymous=True)

    global sub_proc
    sub_proc = rospy.Subscriber("action_status", Action, action_execution_verification)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

    print('shutdown')
