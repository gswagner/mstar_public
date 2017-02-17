#!/usr/bin/python

import unittest
import argparse
import pickle
import time
import sys

import workspace_graph
import robust_policy

from test_utils import debug_on, DebugMeta, DEBUG, do_cprofile

class TestRobustPolicy(unittest.TestCase):

    def build_policy(self, world_descriptor, goal, k):
        return robust_policy.KRobustPolicy(
            workspace_graph.Astar_Graph(
                world_descriptor, goal=goal, connect_8=False,
                diagonal_cost=False, makespan=False),
            k)

    def test_simple(self):
        world = [[0 for i in xrange(10)] for j in xrange(10)]
        k = 1
        policy = self.build_policy(world, (0, 0), k)
        self.assertEqual(policy.get_step(((0, 0), )), ((0, 0), ))
        

def main():
    parser = argparse.ArgumentParser(description='Runs test suite for mstar')
    parser.add_argument('tests', nargs='*', help='Specific tests to run.  ' +
                        'By default, will run all but the long tests.  ' +
                        'Tests to run are specified by ' +
                        'TestCase.test_function.  You can run all tests in ' +
                        'a given test case by just specifying TestCase' +
                        'Overrides any test specification flags, so will ' +
                        'run the map tests')
    parser.add_argument('-v', dest='verbosity', action='store_const', const=2,
                        default=1, help='verbose output')
    # parser.add_argument('-f', dest='full_test', action='store_true',
    #                     help='Full test, including map tests')
    parser.add_argument('--debugon', dest='debug_on', action='store_true',
                        help='Turns on invoking ipdb')
    args = parser.parse_args()
    global DEBUG
    DEBUG = args.debug_on
    if args.tests:
        # Have a list of tests that we want to run
        suite = unittest.TestLoader().loadTestsFromNames(
            args.tests, module=sys.modules[__name__])
        unittest.TextTestRunner(verbosity=args.verbosity).run(suite)
    else:
        unittest.main(verbosity=args.verbosity)

if __name__ == '__main__':
    main()
