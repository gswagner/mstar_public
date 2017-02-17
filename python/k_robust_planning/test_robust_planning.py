#!/usr/bin/python

import unittest
import argparse
import pickle
import time
import sys

import workspace_graph
import robust_policy

import test_utils
from test_utils import debug_on, DebugMeta, DEBUG, do_cprofile


class TestRobustPolicy(unittest.TestCase):

    __metaclass__ = DebugMeta

    def build_policy(self, world_descriptor, goal, k):
        return robust_policy.KRobustPolicy(
            workspace_graph.Astar_Graph(
                world_descriptor, goal=goal, connect_8=False,
                diagonal_cost=False, makespan=False),
            k)

    def test_k_1(self):
        """Tests performance with single step history"""
        world = [[0 for i in xrange(10)] for j in xrange(10)]
        k = 1
        policy = self.build_policy(world, (0, 0), k)
        self.assertEqual(policy.get_step(((0, 0), )), ((0, 0), ))
        self.assertEqual(policy.get_cost(((0, 0), )), 0)

        self.assertEqual(set(policy.get_offsets(((0, 0), ))),
                         set([0, 2]))
        
        self.assertEqual(set(policy.get_offset_neighbors(((0, 0), ), 0)),
                         set([(0, ((0, 0), ))]))
        self.assertEqual(set(policy.get_offset_neighbors(((0, 0), ), 2)),
                         set([(2, ((1, 0), )), (2, ((0, 1), ))]))

    def test_k_2(self):
        """Tests performance with two step history"""
        world = [[0 for i in xrange(10)] for j in xrange(10)]
        k = 2
        policy = self.build_policy(world, (0, 0), k)

        self.assertEqual(policy.get_step(((0, 0), (0, 0))), ((0, 0), (0, 0)))
        self.assertEqual(policy.get_step(((1, 0), (2, 0))), ((0, 0), (1, 0)))

        self.assertEqual(policy.get_cost(((1, 0), (0, 0))), 1)

        self.assertEqual(set(policy.get_neighbors(((0, 0), (1, 0)))),
                         set([((1, 0), (0, 0)), ((0, 1), (0, 0)),
                              ((0, 0), (0, 0))]))
        

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
    test_utils.DEBUG = args.debug_on
    sys.argv = ['mstar_test.py']
    if args.tests:
        # Have a list of tests that we want to run
        suite = unittest.TestLoader().loadTestsFromNames(
            args.tests, module=sys.modules[__name__])
        unittest.TextTestRunner(verbosity=args.verbosity).run(suite)
    else:
        unittest.main(verbosity=args.verbosity)

if __name__ == '__main__':
    main()
