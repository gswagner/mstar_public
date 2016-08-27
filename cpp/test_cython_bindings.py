#!/usr/bin/python
"""Test Suite for python bindings"""

import sys
import unittest
import argparse
try:
    import ipdb as pdb
except ImportError:
    import pdb
from test_utils import debug_on, DebugMeta, do_cprofile
import test_utils
import time
from col_set_addition import NoSolutionError, OutOfTimeError
from itertools import izip
import cPickle as pickle


import modular_mstar
import cython_od_mstar


FULL = False


def py_find_path(world, init_pos, goals, inflation, time_limit):
    """Helper function for running modular ODrM*"""
    import time
    planner = modular_mstar.GridMstar(
        world, goals, recursive=True, connect_8=False,
        inflation=inflation, col_set_memory=False,
        end_time=time.time() + time_limit, mode='OD')
    return planner.find_path(init_pos, time_limit)


class TestCythonOdMstar(unittest.TestCase):

    # __metaclass__ = DebugMeta

    """Testing that modular ODrM* is  working"""
    def setUp(self):
        self.world_descriptor = [[0 for i in xrange(10)] for j in xrange(10)]

    def tupleify(self, path):
        return tuple(map(tuple, path))

    def find_path(self, world, init, goal, inflation, time):
        return self.tupleify(cython_od_mstar.find_path(
            world, init, goal, inflation, time))

    def test_op_decomp_single_robot(self):
        """Tests ODM* on simple single robot cases"""
        from mstar_test import validate_solution, compute_cost
        path = self.find_path(self.world_descriptor,
                              ((0, 0), ), ((1, 0), ), 1, 10)
        self.assertTrue(path == (((0, 0), ), ((1, 0), )))
        path = self.find_path(self.world_descriptor,
                              ((0, 0), ), ((1, 1), ), 1, 10)
        self.assertTrue(path == (((0, 0), ), ((1, 0), ), ((1, 1), )) or
                        path == (((0, 0), ), ((0, 1), ), ((1, 1), )))
        self.assertTrue(validate_solution(path))
        self.world_descriptor[1][0] = 1
        path = self.find_path(self.world_descriptor,
                              ((0, 0), ), ((2, 0), ), 1, 10)
        self.assertTrue(path == (((0, 0), ), ((0, 1), ),
                                 ((1, 1), ), ((2, 1), ), ((2, 0), )))

    def test_od_mstar_multirobot_paths(self):
        """Tests some simple multirobot cases on ODrM*"""
        from mstar_test import validate_solution, compute_cost
        path = self.find_path(
            self.world_descriptor, ((0, 0), (1, 0)), ((1, 0), (0, 0)), 1, 10)
        self.assertTrue(validate_solution(path))
        self.assertTrue(compute_cost(path) == 4)
        path = self.find_path(self.world_descriptor,
                              ((0, 0), (1, 0), (5, 5)),
                              ((1, 0), (0, 0), (6, 6)), 1, 10)
        self.assertTrue(validate_solution(path))
        self.assertTrue(compute_cost(path) == 6)
        # Test a simple three robot collision along edge
        path = self.find_path(self.world_descriptor,
                              ((3, 0), (0, 0), (2, 0)),
                              ((0, 0), (3, 0), (5, 0)), 1, 10)
        self.assertTrue(validate_solution(path))

        # Test with some obstacles
        self.world_descriptor[5][0] = 1
        path = self.find_path(
            self.world_descriptor, ((4, 0), (6, 0)), ((6, 0), (4, 0)), 1, 10)
        self.assertTrue(validate_solution(path))
        self.assertTrue(compute_cost(path) == 10)

    def test_no_solution(self):
        """Tests that the planners correctly identify a problem with no
        solution
        """
        world = [[0], [0]]
        with self.assertRaises(NoSolutionError):
            path = self.find_path(
                world, ((0, 0), (1, 0)), ((1, 0), (0, 0)),
                1, 3)

    def test_map_solutions(self):
        """Tests on full maps"""
        from mstar_test import validate_solution, compute_cost
        if not FULL:
            self.skipTest('Skipped map test for brevity')
        dat = pickle.load(open('maps/5_40_bots_step_5.map'))
        # 281 runs in python, not c
        for i in [0, 30, 60, 103, 141, 151, 161, 163, 164, 209, 211, 212, 213,
                  214, 215, 250, 260, 265, 280, 282, 283, 284, 285, 283, 284,
                  285, 286, 287, 291, 293]:
            print i
            d = dat[i]
            path = self.find_path(d['obs_map'], d['init_pos'], d['goals'],
                                  1, 10)
            py_path = py_find_path(d['obs_map'], d['init_pos'], d['goals'],
                                   1, 10)
            self.assertTrue(validate_solution(path))
            self.assertTrue(compute_cost(path) == compute_cost(py_path))

    def test_map_inflation_bounds(self):
        """Tests on full maps"""
        from mstar_test import validate_solution, compute_cost
        if not FULL:
            self.skipTest('Skipped map test for brevity')
        dat = pickle.load(open('maps/5_40_bots_step_5.map'))
        # 281 runs in python, not c
        for i in [0, 30, 60, 103, 141, 151, 161, 163, 164, 209, 211, 212, 213,
                  214, 215, 250, 260, 265, 280, 282, 283, 284, 285, 283, 284,
                  285, 286, 287, 291, 293]:
            print i
            d = dat[i]
            path = self.find_path(d['obs_map'], d['init_pos'], d['goals'],
                                  1.1, 10)
            py_path = py_find_path(d['obs_map'], d['init_pos'], d['goals'],
                                   1, 10)
            self.assertTrue(validate_solution(path))
            self.assertTrue(compute_cost(path) <= 1.1 * compute_cost(py_path))

    def test_inflated_map(self):
        """Tests on full maps"""
        from mstar_test import validate_solution, compute_cost
        if not FULL:
            self.skipTest('Skipped map test for brevity')
        dat = pickle.load(open('maps/5_40_bots_step_5.map'))
        # 281 runs in python, not c
        for i in [0, 30, 60, 103, 141, 151, 161, 163, 164, 209, 211, 212, 213,
                  214, 215, 250, 260, 265, 280, 282, 283, 284, 285, 283, 284,
                  285, 286, 287, 291, 293, 300, 350, 400, 450, 500, 550]:
            print i
            d = dat[i]
            path = self.find_path(d['obs_map'], d['init_pos'], d['goals'],
                                  3, 10)
            py_path = py_find_path(d['obs_map'], d['init_pos'], d['goals'],
                                   3, 10)
            self.assertTrue(validate_solution(path))


def main():
    parser = argparse.ArgumentParser(description='Runs test suite for ' +
                                     'planning with uncertainty')
    parser.add_argument('tests', nargs='*', help='Specific tests to run.  ' +
                        'By default, will run all but the long tests.  ' +
                        'Tests to run are specified by ' +
                        'TestCase.test_function.  You can run all tests in ' +
                        'a given test case by just specifying TestCase' +
                        'Overrides any test specification flags, so will ' +
                        'run the map tests')
    parser.add_argument('-v', dest='verbosity', action='store_const', const=2,
                        default=1, help='verbose output')
    parser.add_argument('--debugoff', dest='debug_off', action='store_false',
                        help='Disables automatic invocation of pdb')
    parser.add_argument('--vis', action='store_true',
                        help='Visualize sequential composition tests')
    parser.add_argument('-f', dest='full', action='store_true',
                        help='Enable all tests, including longer simulations')
    global VISUALIZE
    args = parser.parse_args()
    test_utils.DEBUG = args.debug_off
    VISUALIZE = args.vis
    global FULL
    FULL = args.full
    modules = [sys.modules[__name__]]
    if args.tests:
        # Have a list of tests that we want to run
        for mod in modules:
            try:
                suite = unittest.TestLoader().loadTestsFromNames(
                    args.tests, module=mod)
            except AttributeError:
                # Not in this module
                continue
            break
        else:
            raise ValueError('Specified test does not exist')
        unittest.TextTestRunner(verbosity=args.verbosity).run(suite)
    else:
        # unittest.main tries to use the command line arguments as well,
        # which can cause problems, so just blank out those
        suite = unittest.TestSuite()
        for mod in modules:
            suite.addTests(unittest.TestLoader().loadTestsFromModule(mod))
        unittest.TextTestRunner(verbosity=args.verbosity).run(suite)


if __name__ == '__main__':
    main()
