#!/usr/bin/python

import dist_map
import cPickle
import cython_od_mstar
import modular_mstar
import argparse
import time
import subprocess
import os
import collections
import cms_world_trials


def worker(world):
    start_time = time.clock()
    try:
        path = cython_od_mstar.find_path(world['obs_map'], world['init_pos'],
                                         world['goals'], world['inflation'],
                                         world['time_limit'])
    except Exception as e:
        return e, time.clock() - start_time
    return path, time.clock() - start_time


def test_func(world):
    """Minor hack around the fact you cannot use lambda closures in
    distmap
    """
    return cms_world_trials.test_worker(world, worker)


def run_cpp_mstar_trial(map_file, out_file, inflation=1, time_limit=60,
                        num_processors=2,
                        hosts=('cobra', 'python', 'anaconda', 'viper')):
    cms_world_trials.run_trial(map_file, out_file, test_func,
                               inflation=inflation,
                               time_limit=time_limit,
                               num_processors=num_processors,
                               hosts=hosts, dep_functions=(worker, test_func),
                               modules=('col_set_addition', 'cms_world_trials',
                                        'dist_map', 'time', 'os',
                                        'cython_od_mstar'))


def main():
    """Runs tests on specified world"""
    parser = argparse.ArgumentParser(
        description='Runs test for C++ implementation of M*')
    parser.add_argument('test_file', help='File describing test cases')
    parser.add_argument('output_file', help='Name of output file')
    parser.add_argument('num_processors', type=int, action='store',
                        help='Number of processes to run on each node. ' +
                        'The local host running the primary server will ' +
                        'run one fewer worker processes')
    parser.add_argument('-i', action='store', type=float, default=1.0,
                        help='Set inflation factor for the heuristic, ' +
                        'defaults to 1', metavar='INF', dest='inflation')
    parser.add_argument('-t', action='store', type=int, default=120,
                        help='Set time limit for planning.  Defaults to 2 ' +
                        'minutes', dest='time_limit')
    parser.add_argument('--hosts', action='store',
                        default=('python', 'cobra', 'viper', 'anaconda'),
                        help='Hostnames/IPs to use as processing nodes.',
                        nargs='*', metavar='HOSTNAME')

    args = parser.parse_args()

    run_cpp_mstar_trial(args.test_file, args.output_file,
                        inflation=args.inflation, time_limit=args.time_limit,
                        hosts=args.hosts, num_processors=args.num_processors)

if __name__ == '__main__':
    main()
