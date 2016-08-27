#!/usr/bin/python

import os.path
import argparse
from itertools import izip
import multiprocessing
import cpp_world_trials

MAP_FOLDER = 'maps/'
OUTPUT_FOLDER = 'cpp_trials/'
TIME_LIMIT = 60
NUM_PROCS = 1

# Specify trials to run
# [map_file, inflation]
TRIALS = [['5_40_bots_step_5.map', 3],
          ['45_60_bots_8con.map', 3],
          ['70_100_8con.map', 3],
          ['110_150_bots_8con.map', 3],
          ['160_200_robots.map', 3],
          ['5_40_bots_step_5.map', 1.1],
          ['45_60_bots_8con.map', 1.1],
          ['5_40_bots_step_5.map', 1],
          ]


def main():
    parser = argparse.ArgumentParser(
        'Runs a hardcoded batch of trials.  Only generates missing results')
    parser.add_argument('--hosts', action='store',
                        default=('python', 'cobra', 'viper', 'anaconda'),
                        help='Hostnames/IPs to use in processing nodes',
                        nargs='*', metavar='HOSTNAME')
    parser.add_argument('--num_processors', type=int, action='store',
                        default=NUM_PROCS,
                        help='Number of proceses to run on each node.\n' +
                        'the local host running the primary server will ' +
                        'run one fewer processes, if the specified number ' +
                        'of processors is greater than 2')
    args = parser.parse_args()

    file_names = [OUTPUT_FOLDER + _to_output_file(map_file, inflation)
                  for map_file, inflation in TRIALS]

    for trial, outfile in izip(TRIALS, file_names):
        if os.path.isfile(outfile):
            print '{0} already completed'.format(outfile)
            continue
        print 'processing {0}'.format(outfile)
        proc = multiprocessing.Process(
            target=cpp_world_trials.run_cpp_mstar_trial,
            args=(MAP_FOLDER + trial[0], outfile),
            kwargs={'time_limit': TIME_LIMIT, 'inflation': trial[1],
                    'hosts': args.hosts,
                    'num_processors': args.num_processors})
        proc.start()
        proc.join()


def _to_output_file(map_file, inflation):
    alg = 'cppODrMstar'
    return '{0}_{1}_{2}_inf.out'.format(map_file.split('.')[0], alg,
                                        inflation)


if __name__ == '__main__':
    main()
