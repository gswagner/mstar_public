#!/usr/bin/python
#This file is purely intended to generate the world arguments, al la
#plan_tester.py and store them in a pickled folder, so you can run multiple
#trials on the same data set.  This now more important, because I am starting
#to compare slightly different versions of mStar, where natural variations in
#the number of failures look to be swamping the actual signal, at least without
#further increasing the number of robots
import multiprocessing
import pickle
import random
import sys
import workspace_graph
import argparse
from Queue import Empty

NUM_TRIALS = 100
PER_BOT = 104
COVERAGE = .35
FIXED_SIZE = 32


def gen_coverage_world(size, num_bots, obs_density, connect_8=False):
    """Generates a random, square world of size cells per side.

    size        - cells per side
    num_bots    - number of robots
    obs_density - liklihood of given cell being an obstacle
    connect_8   - whether to be built for 8 connected graph instead of 4
                  connected
    returns     - [obs_map, init_pos, goals]"""
    print 'gen_coverage'
    if obs_density > 1 or obs_density < 0:
        raise ValueError('Improper obstacle obs_density')

    def foo():
        if random.uniform(0, 1) < obs_density:
            return 1
        return 0

    obs_map = [[foo() for i in xrange(size)] for j in xrange(size)]
    #Generate the initial positions of the robots
    init_pos = []
    goals = []
    # Keeps track of how many times tried to generate valid initial,
    # goal positions
    counter = 0
    #Number of times to try for a given obstacle arrangement
    max_tries = 100 * num_bots
    while len(init_pos) < num_bots:
        #Generate possible initial and goal positions
        ip = (random.randint(0, size - 1), random.randint(0, size - 1))
        g = (random.randint(0, size - 1), random.randint(0, size - 1))
        if g == ip or obs_map[g[0]][g[1]] == 1 or obs_map[ip[0]][ip[1]] == 1:
            # Don't allow the goal and initial positions to be in the
            # same spot or in an obstacle
            if counter > max_tries:
                #obs_map is too hard, try a new one
                init_pos = []
                goals = []
                obs_map = [[foo() for i in range(size)] for j in range(size)]
                counter = 0
                continue
            counter += 1
            continue
        if g in goals or ip in init_pos:
            #Don't allow duplicate elements
            if counter > max_tries:
                #obs_map is too hard, try a new one
                init_pos = []
                goals = []
                obs_map = [[foo() for i in range(size)] for j in range(size)]
                counter = 0
                continue
            counter += 1
            continue
        #Check if we can generate a valid path
        graph = workspace_graph.Astar_Graph(obs_map, g, connect_8=connect_8)
        tpath = graph.get_step(ip)
        if tpath == None or tpath == []:
            #No individual path, so skip
            if counter > max_tries:
                #obs_map is too hard, try a new one
                init_pos = []
                goals = []
                obs_map = [[foo() for i in range(size)] for j in range(size)]
                counter = 0
                continue
            counter += 1
            continue
        #initial position, goal pair is valid
        init_pos.append(ip)
        goals.append(g)
    return obs_map, tuple(init_pos), tuple(goals)


def world_wrapper(args):
    """Wraps gen_coverage world for multithreading purposes"""
    obs_map, init_pos, goals = gen_coverage_world(args[0], args[1], args[2])
    return {"obs_map": obs_map, "init_pos": init_pos, "goals": goals}


def import_dragon_age_map(filename):
    """Used to import obstacle maps from the dragon-age online repository

    filename - name of dragon-age map to import

    """
    def to_obstacle_map(char):
        """Reads in a single character, and maps to 0 free 1 obstacle"""
        if char in ['.', 'G']:
            return 0
        elif char in ['@', 'O', 'T', 'S', 'W']:
            return 1
        raise ValueError('Invalid input')
    f = open(filename)
    l = f.readline()  # Reads the type octile line
    assert l == 'type octile\n'
    l = f.readline()  # Read height
    height = int(l.strip().split()[-1])
    l = f.readline()  # Read width
    width = int(l.strip().split()[-1])
    l = f.readline()  # Stip out another padding
    assert l == 'map\n'
    l = f.readline()
    obs_map = []
    while l != '':
        obs_map.append(map(to_obstacle_map, l.strip()))
        l = f.readline()
    return obs_map


def dragon_age_map_wrapper(args):
    obs_map, init_pos, goals = gen_dragon_age_map(*args)
    return {"obs_map": obs_map, "init_pos": init_pos, "goals": goals}


def gen_dragon_age_map(filename, num_bots):
    """Generates a random set of initial and goal configurations for a
    dragonage map

    connect_8 - whether to be built for 8 connected graph instead of 4
                connected

    returns:
    [obs_map, init_pos, goals]

    """
    obs_map = import_dragon_age_map(filename)
    sizex = len(obs_map)
    sizey = len(obs_map[1])
    # Generate the initial positions of the robots
    init_pos = []
    goals = []
    # Keeps track of how many times tried to generate valid initial,
    # goal positions
    counter = 0
    # Number of times to try for a given obstacle arrangement
    max_tries = 100 * num_bots

    while len(init_pos) < num_bots:
        # Generate possible initial and goal positions
        ip = (random.randint(0, sizex - 1), random.randint(0, sizey - 1))
        g = (random.randint(0, sizex - 1), random.randint(0, sizey - 1))
        if obs_map[g[0]][g[1]] == 1 or obs_map[ip[0]][ip[1]] == 1:
            # Don't allow the goal and initial positions to be in an
            # obstacle
            if counter > max_tries:
                # obs_map is too hard, try a new one
                init_pos = []
                goals = []
                raise ValueError('Couldn\'t Create conditions')
            counter += 1
            continue
        if g in goals or ip in init_pos:
            # Don't allow duplicate elements
            if counter > max_tries:
                # obs_map is too hard, try a new one
                init_pos = []
                goals = []
                obs_map = [[foo() for i in range(sizey)] for j in range(sizex)]
                counter = 0
                continue
            counter += 1
            continue
        # Check if we can generate a valid path
        graph = workspace_graph.Astar_Graph(obs_map, g)
        tpath = graph.get_step(ip)
        if tpath == None or tpath == []:
            # No individual path, so skip
            if counter > max_tries:
                # obs_map is too hard, try a new one
                init_pos = []
                goals = []
                obs_map = [[foo() for i in range(sizey)] for j in range(sizex)]
                counter = 0
                continue
            counter += 1
            continue
        # initial position, goal pair is valid
        init_pos.append(ip)
        goals.append(g)
    return obs_map, tuple(init_pos), tuple(goals)


def main(argv=sys.argv):
    # Read arguments using argparse
    parser = argparse.ArgumentParser(description='Generates test worlds for ' +
                                     'M* an associated worlds')
    parser.add_argument('output_file', help='Specifies the name of the file ' +
                        'in which to save results')
    parser.add_argument('min_robots', action='store', type=int,
                        help='Minimum number of robots for which to ' +
                        'generate trials')
    parser.add_argument('max_robots', action='store', type=int,
                        help='Maximum number of robots for which to ' +
                        'generate trials')
    parser.add_argument('step', action='store', type=int,
                        help='Increment by which to increase the number ' +
                        'of robots between trials')
    parser.add_argument('-p', dest='per_bot', action='store', type=int,
                        metavar='PER_BOT', default=PER_BOT,
                        help='Number of grid cells per robot')
    parser.add_argument('-n', dest='num_trials', action='store', type=int,
                        default=NUM_TRIALS,
                        help='Number of trials for a given number of robots')
    parser.add_argument('-f', dest='world_size', type=int, metavar='WIDTH',
                        help='Sets a fixed square world of the specified ' +
                        'width. Does not change obstacle coverage')
    parser.add_argument('--fixed', dest='fixed', action='store_true',
                        help='forces the world to be 32x32 regardless of '
                        'robot number, does not impact obstacle coverage')
    parser.add_argument('-c', action='store', type=float, default=COVERAGE,
                        metavar='COV', dest='coverage',
                        help='Specifies the probability of a given cell ' +
                        'being an obstacle, defaults to ' + str(COVERAGE))
    parser.add_argument('--dragon_age_map', dest='dragon_age',
                        metavar='MAP_NAME', default=None,
                        help='Creates test cases for the specified dragon ' +
                        'age map file')
    arguments = parser.parse_args()
    world_args = []
    assert(arguments.fixed or (arguments.world_size is not None))
    for num_robots in range(arguments.min_robots, arguments.max_robots + 1,
                            arguments.step):
        # Try a varying number of robots
        for j in range(arguments.num_trials):
            size = int((arguments.per_bot * num_robots) ** .5)
            if arguments.fixed:
                size = FIXED_SIZE
            elif arguments.world_size is not None:
                size = arguments.world_size
                assert(size > 0)
            if arguments.dragon_age != None:
                world_args.append([arguments.dragon_age, num_robots])
            else:
                world_args.append([size, num_robots, arguments.coverage])
    pool = multiprocessing.Pool(multiprocessing.cpu_count())
    worlds = None
    if arguments.dragon_age != None:
        worlds = pool.map(dragon_age_map_wrapper, world_args)
    else:
        worlds = pool.map(world_wrapper, world_args)
    pool.close()
    pickle.dump(worlds, open(arguments.output_file, 'w'))

if __name__ == '__main__':
    main()
