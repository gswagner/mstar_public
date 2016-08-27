#!/usr/bin/python
import matplotlib
matplotlib.use('Agg')

import pylab
import workspace_graph
import random
import sys
import cPickle
import numpy
from numpy import matrix
import time as timer
import multiprocessing
import networkx
import parmap

# Fix seed to fix color between animations
random.seed(5)
COLORS = [(random.uniform(0, 1), random.uniform(0, 1), random.uniform(0, 1))
          for i in range(100)]


def draw_betweenness(obs_map, graph=None, centrality_weight=10):
    """Draws a representation of the world showing which positions in
    the obs map are more important for short paths
    uses networkx graph if provided
    """
    G = graph
    if G is None:
        G = to_networkx_graph(obs_map)
    betweenness = nx.betweenness_centrality(G)
    disp_mat = -numpy.array(obs_map)
    max_bet = max(betweenness.values())
    for i in betweenness:
        disp_mat[i[0]][i[1]] += (centrality_weight * betweenness[i] /
                                 float(max_bet))
    pylab.matshow(disp_mat)


def draw_edges(graph, arrow_width=.1, color=(.7, .7, .7)):
    """Draw the edges of a networkx graph"""
    for i in graph.edges():
        pylab.arrow(i[0][1], i[0][0], .5 * (i[1][1] - i[0][1]),
                    .5 * (i[1][0] - i[0][0]), width=arrow_width, color=color)


def compute_cost(path):
    """Returns the cost of a path

    each robot incurs cost 1 for each time step it is not at its final
    configiration

    path - path from M*

    """
    goals = path[-1]
    cost = 0
    for i in range(1, len(path)):
        start_coord = path[i - 1]
        for k in range(len(goals)):
            if not(start_coord[k] == goals[k] and
                   start_coord[k] == path[i][k]):
                cost += 1
    return cost


def draw_workspace_graph(obs_map, goal, connect_8=False, road_rules=True):
    """Draws the individual policies and cost-to-go field for an
    environment

    obs_map - description of environment.  Matrix with 0 indicating free
              cell, 1 indicating an obstacle
    goal    - [[x1, y1], [x2, y2], ...] joint goal configuration
    """
    graph = workspace_graph.Workspace_Graph(obs_map, goal, connect_8=connect_8,
                                            road_rules=road_rules)
    cost_map = [[graph.get_cost((i, j)) for j in range(len(obs_map[0]))]
                for i in range(len(obs_map))]
    temp = []
    for i in cost_map:
        temp.extend(filter(lambda x: x < workspace_graph.MAX_COST, i))
    max_val = max(temp) * 1.05
    for i in cost_map:
        for j in xrange(len(i)):
            i[j] = min(i[j], max_val)
    pylab.matshow(matrix(cost_map).T)
    pylab.hold(True)
    X = [[0 for i in range(len(obs_map[0]))] for j in range(len(obs_map))]
    Y = [[0 for i in range(len(obs_map[0]))] for j in range(len(obs_map))]
    for i in range(len(obs_map)):
        for j in range(len(obs_map[0])):
            if cost_map[i][j] < max_val:
                pos = graph.get_step((i, j))
                if pos is None:
                    continue
                X[j][i] = (pos[1] - j)
                Y[j][i] = (pos[0] - i)
    pylab.quiver(Y, X)
    sizex = len(obs_map)
    sizey = len(obs_map[0])
    pylab.xlim([-.5, sizex - .5])
    pylab.ylim([-.5, sizey - .5])
    pylab.xticks([])
    pylab.yticks([])
    pylab.jet()
    pylab.hold(False)
    pylab.show()


def draw_astar_graph(obs_map, graph):
    """Debugging function which will show the current state of the m*
    graph
    """
    # Draw the obstacles
    pylab.hold(False)
    sizex = len(obs_map)
    sizey = len(obs_map[0])
    pylab.plot([-.5, sizex - .5, sizex - .5, -.5, -.5],
               [-.5, -.5, sizey - .5, sizey - .5, -.5], 'k')
    pylab.hold(True)
    pylab.matshow((1 - numpy.array(obs_map)).T, fignum=0)
    pylab.gray()
    pylab.clim(-2, 1)
    # Assemble the quiver
    X, Y, U, V = [[] for i in xrange(4)]
    for node in graph.graph.itervalues():
        x, y = node.coord
        u, v = map(lambda w, v: w - v, node.policy, node.coord)
        X.append(x)
        Y.append(y)
        U.append(u)
        V.append(v)
    pylab.quiver(X, Y, U, V)
    pylab.xlim([-.5, sizex - .5])
    pylab.ylim([-.5, sizey - .5])
    pylab.xticks([])
    pylab.yticks([])
    pylab.hold(False)
    pylab.show()


def draw_nonrecursive_search_space(obs_map, init_pos, goals, vis_structure):
    pylab.clf()
    draw_result(None, obs_map, init_pos, goals)
    for i in vis_structure:
        draw_fragment(i)
    pylab.xlim([-.5, len(obs_map) - .5])
    pylab.ylim([-.5, len(obs_map[0]) - .5])


def draw_recursive_search_space(obs_map, init_pos, goals, vis_structure):
    pylab.clf()
    draw_result(None, obs_map, init_pos, goals)
    # Remove empty lists
    vis_structure = filter(lambda x: x != [], vis_structure)
    # Sort by the size of the collision set

    def cmp_func(a, b):
        if a[0]['col_set'] < b[0]['col_set']:
            return 1
        elif a[0]['col_set'] > b[0]['col_set']:
            return -1
        return 0

    vis_structure.sort(cmp=cmp_func)
    max_bots = max(map(lambda x: len(x[0]['rob_id']), vis_structure))
    print max_bots
    for i in vis_structure:
        draw_fragment(i, max(1, i[0]['col_set']) / float(max_bots) * 0.13)


def draw_fragment(moves, width=.1):
    counter = 0
    for fragment in moves:
        counter += 1
        pylab.hold(True)

        def get_color(i):
            while i >= len(COLORS):
                i -= len(COLORS)
            return COLORS[i]

        coord1 = None
        if fragment['standard_1']:
            coord1 = fragment['coord_1']
        else:
            coord1 = list(fragment['coord_1'][0])
        coord2 = None
        if fragment['standard_2']:
            coord2 = fragment['coord_2']
        else:
            coord2 = list(fragment['coord_2'][1])
            coord2.extend(fragment['coord_2'][0][len(coord2):])
        for i in range(len(fragment['rob_id'])):
            color = list(get_color(fragment['rob_id'][i]))
            if fragment['skipped']:
                pylab.plot([coord2[i][0]], [coord2[i][1]], marker='o',
                           color=color)
                continue
            try:
                pylab.arrow(coord1[i][0], coord1[i][1],
                            coord2[i][0] - coord1[i][0],
                            coord2[i][1] - coord1[i][1],
                            length_includes_head=True, width=width,
                            head_width=width * 3, color=color)
            except AssertionError:
                pass
    pylab.hold(False)


def draw_result(paths, obs_map, init_pos, goals, conn_8=False, max_width=5):
    """draws results, assumes square world

    paths   - joint path of the system
              [[[x1(0), y1(0)]...], [[x1(1), y1(1)]]
    obs_map - description of environment.  Matrix with 0 indicating free
              cell, 1 indicating an obstacle
    goal    - [[x1, y1], [x2, y2], ...] joint initial configuration
    goal    - [[x1, y1], [x2, y2], ...] joint goal configuration
    conn_8  - whether to use an 8-connected depiction
    """
    if paths is None:
        print 'No Path, Out of Time'
    elif paths == []:
        print 'No Path, Proved no solution'
    else:
        paths = map(lambda x: map(list, x), paths)
        # Decompose the path from a path in the full c space to
        # individual paths
        temp = [[] for i in range(len(paths[0]))]
        for i in range(len(paths)):
            for j in range(len(paths[i])):
                temp[j].append(paths[i][j])
        paths = temp

    def get_color(i):
        while i >= len(COLORS):
            i -= len(COLORS)
        return COLORS[i]

    pylab.hold(False)
    pylab.clf()
    size = len(obs_map)
    pylab.plot([-.5, size - .5, size - .5, -.5, -.5],
               [-.5, -.5, size - .5, size - .5, -.5], 'k')
    pylab.hold(True)
    if not conn_8:
        # Draw the obstacles
        pylab.matshow((1 - numpy.array(obs_map)).T, fignum=0)
        pylab.gray()
        pylab.clim(-2, 1)
    else:
        for i in range(size):
            for j in range(size):
                if obs_map[i][j] == 1:
                    cir = pylab.Circle((i, j), radius=0.5, alpha=0.5,
                                       fc='white')
                    pylab.gca().add_patch(cir)
        pylab.axis('scaled')
    # draw the paths
    if paths is not None:
        for i in range(len(paths)):
            x, y = zip(*paths[i])
            print x
            pylab.plot(x, y, color=get_color(i),
                       linewidth=max_width * (len(paths) - i) /
                       float(len(paths)))
    # draw the goals
    for i in range(len(goals)):
        pylab.plot(goals[i][0], goals[i][1], color=get_color(i),
                   marker='*', markersize=10)
    for i in range(len(init_pos)):
        pylab.plot(init_pos[i][0], init_pos[i][1], color=get_color(i),
                   marker='o', markersize=10)
    pylab.xlim([-.5, len(obs_map) - .5])
    pylab.ylim([-.5, len(obs_map[0]) - .5])
    # Supress the ticks
    pylab.xticks([])
    pylab.yticks([])
    pylab.hold(False)
    pylab.show()


def animate_result_step(step, paths, obs_map, init_pos, goals, file_base=None,
                        conn_8=False, increments=1):
    """Plots a single step of the path

    optionally saves each image to file.  Optionally interpolates
    between steps to produce a smoother animation

    step       - step of the path to plot
    paths      - joint path of the system
                 [[[x1(0), y1(0)]...], [[x1(1), y1(1)]]
    obs_map    - description of environment.  Matrix with 0 indicating
                  free cell, 1 indicating an obstacle
    goal       - [[x1, y1], [x2, y2], ...] joint initial configuration
    goal       - [[x1, y1], [x2, y2], ...] joint goal configuration
    file_base  -
    conn_8     - whether to use an 8-connected depiction
    increments - How many intermediate frames to generate between step
                 and step + 1
    """
    # set the size of the obstacle/robot representations.  If producing
    # incremental results for 8-connected graphs, want the robots to be
    # a bit smaller
    obs_size = 10
    rob_size = 10
    if increments > 1 and conn_8:
        rob_size = 7
    increments = int(increments)
    assert(increments >= 1)
    # Decompose the path from a path in the full c space to individual
    # paths
    path_length = len(paths)
    temp = [[] for i in range(len(paths[0]))]
    for i in range(len(paths)):
        for j in range(len(paths[i])):
            temp[j].append(paths[i][j])
    paths = temp

    def get_color(i):
        while i >= len(COLORS):
            i -= len(COLORS)
        return COLORS[i]

    t = step
    print t
    for inc in xrange(increments):
        # Turning off hold will overwrite any existing images
        pylab.hold(False)
        sizex = len(obs_map)
        sizey = len(obs_map[0])
        pylab.plot([-.5, sizex - .5, sizex - .5, -.5, -.5],
                   [-.5, -.5, sizey - .5, sizey - .5, -.5], 'k')
        pylab.hold(True)
        if not conn_8:
            pylab.matshow((1 - numpy.array(obs_map)).T, fignum=0)
            pylab.gray()
            pylab.clim(-2, 1)
        else:
            for i in range(sizex):
                for j in range(sizey):
                    if obs_map[i][j] == 1:
                        cir = pylab.Circle((i, j), radius=0.5, alpha=0.5,
                                           fc='gray')
                        pylab.gca().add_patch(cir)
            pylab.axis('scaled')
        # draw the goals
        for i in range(len(goals)):
            pylab.plot(goals[i][0], goals[i][1], color=get_color(i),
                       marker='*', markersize=10)
            pylab.plot(init_pos[i][0], init_pos[i][1], color=get_color(i),
                       marker='+', markersize=10)
        # draw the paths
        if paths is not None:
            for i in range(len(paths)):
                if t >= len(paths[i]):
                    continue
                x, y = paths[i][t]
                if inc > 0 and step < len(paths[i]) - 1:
                    tx, ty = paths[i][t + 1]
                    x += (tx - x) * inc / float(increments)
                    y += (ty - y) * inc / float(increments)
                pylab.plot(x, y, color=get_color(i), marker='o',
                           linewidth=(len(paths) - i), markersize=rob_size)
        pylab.xlim([-.5, sizex - .5])
        pylab.ylim([-.5, sizey - .5])
        pylab.xticks([])
        pylab.yticks([])
        pylab.draw()
        pylab.hold(False)
        if file_base is not None:
            pylab.savefig(file_base + str(t * increments + inc).zfill(3) +
                          '.png',
                          bbox_inches='tight', pad_inches=.1, dpi=(200))


def animate_result(paths, obs_map, init_pos, goals, delay=0, file_base=None,
                   conn_8=True, increments=1):
    """Simple function to animate the path, so it can be viewed

    paths   - joint path of the system
              [ [[x1(0), y1(0)]...], [[x1(1), y1(1)] ]
    obs_map - description of environment.  Matrix with 0 indicating free
              cell, 1 indicating an obstacle
    goal    - [[x1, y1], [x2, y2], ...] joint initial configuration
    goal    - [[x1, y1], [x2, y2], ...] joint goal configuration
    file_base -
    conn_8  - whether to use an 8-connected depiction
    increments - How many intermediate frames to generate between step
                 and step + 1
    """
    for i in range(len(paths)):
        animate_result_step(i, paths, obs_map, init_pos, goals, file_base,
                            conn_8=conn_8, increments=increments)
        timer.sleep(delay)


def gen_movie(paths, obs_map, init_pos, goals, file_base, conn_8=True,
              increments=1):
    """Parallelized animation code

    saves image files for each frame in the animation

    paths   - joint path of the system
              [ [[x1(0), y1(0)]...], [[x1(1), y1(1)] ]
    obs_map - description of environment.  Matrix with 0 indicating free
              cell, 1 indicating an obstacle
    goal    - [[x1, y1], [x2, y2], ...] joint initial configuration
    goal    - [[x1, y1], [x2, y2], ...] joint goal configuration
    file_base - base name for image files
    conn_8  - whether to use an 8-connected depiction
    increments - How many intermediate frames to generate between step
                 and step + 1
    """
    parmap.parmap(lambda x: animate_result_step(x, paths, obs_map, init_pos,
                                                goals, file_base=file_base,
                                                conn_8=conn_8,
                                                increments=increments),
                  xrange(len(paths)))


def gen_plots(source_name, dest_name, large_format=False, plot_mem=False):
    """Generate a specific figure"""
    linewidth = 1
    if large_format:
        params = {'axes.labelsize': 30, 'text.fontsize': 30,
                  'xtick.labelsize': 30, 'ytick.labelsize': 30}
        linewidth = 5
        pylab.rcParams.update(params)
    print 'Loading data'
    dat = cPickle.load(open(source_name))
    bot_numbers = list(set(map(lambda x: len(x['init_pos']), dat)))
    bot_numbers.sort()
    print 'Finished pickling'

    def foo(x):
        return bot_numbers.index(len(x['init_pos']))

    res = [[] for i in range(len(bot_numbers))]
    for i in dat:
        res[foo(i)].append(i)
    suc = [len(filter(lambda y: y['fail_flag'] == '', res[i])) /
           float(len(res[i])) * 100.0 for i in range(len(res))]

    for i in range(len(suc)):
        suc[i] = float(suc[i]) / float(len(res[i])) * 100
    times = map(lambda x: map(lambda y: y['time'], x), res)
    if plot_mem:
        tres = map(lambda x: filter(lambda y: y['fail_flag'] == '', x), res)
        mem_usage = map(lambda x:
                        sum(map(lambda y: y['num_nodes'] -
                                104 * len(y['init_pos']) ** 2, x)) /
                        float(len(x)),
                        tres)
        pylab.plot(bot_numbers, mem_usage)
        pylab.xlabel('Number of Robots')
        pylab.ylabel('Average number of nodes')
        pylab.savefig(dest_name + '_num_nodes.eps')
        pylab.clf()
    for i in times:
        i.sort()
    per_10 = [times[i][int(len(times[i]) * .1)] for i in range(len(times))]
    per_50 = [times[i][int(len(times[i]) * .5)] for i in range(len(times))]
    per_90 = [times[i][int(len(times[i]) * .9)] for i in range(len(times))]
    tot_time = 0
    avg_time = []
    for i in range(len(times)):
        for j in range(len(times[i])):
            tot_time += times[i][j]
        avg_time.append(tot_time / len(times[i]))
        tot_time = 0
    print avg_time
    print suc
    pylab.plot(bot_numbers, suc, linewidth=linewidth)
    pylab.axis((bot_numbers[0], bot_numbers[-1], 0, 110))
    if not large_format:
        pylab.xlabel('Number of robots')
        pylab.ylabel('Percent Success')
    pylab.savefig(dest_name + '_success.eps')
    pylab.clf()
    # pylab.plot(bot_numbers, cost, linewidth=linewidth)
    # if not large_format:
    #    pylab.xlabel('Number of robots')
    #    pylab.ylabel('Average Path Length')
    # pylab.savefig(dest_name + '_cost.eps')
    pylab.clf()
    pylab.semilogy(bot_numbers, per_10, 'r', bot_numbers, per_50, 'b--',
                   bot_numbers, per_90, 'g-.', linewidth=linewidth)
    # pylab.boxplot(times)
    if not large_format:
        pylab.xlabel('Number of Robots')
        pylab.ylabel('Time (seconds)')
    pylab.savefig(dest_name + '_time.eps')


def main(argv=sys.argv):
    """Handles generating plots"""
    print './main infile outfile [-L]\n-L Use large format for publication'
    if len(argv) < 3:
        return
    large_format = False
    if '-L' in argv:
        large_format = True
        argv.pop(argv.index('-L'))
    if len(argv) != 3:
        return
    infile = argv[1]
    outfile = argv[2]
    gen_plots(infile, outfile, large_format)

if __name__ == '__main__':
    main()
