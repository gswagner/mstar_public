#!/usr/bin/python
"""Unit test suite for mstar and associated files"""
import unittest
import argparse
import pickle
import time
import workspace_graph
import od_mstar
try:
    import ipdb as pdb
except ImportError:
    # Default to pdb
    import pdb
import sys
import networkx
import col_set_addition
from itertools import izip
import networkx
from test_utils import debug_on, DebugMeta, DEBUG, do_cprofile

# Remember tests can be marked as skipped with the decorators
# @unittest.skip('message goes here')
# @unittest.skipIf(conditional, 'message')
# @unittest.skipUnless(conditional, 'message')

FULL_TEST = False
EPERMSTAR_FULL = False
EPEMSTAR_FULL = False
OD_RMSTAR_FULL = False
OD_ID_RMSTAR_FULL = False
RMSTAR_FULL = False
CMS_OD_MSTAR_FULL = False


class TestGridGraph(unittest.TestCase):

    __metaclass__ = DebugMeta

    def setUp(self):
        """Is called before every test case is run"""
        self.world_descriptor = [[0 for i in xrange(10)] for j in xrange(10)]

    def tearDown(self):
        """Called after every test function run"""

    def test_grid_graph_get_neighbors_4conn(self):
        """Tests accurate generation of neighbors with 4 conn"""
        new_obs_map = self.world_descriptor
        new_obs_map[5][5] = 1
        new_obs_map[8][9] = 1
        new_obs_map[9][8] = 1
        new_obs_map[8][7] = 1
        new_obs_map[7][8] = 1
        grid_graph = workspace_graph.Grid_Graph(new_obs_map)
        self.assertTrue(set(grid_graph.get_neighbors((4, 5)))
                        == set([(4, 5), (4, 4), (4, 6), (3, 5)]))
        self.assertTrue(set(grid_graph.get_neighbors((2, 2)))
                        == set([(2, 2), (2, 3), (2, 1), (3, 2), (1, 2)]))
        self.assertTrue(set(grid_graph.get_neighbors((8, 8)))
                        == set([(8, 8)]))
        self.assertTrue(set(grid_graph.get_neighbors((0, 9)))
                        == set([(0, 9), (1, 9), (0, 8)]))

    def test_grid_graph_get_neighbors_8conn(self):
        """Tests accurate generation of neighbors with 8 conn"""
        new_obs_map = self.world_descriptor
        new_obs_map[5][5] = 1
        new_obs_map[3][6] = 1
        new_obs_map[8][9] = 1
        new_obs_map[9][8] = 1
        new_obs_map[8][7] = 1
        new_obs_map[7][8] = 1
        new_obs_map[9][9] = 1
        new_obs_map[7][9] = 1
        new_obs_map[9][7] = 1
        new_obs_map[7][7] = 1
        grid_graph = workspace_graph.Grid_Graph_Conn_8(new_obs_map)
        self.assertTrue(set(grid_graph.get_neighbors((4, 5)))
                        == set([(4, 5), (4, 4), (4, 6), (3, 5), (5, 6), (3, 4),
                                (5, 4)]))
        self.assertTrue(set(grid_graph.get_neighbors((2, 2)))
                        == set([(2, 2), (2, 3), (2, 1), (3, 2), (1, 2), (3, 3),
                                (3, 1), (1, 1), (1, 3)]))
        self.assertTrue(set(grid_graph.get_neighbors((8, 8)))
                        == set([(8, 8)]))
        self.assertTrue(set(grid_graph.get_neighbors((0, 9)))
                        == set([(0, 9), (1, 9), (0, 8), (1, 8)]))


class TestGridGraphConn4WaitAtGoal(unittest.TestCase):
    """Tests a grid graph variant which does not penalyze waiting at the
    goal
    """
    def setUp(self):
        """Called before every test function is run"""
        self.obs_map = [[0 for i in xrange(10)] for j in xrange(10)]

    def test_edge_costs(self):
        """Tests that edge costs are computed properly"""
        graph = workspace_graph.GridGraphConn4WaitAtGoal(self.obs_map,
                                                         (0, 0), wait_cost=0)
        self.assertTrue(graph.get_edge_cost((0, 0), (0, 0)) == 0)
        self.assertTrue(graph.get_edge_cost((0, 0), (1, 0)) == 1)
        self.assertTrue(graph.get_edge_cost((1, 0), (0, 0)) == 1)
        self.assertTrue(graph.get_edge_cost((1, 0), (1, 1)) == 1)


class TestNetworkxDiGraph(unittest.TestCase):
    """Test the wrapper for a NetworkX Di Graph"""

    __metaclass__ = DebugMeta

    def setUp(self):
        """Called before every test function is run"""
        self._n_graph = networkx.DiGraph()
        self._n_graph.add_nodes_from(range(4))
        for n1, n2 in [(i, i + 1) for i in xrange(3)]:
            self._n_graph.add_edge(n1, n2, cost=1)
        self._n_graph.add_edge(3, 2, cost=1)

    def test_edge_costs(self):
        """Tests edge cost recovery"""
        g = workspace_graph.Networkx_DiGraph(self._n_graph)
        for n1, n2 in [(i, i + 1) for i in xrange(3)]:
            self.assertTrue(g.get_edge_cost(n1, n2) == 1)
        self.assertTrue(g.get_edge_cost(3, 2) == 1)

    def test_neighbors(self):
        """Tests neighbor generation"""
        g = workspace_graph.Networkx_DiGraph(self._n_graph)
        for i in xrange(3):
            self.assertTrue(g.get_neighbors(i) == [i + 1])
        self.assertTrue(g.get_neighbors(3) == [2])

    def test_in_neighbors(self):
        """Test generation of in neighbors"""
        g = workspace_graph.Networkx_DiGraph(self._n_graph)
        for i in [1, 3]:
            self.assertTrue(g.get_in_neighbors(i) == [i - 1])
        self.assertTrue(set(g.get_in_neighbors(2)) == set([1, 3]))


class TestDiGraphPolicy(unittest.TestCase):
    """Test the policy for digraphs"""

    __metaclass__ = DebugMeta

    def setUp(self):
        """Called before every test function is run"""
        self._n_graph = networkx.DiGraph()
        self._n_graph.add_nodes_from(range(4))
        for n1, n2 in [(i, i + 1) for i in xrange(3)]:
            self._n_graph.add_edge(n1, n2, cost=1)
        self._n_graph.add_edge(3, 2, cost=1)

    def test_cost(self):
        """Tests cost field generation"""
        policy = workspace_graph.Astar_DiGraph_Policy(
            self._n_graph, workspace_graph.Networkx_DiGraph, goal=3,
            compute_heuristic=lambda x, y: 0)
        for i in xrange(3):
            self.assertTrue(policy.get_cost(i) == 3 - i)

        # Test that we get a proper error when trying to plan to an
        # unreachable node
        policy = workspace_graph.Astar_DiGraph_Policy(
            self._n_graph, workspace_graph.Networkx_DiGraph, goal=0,
            compute_heuristic=lambda x, y: 0)
        try:
            policy.get_cost(3)
            self.assertFalse()
        except col_set_addition.NoSolutionError:
            pass


class TestAstarGraph(unittest.TestCase):

    __metaclass__ = DebugMeta

    def setUp(self):
        """Is called before every test function is run"""
        self.world_descriptor = [[0 for i in xrange(10)] for j in xrange(10)]

    def tearDown(self):
        """Called after every test function run"""

    def test_astar_graph_no_obstacles(self):
        """Tests that astar graph performs correctly without obstacles"""
        graph = workspace_graph.Astar_Graph(self.world_descriptor, (5, 5))
        self.assertTrue(graph.get_step((5, 5)) == (5, 5))
        self.assertTrue(graph.get_cost((5, 5)) == 0)
        self.assertTrue(graph.get_step((5, 4)) == (5, 5))
        self.assertTrue(graph.get_cost((5, 4)) == 1)
        self.assertTrue(graph.get_cost((0, 0)) == 10)

    def test_astar_graph_get_edge_cost(self):
        """Tests to ensure that edge cost function works correctly"""
        graph = workspace_graph.Astar_Graph(self.world_descriptor, (5, 5))
        self.assertTrue(graph.get_edge_cost((5, 5), (5, 6)) == 1)
        self.assertTrue(graph.get_edge_cost((5, 5), (5, 5)) == 0)

    def test_astar_graph_get_makespan_edge_cost(self):
        """Tests to ensure that edge cost function works correctly"""
        graph = workspace_graph.Astar_Graph(self.world_descriptor, (5, 5),
                                            makespan=True)
        self.assertTrue(graph.get_edge_cost((5, 5), (5, 6)) == 1)
        self.assertTrue(graph.get_edge_cost((5, 5), (5, 5)) == 1)

    def test_astar_graph_no_obstacles_8conn(self):
        """Tests that 8-connected astar graph performs correctly without
        obstacles"""
        graph = workspace_graph.Astar_Graph(self.world_descriptor, (5, 5),
                                            connect_8=True)
        for i in (-1, 0, 1):
            for j in (-1, 0, 1):
                coord = (5 + i, 5 + j)
                if coord == (5, 5):
                    self.assertTrue(graph.get_step(coord) == (5, 5))
                    self.assertTrue(graph.get_cost(coord) == 0)
                else:
                    self.assertTrue(graph.get_step(coord) == (5, 5))
                    self.assertTrue(graph.get_cost(coord) == 1)
        self.assertTrue(graph.get_step((0, 0)) == (1, 1))
        self.assertTrue(graph.get_cost((0, 0)) == 5)

    def test_astar_graph_obstacles(self):
        """Tests that astar graph performs correctly with simple obstacles"""
        self.world_descriptor[1][0] = 1
        graph = workspace_graph.Astar_Graph(self.world_descriptor, (2, 0))
        self.assertTrue(graph.get_step((0, 0)) == (0, 1))
        self.assertTrue(graph.get_cost((0, 0)) == 4)
        self.assertTrue(graph.get_step((1, 1)) == (2, 1))
        self.assertTrue(graph.get_cost((1, 1)) == 2)

    def test_astar_graph_obstacles_8conn(self):
        """Tests that 8-conencted astar graph performs correctly with simple
        obstacles"""
        self.world_descriptor[1][0] = 1
        graph = workspace_graph.Astar_Graph(self.world_descriptor, (2, 0),
                                            connect_8=True)
        self.assertTrue(graph.get_step((0, 0)) == (1, 1))
        self.assertTrue(graph.get_cost((0, 0)) == 2)
        self.assertTrue(graph.get_step((1, 1)) == (2, 0))
        self.assertTrue(graph.get_cost((1, 1)) == 1)

    def test_astar_threshold_neighbors(self):
        """Tests that cost threshold neighbors work properly for epea*"""
        graph = workspace_graph.Astar_Graph(self.world_descriptor, (2, 2),
                                            connect_8=True)
        neibs = graph.get_limited_offset_neighbors((4, 2), 0)
        self.assertTrue(len(neibs) == 3)
        for t in [(0, (3, 2)), (0, (3, 3)), (0, (3, 1))]:
            self.assertTrue(t in neibs)
        neibs = graph.get_limited_offset_neighbors((4, 2), 1)
        self.assertTrue(len(neibs) == 6)
        for t in [(0, (3, 2)), (0, (3, 3)), (0, (3, 1)), (1, (4, 3)),
                  (1, (4, 2)), (1, (4, 1))]:
            self.assertTrue(t in neibs)
        neibs = graph.get_limited_offset_neighbors((4, 2), 2)
        self.assertTrue(len(neibs) == 9)
        # check minimum offset functionality
        neibs = graph.get_limited_offset_neighbors((4, 2), 1, min_offset=1)
        self.assertTrue(len(neibs) == 3)
        for t in [(1, (4, 3)), (1, (4, 2)), (1, (4, 1))]:
            self.assertTrue(t in neibs)
        neibs = graph.get_limited_offset_neighbors((4, 2), 2, min_offset=1)
        self.assertTrue(len(neibs) == 6)
        for t in [(1, (4, 3)), (1, (4, 2)), (1, (4, 1)), (2, (5, 3)),
                  (2, (5, 2)), (2, (5, 1))]:
            self.assertTrue(t in neibs)

    def test_astar_threshold_neighbors_obstacles(self):
        """Tests that cost threshold neighbors work properly for epea* when
        obstacles are present"""
        self.world_descriptor[2][2] = 1
        graph = workspace_graph.Astar_Graph(self.world_descriptor, (1, 2),
                                            connect_8=True)
        neibs = graph.get_limited_offset_neighbors((3, 2), 0)
        self.assertTrue(len(neibs) == 2)
        for t in [(0, (2, 3)), (0, (2, 1))]:
            self.assertTrue(t in neibs)
        neibs = graph.get_limited_offset_neighbors((3, 2), 1)
        self.assertTrue(len(neibs) == 5)
        for t in [(0, (2, 3)), (0, (2, 1)), (1, (3, 3)), (1, (3, 2)),
                  (1, (3, 1))]:
            self.assertTrue(t in neibs)


class TestUtilityFunctions(unittest.TestCase):

    __metaclass__ = DebugMeta

    def setUp(self):
        """Is called before every test function is run"""
        self.world_descriptor = [[0 for i in xrange(10)] for j in xrange(10)]

    def tearDown(self):
        """Called after every test function run"""

    def test_get_edge_cost(self):
        """Tests edge cost calculation for two coords"""
        self.assertTrue(workspace_graph.Grid_Graph(
                        self.world_descriptor).get_edge_cost((1, 2), (2, 3)) ==
                        1)

    def test_compute_heuristic_conn_4(self):
        """Tests heuristic calculation for 4 connectivity"""
        # Test known values
        compute_heuristic_conn_4 = workspace_graph.compute_heuristic_conn_4
        self.assertTrue(compute_heuristic_conn_4((5, 5), (7, 9)) == 6)
        self.assertTrue(compute_heuristic_conn_4((5, 5), (5, 5)) == 0)
        # Test against procedurally generated answers
        for coord1, coord2 in [[(i, j), (x, y)]
                               for i in xrange(10)
                               for j in xrange(10)
                               for x in xrange(10)
                               for y in xrange(10)]:
            self.assertTrue(compute_heuristic_conn_4(coord1, coord2) ==
                            abs(coord1[0] - coord2[0]) +
                            abs(coord1[1] - coord2[1]))

    def test_compute_heuristic_conn_8(self):
        """Tests heuristic calculation for 8 connectivity"""
        # test known values
        compute_heuristic_conn_8 = workspace_graph.compute_heuristic_conn_8
        self.assertTrue(compute_heuristic_conn_8((5, 5), (7, 9)) == 4)
        self.assertTrue(compute_heuristic_conn_8((5, 5), (1, 4)) == 4)
        self.assertTrue(compute_heuristic_conn_8((5, 5), (5, 5)) == 0)
        # test procedurally generated answers
        for coord1, coord2 in [[(i, j), (x, y)]
                               for i in xrange(10)
                               for j in xrange(10)
                               for x in xrange(10)
                               for y in xrange(10)]:
            self.assertTrue(compute_heuristic_conn_8(coord1, coord2)
                            == max(abs(coord1[0] - coord2[0]),
                                   abs(coord1[1] - coord2[1])))


class TestPriorityGraph(unittest.TestCase):

    __metaclass__ = DebugMeta

    def setUp(self):
        """Is called before every test function is run"""
        self.world_descriptor = [[0 for i in xrange(10)] for j in xrange(10)]

    def tearDown(self):
        """Called after every test function run"""

    def test_priority_graph_no_obstacles(self):
        """Tests that astar graph performs correctly without obstacles"""
        graph = workspace_graph.Priority_Graph(
            workspace_graph.Astar_Graph(self.world_descriptor, (5, 5)))
        self.assertTrue(graph.get_step((5, 5, 1)) == (5, 5, 2))
        self.assertTrue(graph.get_cost((5, 5, 1)) == 0)
        self.assertTrue(graph.get_step((5, 4, 3)) == (5, 5, 4))
        self.assertTrue(graph.get_cost((5, 4, 3)) == 1)
        self.assertTrue(graph.get_cost((0, 0, 6)) == 10)

    def test_priority_graph_no_obstacles_8conn(self):
        """Tests that 8-connected astar graph performs correctly without
        obstacles"""
        graph = workspace_graph.Priority_Graph(
            workspace_graph.Astar_Graph(self.world_descriptor, (5, 5),
                                        connect_8=True))
        for i in (-1, 0, 1):
            for j in (-1, 0, 1):
                coord = (5 + i, 5 + j, 0)
                if coord == (5, 5, 0):
                    self.assertTrue(graph.get_step(coord) == (5, 5, 1))
                    self.assertTrue(graph.get_cost(coord) == 0)
                else:
                    self.assertTrue(graph.get_step(coord) == (5, 5, 1))
                    self.assertTrue(graph.get_cost(coord) == 1)
        self.assertTrue(graph.get_step((0, 0, 10)) == (1, 1, 11))
        self.assertTrue(graph.get_cost((0, 0, 0)) == 5)

    def test_priority_graph_obstacles(self):
        """Tests that astar graph performs correctly with simple obstacles"""
        self.world_descriptor[1][0] = 1
        graph = workspace_graph.Priority_Graph(
            workspace_graph.Astar_Graph(self.world_descriptor, (2, 0)))
        self.assertTrue(graph.get_step((0, 0, 1)) == (0, 1, 2))
        self.assertTrue(graph.get_cost((0, 0, 1)) == 4)
        self.assertTrue(graph.get_step((1, 1, 0)) == (2, 1, 1))
        self.assertTrue(graph.get_cost((1, 1, 0)) == 2)

    def test_priority_graph_obstacles_8conn(self):
        """Tests that 8-conencted astar graph performs correctly with simple
        obstacles"""
        self.world_descriptor[1][0] = 1
        graph = workspace_graph.Priority_Graph(
            workspace_graph.Astar_Graph(self.world_descriptor, (2, 0),
                                        connect_8=True))
        self.assertTrue(graph.get_step((0, 0, 1)) == (1, 1, 2))
        self.assertTrue(graph.get_cost((0, 0, 1)) == 2)
        self.assertTrue(graph.get_step((1, 1, 2)) == (2, 0, 3))
        self.assertTrue(graph.get_cost((1, 1, 2)) == 1)

    def test_priority_max_t(self):
        """Tests that the time limit works as expected"""
        graph = workspace_graph.Priority_Graph(
            workspace_graph.Astar_Graph(self.world_descriptor, (5, 5),
                                        connect_8=True))
        graph.set_max_t(2)
        for t in [0, 1, 2]:
            for i in (-1, 0, 1):
                for j in (-1, 0, 1):
                    coord = (5 + i, 5 + j, t)
                    if coord == (5, 5, t):
                        self.assertTrue(graph.get_step(coord) ==
                                        (5, 5, min(2, t + 1)))
                        self.assertTrue(graph.get_cost(coord) == 0)
                    else:
                        self.assertTrue(graph.get_step(coord) ==
                                        (5, 5, min(2, t + 1)))
                        self.assertTrue(graph.get_cost(coord) == 1)
        self.assertTrue(graph.get_cost((0, 0, 0)) == 5)


class TestrMstar(unittest.TestCase):
    """Testing that ODrM* is  working"""
    def setUp(self):
        self.world_descriptor = [[0 for i in xrange(10)] for j in xrange(10)]

    @debug_on()
    def test_mstar_single_robot(self):
        """Tests ODM* on simple single robot cases"""
        path = od_mstar.find_path(self.world_descriptor, [[0, 0]], [[1, 0]],
                                  astar=True)
        self.assertTrue(path == (((0, 0), ), ((1, 0), )))
        path = od_mstar.find_path(self.world_descriptor, [[0, 0]], [[1, 1]],
                                  astar=True)
        self.assertTrue(path == (((0, 0), ), ((1, 0), ), ((1, 1), )) or
                        path == (((0, 0), ), ((0, 1), ), ((1, 1), )))
        self.assertTrue(validate_solution(path))
        self.world_descriptor[1][0] = 1
        path = od_mstar.find_path(self.world_descriptor, [[0, 0]], [[2, 0]],
                                  astar=True)
        self.assertTrue(path == (((0, 0), ), ((0, 1), ), ((1, 1), ),
                                 ((2, 1), ), ((2, 0), )))
        # TEST 8-connected paths
        path = od_mstar.find_path(self.world_descriptor, [[0, 0]], [[1, 1]],
                                  connect_8=True, astar=True)
        self.assertTrue(path == (((0, 0), ), ((1, 1), )))
        path = od_mstar.find_path(self.world_descriptor, [[0, 0]], [[2, 0]],
                                  connect_8=True, astar=True)
        self.assertTrue(path == (((0, 0), ), ((1, 1), ), ((2, 0), )))

    def test_mstar_multirobot_paths(self):
        """Tests some simple multirobot cases on ODrM*"""
        path = od_mstar.find_path(self.world_descriptor,
                                  [[0, 0], [1, 0]], [[1, 0], [0, 0]],
                                  astar=True)
        self.assertTrue(validate_solution(path))
        self.assertTrue(compute_cost(path) == 4)
        path = od_mstar.find_path(self.world_descriptor,
                                  [[0, 0], [1, 0], [5, 5]],
                                  [[1, 0], [0, 0], [6, 6]], astar=True)
        self.assertTrue(validate_solution(path))
        self.assertTrue(compute_cost(path) == 6)
        # Test a simple three robot collision along edge
        path = od_mstar.find_path(self.world_descriptor,
                                  [[3, 0], [0, 0], [2, 0]],
                                  [[0, 0], [3, 0], [5, 0]], astar=True)
        self.assertTrue(validate_solution(path))
        # TEST 8-connected paths
        path = od_mstar.find_path(self.world_descriptor,
                                  [[0, 0], [1, 0]], [[1, 0], [0, 0]],
                                  connect_8=True, astar=True)
        self.assertTrue(validate_solution(path))
        self.assertTrue(compute_cost(path) == 3)
        path = od_mstar.find_path(self.world_descriptor,
                                  [[0, 0], [1, 0], [5, 5]],
                                  [[1, 0], [0, 0], [6, 6]], connect_8=True,
                                  astar=True)
        self.assertTrue(validate_solution(path))
        self.assertTrue(compute_cost(path) == 4)
        # Test with some obstacles
        self.world_descriptor[5][0] = 1
        path = od_mstar.find_path(self.world_descriptor,
                                  [[4, 0], [6, 0]], [[6, 0], [4, 0]],
                                  astar=True)
        self.assertTrue(validate_solution(path))
        self.assertTrue(compute_cost(path) == 10)

    def test_map_solutions(self):
        """Tests rmstar on a couple of test maps, no constraints"""
        if not (FULL_TEST or RMSTAR_FULL):
            self.skipTest('Skipped map test for brevity')
        print '\nr_mstar long tests'
        dat = pickle.load(open('../maps/5_40_bots_step_5.map'))
        start_time = time.time()
        # Test inlfated
        for i in [0, 30, 60, 101, 131, 141, 151, 161, 200, 250, 300, 350, 400]:
            print i
            d = dat[i]
            path = od_mstar.find_path(d['obs_map'], d['init_pos'], d['goals'],
                                      inflation=10, time_limit=10,
                                      connect_8=True, astar=True)
            self.assertTrue(validate_solution(path))
            self.assertTrue(path[-1] == d['goals'])
        # Test uniflated
        print 'uninflated'
        for i in [0, 30, 60, 101, 131, 141, 151, 161, 200, 227]:
            # , 250, 275, 280]:# , 301, 350]:
            print i
            d = dat[i]
            path = od_mstar.find_path(d['obs_map'], d['init_pos'], d['goals'],
                                      inflation=1, time_limit=60,
                                      connect_8=True, astar=True)
            self.assertTrue(validate_solution(path))
            self.assertTrue(path[-1] == d['goals'])
        print 'time elapsed: %g' % (time.time() - start_time)


class TestODrMstar(unittest.TestCase):
    """Testing that ODrM* is  working"""
    def setUp(self):
        self.world_descriptor = [[0 for i in xrange(10)] for j in xrange(10)]

    def test_op_decomp_single_robot(self):
        """Tests ODM* on simple single robot cases"""
        path = od_mstar.find_path(self.world_descriptor, [[0, 0]], [[1, 0]])
        self.assertTrue(path == (((0, 0), ), ((1, 0), )))
        path = od_mstar.find_path(self.world_descriptor, [[0, 0]], [[1, 1]])
        self.assertTrue(path == (((0, 0), ), ((1, 0), ), ((1, 1), )) or
                        path == (((0, 0), ), ((0, 1), ), ((1, 1), )))
        self.assertTrue(validate_solution(path))
        self.world_descriptor[1][0] = 1
        path = od_mstar.find_path(self.world_descriptor, [[0, 0]], [[2, 0]])
        self.assertTrue(path == (((0, 0), ), ((0, 1), ),
                                 ((1, 1), ), ((2, 1), ), ((2, 0), )))
        # TEST 8-connected paths
        path = od_mstar.find_path(self.world_descriptor, [[0, 0]], [[1, 1]],
                                  connect_8=True)
        self.assertTrue(path == (((0, 0), ), ((1, 1), )))
        path = od_mstar.find_path(self.world_descriptor, [[0, 0]], [[2, 0]],
                                  connect_8=True)
        self.assertTrue(path == (((0, 0), ), ((1, 1), ), ((2, 0), )))

    def test_od_mstar_multirobot_paths(self):
        """Tests some simple multirobot cases on ODrM*"""
        path = od_mstar.find_path(self.world_descriptor,
                                  [[0, 0], [1, 0]], [[1, 0], [0, 0]])
        self.assertTrue(validate_solution(path))
        self.assertTrue(compute_cost(path) == 4)
        path = od_mstar.find_path(self.world_descriptor,
                                  [[0, 0], [1, 0], [5, 5]],
                                  [[1, 0], [0, 0], [6, 6]])
        self.assertTrue(validate_solution(path))
        self.assertTrue(compute_cost(path) == 6)
        # Test a simple three robot collision along edge
        path = od_mstar.find_path(self.world_descriptor,
                                  [[3, 0], [0, 0], [2, 0]],
                                  [[0, 0], [3, 0], [5, 0]])
        self.assertTrue(validate_solution(path))
        # TEST 8-connected paths
        path = od_mstar.find_path(self.world_descriptor,
                                  [[0, 0], [1, 0]], [[1, 0], [0, 0]],
                                  connect_8=True)
        self.assertTrue(validate_solution(path))
        self.assertTrue(compute_cost(path) == 3)
        path = od_mstar.find_path(self.world_descriptor,
                                  [[0, 0], [1, 0], [5, 5]],
                                  [[1, 0], [0, 0], [6, 6]], connect_8=True)
        self.assertTrue(validate_solution(path))
        self.assertTrue(compute_cost(path) == 4)
        # Test with some obstacles
        self.world_descriptor[5][0] = 1
        path = od_mstar.find_path(self.world_descriptor,
                                  [[4, 0], [6, 0]], [[6, 0], [4, 0]])
        self.assertTrue(validate_solution(path))
        self.assertTrue(compute_cost(path) == 10)

    def test_map_solutions(self):
        """Tests odrmstar on a couple of test maps, no constraints"""
        if not (FULL_TEST or OD_RMSTAR_FULL):
            self.skipTest('Skipped map test for brevity')
        print '\nod_rmstar long tests'
        dat = pickle.load(open('../maps/5_40_bots_step_5.map'))
        start_time = time.time()
        # Test inlfated
        for i in [0, 30, 60, 101, 131, 141, 151, 161, 200, 250, 300, 350, 400]:
            print i
            d = dat[i]
            path = od_mstar.find_path(d['obs_map'], d['init_pos'], d['goals'],
                                      inflation=10, time_limit=10,
                                      connect_8=True)
            self.assertTrue(validate_solution(path))
            self.assertTrue(path[-1] == d['goals'])
        # Test uniflated
        print 'uninflated'
        for i in [0, 30, 60, 101, 131, 141, 151, 161, 200, 227, 250,
                  275, 280]:  # , 301, 350]:
            print i
            d = dat[i]
            path = od_mstar.find_path(d['obs_map'], d['init_pos'], d['goals'],
                                      inflation=1, time_limit=60,
                                      connect_8=True)
            self.assertTrue(validate_solution(path))
            self.assertTrue(path[-1] == d['goals'])
        print 'time elapsed: %g' % (time.time() - start_time)


class TestEPEMstar(unittest.TestCase):
    """Testing that EPEM* is  working"""
    def setUp(self):
        self.world_descriptor = [[0 for i in xrange(10)] for j in xrange(10)]

    def test_neighbor(self):
        """Tests the neighbor generation code of EPEM*"""
        o = od_mstar.Od_Mstar(self.world_descriptor,
                              ((2, 2), (8, 8)), False,
                              connect_8=True, epeastar=True)
        node = o.gen_init_nodes(((4, 2), (6, 8)))[0]
        node.col_set = [frozenset([0, 1])]
        neighbors, col_set = o.gen_epeastar_coords(node)
        self.assertTrue(len(neighbors) == 9)
        neighbors, col_set = o.get_epeastar_neighbors(node)
        self.assertTrue(len(neighbors) == 9)

    # @unittest.skip('Debugging other code')
    def test_epemstar_multirobot_paths(self):
        """Tests some simple multirobot cases on EPEM*"""
        # pdb.set_trace()
        recursive = False
        path = od_mstar.find_path(self.world_descriptor,
                                  [[0, 0], [1, 0]],
                                  [[1, 0], [0, 0]], recursive=recursive,
                                  epemstar=True, time_limit=10)
        self.assertTrue(validate_solution(path))
        self.assertTrue(compute_cost(path) == 4)
        path = od_mstar.find_path(self.world_descriptor,
                                  [[0, 0], [1, 0], [5, 5]],
                                  [[1, 0], [0, 0], [6, 6]],
                                  recursive=recursive, epemstar=True,
                                  time_limit=10)
        self.assertTrue(validate_solution(path))
        self.assertTrue(compute_cost(path) == 6)
        # Test a simple three robot collision along edge
        path = od_mstar.find_path(self.world_descriptor,
                                  [[3, 0], [0, 0], [2, 0]],
                                  [[0, 0], [3, 0], [5, 0]],
                                  recursive=recursive,
                                  epemstar=True, time_limit=10)
        self.assertTrue(validate_solution(path))
        # TEST 8-connected paths
        path = od_mstar.find_path(self.world_descriptor,
                                  [[0, 0], [1, 0]],
                                  [[1, 0], [0, 0]], recursive=recursive,
                                  connect_8=True, epemstar=True,
                                  time_limit=10)
        self.assertTrue(validate_solution(path))
        self.assertTrue(compute_cost(path) == 3)
        path = od_mstar.find_path(self.world_descriptor,
                                  [[0, 0], [1, 0], [5, 5]],
                                  [[1, 0], [0, 0], [6, 6]], connect_8=True,
                                  recursive=recursive, epemstar=True,
                                  time_limit=10)
        self.assertTrue(validate_solution(path))
        self.assertTrue(compute_cost(path) == 4)
        # Test with some obstacles
        self.world_descriptor[5][0] = 1
        path = od_mstar.find_path(self.world_descriptor, [[4, 0], [6, 0]],
                                  [[6, 0], [4, 0]], recursive=recursive,
                                  epemstar=True, time_limit=10)
        self.assertTrue(validate_solution(path))
        self.assertTrue(compute_cost(path) == 10)

    def test_epermstar_multirobot_paths(self):
        """Tests some simple multirobot cases on EPEM*"""
        # pdb.set_trace()
        recursive = True
        path = od_mstar.find_path(self.world_descriptor,
                                  [[0, 0], [1, 0]],
                                  [[1, 0], [0, 0]], recursive=recursive,
                                  epemstar=True, time_limit=10)
        self.assertTrue(validate_solution(path))
        self.assertTrue(compute_cost(path) == 4)
        path = od_mstar.find_path(self.world_descriptor,
                                  [[0, 0], [1, 0], [5, 5]],
                                  [[1, 0], [0, 0], [6, 6]],
                                  recursive=recursive, epemstar=True,
                                  time_limit=10)
        self.assertTrue(validate_solution(path))
        self.assertTrue(compute_cost(path) == 6)
        # Test a simple three robot collision along edge
        path = od_mstar.find_path(self.world_descriptor,
                                  [[3, 0], [0, 0], [2, 0]],
                                  [[0, 0], [3, 0], [5, 0]],
                                  recursive=recursive,
                                  epemstar=True, time_limit=10)
        self.assertTrue(validate_solution(path))
        # TEST 8-connected paths
        path = od_mstar.find_path(self.world_descriptor, [[0, 0], [1, 0]],
                                  [[1, 0], [0, 0]], recursive=recursive,
                                  connect_8=True, epemstar=True,
                                  time_limit=10)
        self.assertTrue(validate_solution(path))
        self.assertTrue(compute_cost(path) == 3)
        path = od_mstar.find_path(self.world_descriptor,
                                  [[0, 0], [1, 0], [5, 5]],
                                  [[1, 0], [0, 0], [6, 6]], connect_8=True,
                                  recursive=recursive, epemstar=True,
                                  time_limit=10)
        self.assertTrue(validate_solution(path))
        self.assertTrue(compute_cost(path) == 4)
        # Test with some obstacles
        self.world_descriptor[5][0] = 1
        path = od_mstar.find_path(self.world_descriptor, [[4, 0], [6, 0]],
                                  [[6, 0], [4, 0]], recursive=recursive,
                                  epemstar=True, time_limit=10)
        self.assertTrue(validate_solution(path))
        self.assertTrue(compute_cost(path) == 10)

    def test_map_solutions(self):
        """Tests odrmstar on a couple of test maps, no constraints"""
        if not (FULL_TEST or EPERMSTAR_FULL):
            self.skipTest('Skipped map test for brevity')
        print '\nEPErM* long tests'
        dat = pickle.load(open('../maps/5_40_bots_step_5.map'))
        start_time = time.time()
        # Test inlfated
        for i in [0, 30, 60, 101, 131, 141, 151, 161, 200, 250, 300, 350,
                  370, 400, 500, 600]:
            print i
            d = dat[i]
            path = od_mstar.find_path(d['obs_map'], d['init_pos'], d['goals'],
                                      inflation=10, time_limit=60,
                                      recursive=True, connect_8=True,
                                      epemstar=True)
            self.assertTrue(validate_solution(path))
            self.assertTrue(path[-1] == d['goals'])
        # Test uniflated
        print 'uninflated'
        for i in [0, 30, 60, 101, 131, 141, 151, 161, 200, 227, 250,
                  275, 280]:  # , 301, 350]:
            print i
            d = dat[i]
            path = od_mstar.find_path(d['obs_map'], d['init_pos'], d['goals'],
                                      inflation=1, time_limit=60,
                                      recursive=True, connect_8=True,
                                      epemstar=True)
            self.assertTrue(validate_solution(path))
            self.assertTrue(path[-1] == d['goals'])
        print 'time elapsed: %g' % (time.time() - start_time)

    def test_map_non_recursive_solutions(self):
        """Tests odrmstar on a couple of test maps, no constraints"""
        if not (FULL_TEST or EPEMSTAR_FULL):
            self.skipTest('Skipped map test for brevity')
        print '\nEPEM* long tests'
        dat = pickle.load(open('../maps/5_40_bots_step_5.map'))
        start_time = time.time()
        # Test inlfated
        for i in [0, 30, 60, 101, 131, 141, 151, 161, 200, 250, 300, 350, 400]:
            print i
            d = dat[i]
            path = od_mstar.find_path(d['obs_map'], d['init_pos'], d['goals'],
                                      inflation=10, time_limit=60,
                                      recursive=False, connect_8=True,
                                      epemstar=True)
            self.assertTrue(validate_solution(path))
            self.assertTrue(path[-1] == d['goals'])
        # Test uniflated
        print 'uninflated'
        # for i in [0, 30, 60, 101, 131, 141, 151, 161, 200, 227, 250,
        #           275, 280]
        # , 301, 350]:
        for i in [0, 30, 60, 101, 131, 141, 151, 161]:
            print i
            d = dat[i]
            path = od_mstar.find_path(d['obs_map'], d['init_pos'], d['goals'],
                                      inflation=1, time_limit=60,
                                      recursive=False, connect_8=True,
                                      epemstar=True)
            self.assertTrue(validate_solution(path))
            self.assertTrue(path[-1] == d['goals'])
        print 'time elapsed: %g' % (time.time() - start_time)


class TestColSetMemory(unittest.TestCase):

    @debug_on()
    def test_effective_col_set(self):
        """Tests that the computation of the effective col set is correct"""
        def _assertEqual(col_1, col_2):
            self.assertTrue(set(col_1) == set(col_2))

        fset = frozenset
        ef_col_set = col_set_addition.effective_col_set
        _assertEqual(ef_col_set((fset([1, 2]), ), ()), (fset([1, 2]), ))
        _assertEqual(ef_col_set((), (fset([1, 2]), )), (fset([1, 2]), ))
        # test single containment
        _assertEqual(ef_col_set((fset([1, 2]), ), (fset([1, 2, 3]), )),
                     (fset([1, 2, 3]), ))
        # test overlap
        _assertEqual(ef_col_set((fset([1, 2]), fset([3, 4])),
                                   (fset([1, 2, 3]), )),
                     (fset([1, 2]), fset([3, 4])))
        # test multiple containment
        _assertEqual(ef_col_set((fset([1, 2]), fset([3, 4])),
                                   (fset([1, 2, 3, 4]), )),
                     (fset([1, 2, 3, 4]), ))
        _assertEqual(ef_col_set((fset([1, 2]), fset([3, 4]), fset([5, 6])),
                                   (fset([1, 2, 3, 4]), )),
                     (fset([1, 2, 3, 4]), fset([5, 6])))
        _assertEqual(ef_col_set((fset([1, 2]), fset([3, 4]), fset([5, 6])),
                                   (fset([1, 2, 3, 4]), fset([5, 6, 7]))),
                     (fset([1, 2, 3, 4]), fset([5, 6, 7])))
        _assertEqual(ef_col_set((fset([1, 2]), fset([3, 4]), fset([5, 6])),
                                   (fset([1, 2, 3, 4]), fset([6, 7]))),
                     (fset([1, 2, 3, 4]), fset([5, 6])))



#########################################################################
#
# HEPLER FUNCTIONS
#
#########################################################################
def validate_4_conn_path(path):
    """Validates that a given single robot would be a valid single robot
    path on a 4 connected grid

    path - [(x0, y0), ..., (xn, yn)] path to validate

    returns:
    True if path is valid, else False
    """
    for cur_state, next_state in zip(path[:-1], path[1:]):
        diffx = next_state[0] - cur_state[0]
        diffy = next_state[1] - cur_state[1]
        if abs(diffx) > 1 or abs(diffy) > 1:
            # Trying to move too far
            return False
        if abs(diffx) == 1 and abs(diffy) == 1:
            # diagonal move
            return False
    return True


def validate_solution(solution, conn_8=False, CMS=False):
    """Takes in a path in the full configuration space and tests whether
    it is valid.  (Will only check that no collisions exist, and will not
    check that all edges should exist, due to either connectivity or
    presence of obstacles
    """
    # Decompose joint path so it can be passed to validate path pair
    if not CMS:
        paths = [[(tuple(solution[dex][rob]), )
                  for dex in xrange(len(solution))]
                 for rob in xrange(len(solution[0]))]
    else:
        # Need to process via a rectangle collision checker.  Not
        # perfect, because this is the same collision checking code used
        # when running the planner, but don't have any other reasonable
        # way of testing, and at least this will catch any paths put
        # together wrong
        import rectangle_team_graphs
        col_checker = rectangle_team_graphs.Rectangle_Team_Planner_Checker()
        for coord in solution:
            if col_checker.col_check(coord):
                return False
        for coord1, coord2 in izip(solution, solution[1:]):
            if col_checker.cross_over(coord1, coord2):
                return False
        return True
    for i in xrange(len(paths) - 1):
        for j in xrange(i + 1, len(paths)):
            if (validate_path_pair(paths[i], (i, ), paths[j], (j, ))
                    is not None):
                return False
    return True


def validate_path_pair(path1, rob1, path2, rob2, conn_8=False):
    """Takes in a pair of paths, and the robots associated with them, and
    returns the indicies of the first pair of conflicting robots.  Assumes
    each path is itself collision free, so only need to worry about
    inter-path conflicts. Specific to grid worlds.  Returns a pair of
    constraints representing the first observed collision.  The constraints
    are applied to the joint set of robots


    path1 - (((x1, y1), (x2, y2), ...), ....) first path
    rob1 - tuple of robots in order executing first path
    path2 - (((x1, y1), (x2, y2), ...), ....) second path
    rob2 - tuple of robots executing second path

    returns - [robs], [nodes], t
       robs - index of involved robos
       nodes - coordinates of the node(s) at which the conflict occurs
       t - time at which coordinate
    """
    for t in xrange(max(len(path1), len(path2))):
        # extract the current coordinates along the path
        c1 = path1[min(t, len(path1) - 1)]
        c2 = path2[min(t, len(path2) - 1)]
        # extract the previsou coordinates along the path (for edge checking)
        p1 = path1[min(max(0, t - 1), len(path1) - 1)]
        p2 = path2[min(max(0, t - 1), len(path2) - 1)]
        for dex1, rob_id1 in enumerate(rob1):
            for dex2, rob_id2 in enumerate(rob2):
                diff = [c2[dex2][0] - c1[dex1][0], c2[dex2][1] - c1[dex1][1]]
                # Check for basic on top collisions
                if abs(diff[0]) > 1 or abs(diff[1]) > 1:
                    # No possible collisions
                    continue
                if c1[dex1] == c2[dex2]:
                    # both robots in the same location
                    con1 = con_empty_constraint(rob1)
                    con1 = con_add_node_constraint(con1, t, c1[dex1])
                    con2 = con_empty_constraint(rob2)
                    con2 = con_add_node_constraint(con2, t, c2[dex2])
                    return [con1, con2]
                if t == 0:
                    # Can't have edge collision
                    continue
                # Check for cross over collisions, will also catch pass
                # through collisions
                # if conn_8:
                # Compute the offset of the previous position for checking
                # cross over collisions
                pdiff = [p2[dex2][0] - p1[dex1][0], p2[dex2][1] - p1[dex1][1]]
                # Have a cross over if the difference has been reversed
                if diff[0] == -pdiff[0] and diff[1] == -pdiff[1]:
                    con1 = con_empty_constraint(rob1)
                    con1 = con_add_edge_constraint(con1, t, p1[dex1], c1[dex1])
                    con2 = con_empty_constraint(rob2)
                    con2 = con_add_edge_constraint(con2, t, p2[dex2], c2[dex2])
                    return [con1, con2]
    return None


def compute_cost(path):
    goals = path[-1]
    cost = 0
    for i in xrange(1, len(path)):
        start_coord = path[i - 1]
        for k in xrange(len(goals)):
            if not(start_coord[k] == goals[k] and
                   start_coord[k] == path[i][k]):
                cost += 1
    return cost


def compute_makespan(path):
    """Computes the makespan (time until all robots are done)

    path - path to compute makespan
    """
    return len(path) - 1


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
    parser.add_argument('-f', dest='full_test', action='store_true',
                        help='Full test, including map tests')
    parser.add_argument('--odrmstar', dest='odrmstar_full',
                        action='store_true',
                        help='Full test for ODrM*, including map tests')
    parser.add_argument('--rmstar', dest='rmstar_full', action='store_true',
                        help='Full test for rM*, including map tests')
    parser.add_argument('--opdecomp', dest='opdecomp_full',
                        action='store_true',
                        help='Full test for operator decomposition, ' +
                        'including map tests')
    parser.add_argument('--epermstar', dest='epermstar_full',
                        action='store_true',
                        help='Full test of EPErM*, including map tests')
    parser.add_argument('--epemstar', dest='epemstar_full',
                        action='store_true',
                        help='Full test of EPEM*, including map tests')
    parser.add_argument('--debugoff', dest='debug_off', action='store_false',
                        help='Turns off activating pdb on error')
    args = parser.parse_args()
    global DEBUG
    DEBUG = args.debug_off
    # global FULL_TEST
    sys.argv = ['mstar_test.py']
    global FULL_TEST, OD_RMSTAR_FULL
    global EPERMSTAR_FULL, RMSTAR_FULL, EPEMSTAR_FULL
    # Enable full test if the user manually specified the tests to run
    FULL_TEST = args.full_test or args.tests
    OD_RMSTAR_FULL = args.odrmstar_full
    EPERMSTAR_FULL = args.epermstar_full
    RMSTAR_FULL = args.rmstar_full
    EPEMSTAR_FULL = args.epemstar_full
    if args.tests:
        # Have a list of tests that we want to run
        suite = unittest.TestLoader().loadTestsFromNames(
            args.tests, module=sys.modules[__name__])
        unittest.TextTestRunner(verbosity=args.verbosity).run(suite)
    else:
        unittest.main(verbosity=args.verbosity)


if __name__ == '__main__':
    main()
