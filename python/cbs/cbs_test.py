#!/usr/bin/python
import unittest
import argparse
import pickle
import time
import sys
import cbs
import constrained_od_mstar
import constrained_planner
import ipdb

"""Unit test suite for cbs"""

#Remember tests can be marked as skipped with the decorators
# @unittest.skip('message goes here')
# @unittest.skipIf(conditional, 'message')
# @unittest.skipUnless(conditional, 'message')


FULL_TEST = False
CBS_FULL = False
MCBS_FULL = False
COD_rMSTAR_FULL = False
CEPErMSTAR_FULL = False
MERGE_THRESH = 10

class TestCBSConstraintFunctions(unittest.TestCase):
    def setUp(self):
        """Is called before every test function is run

        """
        self.con = cbs.con_empty_constraint([1])
        self.con2 = cbs.con_empty_constraint([1])
        
        self.pop_con = cbs.con_empty_constraint([1])
        self.pop_con = cbs.con_add_node_constraint(self.pop_con, 1, (2, 2))
        self.pop_con = cbs.con_add_node_constraint(self.pop_con, 3, (2, 2))
        self.pop_con = cbs.con_add_node_constraint(self.pop_con, 3, (3, 3))
        self.pop_con = cbs.con_add_node_constraint(self.pop_con, 3, (4, 4))
        self.pop_con = cbs.con_add_edge_constraint(self.pop_con, 4, (2, 2), 
                                                   (2, 3))
        self.pop_con = cbs.con_add_edge_constraint(self.pop_con, 5, (2, 2), 
                                                   (2, 3))
        self.pop_con = cbs.con_add_edge_constraint(self.pop_con, 5, (2, 2), 
                                                   (3, 3))
        self.pop_con = cbs.con_add_edge_constraint(self.pop_con, 5, (2, 2), 
                                                   (4, 4))

    def tearDown(self):
        """Callled after every test function run"""
        pass

    def test_constraint_initialization(self):
        """Tests that the empty constraints are initialized properly"""
        self.assertTrue(self.con == ((1, ), ()))
        self.assertTrue(self.con == cbs.con_empty_constraint(1))
        self.assertTrue(self.con == cbs.con_empty_constraint((1, )))
        self.assertTrue(not(self.con == cbs.con_empty_constraint((1, 2))))

    def test_get_robot(self):
        """tests that the get robot command works as expected"""
        self.assertTrue(cbs.con_get_robots(self.con) == (1, ))
        temp = cbs.con_empty_constraint((1, 2))
        self.assertTrue(cbs.con_get_robots(temp) == (1, 2))

    def test_constraint_sorting(self):
        """Constraint object should be independent of order of constraint 
        addition

        """
        self.con = cbs.con_add_node_constraint(self.con, 1, (2, 2))
        self.con = cbs.con_add_node_constraint(self.con, 3, (2, 2))
        #Check that this has actually changed the constraints
        self.assertTrue(self.con != self.con2)
        
        self.con2 = cbs.con_add_node_constraint(self.con2, 3, (2, 2))
        self.con2 = cbs.con_add_node_constraint(self.con2, 1, (2, 2))
        #Make sure the constraints remain sorted
        self.assertTrue(self.con == self.con2)
        
        #Test the addition of edges
        self.con = cbs.con_add_edge_constraint(self.con, 4, (2, 2), (2, 3))
        self.con = cbs.con_add_edge_constraint(self.con, 5, (2, 2), (2, 3))
        #Check that this has actually changed the constraints
        self.assertTrue(self.con != self.con2)
        
        self.con2 = cbs.con_add_edge_constraint(self.con2, 4, (2, 2), (2, 3))
        self.con2 = cbs.con_add_edge_constraint(self.con2, 5, (2, 2), (2, 3))
        self.assertTrue(self.con == self.con2)

    def test_constraint_positive_detection(self):
        """confirm that collisions are detected"""
        #Confirm detect positive node collision
        self.assertTrue(cbs.con_is_constrained(self.pop_con, 1, (0, 0), (2, 2)))
        self.assertTrue(cbs.con_is_constrained(self.pop_con, 3, (0, 0), (2, 2)))
        self.assertTrue(cbs.con_is_constrained(self.pop_con, 3, (0, 0), (3, 3)))
        self.assertTrue(cbs.con_is_constrained(self.pop_con, 3, (0, 0), (4, 4)))
        #confirm positive edge detection
        self.assertTrue(cbs.con_is_constrained(self.pop_con, 4, (2, 2), (2, 3)))
        self.assertTrue(cbs.con_is_constrained(self.pop_con, 5, (2, 2), (2, 3)))
        self.assertTrue(cbs.con_is_constrained(self.pop_con, 5, (2, 2), (3, 3)))
        self.assertTrue(cbs.con_is_constrained(self.pop_con, 5, (2, 2), (4, 4)))

    def test_constraint_negative_detection(self):
        """Confirm that there are not false positives"""
        #Confirm ignore non-collision
        self.assertTrue(not cbs.con_is_constrained(self.pop_con, 1, (0, 0), 
                                                   (1, 1)))
        #Confirm recognizes time offset
        self.assertTrue(not cbs.con_is_constrained(self.pop_con, 2, (0, 0), 
                                                   (2, 2)))
        #confirm edge time offset
        self.assertTrue(not cbs.con_is_constrained(self.pop_con, 3, (2, 2), 
                                                   (2, 3)))
        #confirm edge single vertex offsets
        self.assertTrue(not cbs.con_is_constrained(self.pop_con, 4, (2, 1), 
                                                   (2, 3)))
        self.assertTrue(not cbs.con_is_constrained(self.pop_con, 4, (2, 2), 
                                                   (2, 1)))
        #confirm reversed edges not detected
        self.assertTrue(not cbs.con_is_constrained(self.pop_con, 4, (2, 3), 
                                                   (2, 2)))
        self.assertTrue(not cbs.con_is_constrained(self.pop_con, 5, (2, 3), 
                                                   (2, 2)))

    def test_constraint_max_time(self):
        """Verify detection of maximum time in constraint"""
        self.assertTrue(cbs.con_get_max_time(self.pop_con) == 5)

    def test_constraint_merge(self):
        """Test that con_merge_constraints operates correctly"""
        self.con = cbs.con_add_node_constraint(self.con, 1, (0, 0))
        self.con = cbs.con_add_node_constraint(self.con, 1, (1, 0))
        self.con = cbs.con_add_edge_constraint(self.con, 0, (3, 3), (4, 4))
        distractor =  cbs.con_empty_constraint([3])
        distractor = cbs.con_add_node_constraint(distractor, 1, (0, 0))
        con_set = (self.con2, distractor)
        temp =  cbs.con_empty_constraint([1])
        temp = cbs.con_add_node_constraint(temp, 1, (0, 0))
        con_set = cbs.con_merge_constraints(con_set, temp)
        temp =  cbs.con_empty_constraint([1])
        temp = cbs.con_add_node_constraint(temp, 1, (1, 0))
        con_set = cbs.con_merge_constraints(con_set, temp)
        temp =  cbs.con_empty_constraint([1])
        temp = cbs.con_add_edge_constraint(temp, 0, (3, 3), (4, 4))
        con_set = cbs.con_merge_constraints(con_set, temp)
        self.assertTrue(con_set == (self.con, distractor))

    def test_agent_merge(self):
        """Test that mergeing agents behaves as expected"""
        constraints = [cbs.con_empty_constraint([i]) for i in range(10)]
        new_con = cbs.con_merge_agents(constraints, (2, ), (3, ))

        self.assertTrue(cbs.con_empty_constraint((2, 3)) in new_con)
        self.assertTrue(len(new_con) == 9)
        for i in ([0, 1] + range(4, 10)):
            self.assertTrue(cbs.con_empty_constraint((i, )) in new_con)
        #Check merging a single robot with a meta agent
        new_con = cbs.con_merge_agents(new_con, (9, ), (2, 3))
        self.assertTrue(cbs.con_empty_constraint((2, 3, 9)) in new_con)
        self.assertTrue(len(new_con) == 8)
        for i in ([0, 1] + range(4, 9)):
            self.assertTrue(cbs.con_empty_constraint((i, )) in new_con)
        #Check merging between two meta agents
        new_con = cbs.con_merge_agents(new_con, (1, ), (0, ))
        new_con = cbs.con_merge_agents(new_con, (0, 1), (2, 3, 9))
        self.assertTrue(cbs.con_empty_constraint((0, 1, 2, 3, 9)) in new_con)
        self.assertTrue(len(new_con) == 6)
        for i in (range(4, 9)):
            self.assertTrue(cbs.con_empty_constraint((i, )) in new_con)
        #Test the behavior of constraints with prohibited states
        c = cbs.con_empty_constraint((10, ))
        c = cbs.con_add_node_constraint(c, 1, (3, 4))
        new_con += (c, )
        #Test that merging an uninvolved robot does not alter other constraints
        new_con = cbs.con_merge_agents(new_con, (4, ), (5, ))
        self.assertTrue(cbs.con_empty_constraint((0, 1, 2, 3, 9)) in new_con)
        self.assertTrue(c in new_con)
        self.assertTrue(cbs.con_empty_constraint((4, 5)) in new_con)
        #Test that merging in a constraint with prohibited states sees those
        #prohibited states removed
        new_con = cbs.con_merge_agents(new_con, (4, 5), (10, ))
        self.assertTrue(cbs.con_empty_constraint((0, 1, 2, 3, 9)) in new_con)
        self.assertTrue(cbs.con_empty_constraint((4, 5, 10)) in new_con)
        

class TestConstraintPlanner(unittest.TestCase):
    def setUp(self):
        """Is called before every test function is run"""
        self.obs_map = [[0 for i in xrange(10)] for j in xrange(10)]
        self.con = cbs.con_empty_constraint([1])

    def test_execution(self):
        """Tests if a simple straight path can be found"""
        path, cost = constrained_planner.find_constrained_path(
            self.obs_map, (0, 0), (1, 0), self.con)
        self.assertTrue(cost == 1)
        self.assertTrue(path == ((0, 0), (1, 0)))

    def test_simple_node_avoidance(self):
        """Tests if a simple node constraint will be obeyed"""
        self.con = cbs.con_add_node_constraint(self.con, 1, (1, 0))
        path, cost = constrained_planner.find_constrained_path(
            self.obs_map, (0, 0), (1, 0), self.con)
        self.assertTrue(path[1] != (1, 0))
        self.assertTrue(cost == 2)
        self.assertTrue(path[-1] == (1, 0))

    def test_simple_edge_avoidance(self):
        """Tests if a simple node constraint will be obeyed"""
        self.con = cbs.con_add_edge_constraint(self.con, 1, (0, 0), (1, 0))
        path, cost = constrained_planner.find_constrained_path(
            self.obs_map, (0, 0), (1, 0), self.con)
        self.assertTrue(path[1] != (1, 0))
        self.assertTrue(cost == 2)
        self.assertTrue(path[-1] == (1, 0))


    def test_false_positives(self):
        """Test if non-conflicting constraints can be ignored"""
        self.con = cbs.con_add_edge_constraint(self.con, 1, (0, 1), (1, 0))
        self.con = cbs.con_add_edge_constraint(self.con, 1, (1, 0), (0, 0))
        self.con = cbs.con_add_edge_constraint(self.con, 2, (0, 0), (1, 0))
        self.con = cbs.con_add_node_constraint(self.con, 1, (0, 0))
        self.con = cbs.con_add_node_constraint(self.con, 2, (1, 1))
        path, cost = constrained_planner.find_constrained_path(
            self.obs_map, (0, 0), (1, 0), self.con)
        self.assertTrue(cost == 1)
        self.assertTrue(path[-1] == (1, 0))


    def test_obstacle_avoidance(self):
        """Test to ensure that obstacles are actually avoided"""
        self.obs_map[1][0] = 1
        path, cost = constrained_planner.find_constrained_path(
            self.obs_map, (0, 0), (2, 0), self.con)
        self.assertTrue(cost == 4)
        self.assertTrue(not (1, 0) in path)

    def test_constraint_delay(self):
        """Tests that the constrained planner will plan beyond the constraint"""
        #Test with an irrelevant constraint
        con = cbs.con_add_node_constraint(self.con, 10, (3, 3))
        path, cost = constrained_planner.find_constrained_path(
            self.obs_map, (0, 0), (1, 0), con)
        self.assertTrue(len(path) == 12)
        self.assertTrue(cost == 1)
        #Test with a constraint that appears at the goal
        con = cbs.con_add_node_constraint(self.con, 10, (1, 0))
        path, cost = constrained_planner.find_constrained_path(
            self.obs_map, (0, 0), (1, 0), con)
        self.assertTrue(len(path) == 12)
        self.assertTrue(cost == 3)

    def test_graph_get_neighbors(self):
        """Test constraint_planners get neighbors graph functionality"""
        #Test with empty con
        p = constrained_planner.Constrained_Planner(self.obs_map, [0, 0], 
                                                    [2, 0], self.con)
        n = p.get_neighbors((1, 1, 0))
        for i in [(0, 1, 1), (1, 2, 1), (2, 1, 1), (1, 0, 1)]:
            self.assertTrue(i in n)
        n = p.get_neighbors((1, 1, 1))
        for i in [(0, 1, 1), (1, 2, 1), (2, 1, 1), (1, 0, 1)]:
            self.assertTrue(i in n)

        #Test that constraint violating nodes are correctly rejected
        con = cbs.con_add_node_constraint(self.con, 3, (1, 2))
        p = constrained_planner.Constrained_Planner(self.obs_map, [0, 0], 
                                                    [2, 2], con)
        n = p.get_neighbors((1, 1, 2))
        for i in [(0, 1, 3), (2, 1, 3), (1, 0, 3)]:
            self.assertTrue(i in n)
        self.assertTrue((1, 2, 3) not in n)
        #Check that constraints don't bleed in time
        n = p.get_neighbors((1, 1, 3))
        for i in [(0, 1, 4), (1, 2, 4), (2, 1, 4), (1, 0, 4)]:
            self.assertTrue(i in n)

        #Test edge constraints
        con = cbs.con_add_edge_constraint(self.con, 3, (1, 1), (1, 2))
        p = constrained_planner.Constrained_Planner(self.obs_map, [0, 0], 
                                                    [2, 0], con)
        n = p.get_neighbors((1, 1, 2))
        for i in [(0, 1, 3), (2, 1, 3), (1, 0, 3)]:
            self.assertTrue(i in n)
        self.assertTrue((1, 2, 3) not in n)
        #Check that constraints don't bleed in time
        n = p.get_neighbors((1, 1, 3))
        for i in [(0, 1, 4), (1, 2, 4), (2, 1, 4), (1, 0, 4)]:
            self.assertTrue(i in n)

    def test_graph_get_limited_offset_neighbors(self):
        """Tests the generation of neighbors with specified offsets"""
        p = constrained_planner.Constrained_Planner(self.obs_map, [5, 5], 
                                                    [3, 5], self.con)
        offset_neighbors = p.get_limited_offset_neighbors((5, 5, 0), 0)
        offset_neighbors = [i[-1] for i in offset_neighbors]
        self.assertTrue(len(offset_neighbors) == 1)
        for i in [(4, 5, 1)]:
            self.assertTrue(i in offset_neighbors)

        offset_neighbors = p.get_limited_offset_neighbors((5, 5, 0), 1)
        offset_neighbors = [i[-1] for i in offset_neighbors]
        self.assertTrue(len(offset_neighbors) == 2)
        for i in [(4, 5, 1), (5, 5, 1)]:
            self.assertTrue(i in offset_neighbors)

        offset_neighbors = p.get_limited_offset_neighbors((5, 5, 0), 2)
        offset_neighbors = [i[-1] for i in offset_neighbors]
        self.assertTrue(len(offset_neighbors) == 5)
        for i in [(4, 5, 1), (5, 5, 1), (5, 4, 1), (5, 6, 1), (6, 5, 1)]:
            self.assertTrue(i in offset_neighbors)

    def test_graph_get_limited_offset_neighbors_8_conn(self):
        """Tests the generation of neighbors with specified offsets"""
        p = constrained_planner.Constrained_Planner(self.obs_map, [5, 5], 
                                                    [3, 5], self.con, 
                                                    conn_8=True)
        offset_neighbors = p.get_limited_offset_neighbors((5, 5, 0), 0)
        offset_neighbors = [i[-1] for i in offset_neighbors]
        self.assertTrue(len(offset_neighbors) == 3)
        for i in [(4, 5, 1), (4, 4, 1), (4, 6, 1)]:
            self.assertTrue(i in offset_neighbors)

        offset_neighbors = p.get_limited_offset_neighbors((5, 5, 0), 1)
        offset_neighbors = [i[-1] for i in offset_neighbors]
        self.assertTrue(len(offset_neighbors) == 6)
        for i in [(4, 5, 1), (4, 4, 1), (4, 6, 1), (5, 5, 1), (5, 4, 1), 
                  (5, 6, 1)]:
            self.assertTrue(i in offset_neighbors)

        offset_neighbors = p.get_limited_offset_neighbors((5, 5, 0), 2)
        offset_neighbors = [i[-1] for i in offset_neighbors]
        self.assertTrue(len(offset_neighbors) == 9)
        for i in  [(4, 5, 1), (4, 4, 1), (4, 6, 1), (5, 5, 1), (5, 4, 1), 
                   (5, 6, 1), (6, 6, 1), (6, 5, 1), (6, 4, 1)]:
            self.assertTrue(i in offset_neighbors)

    def test_graph_get_step(self):
        """Test constrain_planner's policy generation methods"""
        p = constrained_planner.Constrained_Planner(self.obs_map, [0, 0], 
                                                    [2, 0], self.con)
        # print p.get_step((0, 0, 0)) == (1, 0, 1)
        self.assertTrue(p.get_step((0, 0, 0)) == (1, 0, 1))
        self.assertTrue(p.get_step((1, 0, 1)) == (2, 0, 1))
        con = cbs.con_add_node_constraint(self.con, 13, (9, 9))
        p = constrained_planner.Constrained_Planner(self.obs_map, [0, 0], 
                                                    [2, 0], con)
        self.assertTrue(p.get_step((2, 9, 11)) == (2, 8, 12))

    def test_tie_breaking(self):
        """Tests that Constrained__Planner performs tie breaking properly"""
        path, cost = constrained_planner.find_constrained_path(
            self.obs_map, (0, 0), (1, 1), self.con, out_paths=(((1, 0), ), 
                                                               ((1, 0), )))
        self.assertTrue(path == ((0, 0), (0, 1), (1, 1)))
        self.assertTrue(cost == 2)
        path, cost = constrained_planner.find_constrained_path(
            self.obs_map, (0, 0), (1, 1), self.con, 
            out_paths=(((1, 0), ), ((1, 0), ), ((1, 0), ), ((1, 0), )))
        self.assertTrue(path[:3] == ((0, 0), (0, 1), (1, 1)))
        self.assertTrue(cost == 2)
        path, cost = constrained_planner.find_constrained_path(
            self.obs_map, (0, 0) , (1, 1), self.con, out_paths=(((0, 1), ), 
                                                                ((0, 1), )))
        self.assertTrue(path == ((0, 0), (1, 0), (1, 1)))
        self.assertTrue(cost == 2)
        #Tests that it will ride over constraints when necessary
        path, cost = constrained_planner.find_constrained_path(
            self.obs_map, (0, 0), (1, 1), self.con, 
            out_paths=(((0, 1), (1, 0)), ((0, 1), (1, 0))))
        self.assertTrue(path[:3] == ((0, 0), (1, 0), (1, 1)) or 
                        path[:3] == ((0, 0), (0, 1), (1, 1)))
        self.assertTrue(cost == 2)
        #Tests planning when given a longer out_path
        path, cost = constrained_planner.find_constrained_path(
            self.obs_map, (0, 0), (1, 1), self.con, out_paths=(((3, 3), ), 
                                                               ((3, 3), ), 
                                                               ((3, 3), ), 
                                                               ((3, 3), ), 
                                                               ((3, 3), ), 
                                                               ((3, 3), ), ))
        self.assertTrue(cost == 2)
        self.assertTrue(path[:3] == ((0, 0), (1, 0), (1, 1)) or
                        path[:3] == ((0, 0), (0, 1), (1, 1)))


class TestForwardsConstraintPlanner(unittest.TestCase):

    def setUp(self):
        """Is called before every test function is run"""
        self.obs_map = [[0 for i in xrange(10)] for j in xrange(10)]
        self.con = cbs.con_empty_constraint([1])

    def test_execution(self):
        """Tests if a simple straight path can be found"""
        path, cost = constrained_planner.find_forwards_constrained_path(
                                                              self.obs_map, 
                                                              (0, 0), (1, 0), 
                                                              self.con)
        self.assertTrue(cost == 1)
        self.assertTrue(path == ((0, 0), (1, 0)))

    def test_simple_node_avoidance(self):
        """Tests if a simple node constraint will be obeyed"""
        self.con = cbs.con_add_node_constraint(self.con, 1, (1, 0))
        path, cost = constrained_planner.find_forwards_constrained_path(
                                                              self.obs_map, 
                                                              (0, 0), (1, 0)
                                                              , self.con)
        self.assertTrue(path[1] != (1, 0))
        self.assertTrue(cost == 2)
        self.assertTrue(path[-1] == (1, 0))

    def test_simple_edge_avoidance(self):
        """Tests if a simple node constraint will be obeyed"""
        self.con = cbs.con_add_edge_constraint(self.con, 1, (0, 0), (1, 0))
        path, cost = constrained_planner.find_forwards_constrained_path(
                                                              self.obs_map, 
                                                              (0, 0), (1, 0), 
                                                              self.con)
        self.assertTrue(path[1] != (1, 0))
        self.assertTrue(cost == 2)
        self.assertTrue(path[-1] == (1, 0))

    def test_false_positives(self):
        """Test if non-conflicting constraints can be ignored"""
        self.con = cbs.con_add_edge_constraint(self.con, 1, (0, 1), (1, 0))
        self.con = cbs.con_add_edge_constraint(self.con, 1, (1, 0), (0, 0))
        self.con = cbs.con_add_edge_constraint(self.con, 2, (0, 0), (1, 0))
        self.con = cbs.con_add_node_constraint(self.con, 1, (0, 0))
        self.con = cbs.con_add_node_constraint(self.con, 2, (1, 1))
        path, cost = constrained_planner.find_forwards_constrained_path(
                                                              self.obs_map, 
                                                              (0, 0), (1, 0), 
                                                              self.con)
        self.assertTrue(cost == 1)
        self.assertTrue(path[-1] == (1, 0))

    def test_obstacle_avoidance(self):
        """Test to ensure that obstacles are actually avoided"""
        self.obs_map[1][0] = 1
        path, cost = constrained_planner.find_forwards_constrained_path(
                                                              self.obs_map, 
                                                              (0, 0), (2, 0), 
                                                              self.con)
        self.assertTrue(cost == 4)
        self.assertTrue(not (1, 0) in path)

    def test_constraint_delay(self):
        """Tests that the constrained planner will plan beyond the 
        constraint
        """
        #Test with an irrelevant constraint
        con = cbs.con_add_node_constraint(self.con, 10, (3, 3))
        path, cost = constrained_planner.find_forwards_constrained_path(
                                                              self.obs_map, 
                                                              (0, 0), (1, 0),
                                                              con)
        self.assertTrue(len(path) == 12)
        self.assertTrue(cost == 1)
        #Test with a constraint that appears at the goal
        con = cbs.con_add_node_constraint(self.con, 10, (1, 0))
        path, cost = constrained_planner.find_forwards_constrained_path(
                                                              self.obs_map, 
                                                              (0, 0), (1, 0),
                                                              con)
        self.assertTrue(len(path) == 12)
        self.assertTrue(cost == 3)

    def test_tie_breaking(self):
        """Tests that Constrained_Forwards_Planner performs tie breaking 
        properly

        """
        path, cost = constrained_planner.find_forwards_constrained_path(
            self.obs_map, (0, 0), (1, 1), self.con, out_paths=(((1, 0), ),
                                                               ((1, 0), )))
        self.assertTrue(path == ((0, 0), (0, 1), (1, 1)))
        self.assertTrue(cost == 2)
        path, cost = constrained_planner.find_forwards_constrained_path(
            self.obs_map, (0, 0), (1, 1), self.con, out_paths=(((0, 1), ),
                                                               ((0, 1), )))
        self.assertTrue(path == ((0, 0), (1, 0), (1, 1)))
        self.assertTrue(cost == 2)
        #Tests that it will ride over constraints when necessary
        path, cost = constrained_planner.find_forwards_constrained_path(
            self.obs_map, (0, 0), (1, 1), self.con, 
            out_paths=(((0, 1), (1, 0)), ((0, 1), (1, 0))))
        self.assertTrue(path == ((0, 0), (1, 0), (1, 1)) or 
                        path == ((0, 0), (0, 1), (1, 1)))
        self.assertTrue(cost == 2)
        #Tests planning when given a longer out_path
        path, cost = constrained_planner.find_forwards_constrained_path(
            self.obs_map, (0, 0), (1, 1), self.con, 
            out_paths= (((3, 3), ), ((3, 3), ), ((3, 3), ), ((3, 3), ), 
                        ((3, 3), ), ((3, 3), ), ) )
        self.assertTrue(cost == 2)
        self.assertTrue(path == ((0, 0), (1, 0), (1, 1)) or
                        path == ((0, 0), (0, 1), (1, 1)))

    def test_graph_get_neighbors(self):
        """Test constraint_planners get neighbors graph functionality"""
        #Test with empty con
        p = constrained_planner.Constrained_Planner(self.obs_map, [0, 0], 
                                                    [2, 0], self.con)
        n = p.get_neighbors((1, 1, 0))
        for i in [(0, 1, 1), (1, 2, 1), (2, 1, 1), (1, 0, 1)]:
            self.assertTrue(i in n)
        n = p.get_neighbors((1, 1, 1))
        for i in [(0, 1, 1), (1, 2, 1), (2, 1, 1), (1, 0, 1)]:
            self.assertTrue(i in n)

        #Test that constraint violating nodes are correctly rejected
        con = cbs.con_add_node_constraint(self.con, 3, (1, 2))
        p = constrained_planner.Constrained_Planner(self.obs_map, [0, 0], 
                                                    [2, 2], con)
        n = p.get_neighbors((1, 1, 2))
        for i in [(0, 1, 3), (2, 1, 3), (1, 0, 3)]:
            self.assertTrue(i in n)
        self.assertTrue((1, 2, 3) not in n)
        #Check that constraints don't bleed in time
        n = p.get_neighbors((1, 1, 3))
        for i in [(0, 1, 4), (1, 2, 4), (2, 1, 4), (1, 0, 4)]:
            self.assertTrue(i in n)

        #Test edge constraints
        con = cbs.con_add_edge_constraint(self.con, 3, (1, 1), (1, 2))
        p = constrained_planner.Constrained_Planner(self.obs_map, [0, 0], 
                                                    [2, 0], con)
        n = p.get_neighbors((1, 1, 2))
        for i in [(0, 1, 3), (2, 1, 3), (1, 0, 3)]:
            self.assertTrue(i in n)
        self.assertTrue((1, 2, 3) not in n)
        #Check that constraints don't bleed in time
        n = p.get_neighbors((1, 1, 3))
        for i in [(0, 1, 4), (1, 2, 4), (2, 1, 4), (1, 0, 4)]:
            self.assertTrue(i in n)

    def test_graph_get_step(self):
        """Test constrain_planner's policy generation methods"""
        p = constrained_planner.Constrained_Planner(self.obs_map, [0, 0], 
                                                    [2, 0], self.con)
        # print p.get_step((0, 0, 0)) == (1, 0, 1)
        self.assertTrue(p.get_step((0, 0, 0)) == (1, 0, 1))
        self.assertTrue(p.get_step((1, 0, 1)) == (2, 0, 1))
        con = cbs.con_add_node_constraint(self.con, 13, (9, 9))
        p = constrained_planner.Constrained_Planner(self.obs_map, [0, 0], 
                                                    [2, 0], con)
        self.assertTrue(p.get_step((2, 9, 11)) == (2, 8, 12))


class TestSum_Of_Costs_Constrained_Forwards_Planner(unittest.TestCase):

    def setUp(self):
        """Is called before every test function is run"""
        self.obs_map = [[0 for i in xrange(10)] for j in xrange(10)]
        self.con = cbs.con_empty_constraint([1])

    def test_delay_cost(self):
        con = cbs.con_add_node_constraint(self.con, 5, (0, 0))

        p = constrained_planner.Sum_Of_Cost_Constrained_Forwards_Planner(
            self.obs_map, (0, 0), (0, 0), con)
        path, cost = p.find_path(time_limit=10)
        self.assertTrue(cost == 6)

        p = constrained_planner.Constrained_Forwards_Planner(
            self.obs_map, (0, 0), (0, 0), con)
        path, cost = p.find_path(time_limit=10)
        self.assertTrue(cost == 2)

        # Check with two sequential constraints
        con = cbs.con_add_node_constraint(con, 10, (0, 0))

        p = constrained_planner.Sum_Of_Cost_Constrained_Forwards_Planner(
            self.obs_map, (0, 0), (0, 0), con)
        path, cost = p.find_path(time_limit=10)
        self.assertTrue(cost == 11)

        p = constrained_planner.Constrained_Forwards_Planner(
            self.obs_map, (0, 0), (0, 0), con)
        path, cost = p.find_path(time_limit=10)
        self.assertTrue(cost == 4)

class TestPathValidation(unittest.TestCase):
    # def setUp(self):
    #     """Is called before every test function is run"""
    #     self.

    def test_single_false_positives(self):
        """Tests for false positives"""
        p1 = map(lambda x:(x, ), ((0, 0), (0, 1), (0, 2), (0, 3), (0, 4)))
        p2 = map(lambda x:(x, ), ((0, 3), (0, 2), (0, 3), (1, 3), (2, 3)))
        new_cons = cbs.validate_path_pair(p1, (1, ), p2, (2, ))
        self.assertTrue(new_cons == None)
        p1 = map(lambda x:(x, ), ((0, 0), (0, 1), (0, 2), (0, 3), (0, 4)))
        p2 = map(lambda x:(x, ), ((0, 5), ))
        new_cons = cbs.validate_path_pair(p1, (1, ), p2, (2, ))
        self.assertTrue(new_cons == None)

    def test_single_bots_simultaneous(self):
        """Tests to make sure simultaneous occupation is detected"""
        p1 = map(lambda x:(x, ), ((0, 0), (0, 1), (0, 2), (0, 3), (0, 4)))
        p2 = map(lambda x:(x, ), ((0, 4), (0, 3), (0, 2), (0, 1), (0, 0)))
        new_cons = cbs.validate_path_pair(p1, (1, ), p2, (2, ))
        con1 = cbs.con_add_node_constraint(cbs.con_empty_constraint((1, )), 2, 
                                           (0, 2))
        con2 = cbs.con_add_node_constraint(cbs.con_empty_constraint((2, )), 2, 
                                           (0, 2))
        self.assertTrue( con1 in new_cons and con2 in new_cons)
        p1 = map(lambda x:(x, ), ((0, 0), (0, 1), (0, 2), (0, 3), (0, 4)))
        p2 = map(lambda x:(x, ), ((0, 4), ))
        new_cons = cbs.validate_path_pair(p1, (1, ), p2, (2, ))
        con1 = cbs.con_add_node_constraint( cbs.con_empty_constraint((1, )) , 4,
                                            (0, 4))
        con2 = cbs.con_add_node_constraint( cbs.con_empty_constraint((2, )) , 4,
                                            (0, 4))
        self.assertTrue( con1 in new_cons and con2 in new_cons)

    def test_single_bots_pass_through(self):
        """Tests to make sure simultaneous occupation is detected"""
        p1 = map(lambda x:(x, ), ((0, 0), (0, 1), (0, 2), (0, 3), (0, 4)))
        p2 = map(lambda x:(x, ), ((0, 5), (0, 4), (0, 3), (0, 2), (0, 1)))
        new_cons = cbs.validate_path_pair(p1, (1, ), p2, (2, ))
        con1 = cbs.con_add_edge_constraint(cbs.con_empty_constraint((1, )) , 3,
                                           (0, 2), (0, 3))
        con2 = cbs.con_add_edge_constraint(cbs.con_empty_constraint((2, )) , 3,
                                           (0, 3), (0, 2))
        self.assertTrue( con1 in new_cons and con2 in new_cons)

    def test_meta_agent_false_positives(self):
        """tests for false positives with meta-agents"""
        p11 = ((0, 0), (0, 1), (0, 2), (0, 3), (0, 4))
        p12 = ((5, 5), (5, 6), (5, 7), (5, 8), (5, 9))
        p1 = map(list, zip(p11, p12))
        p2 = map(lambda x:(x, ), ((0, 3), (0, 2), (0, 3), (1, 3), (2, 3)))
        new_cons = cbs.validate_path_pair(p1, (1, 3), p2, (2, ))
        self.assertTrue(new_cons == None)

    def test_meta_agent_simultaneous(self):
        """tests meta_agents for simultaneous occupation"""
        p11 = ((0, 0), (0, 1), (0, 2), (0, 3), (0, 4))
        p12 = ((5, 5), (5, 6), (5, 7), (5, 8), (5, 9))
        p1 = map(list, zip(p11, p12))
        p2 = map(lambda x:(x, ), ((0, 3), (0, 3), (0, 2), (1, 3), (2, 3)))
        new_cons = cbs.validate_path_pair(p1, (1, 3), p2, (2, ))
        con1 = cbs.con_add_node_constraint(cbs.con_empty_constraint((1, 3)), 2, 
                                           (0, 2))
        con2 = cbs.con_add_node_constraint(cbs.con_empty_constraint((2, )), 2, 
                                           (0, 2))
        self.assertTrue(con1 in new_cons and con2 in new_cons)

    def test_8_conn_validation_rejection(self):
        """test that non-cross over events are rejected"""
        path1 = map(lambda x:(x, ), ((0, 0), (1, 0)))
        for i in [(0, 1), (1, 1), (1, 2), (0, 2), (-1, 2), (-1, 1), (-1, 0), 
                  (0, 0)]:
            path2 = map(lambda x:(x, ), ((0, 1), i))
            con = cbs.validate_path_pair(path1, ((1, )), path2, ((2, )))
            self.assertTrue(con == None)
        path1 = map(lambda x:(x, ), ((0, 0), (1, 1)))
        for i in [(0, 1), (1, 2), (0, 2), (-1, 2), (-1, 1), (-1, 0), (0, 0)]:
            path2 = map(lambda x:(x, ), ((0, 1), i))
            con = cbs.validate_path_pair(path1, ((1, )), path2, ((2, )))
            self.assertTrue(con == None)

    def test_8_conn_validation_true_positive(self):
        """test that cross over events are detected"""
        path1 = map(lambda x:(x, ), ((0, 0), (1, 1)))
        for i in [((0, 1), (1, 0)), ((1, 0), (0, 1))]:
            path2 = map(lambda x:(x, ), i)
            con = cbs.validate_path_pair(path1, ((1, )), path2, ((2, )))
            self.assertTrue(con != None)

        for i in [((-1, 0), (0, 1)), ((0, 1), (-1, 0))]:
            path1 = map(lambda x:(x, ), ((0, 0), (-1, 1)))
            path2 = map(lambda x:(x, ), i)
            con = cbs.validate_path_pair(path1, ((1, )), path2, ((2, )))
            self.assertTrue(con != None)

        for i in [((-1, 0), (0, -1)), ((0, -1), (-1, 0))]:
            path1 = map(lambda x:(x, ), ((0, 0), (-1, -1)))
            path2 = map(lambda x:(x, ), i)
            con = cbs.validate_path_pair(path1, ((1, )), path2, ((2, )))
            self.assertTrue(con != None)

        for i in [((1, 0), (0, -1)), ((0, -1), (1, 0))]:
            path1 = map(lambda x:(x, ), ((0, 0), (1, -1)))
            path2 = map(lambda x:(x, ), i)
            con = cbs.validate_path_pair(path1, ((1, )), path2, ((2, )))
            self.assertTrue(con != None)
        
    def test_solution_validation(self):
        """tests the solution validation function"""
        #Testing some simple collisions
        path1 =  ((0, 0), (1, 0), (2, 0))
        path2 =  ((2, 0), (1, 0), (0, 0))
        path3 = ((4, 4), (5, 4), (6, 4))
        path4 = ((4, 5), (5, 5), (6, 5))
        self.assertTrue(not cbs.validate_solution(zip(path1, path2)))
        self.assertTrue(not cbs.validate_solution(zip(path1, path2, 
                                                      path3, path4)))
        self.assertTrue(not cbs.validate_solution(zip(path3, path1, 
                                                      path4, path2)))
        path1 =  ((0, 0), (1, 0), (1, 0))
        path2 =  ((1, 0), (0, 0), (0, 0))
        self.assertTrue(not cbs.validate_solution(zip(path1, path2)))
        self.assertTrue(not cbs.validate_solution(zip(path1, path2, 
                                                      path3, path4)))
        self.assertTrue(not cbs.validate_solution(zip(path3, path1,
                                                      path4, path2)))
        path1 =  ((0, 0), (1, 1), (1, 1))
        path2 =  ((0, 1), (1, 0), (1, 0))
        self.assertTrue(not cbs.validate_solution(zip(path1, path2)))
        self.assertTrue(not cbs.validate_solution(zip(path1, path2,
                                                      path3, path4)))
        self.assertTrue(not cbs.validate_solution(zip(path3, path1, 
                                                      path4, path2)))
        #Test a known-good path
        path1 =  ((0, 0), (1, 0), (2, 0))
        path2 =  ((2, 0), (1, 1), (0, 0))
        self.assertTrue(cbs.validate_solution(zip(path1, path2)))
        self.assertTrue(cbs.validate_solution(zip(path1, path2, path3, path4)))
        self.assertTrue(cbs.validate_solution(zip(path3, path1, path4, path2)))


class TestCBSPlanner(unittest.TestCase):
    def setUp(self):
        """Sets up a basic empty environment"""
        self.obs_map = [[0 for i in xrange(10)] for j in xrange(10)]
        self.empty_con = cbs.con_empty_constraint((1, ))

    def test_single_robot_path(self):
        """test that the CBS planner can still handle single robot problems"""
        p = cbs.CBS_Planner(self.obs_map, [(0, 0)])
        path = p.find_solution([[3, 0]])
        self.assertTrue((((3, 0), ), ((2, 0), ), ((1, 0), ), ((0, 0), )) == path)
        #Test block
        self.obs_map[1][0] = 1
        p = cbs.CBS_Planner(self.obs_map, [(0, 0)])
        path = p.find_solution([[3, 0]])
        self.assertTrue(not ((1, 0), ) in path)
        self.assertTrue(cbs.compute_cost(path) == 5)

    def test_non_colliding_multirobot_path(self):
        """Test that non-colliding paths do not incur needless costs"""
        p = cbs.CBS_Planner(self.obs_map, [(0, 0), (3, 3)])
        path = p.find_solution([[1, 1], [4, 4]])
        self.assertTrue(cbs.compute_cost(path) == 4)
        self.assertTrue(cbs.validate_solution(path))

    def test_colliding_multirobot_path(self):
        """Test colliding systems of robots"""
        #Tests a collision occuring along an edge (1, 2)->(2, 1)
        p = cbs.CBS_Planner(self.obs_map, [(0, 0), (3, 0)])
        path = p.find_solution([[3, 0], [0, 0]])
        self.assertTrue(cbs.compute_cost(path) == 8)
        #Test a collision due to simultaneous occupation
        p = cbs.CBS_Planner(self.obs_map, [(0, 0), (2, 0)])
        path = p.find_solution([[2, 0], [0, 0]])
        self.assertTrue(cbs.compute_cost(path) == 6)
        #Testing a simple, three robot problem
        #Tests a collision occuring along an edge (1, 2)->(2, 1)
        p = cbs.CBS_Planner(self.obs_map, [(0, 0), (3, 0), (5, 0)])
        path = p.find_solution([[3, 0], [0, 0], [2, 0]])
        self.assertTrue(cbs.compute_cost(path) == 11)

    # @unittest.skip('For speed')
    def test_8_connected_path(self):
        """Tests solutions found on an 8-connected grid"""
        #Tests a collision occuring along an edge (1, 2)->(2, 1)
        p = cbs.CBS_Planner(self.obs_map, [(0, 0), (3, 0)], conn_8=True)
        path = p.find_solution([[3, 0], [0, 3]])
        self.assertTrue(cbs.compute_cost(path) == 6)
        #Test a collision due to simultaneous occupation
        p = cbs.CBS_Planner(self.obs_map, [(0, 0), (2, 0)], conn_8=True)
        path = p.find_solution([[2, 0], [0, 0]])
        self.assertTrue(cbs.compute_cost(path) == 4)
        #Tests a collision occuring along an edge (1, 2)->(2, 1)
        p = cbs.CBS_Planner(self.obs_map, [(0, 0), (3, 0), (5, 0)], conn_8=True)
        path = p.find_solution([[3, 0], [0, 0], [2, 0]])
        self.assertTrue(cbs.compute_cost(path) == 9)

    # @unittest.skip('For speed')
    def test_map_solutions(self):
        """Tests CBS on a couple of test maps"""
        if not (FULL_TEST or CBS_FULL):
            self.skipTest('Skipped map test for brevity')
        print '\ncbs long tests'
        start_time = time.time()
        dat = pickle.load(open('maps/5_40_bots_step_5.map'))
        for i in [0, 30, 60, 101, 130, 151, 200, 230, 250, 300, 330, 350, 380, 
                  400, 450, 451, 531, 532]:#, 534]:
                  #[0, 30, 60, 101, 151, 200, 250, 300, 350]:
            print i
            d = dat[i]
            p = cbs.CBS_Planner(d['obs_map'], d['goals'], conn_8=True)
            path = p.find_solution(d['init_pos'], time_limit=20)
            self.assertTrue(cbs.validate_solution(path))
            self.assertTrue(path[-1] == d['goals'])
        print 'time elapsed: %g' %(time.time()-start_time)


# @unittest.skip('message goes here')
class TestMetaCBSPlanner(unittest.TestCase):
    def setUp(self):
        """Sets up a basic empty environment"""
        self.obs_map = [[0 for i in xrange(10)] for j in xrange(10)]
        self.empty_con = cbs.con_empty_constraint((0, ))
        self.merge_thresh = MERGE_THRESH
        self.planners = ['od_rmstar', 'epermstar']

    def test_single_robot_path(self):
        """test that the CBS planner can still handle single robot problems"""
        for planner in self.planners:
            self.setUp()
            p = cbs.CBS_Planner(self.obs_map, [(0, 0)], meta_agents=True, 
                                merge_thresh=self.merge_thresh, 
                                meta_planner=planner)
            path = p.find_solution([[3, 0]])
            self.assertTrue((((3, 0), ), ((2, 0), ), ((1, 0), ), ((0, 0), ))
                            == path)
            #Test block
            self.obs_map[1][0] = 1
            p = cbs.CBS_Planner(self.obs_map, [(0, 0)], meta_agents=True, 
                                merge_thresh=self.merge_thresh, 
                                meta_planner=planner)
            path = p.find_solution([[3, 0]])
            self.assertTrue(not ((1, 0), ) in path)
            self.assertTrue(cbs.compute_cost(path) == 5)
    

    def test_non_colliding_multirobot_path(self):
        """Test that non-colliding paths do not encure additional, needless 
        costs

        """
        for planner in self.planners:
            self.setUp()
            p = cbs.CBS_Planner(self.obs_map, [(0, 0), (3, 3)], meta_agents=True,
                                merge_thresh=self.merge_thresh, 
                                meta_planner=planner)
            path = p.find_solution([[1, 1], [4, 4]])
            self.assertTrue(cbs.compute_cost(path) == 4)
            self.assertTrue(cbs.validate_solution(path))


    def test_colliding_multirobot_path(self):
        """Test colliding systems of robots"""
        for planner in self.planners:
            self.setUp()
            #Tests a collision occuring along an edge (1, 2)->(2, 1)
            p = cbs.CBS_Planner(self.obs_map, [(0, 0), (3, 0)], meta_agents=True,
                                merge_thresh=self.merge_thresh, 
                                meta_planner=planner)
            path = p.find_solution([[3, 0], [0, 0]])
            self.assertTrue(cbs.compute_cost(path) == 8)
            #Test a collision due to simultaneous occupation
            p = cbs.CBS_Planner(self.obs_map, [(0, 0), (2, 0)], meta_agents=True,
                                merge_thresh=self.merge_thresh, 
                                meta_planner=planner)
            path = p.find_solution([[2, 0], [0, 0]])
            self.assertTrue(cbs.compute_cost(path) == 6)
            #Testing a simple, three robot problem
            #Tests a collision occuring along an edge (1, 2)->(2, 1)
            p = cbs.CBS_Planner(self.obs_map, [(0, 0), (3, 0), (5, 0)], 
                                meta_agents=True, 
                                merge_thresh=self.merge_thresh, 
                                meta_planner=planner)
            path = p.find_solution([[3, 0], [0, 0], [2, 0]])
            self.assertTrue(cbs.compute_cost(path) == 11)


    def test_8_connected_path(self):
        """Tests solutions found on an 8-connected grid"""
        for planner in self.planners:
            self.setUp()
            #Tests a collision occuring along an edge (1, 2)->(2, 1)
            p = cbs.CBS_Planner(self.obs_map, [(0, 0), (3, 0)], conn_8=True, 
                                meta_agents=True, 
                                merge_thresh=self.merge_thresh, 
                                meta_planner=planner)
            path = p.find_solution([[3, 0], [0, 3]])
            self.assertTrue(cbs.compute_cost(path) == 6)
            #Test a collision due to simultaneous occupation
            p = cbs.CBS_Planner(self.obs_map, [(0, 0), (2, 0)], conn_8=True, 
                                meta_agents=True, 
                                merge_thresh=self.merge_thresh, 
                                meta_planner=planner)
            path = p.find_solution([(2, 0), (0, 0)])
            self.assertTrue(cbs.compute_cost(path) == 4)
            #Tests a collision occuring along an edge (1, 2)->(2, 1)
            p = cbs.CBS_Planner(self.obs_map, [(0, 0), (3, 0), (5, 0)], 
                                conn_8=True, meta_agents=True, 
                                merge_thresh=self.merge_thresh, 
                                meta_planner=planner)
            path = p.find_solution([[3, 0], [0, 0], [2, 0]])
            self.assertTrue(cbs.compute_cost(path) == 9)

    # @unittest.skip('For speed')
    def test_map_solutions(self):
        """Tests CBS on a couple of test maps"""
        if not (FULL_TEST or MCBS_FULL):
            self.skipTest('Skipped map test for brevity')
        for planner in self.planners:
            self.setUp()
            print '\nmeta cbs long tests w/%s' %(planner)
            start_time = time.time()
            dat = pickle.load(open('maps/5_40_bots_step_5.map'))
            # for i in [0, 30, 60, 101, 151, 200, 250, 300, 350]:
            for i in [0, 20, 30, 40, 60, 70, 80, 100, 101, 130, 151, 160, 170, 
                      200, 230, 250, 300, 330, 350, 380, 400, 450, 
                      451, 510, 520, 531, 532, 534, 540, 550]:#[456]:#, 485]
                print i
                d = dat[i]
                p = cbs.CBS_Planner(
                    d['obs_map'], d['goals'], conn_8=True, meta_agents=True, 
                    merge_thresh=self.merge_thresh, meta_planner=planner)
                path = p.find_solution(d['init_pos'], time_limit=20)
                self.assertTrue(cbs.validate_solution(path))
                self.assertTrue(path[-1] == d['goals'])
            print 'time elapsed: %g' %(time.time()-start_time)

# @unittest.skip('for_debugging')
class TestConstrainedODrMstarPlanner(unittest.TestCase):
    def setUp(self):
        """Sets up a basic empty environment"""
        self.obs_map = [[0 for i in xrange(10)] for j in xrange(10)]
        self.empty_con =  cbs.con_empty_constraint([1])
        self.empty_2con = cbs.con_empty_constraint((0, 1))
        self.empty_3con = cbs.con_empty_constraint((0, 1, 2))

    def test_single_robot_path(self):
        """test that the constrained op_decomp planner can still handle single
        robot problems"""
        p = constrained_od_mstar.Constrained_Od_Mstar(self.obs_map, [[3, 0]], 
                                                      [[0, 0]], self.empty_con)
        path = p.find_path([[3, 0, 0]])
        self.assertTrue((((3, 0), ), ((2, 0), ), ((1, 0), ), ((0, 0), )) == 
                        path)
        #Test block by obstacles
        self.obs_map[1][0] = 1
        p = constrained_od_mstar.Constrained_Od_Mstar(self.obs_map, [[3, 0]], 
                                                     [[0, 0]], self.empty_con)
        path = p.find_path([[3, 0, 0]])
        self.assertTrue(not ((1, 0), ) in path)
        self.assertTrue(cbs.compute_cost(path) == 5)
        self.assertTrue(p.get_path_cost() == 5)

        self.obs_map[1][0] = 0
        
        #Test that it obeys constraints
        basic_con = cbs.con_add_node_constraint(self.empty_con, 1, (2, 0))
        p = constrained_od_mstar.Constrained_Od_Mstar(self.obs_map, [[3, 0]], 
                                                        [[0, 0]], basic_con)
        path = p.find_path([[3, 0, 0]])
        self.assertTrue(path[1][0] != (2, 0))
        self.assertTrue(cbs.compute_cost(path) == 4)
        self.assertTrue(p.get_path_cost() == 4)

        basic_con = cbs.con_add_edge_constraint(self.empty_con, 1, (3, 0), 
                                                (2, 0))
        p = constrained_od_mstar.Constrained_Od_Mstar(self.obs_map, [[3, 0]], 
                                                        [[0, 0]], basic_con)
        path = p.find_path([[3, 0, 0]])
        self.assertTrue(path[1][0] != (2, 0))
        self.assertTrue(cbs.compute_cost(path) == 4)
        self.assertTrue(p.get_path_cost() == 4)

    def test_forced_delay(self):
        """Test that the constrained op_decomp planner can be forced to delay
        by constraints"""
        con = cbs.con_add_node_constraint(self.empty_con, 1, (1, 0))
        con = cbs.con_add_node_constraint(con, 1, (0, 1))
        p = constrained_od_mstar.Constrained_Od_Mstar(self.obs_map, [[0, 0]], 
                                                        [[1, 0]], con)
        path = p.find_path([[0, 0, 0]])
        self.assertTrue(cbs.compute_cost(path) == 2)

    def test_constraint_delay(self):
        """Tests that the constrained planner will plan beyond the constraint"""
        #Test with an irrelevant constraint
        con = cbs.con_add_node_constraint(self.empty_con, 10, (3, 3))
        p = constrained_od_mstar.Constrained_Od_Mstar(self.obs_map, [[0, 0]], 
                                                        [[1, 0]], con)
        path = p.find_path([[0, 0, 0]])
        self.assertTrue(len(path) == 12)
        self.assertTrue(p.get_path_cost() == 1)
        #Test with a constraint that appears at the goal
        con = cbs.con_add_node_constraint(self.empty_con, 10, (1, 0))
        p = constrained_od_mstar.Constrained_Od_Mstar(self.obs_map, [[0, 0]], 
                                                        [[1, 0]], con)
        path = p.find_path([[0, 0, 0]])
        self.assertTrue(len(path) == 12)
        self.assertTrue(p.get_path_cost() == 3)

    def test_non_colliding_multirobot_path(self):
        """Test that non-colliding paths do not encure additional, needless 
        costs"""
        p = constrained_od_mstar.Constrained_Od_Mstar(self.obs_map, 
                                                      [[1, 1], [4, 4]], 
                                                      [[0, 0], [3, 3]], 
                                                      self.empty_2con)
        path = p.find_path([[1, 1, 0], [4, 4, 0]])
        self.assertTrue(cbs.compute_cost(path) == 4)
        self.assertTrue(cbs.validate_solution(path))
        #Test non- miportant constraints
        d_con = cbs.con_add_node_constraint(self.empty_2con, 1, (5, 5))
        d_con = cbs.con_add_edge_constraint(d_con, 1, (2, 3), (3, 2))
        p = constrained_od_mstar.Constrained_Od_Mstar(self.obs_map, 
                                                        [[1, 1], [4, 4]], 
                                                        [[0, 0], [3, 3]], d_con)
        path = p.find_path([[1, 1, 0], [4, 4, 0]])
        self.assertTrue(cbs.compute_cost(path) == 4)
        self.assertTrue(p.get_path_cost() == 4)
        self.assertTrue(cbs.validate_solution(path))

    def test_colliding_multirobot_path(self):
        """Test colliding systems of robots"""
        #Tests a collision occuring along an edge (1, 2)->(2, 1)
        p = constrained_od_mstar.Constrained_Od_Mstar(self.obs_map, 
                                                        [[3, 0], [0, 0]], 
                                                        [[0, 0], [3, 0]], 
                                                        self.empty_2con)
        path = p.find_path([[3, 0, 0], [0, 0, 0]])
        self.assertTrue(cbs.validate_solution(path))
        self.assertTrue(cbs.compute_cost(path) == 8)
        #Test a collision due to simultaneous occupation
        p = constrained_od_mstar.Constrained_Od_Mstar(self.obs_map, 
                                                        [[2, 0], [0, 0]], 
                                                        [[0, 0], [2, 0]], 
                                                        self.empty_2con)
        path = p.find_path([[2, 0, 0], [0, 0, 0]])
        self.assertTrue(cbs.validate_solution(path))
        self.assertTrue(cbs.compute_cost(path) == 6)
        #Testing a simple, three robot problem
        #Tests a collision occuring along an edge (1, 2)->(2, 1)
        p = constrained_od_mstar.Constrained_Od_Mstar(self.obs_map, 
                                                      [[3, 0], [0, 0], [2, 0]], 
                                                      [[0, 0], [3, 0], [5, 0]], 
                                                        self.empty_3con)
        path = p.find_path([[3, 0, 0], [0, 0, 0], [2, 0, 0]])
        self.assertTrue(cbs.compute_cost(path) == 11)
        self.assertTrue(cbs.validate_solution(path))

        temp = cbs.con_add_node_constraint(self.empty_2con, 1, (1, 0))
        temp = cbs.con_add_node_constraint(temp, 1, (2, 0))
        p = constrained_od_mstar.Constrained_Od_Mstar(self.obs_map, 
                                                        [[3, 0], [0, 0]], 
                                                        [[0, 0], [3, 0]], temp)
        path = p.find_path([[3, 0, 0], [0, 0, 0]])
        self.assertTrue((1, 0) not in path[1] and (2, 0) not in path[1])
        self.assertTrue(cbs.compute_cost(path) == 9)
        self.assertTrue(cbs.validate_solution(path))

        temp = cbs.con_add_node_constraint(self.empty_3con, 1, (1, 0))
        temp = cbs.con_add_node_constraint(temp, 1, (2, 0))
        p = constrained_od_mstar.Constrained_Od_Mstar(self.obs_map, 
                                                      [[3, 0], [0, 0], [2, 0]], 
                                                      [[0, 0], [3, 0], [5, 0]], 
                                                        temp)
        path = p.find_path([[3, 0, 0], [0, 0, 0], [2, 0, 0]])
        self.assertTrue((1, 0) not in path[1] and (2, 0) not in path[1])

    def test_8_connected_path(self):
        """Tests solutions found on an 8-connected grid"""
        #Tests a collision occuring along an edge (1, 2)->(2, 1)
        p = constrained_od_mstar.Constrained_Od_Mstar(self.obs_map, 
                                                        [[3, 0], [0, 0]], 
                                                        [[0, 0], [3, 0]], 
                                                        self.empty_2con, 
                                                        conn_8=True)
        path = p.find_path([[3, 0, 0], [0, 0, 0]])
        self.assertTrue(cbs.compute_cost(path) == 6)
        #Test a collision due to simultaneous occupation
        p = constrained_od_mstar.Constrained_Od_Mstar(self.obs_map, 
                                                        [[2, 0], [0, 0]], 
                                                        [[0, 0], [2, 0]], 
                                                        self.empty_2con, 
                                                        conn_8=True)
        path = p.find_path([[2, 0, 0], [0, 0, 0]])
        self.assertTrue(cbs.compute_cost(path) == 4)
        #Tests a collision occuring along an edge (1, 2)->(2, 1)
        p = constrained_od_mstar.Constrained_Od_Mstar(self.obs_map, 
                                                      [[3, 0], [0, 0], [2, 0]], 
                                                      [[0, 0], [3, 0], [5, 0]], 
                                                        self.empty_3con, 
                                                        conn_8=True)
        path = p.find_path([[3, 0, 0], [0, 0, 0], [2, 0, 0]])
        self.assertTrue(cbs.compute_cost(path) == 9)

        #Test some constraints
        temp = cbs.con_add_node_constraint(self.empty_2con, 1, (1, 0))
        temp = cbs.con_add_node_constraint(temp, 1, (1, 1))
        p = constrained_od_mstar.Constrained_Od_Mstar(self.obs_map, 
                                                        [[3, 0], [0, 0]], 
                                                        [[0, 0], [3, 0]], temp, 
                                                        conn_8=True)
        path = p.find_path([[3, 0, 0], [0, 0, 0]])
        self.assertTrue(cbs.compute_cost(path) == 7)

    def test_map_solutions(self):
        """Tests cOD_rM* on a couple of test maps, no constraints"""
        if not (COD_rMSTAR_FULL or FULL_TEST):
            self.skipTest('Skipped map test for brevity')
        print '\nconstrained ODrM* long tests'
        dat = pickle.load(open('maps/5_40_bots_step_5.map'))
        start_time = time.time()
        for i in [0, 30, 60, 101, 151]:#, 200, 250, 300, 350]:
            print i
            d = dat[i]
            p = constrained_od_mstar.Constrained_Od_Mstar(d['obs_map'], 
                                           d['init_pos'], d['goals'], 
                                           cbs.con_empty_constraint(range(
                                               len(d['goals']))), conn_8=True)
            path = p.find_path_time_pad(d['init_pos'], time_limit=20)
            self.assertTrue(cbs.validate_solution(path))
            self.assertTrue(path[-1] == d['goals'])
        print 'time elapsed: %g' %(time.time()-start_time)

# @unittest.skip('Skipping constrained EPErM* testing to ensure that cbs test')
class TestConstrainedEPErMstarPlanner(unittest.TestCase):
    def setUp(self):
        """Sets up a basic empty environment"""
        self.obs_map = [[0 for i in xrange(10)] for j in xrange(10)]
        self.empty_con =  cbs.con_empty_constraint([1])
        self.empty_2con = cbs.con_empty_constraint((0, 1))
        self.empty_3con = cbs.con_empty_constraint((0, 1, 2))

    def test_single_robot_path(self):
        """test that the constrained op_decomp planner can still handle single
        robot problems"""
        p = constrained_od_mstar.Constrained_Od_Mstar(
            self.obs_map, [[3, 0]], [[0, 0]], self.empty_con, epeastar=True)
        path = p.find_path([[3, 0, 0]])
        self.assertTrue((((3, 0), ), ((2, 0), ), ((1, 0), ), ((0, 0), )) == 
                        path)
        #Test block by obstacles
        self.obs_map[1][0] = 1
        p = constrained_od_mstar.Constrained_Od_Mstar(
            self.obs_map, [[3, 0]], [[0, 0]], self.empty_con, epeastar=True)
        path = p.find_path([[3, 0, 0]])
        self.assertTrue(not ((1, 0), ) in path)
        self.assertTrue(cbs.compute_cost(path) == 5)
        self.assertTrue(p.get_path_cost() == 5)

        self.obs_map[1][0] = 0
        
        #Test that it obeys constraints
        basic_con = cbs.con_add_node_constraint(self.empty_con, 1, (2, 0))
        p = constrained_od_mstar.Constrained_Od_Mstar(
            self.obs_map, [[3, 0]], [[0, 0]], basic_con, epeastar=True)
        path = p.find_path([[3, 0, 0]])
        self.assertTrue(path[1][0] != (2, 0))
        self.assertTrue(cbs.compute_cost(path) == 4)
        self.assertTrue(p.get_path_cost() == 4)

        basic_con = cbs.con_add_edge_constraint(self.empty_con, 1, (3, 0), 
                                                (2, 0))
        p = constrained_od_mstar.Constrained_Od_Mstar(
            self.obs_map, [[3, 0]], [[0, 0]], basic_con, epeastar=True)
        path = p.find_path([[3, 0, 0]])
        self.assertTrue(path[1][0] != (2, 0))
        self.assertTrue(cbs.compute_cost(path) == 4)
        self.assertTrue(p.get_path_cost() == 4)

    def test_forced_delay(self):
        """Test that the constrained op_decomp planner can be forced to delay
        by constraints"""
        con = cbs.con_add_node_constraint(self.empty_con, 1, (1, 0))
        con = cbs.con_add_node_constraint(con, 1, (0, 1))
        p = constrained_od_mstar.Constrained_Od_Mstar(
            self.obs_map, [[0, 0]], [[1, 0]], con, epeastar=True)
        path = p.find_path([[0, 0, 0]])
        self.assertTrue(cbs.compute_cost(path) == 2)

    def test_constraint_delay(self):
        """Tests that the constrained planner will plan beyond the constraint"""
        #Test with an irrelevant constraint
        con = cbs.con_add_node_constraint(self.empty_con, 10, (3, 3))
        p = constrained_od_mstar.Constrained_Od_Mstar(
            self.obs_map, [[0, 0]], [[1, 0]], con, epeastar=True)
        path = p.find_path([[0, 0, 0]])
        self.assertTrue(len(path) == 12)
        self.assertTrue(p.get_path_cost() == 1)
        #Test with a constraint that appears at the goal
        con = cbs.con_add_node_constraint(self.empty_con, 10, (1, 0))
        p = constrained_od_mstar.Constrained_Od_Mstar(
            self.obs_map, [[0, 0]], [[1, 0]], con, epeastar=True)
        path = p.find_path([[0, 0, 0]])
        self.assertTrue(len(path) == 12)
        self.assertTrue(p.get_path_cost() == 3)

    def test_non_colliding_multirobot_path(self):
        """Test that non-colliding paths do not encure additional, needless 
        costs"""
        p = constrained_od_mstar.Constrained_Od_Mstar(
            self.obs_map, [[1, 1], [4, 4]], [[0, 0], [3, 3]], self.empty_2con, 
            epeastar=True)
        path = p.find_path([[1, 1, 0], [4, 4, 0]])
        self.assertTrue(cbs.compute_cost(path) == 4)
        self.assertTrue(cbs.validate_solution(path))
        #Test non- miportant constraints
        d_con = cbs.con_add_node_constraint(self.empty_2con, 1, (5, 5))
        d_con = cbs.con_add_edge_constraint(d_con, 1, (2, 3), (3, 2))
        p = constrained_od_mstar.Constrained_Od_Mstar(
            self.obs_map, [[1, 1], [4, 4]], [[0, 0], [3, 3]], d_con, 
            epeastar=True)
        path = p.find_path([[1, 1, 0], [4, 4, 0]])
        self.assertTrue(cbs.compute_cost(path) == 4)
        self.assertTrue(p.get_path_cost() == 4)
        self.assertTrue(cbs.validate_solution(path))

    def test_colliding_multirobot_path(self):
        """Test colliding systems of robots"""
        #Tests a collision occuring along an edge (1, 2)->(2, 1)
        p = constrained_od_mstar.Constrained_Od_Mstar(
            self.obs_map, [[3, 0], [0, 0]], [[0, 0], [3, 0]], self.empty_2con, 
            epeastar=True)
        path = p.find_path([[3, 0, 0], [0, 0, 0]])
        self.assertTrue(cbs.validate_solution(path))
        self.assertTrue(cbs.compute_cost(path) == 8)
        #Test a collision due to simultaneous occupation
        p = constrained_od_mstar.Constrained_Od_Mstar(
            self.obs_map, [[2, 0], [0, 0]], [[0, 0], [2, 0]], self.empty_2con, 
            epeastar=True)
        path = p.find_path([[2, 0, 0], [0, 0, 0]])
        self.assertTrue(cbs.validate_solution(path))
        self.assertTrue(cbs.compute_cost(path) == 6)
        #Testing a simple, three robot problem
        #Tests a collision occuring along an edge (1, 2)->(2, 1)
        p = constrained_od_mstar.Constrained_Od_Mstar(
            self.obs_map, [[3, 0], [0, 0], [2, 0]], [[0, 0], [3, 0], [5, 0]], 
            self.empty_3con, epeastar=True)
        path = p.find_path([[3, 0, 0], [0, 0, 0], [2, 0, 0]])
        self.assertTrue(cbs.compute_cost(path) == 11)
        self.assertTrue(cbs.validate_solution(path))

        temp = cbs.con_add_node_constraint(self.empty_2con, 1, (1, 0))
        temp = cbs.con_add_node_constraint(temp, 1, (2, 0))
        p = constrained_od_mstar.Constrained_Od_Mstar(
            self.obs_map, [[3, 0], [0, 0]], [[0, 0], [3, 0]], temp, 
            epeastar=True)
        path = p.find_path([[3, 0, 0], [0, 0, 0]])
        self.assertTrue((1, 0) not in path[1] and (2, 0) not in path[1])
        self.assertTrue(cbs.compute_cost(path) == 9)
        self.assertTrue(cbs.validate_solution(path))

        temp = cbs.con_add_node_constraint(self.empty_3con, 1, (1, 0))
        temp = cbs.con_add_node_constraint(temp, 1, (2, 0))
        p = constrained_od_mstar.Constrained_Od_Mstar(
            self.obs_map, [[3, 0], [0, 0], [2, 0]], [[0, 0], [3, 0], [5, 0]], 
            temp, epeastar=True)
        path = p.find_path([[3, 0, 0], [0, 0, 0], [2, 0, 0]])
        self.assertTrue((1, 0) not in path[1] and (2, 0) not in path[1])

    def test_8_connected_path(self):
        """Tests solutions found on an 8-connected grid"""
        #Tests a collision occuring along an edge (1, 2)->(2, 1)
        p = constrained_od_mstar.Constrained_Od_Mstar(
            self.obs_map, [[3, 0], [0, 0]], [[0, 0], [3, 0]], self.empty_2con, 
            conn_8=True, epeastar=True)
        path = p.find_path([[3, 0, 0], [0, 0, 0]])
        self.assertTrue(cbs.compute_cost(path) == 6)
        #Test a collision due to simultaneous occupation
        p = constrained_od_mstar.Constrained_Od_Mstar(
            self.obs_map, [[2, 0], [0, 0]], [[0, 0], [2, 0]], self.empty_2con, 
            conn_8=True, epeastar=True)
        path = p.find_path([[2, 0, 0], [0, 0, 0]])
        self.assertTrue(cbs.compute_cost(path) == 4)
        #Tests a collision occuring along an edge (1, 2)->(2, 1)
        p = constrained_od_mstar.Constrained_Od_Mstar(
            self.obs_map, [[3, 0], [0, 0], [2, 0]], [[0, 0], [3, 0], [5, 0]], 
            self.empty_3con, conn_8=True, epeastar=True)
        path = p.find_path([[3, 0, 0], [0, 0, 0], [2, 0, 0]])
        self.assertTrue(cbs.compute_cost(path) == 9)

        #Test some constraints
        temp = cbs.con_add_node_constraint(self.empty_2con, 1, (1, 0))
        temp = cbs.con_add_node_constraint(temp, 1, (1, 1))
        p = constrained_od_mstar.Constrained_Od_Mstar(
            self.obs_map, [[3, 0], [0, 0]], [[0, 0], [3, 0]], temp, 
            conn_8=True, epeastar=True)
        path = p.find_path([[3, 0, 0], [0, 0, 0]])
        self.assertTrue(cbs.compute_cost(path) == 7)

    def test_map_solutions(self):
        """Tests cOD_rM* on a couple of test maps, no constraints"""
        if not (CEPErMSTAR_FULL or FULL_TEST):
            self.skipTest('Skipped map test for brevity')
        print '\nconstrained EPErM* long tests'
        dat = pickle.load(open('maps/5_40_bots_step_5.map'))
        start_time = time.time()
        for i in [0, 30, 60, 101, 151, 200, 250]:#, 300]:#, 350]:
            print i
            d = dat[i]
            p = constrained_od_mstar.Constrained_Od_Mstar(
                d['obs_map'], d['init_pos'], d['goals'], 
                cbs.con_empty_constraint(range(len(d['goals']))), conn_8=True, 
                epeastar=True)
            path = p.find_path_time_pad(d['init_pos'], time_limit=20)
            self.assertTrue(cbs.validate_solution(path))
            self.assertTrue(path[-1] == d['goals'])
        print 'time elapsed: %g' %(time.time()-start_time)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Runs test suite for cbs')
    parser.add_argument('-v', dest='verbosity', action='store_const', const=2, 
                        default=1, help='verbose output')
    parser.add_argument('-f', dest='full_test', action='store_true', 
                        help='Full test, including map tests')
    parser.add_argument('--cbs', dest='cbs_full', action='store_true', 
                        help='Full test for cbs, including map tests')
    parser.add_argument('--mcbs', dest='mcbs_full', action='store_true', 
                        help='Full test for meta-cbs, including map tests')
    parser.add_argument('--merge-thresh', dest='merge_thresh', action='store', 
                        type=int, default=10, help='Merge threshold for M-CBS')
    parser.add_argument('--codrmstar', dest='cod_rmstar_full', 
                        action='store_true', help='Full test for constrained '+
                        'ODrM*, including map tests')
    parser.add_argument('--cepermstar', dest='cepermstar_full', 
                        action='store_true', 
                        help = 'Full test for constrained EPErM*')
    
    args = parser.parse_args()
    # global FULL_TEST 
    FULL_TEST = args.full_test
    CBS_FULL = args.cbs_full
    MCBS_FULL = args.mcbs_full
    MERGE_THRESH = args.merge_thresh
    COD_rMSTAR_FULL = args.cod_rmstar_full
    CEPErMSTAR_FULL = args.cepermstar_full
    sys.argv = ['cbs_test.py']
    unittest.main(verbosity=args.verbosity)
    # cur_module = __import__('__main__')
    # suite = unittest.TestLoader().loadTestsFromModule(cur_module)
    # unittest.TextTestRunner(verbosity=args.verbosity).debug(suite)

    # suite = unittest.TestLoader().loadTestsFromTestCase(
    #                                   TestCBSConstraintFunctions)
    # unittest.TextTestRunner(verbosity=3).run(suite)
