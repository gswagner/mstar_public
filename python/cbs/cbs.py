"""Implementation of the Conlift-Based Search and Meta-agent Conflict based
search algorithms from

Meta-agent Conflict-Based Search for Optimal Multi-Agent Path Finding by
Guni Sharon, Roni Stern, Ariel Felner and Nathan Sturtevant

Basic idea is independent planning, then check for pairwise collisions.  Branch
into two seperate searches, which each require that one robot avoid that
particular space-time position.  The search over the conflict tree continues
until a set of collision free paths are found.

constraints will be of the form ((robot_ids),(disallowed states,...))  where
each disallowed state will have the form (time,coord,[coord2]), where the
optional second coord indicates that this is an edge constraint.  Note that
constraints are always expressed in the single robot space, so (x,y), not
((x,y),)  THe time for an edge constraint refers to the time at which edge
traversal is finished.  Not using objects so I can use constraints in
dictionary keys.  Constraints will be ordered by time"""

import workspace_graph
import heapq
import bisect
import SortedCollection
import hashlib
import itertools
import constrained_od_mstar
import constrained_planner
import collections
import time as timer
import random
from col_set_addition import NoSolutionError, OutOfTimeError

MAX_COST = 1000000
# Effectively defining a fake object for constraints
# Indicies for elements of a constraint
CON_ROB = 0
CON_DISALLOWED = 1
CON_TIME = 0
CON_STATES = 1

VALID_META_PLANNERS = ['od_rmstar', 'op_decomp', 'epea', 'epermstar']


#Defining some helper functions for manipulating constraints
def con_empty_constraint(robots):
    """creates a new constraint for robots, containing no actual
    prohibited states, to be populated later

    robots - list of robots making up the meta-agent, can provide a
             single integer id if desired (will be wrapped in a tuple)

    returns:
    Constraint specifying a set of robots, but not a set of disallowed states
    """
    if isinstance(robots, int):
        robots = (robots, )
    if isinstance(robots, list):
        robots = tuple(sorted(robots))
    return (robots, ())


def con_get_robots(constraint):
    """returns the robots to which the constraint applies

    constraint - the constraint to extract the applied robots from

    returns:
    the robots subject to a specific constraint
    """
    return constraint[CON_ROB]


def con_subset_robots(constraint, robots):
    """Creates a new constraint with the prohibitted state from
    constraints, but the robots in robots

    constraint - a constraint tuple describing the disallowed states
    robots     - list of robots to be prohibitted from the disallowed
                 states

    returns
    new constraint based on the specified constraint, but applying
    only to a limited set of robots
    """
    return (tuple(sorted(robots)), constraint[CON_DISALLOWED])


def con_add_node_constraint(constraint, time, coord):
    """Creates a new constraint which is not allowed to occupy a
    specific space time location

    constraint - constraint the new constraint is based on
    time       - the time of the new prohibited state
    coord      - the coordinate that the robots are prohibited from
                 occupying

    returns
    A copy of constraint with an additional disallowed space-time
    location
    """
    # Can do tuple concatenation by addition
    # Maintain sorted, to ensure duplicates will be found, and allow
    # more rapid searching of constraint violations
    tup = (time, (coord, ))
    # Bisecting allows us to find the first constraint at the proper
    # time with O(log(n)) complexity, nice when checking constraints
    # will be a common operation
    dex = bisect.bisect_left(constraint[CON_DISALLOWED], tup)
    new_cons = (constraint[CON_DISALLOWED][:dex] + (tup, ) +
                constraint[CON_DISALLOWED][dex:])
    return (constraint[CON_ROB], new_cons)


def con_add_edge_constraint(constraint, time, coord1, coord2):
    """Creates a new constraint which is not allowed to move from coord1
    at time - 1 to coord2 at time

    constraint - constraint to which a new dissalowed edge will be added
    time       - time of arrival at new coordinate for the edge to be
                 disallowed
    coord1     - source node for the edge, at time - 1
    coord2     - destination node for the edge, and time

    returns:
    copy of constraint to which a new disallowed edge has been added
    """
    # Can do tuple concatenation by addition
    # Maintain sorted, to ensure duplicates will be found, and allow more rapid
    # searching of constraint violations
    tup = (time, (coord1, coord2))
    dex = bisect.bisect_left(constraint[CON_DISALLOWED], tup)
    new_cons = (constraint[CON_DISALLOWED][:dex] + (tup, ) +
                constraint[CON_DISALLOWED][dex:])
    return (constraint[CON_ROB], new_cons)


def con_is_constrained(constraint, time, coord1, coord2):
    """Checks if transitioning between two coordinates violates a
    constraint

    Checks if moving from coord1 to coord2 at time (time being the time
    step you will be at coord2) is a constraint violation, returns True
    if violation.  Does no checks for which robots are triggering these
    constraints.  Operates in the single robot space, so coord1 must be
    the position of a single robot, i.e. (x, y), and not ((x, y), ).
    Therefore, points in a potentially joint configuration space must
    broken down

    # TODO:  SWAP TO HANDLING COORDINATES GIVEN IN THE FULL SPACE?

    constraint - constraint to test
    time       - time of arrival at the destination node
    coord1     - source node for transition, at time - 1
    coord2     - destination node of the transition, at time

    returns
    True if a constraint is violated, False otherwise
    """
    # find the dex of the first constraint with a time that may be equal
    # to the querry
    dex = bisect.bisect_left(constraint[CON_DISALLOWED], (time, ))
    while (dex < len(constraint[CON_DISALLOWED]) and
           constraint[CON_DISALLOWED][dex][CON_TIME] == time):
        sub = constraint[CON_DISALLOWED][dex]
        if sub[CON_TIME] == time:
            if len(sub[CON_STATES]) == 2:
                # Edge constraint
                if (sub[CON_STATES][0] == coord1 and
                    sub[CON_STATES][1] == coord2):
                    return True
            else:
                # Node constraint
                if (sub[CON_STATES][0] == coord2):
                    return True
        dex += 1
    return False


def con_get_max_time(constraint):
    """Returns the greatest timestep value in the constraints

    Used for determining the temporal planning window for planning with
    constraints

    constraint - constraint to process

    returns:
    latest time step of a disallowed state in constraint, 0 if the
    constraint is empty
    """
    if len(constraint[CON_DISALLOWED]) == 0:
        return 0
    return constraint[CON_DISALLOWED][-1][CON_TIME]


def con_merge_constraints(existing_constraints, new_constraint):
    """Adds the disallowed states in new_constraint to
    existing_constraints.

    Keep the constraints sorted to allow for duplicate detection

    existing_constraints - source of new disallowed states
    new_constraints      - constraint object to add additional
                           disallowed states

    returns
    copy of new_constraints with disallowed states in
    existing_constraints added
    """
    temp = list(existing_constraints)
    new_bots = con_get_robots(new_constraint)
    for dex, sub_con in enumerate(temp):
        if new_bots == con_get_robots(sub_con):
            for disallowed in new_constraint[CON_DISALLOWED]:
                if len(disallowed[CON_STATES]) == 1:
                    # Insert node constraint
                    sub_con = con_add_node_constraint(
                        sub_con, disallowed[CON_TIME],
                        disallowed[CON_STATES][0])
                else:
                    # Insert edge constraint
                    sub_con = con_add_edge_constraint(
                        sub_con, disallowed[CON_TIME],
                        disallowed[CON_STATES][0], disallowed[CON_STATES][1])
            temp[dex] = sub_con
            return tuple(sorted(temp))
    raise ValueError('Missing a constraint for robots: ' +
                     str(con_get_robots(new_constraint)))


def con_merge_agents(constraints, robs1, robs2):
    """Takes in a set of constraints, and produces the constraints
    resulting from merging the agents in robs1 and robs2 into a single
    meta agent.

    CURRENTLY RETURNS SETS THE PROHIBITED STATES FOR THE NEW
    META-AGENT AS THE EMPTY SET

    contstraints - list of current constraints prior to meta-agent
                   merging
    robs1        - tuple of robot ids representing the first meta-agent
    robs2        - tuple of robot ids representing the second meta-agent

    returns
    a constraint containing a meta-agent consisting of the robots in
    robs1 and robs2
    """
    constraints = list(constraints)  # Need to be able to modify stuff
    # Approach is to remove the current constraints for the specified
    # robots, then insert a new, empty constraint for the merged
    # meta-agent
    dex = 0
    while dex < len(constraints):
        if (robs1 == con_get_robots(constraints[dex]) or
            robs2 == con_get_robots(constraints[dex])):
            constraints.pop(dex)
        else:
            dex += 1
    # Create the tuple of the merged meta-agent, sorted for duplicate
    # detection
    meta_agent = tuple(sorted(robs1 + robs2))
    constraints.append(con_empty_constraint(meta_agent))
    return tuple(constraints)


def con_to_single_robot(constraint):
    """Splits constraints into n constraints, each specific to a single
    robot.

    Used for generating the necessary single robot policies

    constraint - constraint to split into single-robot constraints

    returns:
    a list of n constraints, each containing the constraints applied to
    a single robot
    """
    return [((i, ), constraint[CON_DISALLOWED]) for i in
            con_get_robots(constraint)]


# IMPLICITLY ASSUMES THAT THE GRAPH IS A 4 or 8 connected grid
def validate_path_pair(path1, rob1, path2, rob2):
    """Takes in a pair of paths, and the robots associated with them,
    and returns the indicies of the first pair of conflicting robots.

    Assumes each path is itself collision free, so only need to worry
    about inter-path conflicts. Specific to grid worlds.  Returns a pair
    of constraints representing the first observed collision.  The
    constraints are applied to the joint set of robots

    # TODO Migrate to edge-checker, or other graph specific collision
    # checking code


    path1 - (((x1, y1), (x2, y2), ...), ....) first path
    rob1 - tuple of robots in order executing first path
    path2 - (((x1, y1), (x2, y2), ...), ....) second path
    rob2 - tuple of robots executing second path

    returns: [robs], [nodes], t
       robs - index of involved robots
       nodes - coordinates of the node(s) at which the conflict occurs
       t - time at which coordinate
    """
    for t in xrange(max(len(path1), len(path2))):
        # extract the current coordinates along the path
        c1 = path1[min(t, len(path1) - 1)]
        c2 = path2[min(t, len(path2) - 1)]
        # extract the previous coordinates along the path
        # (for edge checking)
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
                # Compute the offset of the previous position for
                # checking cross over collisions
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
    for i in range(1, len(path)):
        start_coord = path[i - 1]
        for k in range(len(goals)):
            if not(start_coord[k] == goals[k] and
                   start_coord[k] == path[i][k]):
                cost += 1
    return cost


def find_path(obs_map, init_pos, goals, conn_8=False, meta_agents=False,
              merge_thresh=10, meta_planner='op_decomp', time_limit=5 * 60,
              sum_of_costs=False, return_cost=False):
    """Finds a path using CBS or MA-CBS

    obs_map      - list of lists specifying obstacle positions, as per
                   workspace graph
    init_pos     - [p1, p2, .., pn] list of initial positions
    goals        - [p1, p2, .., pn] list of goal positions
    sub_search   - specifies individual robot planners
    conn_8       - whether to use an 8-connected graph instead of a
                   4-connected  graph
    meta_agents  - whether to use meta_agents
    merge_thresh - number of collisions before merging
    meta_planner - which coupled planner to use, options are
                   'op_decomp' (default) 'od_rmstar', 'epea', and
                   'epermstar'
    time_limit   - maximum time to search for a solution before raising
                   an error
    sum_of_costs - use sum of costs metric
    return_cost  - If true, also return cost of path

    returns -
    path in the joint configuration graph

    if return_cost is true, returns path, cost

    raises:
    NoSolutionError - if there is no solution to the problem
    OutOfTimeError  - if could not find a solution in the allowed time
    """
    p = CBS_Planner(obs_map, goals, conn_8=conn_8, meta_agents=meta_agents,
                    merge_thresh=merge_thresh, meta_planner=meta_planner,
                    sum_of_costs=sum_of_costs, return_cost=return_cost)
    return p.find_solution(init_pos, time_limit)


# TODO: NEED TO DETERMINE HOW TO HANDLE COLLISION COUNT AFTER AGENTS ARE
# MERGED
class CBS_Planner(object):
    """Top level planner for performing CBS and MA-CBS search

    By default, each agent incurs cost 1 for every action except for
    waiting at its goal, which incurs cost 0.  Alternatively, can use
    the sum-of-costs formulation, where a robot incurs cost equal to the
    time until it reaches its goal for the last time and stays there
    from then on out
    """
    def __init__(self, obs_map, goals, sub_search=None, conn_8=False,
                 meta_agents=False, merge_thresh=1, meta_planner='op_decomp',
                 sum_of_costs=False, return_cost=False):
        """
        obs_map      - list of lists specifying obstacle positions,
                       as per workspace graph
        goals        - [p1, p2, .., pn] list of goal positions
        sub_search   - specifies individual robot planners
        conn_8       - whether to use an 8-connected graph instead of
                       a 4-connected  graph
        meta_agents  - whether to use meta_agents
        merge_thresh - number of collisions before merging
        meta_planner - which coupled planner to use, options are
                       'od_rmstar' and 'epermstar'
        sum_of_costs - use the sum of costs formulation
        return_cost  - if True, return both the path and its cost
        """
        self.obs_map = obs_map
        self.goals = tuple(map(tuple, goals))
        self.conn_8 = conn_8
        self.meta_agents = meta_agents
        self.merge_thresh = merge_thresh
        self.return_cost = return_cost
        # Cache result paths for reuse/cost calculation
        # store in the form (cost, path) with the key being the sorted
        # tuple of constraints (sorted to ensure duplicates are found)
        self.paths = {}
        # Catch duplicate nodes, as we never need to revisit these, using a
        # dictionary, for easy constant time inclusion tests
        self.closed = {}
        self.meta_planner = meta_planner
        self.sum_of_costs = sum_of_costs
        assert self.meta_planner in [None, 'od_rmstar', 'op_decomp', 'epea',
                                     'epermstar']
        # Counts the number of collisions for determining when to merge
        self.col_count = [[0 for i in xrange(len(goals))]
                                   for j in xrange(len(goals))]
        if sub_search == None:
            self.sub_search = {}
            for i in xrange(len(self.goals)):
                self.sub_search[tuple([i])] = workspace_graph.Astar_Graph(
                    obs_map, goals[i], connect_8=conn_8)
        else:
            self.sub_search = sub_search
        # Most of the time is taken by computing constraints.  A lot of
        # checks will be duplicates, so maintain a cache
        self.solution_validation_results = {}

    def find_solution(self, init_pos, time_limit=5 * 60):
        """Finds a joint solution for all robots from the specified
        initial position to the goal specified at object creation time

        init_pos   - [p1, p2, ....]
        time_limit - maximum amount of execution time allowed

        returns:
        a solution of form [[p11, p12, p13, ,.p1n], ...]

        or path, cost if self.return_cost

        raises:
        NoSolutionError - if there is no solution to the problem
        OutOfTimeError  - if could not find a solution in the allowed
                          time
        """
        init_pos = tuple(tuple(i) for i in init_pos)
        start_time = timer.time()
        cons = tuple([con_empty_constraint((i, ))
                      for i in xrange(len(init_pos))])
        cost = self.compute_paths(init_pos, cons, time_limit)
        # Create an initial search node, consists of
        # [cost, constraints, solution]
        node = [cost, {'cost': cost, 'con': cons}]
        open_list = []
        while timer.time() - start_time < time_limit:
            constraints = node[1]['con']
            # Representing a solution by the set of constraints which
            # are the indices in self.paths
            solution = constraints
            # Get the two new constraints that result from
            new_constraints = self.compute_constraints(solution)
            # Any non-empty list evaluates as True
            if not new_constraints:
                # Valid
                if self.return_cost:
                    return self.construct_joint_path(solution), node[0]
                else:
                    return self.construct_joint_path(solution)
            # Need to merge the new constraint into the existing
            # constraints, and generate new nodes, branch based on
            # whether we are considering meta-agents
            assert(len(new_constraints) == 2)
            try:
                if self.meta_agents:
                    # Need to update collision counts, and track the
                    # number of collisions between the currently
                    # colliding groups

                    # Number of collisions between the constrained bots
                    cur_col = 0
                    for bot1 in con_get_robots(new_constraints[0]):
                        for bot2 in con_get_robots(new_constraints[1]):
                            self.col_count[bot1][bot2] += 1
                            cur_col += self.col_count[bot1][bot2]
                            cur_col += self.col_count[bot2][bot1]
                    if cur_col > self.merge_thresh:
                        con = con_merge_agents(
                            constraints, con_get_robots(new_constraints[0]),
                            con_get_robots(new_constraints[1]))
                        cost = self.compute_paths(
                            init_pos, con,
                            time_limit - timer.time() + start_time)
                        # add the new node to tree
                        heapq.heappush(open_list,
                                       [cost, {'cost': cost, 'con': con}])
                    else:
                        # Didn't merge, so branch normally
                        for new_con in new_constraints:
                            con = con_merge_constraints(constraints, new_con)
                            cost = self.compute_paths(
                                init_pos, con, time_limit - timer.time() +
                                start_time)
                            # add the new node to tree
                            heapq.heappush(open_list,
                                           [cost, {'cost': cost, 'con': con}])
                else:
                    # For each constraint option, compute new paths,
                    # then branch search
                    for new_con in new_constraints:
                        con = con_merge_constraints(constraints, new_con)
                        cost = self.compute_paths(
                            init_pos, con,
                            time_limit - timer.time() + start_time)
                        # add the new node to tree
                        heapq.heappush(open_list, [cost,
                                                   {'cost': cost, 'con': con}])
            except NoSolutionError:
                # A set of constraints were generated that prevent any
                # solution for at least one robot from being found.
                # Just skip all other components, and move to the
                # next node
                pass
            try:
                node = heapq.heappop(open_list)
            except IndexError:
                # Have emptied the open list, so no path exists,
                # currently
                raise NoSolutionError('No solution found')
        raise OutOfTimeError('Solution ran out of time')

    def compute_constraints(self, constraints):
        """Takes in a set of constraints that generate a solution and
        returns a new pair of constraints, generating two branches in
        the constraint tree

        Note that the actual solution is pulled from self.paths

        constraints - the constraints imposed to generate the solution
                      in question

        Returns:
        None if the solution is valid
        otherwise: generates two new constraints to prevent collisions
        """
        for dex1, con1 in enumerate(constraints[:-1]):
            for dex2, con2 in enumerate(constraints[dex1 + 1:]):
                if (con1, con2) in self.solution_validation_results:
                    new_cons = self.solution_validation_results[(con1, con2)]
                    if new_cons != None:
                        return new_cons
                    continue
                rob1 = con_get_robots(con1)
                rob2 = con_get_robots(con2)
                cost1, path1 = self.paths[con1]
                cost2, path2 = self.paths[con2]
                new_cons = validate_path_pair(path1, rob1, path2, rob2)
                self.solution_validation_results[(con1, con2)] = new_cons
                if new_cons != None:
                    return new_cons
        return None

    def compute_paths(self, init_pos, constraints, time_limit):
        """Computes single agent/meta_agent path for each robot subject
        to the specified constraints.

        Operates by side-effecting self.paths, which will store all
        paths

        init_pos    - initial position of all robots, to guide search
        constraints - list of the form
                      [(robots, time, [invalid nodes/edges])..]
                      if a robot is not in the list of constraints,
                      defaults to an empty constraint
        time_limit  - available time to compute paths

        returns: solution, cost
        solution - path in the joint configuration space
        cost     - cost of solution
        """
        temp_time = timer.time()
        cost = 0
        precomputed = filter(lambda x: x in self.paths, constraints)
        if len(precomputed) > 0:
            paths = [self.paths[x][1] for x in precomputed]
            out_paths = to_joint_path(paths)
            first_pass = False
        else:
            out_paths = None
            first_pass = True
        for con in constraints:
            if con in self.paths:
                # Already have computed this previously
                cost += self.paths[con][0]
                continue
            # Have to compute and store this path
            # TODO: ADAPT TO HANDLE MERGED PATHS!!!!!!!
            bots = con_get_robots(con)
            if len(bots) == 1:
                bots = con_get_robots(con)[0]
                path, tcost =\
                        constrained_planner.find_forwards_constrained_path(
                            self.obs_map, init_pos[bots], self.goals[bots],
                            con, sub_search=self.sub_search,
                            conn_8=self.conn_8, out_paths=out_paths,
                            time_limit=time_limit - timer.time() + temp_time,
                            sum_of_costs=self.sum_of_costs)
                if first_pass:
                    if out_paths == None:
                        # out_paths = path
                        out_paths = tuple((x, ) for x in path)
                    else:
                        temp = tuple((x, ) for x in path)
                        out_paths = to_joint_path([out_paths, temp])
                # convert path to multi-robot format
                path = tuple(map(lambda x: (x, ), path))
            else:
                t_init_pos = tuple([init_pos[bot] for bot in bots])
                t_goals = tuple([self.goals[bot] for bot in bots])
                if self.meta_planner == 'op_decomp':
                    assert(False)
                elif self.meta_planner == 'epea':
                    assert(False)
                elif self.meta_planner in ['od_rmstar', 'epermstar']:
                    # First time this set of constraints has been found.
                    # Adding the rmstar planner to sub_search, so if the
                    # constraint is later expanded, this can still be
                    # used as a sub planner
                    epeastar = self.meta_planner == 'epermstar'
                    planner = constrained_od_mstar.Constrained_Od_Mstar(
                        self.obs_map, t_init_pos, t_goals, con, recursive=True,
                        sub_search=self.sub_search, conn_8=self.conn_8,
                        out_paths=out_paths, epeastar=epeastar)
                    self.sub_search[con] = planner
                    path = planner.find_path_time_pad(
                        t_init_pos,
                        time_limit=time_limit - timer.time() + temp_time)
                    tcost = planner.get_path_cost()
            self.paths[con] = (tcost, path)
            cost += tcost
        return cost

    def construct_joint_path(self, solution):
        """takes in a solution and returns a standardized path, where
        each entry provides the configuration of the entire system at a
        given timestep

        solution - a joint solution for the path planning problem, of
                   form to be determined

        returns:
        a path in the joint configuration graph"""
        num_bots = len(self.goals)
        paths = [None for i in xrange(num_bots)]
        # Gather the individual paths
        for bot in xrange(num_bots):
            for sub_con in solution:
                if bot in con_get_robots(sub_con):
                    cost, tpath = self.paths[sub_con]
                    temp = con_get_robots(sub_con).index(bot)
                    paths[bot] = map(lambda x: x[temp], tpath)
        solution_length = max(map(len, paths))
        # aggregate paths into a path in the joint configuration space
        for path in paths:
            for i in range(len(path), solution_length):
                path.append(path[-1])
        return tuple(zip(*paths))


def permuted_cbs_find_path(
        obs_map, init_pos, goals, conn_8=False, meta_agents=False,
        merge_thresh=10, meta_planner='op_decomp', time_limit=5 * 60,
        sum_of_costs=False, num_restarts=4, seed=10, return_cost=False):
    """Finds a path using CBS or MA-CBS with randomized restarts

    obs_map      - list of lists specifying obstacle positions, as per
                   workspace graph
    init_pos     - [p1, p2, .., pn] list of initial positions
    goals        - [p1, p2, .., pn] list of goal positions
    sub_search   - specifies individual robot planners
    conn_8       - whether to use an 8-connected graph instead of a
                   4-connected  graph
    meta_agents  - whether to use meta_agents
    merge_thresh - number of collisions before merging
    meta_planner - which coupled planner to use, options are
                   'op_decomp' (default) 'od_rmstar', 'epea', and
                   'epermstar'
    time_limit   - maximum time to search for a solution before raising
                   an error
    sum_of_costs - use sum of costs metric
    num_restarts - how many random restarts to run
    seed         - seed to use in randomimzed planning.  If None, does
                   not set the random seed, so python will use clock
                   time
    return_cost  - whether to return the cost of the path along with the
                   path itself

    returns: 
    path in the joint configuration graph

    if return_cost is true, returns path, cost

    raises:
    NoSolutionError - if there is no solution to the problem
    OutOfTimeError  - if could not find a solution in the allowed time
    """
    if seed is not None:
        random.seed(seed)
    time_per_iter = time_limit / float(num_restarts)
    planner = CBS_Planner(obs_map, goals, conn_8=conn_8,
                          meta_agents=meta_agents, return_cost=True,
                          merge_thresh=merge_thresh, meta_planner=meta_planner,
                          sum_of_costs=sum_of_costs)
    new_init_pos = init_pos
    permutation = range(len(init_pos))
    for i in xrange(num_restarts):
        try:
            path, cost = planner.find_path(new_init_pos, time_per_iter)
            break
        except OutOfTimeError:
            pass
        permutation = range(len(init_pos))
        random.shuffle(permutation)
        new_init_pos = permute(init_pos, permutation)
    else:
        raise OutOfTimeError()
    path = [unpermute_coord(coord) for coord in path]
    if return_cost:
        return path, cost
    return path


def permute(iterable, permutation):
    """Permutes the iterable

    iterable - list/tuple to permute
    permutation - permutation[i] gives the new position of the original
                  i'th entry

    return:
    list with the elements of iterable permuated
    """
    temp = sorted([(permutation[i], i) for i in xrange(len(permutation))])
    out = []
    for null, val in temp:
        out.append(iterable[val])
    return out


def unpermute_coord(coord, permutation):
    """Reverses the permutation of a prob_coord

    prob_coord  - (coord1, coord2, ...) list of robot coordinates to
                  unpermute
    permutation - permutation[i] gives the new position of the original
                  i'th entry

    returns:
    a new ProbCoord with permutation undone
    """
    return [coord[i] for i in permutation]


def to_joint_path(paths):
    """Converts a set of individual paths to a joint path"""
    length = max(len(p) for p in paths)
    paths = [list(p) for p in paths]
    for p in paths:
        for i in xrange(len(p), length):
            p.append(p[-1])
    temp = map(list, paths[0])
    for dex in xrange(1, len(paths)):
        for t in xrange(len(paths[dex])):
            temp[t].extend(paths[dex][t])
    return tuple(temp)


def validate_solution(solution):
    """Takes in a path in the full configuration space and tests whether
    it is valid.

    Will only check that no collisions exist, and will not
    check that all edges should exist, due to either connectivity or
    presence of obstacles

    solution - [[coord1, coord2, ...], ...] list of joint configuration
               coordinates defining a path

    returns:
    True if valid, False if robot-robot collisions occur
    """
    # Decompose joint path so it can be passed to validate path pair
    paths = [[(tuple(solution[dex][rob]), ) for dex in xrange(len(solution))]
               for rob in xrange(len(solution[0]))]
    for i in xrange(len(paths) - 1):
        for j in xrange(i + 1, len(paths)):
            if validate_path_pair(paths[i], (i, ), paths[j], (j, )) != None:
                return False
    return True


def interactive_validate_solution(solution):
    """Takes in a path in the full configuration space and tests whether
    it is valid, printing conflicts to stdout.

    Will only check that no collisions exist, and will not check that
    all edges should exist, due to either connectivity or presence of
    obstacles

    solution - [[coord1, coord2, ...], ...] list of joint configuration
               coordinates defining a path

    returns:
    True if valid, False if robot-robot collisions occur
    """
    # Decompose joint path so it can be passed to validate path pair
    paths = [[(tuple(solution[dex][rob]), ) for dex in xrange(len(solution))]
               for rob in xrange(len(solution[0]))]
    for i in xrange(len(paths) - 1):
        for j in xrange(i + 1, len(paths)):
            if validate_path_pair(paths[i], (i, ), paths[j], (j, )) != None:
                cons = validate_path_pair(paths[i], (i, ), paths[j], (j, ))
                for con in cons:
                    if len(con[1][0][1]) == 1:
                        # Node conflict
                        print con
                        print 'Robot: %s\ntime: %s\nnode: %s' % (
                            str(con[0]), str(con[1][0][0]),
                            str(con[1][0][1][0]))
                    else:
                        # Edge conflict
                        print 'Robot: %s\ntime: %s\n:edge: %s to %s' % (
                            str(con[0]), str(con[1][0][0]),
                            str(con[1][0][1][0]),
                            str(con[1][0][1][1]))
                return False
    return True
