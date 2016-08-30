import cbs
import SortedCollection
import workspace_graph
import time as timer
from col_set_addition import NoSolutionError, OutOfTimeError
from collections import defaultdict

MAX_COST = 1000000


def find_constrained_path(obs_map, init_pos, goal, constraints,
                          sub_search=None, out_paths=None, conn_8=False,
                          inflation=1.0, time_limit=5 * 60):
    '''initialize variables (obstacle map, positions, ...).
    obs_map     - binary obstacle map
    init_pos    - start positions
    goal        - goal position
    constraints - constraints defining where robot cannot go, should
                    consist of a single tuple, with only the appropriate
                    robots
    out_paths   - paths of out-of group robots to try to avoid if
                  possible, but will not incur extra cost to do so
    conn_8      - 8-connected grid if True; otherwise 4-connected'''
    p = Constrained_Planner(obs_map, init_pos, goal, constraints,
                            sub_search=sub_search, conn_8=conn_8,
                            inflation=inflation, out_paths=out_paths)
    #Need to specify when you are starting your path from
    init_pos = (init_pos[0], init_pos[1], 0)
    return p.find_path(init_pos, time_limit=time_limit)


class Constrained_Planner(object):
    '''Performs constrained planning for a single robot.  Plans from the
    goals towards the initial position, so that this can be used as a
    source of heuristics by coupled planners.  We are also computing
    a reviersed non-constrained plan, to provide heuristics for this
    planner

    set to provide resumable planning to different positions and times
    to provide policy style planning
    '''
    def __init__(self, obs_map, init_pos, goal, constraints, out_paths=None,
                 sub_search=None, conn_8=False, inflation=1, prune_paths=True):
        '''initialize variables (obstacle map, positions, ...)
        obs_map     - binary obstacle map
        init_pos    - (x,y,t) start position and time
        goal        - goal position
        constraints - constraints defining where robot cannot go, should
                      consist of a single tuple, with only the
                      appropriate  robots
        out_paths   - paths of out of group robots to treat as soft
                      constraints
        conn_8      - 8-connected grid if True; otherwise 4-connected
        only_init   - will only find paths to the original initial
                      position (pruning paths which cannot reach the
                      specified position in time).  If false, will not
                      perform pruning
        '''
        self.obs_map = obs_map

        self.init_pos = tuple(init_pos)
        self.constraints = constraints
        self.out_paths = out_paths
        assert(len(cbs.con_get_robots(constraints)) == 1)
        # Store as tuple, as just need as dictionary key
        self.rob_id = cbs.con_get_robots(constraints)
        self.conn_8 = conn_8
        # Sub-policies as weighting functions
        self.sub_search = sub_search
        # Graph is stored as dictionary indexed by strings of coord and
        # tuple
        self.graph = {}
        # initialize with "normal" indices; is overwritten if other
        # priority
        # ordering is applied. Need time one greater than that of the
        # last constraint to ensure happy
        self.max_t = 0
        self.con_max_t = 0
        if self.constraints is not None:
            self.max_t = cbs.con_get_max_time(self.constraints) + 1
            self.con_max_t = self.max_t
        if self.out_paths is not None:
            # Set max_t one greater than len(self.out_paths) to make
            self.max_t = max(self.max_t, len(self.out_paths))
        self.inflation = inflation
        self.col_check = workspace_graph.Edge_Checker()
        # Generate policy for each robot, for moving towards the initial
        # configuration
        self.policy_key = self.rob_id + ('backpolicy', )
        if self.sub_search is None:
            self.sub_search = {
                self.policy_key: workspace_graph.Back_Priority_Graph(
                    workspace_graph.Astar_Graph(
                        obs_map, self.init_pos[:2], self.conn_8),
                    max_t=self.max_t)}
        # elif not self.policy_key in self.sub_search:
        elif not self.policy_key in self.sub_search:
            # Don't have the individual robot policy here
            self.sub_search[self.policy_key] = \
                workspace_graph.Back_Priority_Graph(
                    workspace_graph.Astar_Graph(
                        obs_map, self.init_pos[:2], self.conn_8))
        self.sub_search[self.policy_key].prune_paths = prune_paths
        assert (self.init_pos[:2] ==
                self.sub_search[self.policy_key].astar_policy.goal)
        # Need persisitant open list for repeated planning, add the goal
        # node now, as it will be the initial search posotion
        # Convert goal to goal in space time
        self.goal = tuple(goal) + (self.con_max_t, )
        goal_node = self.get_node(self.goal)
        goal_node.cost = (0, 0)
        goal_node.back_ptr = goal_node
        self.open_list = SortedCollection.SortedCollection(
            [goal_node], key=lambda x: [
                -x.cost[0] - x.h * self.inflation, -x.cost[1]])
        # potentially need to depart the goal at multiple different
        # times.  Including all possible departure times will be a bit
        # more efficient than forcing exploration
        prev = goal_node
        for t in xrange(self.con_max_t + 1, self.max_t + 1):
            t_coord = tuple(goal) + (t, )
            t_node = self.get_node(t_coord)
            t_node.cost = (0, 0)
            t_node.back_ptr = t_node
            prev.back_ptr = t_node
            prev = t_node
            self.open_list.insert(t_node)

    def heuristic(self, coord):
        '''Computes the heuristic costs for the robot from its position
        (coord) to the goal
        '''
        return self.sub_search[self.policy_key].get_cost(coord, self.max_t)

    def get_node(self, coord):
        '''To expand the graph at the specified coordinate and time'''
        # Node already exists in the graph
        if coord in self.graph:
            node = self.graph[coord]
            return node
        # Otherwise initiate node
        else:
            # Add node to graph
            node = Constraint_Node(coord)
            self.graph[coord] = node
            node.h = self.heuristic(node.coord)
            return node

    def get_transition_cost(self, old_coord, new_coord):
        '''Returns the increment in collision costs.

        Note that this will return a partial tuple, that needs to be
        added elementwise to the old cost to get the new cost

        returns - (transition_cost, collision_count), where
                  collision_count is the number of collision with
                  outpath robots
        '''
        step_cost = 1
        # cost = 0 when robot stays at goal position
        if (old_coord[:-1] == self.goal[:-1] and
                new_coord[:-1] == self.goal[:-1]):
            step_cost = 0
        new_cols = 0
        if self.out_paths is not None:
            new_cols = self.col_check.single_bot_outpath_check(
                new_coord, old_coord, new_coord[-1], self.out_paths)
        return (step_cost, new_cols)

    def expand(self, node, open_list):
        '''Calls get_neighbor from workspace_graph, checks for
        collisions and adds all found neighbors to the open_list

        used for performing constrained planning
        '''
        # Need to both expand along at max_t, and prevent going beyond
        # t=0
        # TODO: should opt be implemented as flag?
        coords = self.sub_search[self.policy_key].get_neighbors(node.coord,
                                                                self.max_t)
        for i in coords:
            new_node = self.get_node(i)
            if new_node.closed:
                continue
            step_cost, new_cols = self.get_transition_cost(node.coord, i)
            new_cost = (node.cost[0] + step_cost, node.cost[1] + new_cols)
            assert new_cost >= node.cost
            if (new_cost < new_node.cost):
                # Check for constraint violation, stepping from i to
                # node. Remember, that i is temporally before node
                if cbs.con_is_constrained(self.constraints, node.coord[-1],
                                          i[:2], node.coord[:2]):
                    # Hit constraint, can skip rest of tests
                    continue
                new_node.tmp_cost = new_cost
                new_node.back_ptr = node
                if new_node.open:
                    # Need to do remove operation in open list to ensure
                    # sorted correctly
                    # open_list.remove(new_node)
                    new_node.cost = new_node.tmp_cost
                    open_list.insert_right(new_node)
                    continue
                else:
                    new_node.cost = new_node.tmp_cost
                    new_node.open = True
                    open_list.insert_right(new_node)

    def find_path(self, start_pos, time_limit=5 * 60):
        '''Finds a path for a single robot from its init_pos to its goal
        position.

        All internal computation is done in an explicitly single robot
        space.  Actually plans from goal to init pos

        start_pos - (x, y, t) position

        returns - Path, cost of path
        '''
        # Check if we have already found a solution
        start_pos = tuple(start_pos)
        end_node = self.get_node(start_pos)
        if end_node.closed:
            # Have our solution already
            return tuple(end_node.back_trace_path()), end_node.cost
        end_time = timer.time() + time_limit
        while len(self.open_list) != 0:
            if timer.time() > end_time:
                raise OutOfTimeError()
            node = self.open_list.pop()
            if node.closed:
                # Don't remove previous references to node from the open
                # list, instead just skip duplicates stored in more
                # expensive places
                continue
            node.open = False
            node.closed = True
            self.expand(node, self.open_list)
            # robot has reached goal and other robots are already at
            # goal position.  Have to terminate search after expansion,
            # so that we don't lose this nodes neighbors for later
            # search
            if node.coord == start_pos:
                # Convert
                path = reversed(node.back_trace_path())
                # Remove the time component
                path = tuple(((n[0], n[1]) for n in path))
                return path, node.cost[0]
        raise NoSolutionError('init_pos ' + str(self.init_pos) +
                              ' start_pos ' + str(start_pos) + ' goals ' +
                              str(self.goal) + ' constraints ' +
                              str(self.constraints) + 'paths: ' +
                              str(self.out_paths))

    def get_cost(self, coord):
        '''Finds the cost of moving from a specified configuration to
        the goal

        used for generating more informed heuristics

        coord - (x, y, t)
        '''
        coord = tuple(coord)
        end_node = self.get_node(coord)
        if end_node.closed:
            # Have already found a path from the specified configuration
            # to the goal, so can go ahead and return the cost
            return end_node.cost[0]
        # Need to extend our search to connect to the specified
        # configuration
        path, cost = self.find_path(coord)
        return cost

    def get_h_cost(self, coord):
        '''Finds the cost of moving from a specified configuration to
        the goal, including both cost to go and expected collisions

        coord - (x, y, t)
        '''
        coord = tuple(coord)
        end_node = self.get_node(coord)
        if end_node.closed:
            # Have already found a path from the specified configuration
            # to the goal, so can go ahead and return the cost
            return end_node.cost
        # Need to extend our search to connect to the specified
        # configuration
        path, cost = self.find_path(coord)
        return end_node.cost

    def get_neighbors(self, coord):
        '''Returns the collision free neighbors of the specified
        coordinates

        coord - (x, y, t)
        '''
        coords = self.sub_search[self.policy_key].get_forwards_neighbors(
            coord, self.max_t)
        return [c for c in coords if not cbs.con_is_constrained(
                self.constraints, c[-1], coord[:2], c[:2])]

    def get_step(self, coord):
        '''Gets the policy for the given coordinates, extending planning
        to reach if necessary

        coord - (x, y, t)
        '''
        coord = tuple(coord)
        node = self.get_node(coord)
        if node.closed:
            # Have already found a path here
            return node.back_ptr.coord
        # Need to compute the path first
        self.find_path(coord)
        return node.back_ptr.coord

    def gen_limited_offset_neighbors(self, node):
        '''Stores the neighbors of a node in terms of changes in f-value

        i.e. sum of cost to reach the node and the cost to go.  Only
        considering the direct f-value cost, does not use the collision
        count, as finding the proper time to transition from
        incrementing direct cost to collision count would be hard
        '''
        node.offset_neighbors = defaultdict(lambda: [])
        coord = node.coord
        base_cost = self.get_cost(coord)
        for neib in self.get_neighbors(coord):
            offset, cols = self.get_transition_cost(coord, neib)
            new_cost = self.get_cost(neib)
            new_offset = new_cost + offset - node.cost[0]
            node.offset_neighbors[new_offset].append((new_offset, neib))
        node.offset_neighbors = dict(node.offset_neighbors)

    def get_offset_neighbors(self, coord, offset):
        '''Returns all neighbors of coord with the specified EPEA*
        offset (i.e. difference in f-value).

        This is with respect to some multirobot planner, so the f-value
        is capturing the true cost to go at the single robot level
        '''
        node = self.get_node(coord)
        try:
            return node.offset_neighbors[offset]
        except KeyError:
            self.gen_limited_offset_neighbors(coord)
            return node.offset_neighbors[offset]

    def get_limited_offset_neighbors(self, coord, max_offset, min_offset=0):
        '''Returns the set of neighbors for which the maximum difference
        in path cost if passed through is less than the specified value.

        (i.e. if you are forced to pass through coordinate x, instead of
        the optimal step, what is the difference in cost?)  Does not
        include the count of outpath collisions in this calculation

        coord      - coordinates of the node to find neihbors of
        max_offset - the maximum increase in path cost to encur in
                     choice of neibhors
        min_offset - minimum increae in path cost to encur in a neighbor

        returns - a list of tuples of the form (offset, coordinate)
        '''
        node = self.get_node(coord)
        if not node.offset_neighbors:
            self.gen_limited_offset_neighbors(node)
        # Have already pre-computed the results
        out = []
        for offset, neighbors in node.offset_neighbors.iteritems():
            if offset < min_offset:
                continue
            if offset > max_offset:
                return out
            out.extend(neighbors)
        return out

    def get_offsets(self, coord):
        '''Return the possible offsetes (i.e. differences in f-value) of
        the neighbors

        coord - coordinate at which to compute the offsets

        returns:
        list of offset values

        '''
        node = self.get_node(coord)
        if not node.offset_neighbors:
            self.gen_limited_offset_neighbors(node)
        return sorted(node.offset_neighbors.keys())


class Constraint_Node:
    '''All data needed concerning one single node, coord is to hold both
    position and time information
    '''
    def __init__(self, coord, back_ptr=None):
        '''
        coord    - coordinates of the node
        t        - time slice
        free     - is node occupied by an obstacle or obstacle free
        back_ptr - pointer to previous node in a path found for a robot
        '''
        self.coord = coord
        self.back_ptr = back_ptr
                # only tmp_cost is updated within find_single_path
                # cost is updated within expand (--> Sorted Collection)
        self.cost = (MAX_COST, MAX_COST)
        self.closed = False
        self.open = False
        self.h = 0
        self.offset_neighbors = None

    def back_trace_path(self, path=None):
        '''Returns the full path starting at the node from which it is
        called'''
        if path is None:
            path = []
        path.insert(0, self.coord[:])
        if self.back_ptr != self:
            return self.back_ptr.back_trace_path(path)
        return path


class Forward_Constraint_Node(object):
    '''All data needed concerning one single node, coord is to hold both
    position and time information
    '''
    def __init__(self, coord, t, back_ptr=None):
        '''
        coord    - coordinates of the node
        t        - time slice
        free     - is node occupied by an obstacle or obstacle free
        back_ptr - pointer to previous node in a path found for a robot'''
        self.coord = coord
        self.back_ptr = back_ptr
                # only tmp_cost is updated within find_single_path
                # cost is updated within expand (--> Sorted Collection)
        self.cost = [MAX_COST, MAX_COST]
        self.closed = False
        self.open = False
        self.h = 0
        # This is the time coordinate of the node.  This has a maximum
        # value equal to one +  the greater of the length of the out
        # paths or the time of the last constraint.  This is because
        # once the rest of the environment is static, time doesn't
        # matter for the trajectory
        self.t = t 
        # holds the current time without max operations.  Only used to
        # compute the sum_of_costs metric
        self.time_for_cost = 0

    def back_trace_path(self, path=None):
        '''Returns the full path starting at the node from which it is
        called
        '''
        if path is None:
            path = []
        path.insert(0, self.coord[:])
        if self.back_ptr != self:
            return self.back_ptr.back_trace_path(path)
        return path


class Constrained_Forwards_Planner(object):
    '''Performs constrained planning for a single robot'''
    def __init__(self, obs_map, init_pos, goal, constraints, sub_search=None,
                 conn_8=False, inflation=1.0, out_paths=None,
                 sum_of_costs=False):
        '''initialize variables (obstacle map, positions, ...)
        
        Every action except waiting at the goal configuration incurs
        cost 1. Waiting at the goal configuration incurs zero cost

        obs_map     - binary obstacle map
        init_pos    - start positions
        goal        - goal position
        constraints - constraints defining where robot cannot go, should
                      consist of a single tuple, with only the
                      appropriate robots
        conn_8      - 8-connected grid if True; otherwise 4-connected
        out_paths   - paths of other robots to avoid, if possible
                       without incuring extra cost
        '''
        self.obs_map = obs_map
        self.goal = tuple(goal)
        self.init_pos = tuple(init_pos)
        self.constraints = constraints
        assert(len(cbs.con_get_robots(constraints)) == 1)
        # Store as tuple, as just need as dictionary key
        self.rob_id = cbs.con_get_robots(constraints)
        self.conn_8 = conn_8
        # Sub-policies as weighting functions
        self.sub_search = sub_search
        # Graph is stored as dictionary indexed by strings of coord and
        # tuple
        self.graph = {}
        # initialize with "normal" indices; is overwritten if other
        # priority ordering is applied. Need time one greater than that
        # of the last constraint to ensure happy
        self.goal_t = cbs.con_get_max_time(self.constraints) + 1
        self.max_t = self.goal_t
        if out_paths is not None:
                self.max_t = max(self.goal_t, len(out_paths))
        self.inflation = inflation
        self.out_paths = out_paths
        self.col_check = workspace_graph.Edge_Checker()
        # Generate policy for each robot
        if self.sub_search is None:
            self.sub_search = {
                self.rob_id: workspace_graph.Astar_Graph(obs_map, goal,
                                                         self.conn_8)}
        elif not self.rob_id in self.sub_search:
            self.sub_search[self.rob_id] = workspace_graph.Astar_Graph(
                obs_map, goal, self.conn_8)

    def heuristic(self, coord):
        '''Computes the heuristic costs for the robot from its position
        (coord) to the goal
        '''
        return self.sub_search[self.rob_id].get_cost(coord)

    def get_node(self, coord, t):
        '''To expand the graph at the specified coordinate and time'''
        c = coord + (t, )
        # Node already exists in the graph
        if c in self.graph:
            node = self.graph[c]
            return node
        # Otherwise initiate node
        else:
            # Add node to graph
            node = Forward_Constraint_Node(coord, t)
            self.graph[c] = node
            node.h = self.heuristic(node.coord)
            return node

    def expand(self, node, open_list):
        '''Calls get_neighbor from workspace_graph, checks for
        collisions and  adds all found neighbors to the open_list

        used for performing  constrained planning
        '''
        nt = min(node.t + 1, self.max_t)
        neighbors = []
        # TODO: should opt be implemented as flag?
        coords = self.sub_search[self.rob_id].get_neighbors(node.coord)
        for i in coords:
            new_node = self.get_node(i, nt)
            if new_node.closed:
                continue
            temp_cost = self.transition_cost(node, new_node)
            if (temp_cost < new_node.cost):
                # Check for constraint violation
                if cbs.con_is_constrained(self.constraints, nt, node.coord, i):
                    # Hit constraint, can skip rest of tests
                    continue
                # new_node.tmp_cost = node.cost + step_cost
                new_node.cost = temp_cost
                new_node.back_ptr = node
                new_node.open = True
                open_list.insert_right(new_node)

    def transition_cost(self, prev_node, new_node):
        '''Computes the cost of the new neighbor when steping from
        prev_node to the specified coordinates
        '''
        step_cost = 1
        # cost = 0 when robot stays at goal position
        if prev_node.coord == self.goal and new_node.coord == self.goal:
            step_cost = 0
        # Can add booleans to ints, True == 1, False == 0
        out_col = self.col_check.single_bot_outpath_check(
            new_node.coord, prev_node.coord, new_node.t, self.out_paths)
        return [prev_node.cost[0] + step_cost, prev_node.cost[1] + out_col]

    def find_path(self, time_limit=5 * 60):
        '''Finds a path for a single robot from its init_pos to its goal
        position

        all internal computation is done in an explicitly single robot
        space.

        returns - Path, cost of path
        '''
        end_time = timer.time() + time_limit
        first = self.get_node(self.init_pos, 0)
        first.open = True
        first.cost = [0, 0]
        first.back_ptr = first
        open_list = SortedCollection.SortedCollection(
            [first], key=lambda x: [-x.cost[0]-x.h * self.inflation,
                                    -x.cost[1]])
        while len(open_list) != 0:
            if timer.time() > end_time:
                raise OutOfTimeError()
            node = open_list.pop()
            if node.closed:
                continue
            node.open = False
            node.closed = True
            # robot has reached goal and other robots are already at
            # goal position
            if node.coord == self.goal and node.t >= self.goal_t:
                # Convert
                return tuple(node.back_trace_path()), node.cost[0]
            self.expand(node, open_list)
        raise NoSolutionError(str(self.init_pos) + ' ' + str(self.constraints))


def find_forwards_constrained_path(
        obs_map, init_pos, goal, constraints, sub_search=None, conn_8=False,
        inflation=1.0, time_limit=5 * 60, out_paths=None, sum_of_costs=False):
        '''initialize variables (obstacle map, positions, ...).
        obs_map      - binary obstacle map
        init_pos     - start positions
        goal         - goal position
        constraints  - constraints defining where robot cannot go,
                       should consist of a single tuple, with only the
                       appropriate robots
        conn_8       - 8-connected grid if True; otherwise 4-connected
        sum_of_costs - use sum of costs, i.e. cost = time until agent
                       reaches the goal and does not depart
        '''
        if sum_of_costs:
            p = Sum_Of_Cost_Constrained_Forwards_Planner(
                obs_map, init_pos, goal, constraints, sub_search=sub_search,
                conn_8=conn_8, inflation=inflation, out_paths=out_paths)
        else:
            p = Constrained_Forwards_Planner(
                obs_map, init_pos, goal, constraints, sub_search=sub_search,
                conn_8=conn_8, inflation=inflation, out_paths=out_paths)
        # Need to specify when you are starting your path from
        return p.find_path(time_limit=time_limit)

class Sum_Of_Cost_Constrained_Forwards_Planner(Constrained_Forwards_Planner):
    '''Variant of Constrained_Forwards_Planner that uses a sum-of-costs
    function

    The sum-of-costs cost function is equal to the time required for an
    agent to reach its goal for the last time, i.e. time until the agent
    reaches its goal and stays there for good
    '''

    def transition_cost(self, prev_node, new_node):
        '''Computes the cost of new_node when reached from prev_node

        If away from goal, or just reaching the goal the total cost is
        equal to the current time.  If remains at the goal

        prev_node - Forwards_Constraint_Node specifying source node
        new_node  - Forwards_Constraint_Node specifying target node

        returns:
        cost of prev_node
        '''
        new_node.time_for_cost = prev_node.time_for_cost + 1
        g_value = new_node.time_for_cost
        if prev_node.coord == self.goal and new_node.coord == self.goal:
            g_value = prev_node.cost
        # Can add booleans to ints, True == 1, False == 0
        out_col = self.col_check.single_bot_outpath_check(
            new_node.coord, prev_node.coord, new_node.t, self.out_paths)
        return [new_node.time_for_cost, prev_node.cost[1] + out_col]
