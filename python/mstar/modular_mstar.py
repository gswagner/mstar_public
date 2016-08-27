"""Implementation of subdimensional expansion using operator
decomposition instead of vanilla A*, with better graph abstraction.
All coordinates are to be tuples and all collision sets are to be lists
of immutable sets (frozenset). This partial rewrite will focus on
converting everything that can be immutable into an immutable structure

Intended to support both mstar and rMstar."""


import workspace_graph
import sys
import time as timer  # So that we can use the time command in ipython
import SortedCollection
from col_set_addition import add_col_set_recursive, add_col_set
from col_set_addition import effective_col_set
from col_set_addition import OutOfTimeError, NoSolutionError, col_set_add
from od_mstar import MAX_COST
from itertools import izip
import copy
import collections
try:
    import ipdb as pdb
except ImportError:
    # Default to pdb
    import pdb
import inspect


MAX_COST = workspace_graph.MAX_COST
PER_ROBOT_COST = 1  # Cost a robot accrues for not being at its goal position
POSITION = 0
MOVE_TUPLE = 1  # Tuple of destination coordinate tuples for each robot's move

global_move_list = []  # Used for visualization

ODCoord = collections.namedtuple('ODCoord', ['coord', 'move_list'])


def __init__(self, world, goals, recursive, sub_search=None,
             rob_id=None, inflation=1.0, end_time=10 ** 15,
             col_set_memory=False):
    """
    world             - obstacle map,  matrix with 0 for free,  1
                        for obstacle
    goals             - ((x1, y1), (x2, y2), ...) coordinates of the
                        goal, should be tuples
    recursive         - True for rM*, false for basic M*
    sub_search        - Sub planners, should be None for the full
                        configuration space
    rob_id            - maps local robot identity to full
                        configuration space identity,  should be
                        None for the full configuration space
                        instance
    inflation         - how much the metric should be inflated by
    end_time          - when the search should be terminated
    col_set_memory    - remember previous step collision set,
                        intended to provide more efficient cached
                        path utillization.  False by default
    """
    # visualize - turn on visualization code - DISABLED
    self._world = world
    self._recursive = recursive
    self._sub_search = sub_search
    # Stores the global ids of the robots in order of their position
    # in coord
    self._rob_id = rob_id
    self._goals = goals
    # Graph that holds the graph representing the joint configuration space
    self._graph = {}
    self._end_time = end_time
    self._inflation = float(inflation)

    # Store some useful values
    self._updated = 0
    self._num_bots = len(goals)
    self._open_list_key = lambda x: -x.cost - x.h * self._inflation
    if self._rob_id is None:
        self._rob_id = tuple(range(len(goals)))
    self._col_checker = self._default_col_checker
    # Making everything that can be immutable,  immutable
    self._col_set_memory = col_set_memory
    self.gen_policy_planners(sub_search, self._world, self._goals)


def epea__init__(self, world, goals, recursive, sub_search=None,
                 rob_id=None, inflation=1.0, end_time=10 ** 15,
                 col_set_memory=False):
    __init__(self, world, goals, recursive, sub_search=sub_search,
             rob_id=rob_id, inflation=inflation, end_time=end_time,
             col_set_memory=col_set_memory)
    self._open_list_key = lambda x: -x.cost - x.h * self._inflation - x.offset


def gen_policy_planners(self, sub_search, world, goals):
    """Creates the sub-planners and necessary policy keys.  This is
    because pretty much every sub-planner I've made requires
    adjusting the graph used to create the policies and passing
    around dummy sub_searches

    side effects to generate self._sub_search and self.policy_keys
    """
    self.policy_keys = tuple([(i, ) for i in self._rob_id])
    self._sub_search = sub_search
    if self._sub_search is None:
        self._sub_search = {}
        # Wrapping the robot number in a tuple so we can use the same
        # structure for planners for compound robots
        for dex, key in enumerate(self.policy_keys):
            self._sub_search[key] = workspace_graph.Astar_Graph(
                world, goals[dex], connect_8=False,
                makespan=False)


def _reset(self):
    """resets the map for later searches

    does not remove forwards_pointer
    """
    self._updated += 1


########################################################################
# heuristic
########################################################################
def od_heuristic(self, coord):
    """Returns the heuristic value of the specified coordinate for
    operator decomposition

    Does not handle inflation naturally so we can update the
    heuristic properly

    coord - ODCoord coordinate of the node at which to compute the

    return:
    heuristic cost to go
    """
    if not coord.move_list:
        # standard node
        cost = sum(self._sub_search[key].get_cost(coord.coord[dex])
                   for dex, key in enumerate(self.policy_keys))
        # return self._inflation * cost
        return cost
    else:
        # Compute heuristic for robots which have moved
        cost = sum(self._sub_search[key].get_cost(coord.move_list[dex])
                   for dex, key in enumerate(
                       self.policy_keys[:len(coord.move_list)]))
        # compute heuristic for robots which have not moved
        cost += sum(
            self._sub_search[key].get_cost(
                coord.coord[dex + len(coord.move_list)]) for dex, key in
            enumerate(self.policy_keys[len(coord.move_list):]))
        return cost


def heuristic(self, coord):
    """Returns the heuristic value of the specified coordinate for
    operator decomposition

    Does not handle inflation naturally so we can update the
    heuristic properly

    coord - coordinate of the node at which to compute the

    return:
    heuristic cost to go
    """
    cost = sum(self._sub_search[key].get_cost(coord[dex])
               for dex, key in enumerate(self.policy_keys))
    # return self._inflation * cost
    return cost


########################################################################
# pass_through
########################################################################
def od_pass_through(self, coord1, coord2):
    """Tests for a collision during transition from coord 1 to coord
    2.

    coord1, coord2 - joint coordinates of multirobot system as ODCoord
                     instances

    returns:

    collision set for the edge,  empty list if there are no
    collisions
    """
    if coord2.move_list:
        return self._col_checker.cross_over(
            coord1.coord[:len(coord2.move_list)], coord2.move_list,
            self._recursive)
    return self._col_checker.cross_over(coord1.coord, coord2.coord,
                                        self._recursive)


def pass_through(self, coord1, coord2):
    """Tests for a collision during transition from coord 1 to coord
    2.

    coord1, coord2 - joint coordinates of multirobot system

    returns:

    collision set for the edge,  empty list if there are no
    collisions
    """
    return self._col_checker.cross_over(coord1, coord2, self._recursive)


def incremental_col_check(self, start_coord, new_coord):
    """Performs an incremental collision check for new coord.

    Assumes that the position of a single new robot has been added to
    a list of coordinates of robots.  Checks whether adding this new
    robot will lead to a collision.  Start coord is the joint state
    before the action being built in new_coord,  and may contain more
    robots than new_coord. counts on the implementation of the
    incremental collision checks to be intelligent to avoid issues

    start_coord - coordinate at which the system starts
    new_coord   - coordinate to which the system moves

    returns:

    collision_set formed form the colliding robots during the move
    """
    col_set = self._col_checker.incremental_cross_over(
        start_coord, new_coord, self._recursive)
    if col_set:
        return col_set
    return self._col_checker.incremental_col_check(
        new_coord, self._recursive)


########################################################################
# get_node
########################################################################
def get_od_node(self, coord):
    """Returns the node at the specified coordinates.

    coordinates are ODCoord instances

    coord - coordinates of the node,  potentially an intermediate node
    """
    if coord in self._graph:
        # Node already exists.  reset if necessary
        t_node = self._graph[coord]
        t_node.reset(self._updated)
        return t_node
    # Need to instantiate the node
    if coord.move_list:
        # Only check for collisions between robots whose move has
        # been determined
        col = self._col_checker.col_check(coord.move_list, self._recursive)
    else:
        col = self._col_checker.col_check(coord.coord, self._recursive)
    free = (len(col) == 0)
    t_node = ODNode(coord, free, self._recursive)
    # Cache the resultant col_set
    t_node.col_set = col
    t_node.updated = self._updated
    t_node.h = self.heuristic(coord)
    # Add the node to the graph
    self._graph[coord] = t_node
    return t_node


def get_epea_node(self, coord):
    """Returns the node at the specified coordinates.

    coord - coordinates of the node
    """
    if coord in self._graph:
        # Node already exists.  reset if necessary
        t_node = self._graph[coord]
        t_node.reset(self._updated)
        return t_node
    # Need to instantiate the node
    col = self._col_checker.col_check(coord, self._recursive)
    free = (len(col) == 0)
    t_node = EPEANode(coord, free, self._recursive)
    # Need to compute the maximum offset for use in terminating search
    offsets = [self._sub_search[self.policy_keys[i]].get_offsets(c)
               for i, c in enumerate(coord)]
    t_node.max_offset = sum(max(offsets))
    # Cache the resultant col_set
    t_node.col_set = col
    t_node.updated = self._updated
    t_node.h = self.heuristic(coord)
    # Add the node to the graph
    self._graph[coord] = t_node
    return t_node


def get_step(self, init_pos):
    """Get the optimal step from init_pos.

    Computes the entire optimal path if necessary, but preferentially
    relying on the cached paths stored in mstar_node.forwards_ptr.

    init_pos - coordinate of the node to compute the step from

    returns:
    coordinate of the optimal step towards the goal
    """
    cur_node = self.get_node(init_pos)
    temp = cur_node.get_step()
    if temp is not None:
        return temp
    # Use a zero time limit,  so the end time will not be modified
    path = self._find_path(init_pos, time_limit=-1)
    return cur_node.get_step()


def gen_init_nodes(self, init_pos):
    """Generate the initial search nodes.

    Potentially more than one node is generated, but in practice
    will usually just one will be generated

    init_pos - initial position

    returns:

    list of initial nodes
    """
    first = self.get_node(init_pos)
    first.open = True
    first.cost = 0
    first.back_ptr = first
    return [first]


########################################################################
# find_path
########################################################################
def find_path(self, init_pos, time_limit=5 * 60):
    """Public function for finding a path

    init_pos   - (q1, ..., qn) list of coordinates defining initial
                 configuration
    time_limit - time allocated to find a solution

    returns:
    path in the joint configuration space

    raises:
    OutOfTimeError
    NoSolutionError
    """
    return self._find_path(init_pos, time_limit=time_limit)


def od_find_path(self, init_pos, time_limit=5 * 60):
    """Public function for finding a path

    converts from configurations to ODCoord

    init_pos   - (q1, ..., qn) list of coordinates defining initial
                 configuration
    time_limit - time allocated to find a solution

    returns:
    path in the joint configuration space

    raises:
    OutOfTimeError
    NoSolutionError
    """
    path = self._find_path(ODCoord(coord=init_pos, move_list=()), time_limit)
    # convert back to standard coordinates
    return tuple(coord.coord for coord in path)


def _find_path(self, init_pos, time_limit=5 * 60):
    """Private function for finding a path

    Used for recursive calls, etc, and only works with the internal
    coordinate representation

    init_pos   - ((x1, y1), (x2, y2), ...) coordinates of initial
                 position
    time_limit - time allocated to find a solution.  Will raise an
                 exception if a path cannot be found within this time
                 period

    returns:
    path in the joint configuration space

    raises:
    OutOfTimeError
    NoSolutionError
    """
    self._reset()
    if time_limit > 0:
        self._end_time = timer.time() + time_limit
        # For replanning to work correctly, need to update the end
        # time for all subplanners.  Otherwise, the end time of the
        # subplanners will never be updated, so if you make a query
        # more than the original time_limit seconds after the first
        # query to this object, you will always get a timeout,
        # regardless of the time limit used on the second query
        for planner in self._sub_search.itervalues():
            if hasattr(planner, '_end_time'):
                planner._end_time = self._end_time

    # Use the negation of the cost,  so SortedCollection will put the
    # lowest value item at the right of its internal list
    init_nodes = self.gen_init_nodes(init_pos)
    open_list = SortedCollection.SortedCollection(init_nodes,
                                                  key=self._open_list_key)

    while len(open_list) > 0:
        if timer.time() > self._end_time:
            raise OutOfTimeError(timer.time())
        node, consistent = open_list.consistent_pop()
        if not consistent:
            continue
        node.open = False
        if self.solution_condition(node):
            path = node.get_path()
            return tuple(path)
        self.expand(node, open_list)
    raise NoSolutionError(self._rob_id)


########################################################################
# solution_condition
########################################################################
def od_solution_condition(self, node):
    """Checks whether we have finished finding a path when node has
    been reached

    Checks whether node.forwards_ptr indicates that a path to the
    goal has been found

    node - node to check for indicating a path to the goal

    returns:

    True if goal has been reached or a cached path to the goal has
    been reached, else False
    """
    if node.forwards_ptr is not None:
        return True
    if not node.coord.move_list and node.coord.coord == self._goals:
        node.forwards_ptr = node
        return True
    return False


def solution_condition(self, node):
    """Checks whether we have finished finding a path when node has
    been reached

    Checks whether node.forwards_ptr indicates that a path to the
    goal has been found

    node - node to check for indicating a path to the goal

    returns:

    True if goal has been reached or a cached path to the goal has
    been reached, else False
    """
    if node.forwards_ptr is not None:
        return True
    if node.coord == self._goals:
        node.forwards_ptr = node
        return True
    return False


########################################################################
# expand
########################################################################
def epeastar_expand(self, node, open_list):
    """Handles the expansion of the given node and the addition of
    its neighbors to the open list

    node      - node to expand
    open_list - open list used during the search
    """
    old_col_set = node.col_set
    self._expand(node, open_list)
    if old_col_set == node.col_set:
        # If the collision set changed,  then adding the node
        # back to the open list with properly updated collision
        # set has been handled by the backprop function
        node.offset += self._offset_increment
        if node.offset <= node.max_offset:
            open_list.insert(node)


def expand(self, node, open_list):
    """Handles the expansion of the given node and the addition of
    its neighbors to the open list

    node      - node to expand
    open_list - open list used during the search
    """
    node.closed = True

    generating_col_set = ()
    if self._recursive:
        neighbors, generating_col_set = self.recursive_neighbors(
            node.coord, node.col_set, node.prev_col_set, node)
    else:
        neighbors = self.nonrecursive_neighbors(node.coord, node.col_set, node)

    col_set = []
    for new_coord in neighbors:
        # Check for collisions in the transition to a new position
        pass_col = self.pass_through(node.coord, new_coord)
        if pass_col:
            # Have a robot-robot collision along the edge
            col_set = col_set_add(pass_col, col_set, self._recursive)
            continue
        try:
            new_node = self.get_node(new_coord)
        except NoSolutionError:
            # We constructed a state from which there is no path to the
            # goal
            continue

        if node not in new_node.back_prop_set:
            new_node.back_prop_set.append(node)
        # Always need to add the col_set of any vertex that we can
        # actually reach,  as otherwise,  we would need to wait for
        # another robot to collide downstream of the reached vertex
        # before that vertex would back propagate its col_set
        col_set = col_set_add(new_node.col_set, col_set, self._recursive)
        if new_node.closed or not new_node.free:
            continue
        new_cost = self.transition_cost(node.coord, new_coord, node.cost)
        if new_cost >= new_node.cost:
            continue
        new_node.cost = new_cost

        # Annoying hack to hopefully improve ODM* performance (should
        # probably check that this actually helps)
        try:
            # Set the intermediate nod's col_set equal to its parent,
            # so later elements will actually be explored.  Not
            # technically required but will cut back on thrashing
            if new_node.coord.move_list:
                new_node.add_col_set(node.col_set)
        except AttributeError:
            # not ODM*, so skip
            pass

        if self._col_set_memory:
            node.prev_col_set = generating_col_set

        new_node.back_ptr = node
        # Even if the node is already in the open list,  removing if
        # from its old position (given by the old cost value) is too
        # expensive, requiring an O(N) operation to delete.  Simply
        # add the new value and reject the old copy (which will be
        # marked as closed),  when you come to it
        new_node.open = True
        open_list.insert_right(new_node)
    node.back_prop_col_set(col_set, open_list)


########################################################################
# nonrecursive_neighbors
########################################################################
def od_mstar_neighbors(self, coord, col_set, node):
    """Returns the coordinates of all legal neighbors using operator
    decomposition

    coord   - coord to generate neighbors of.  Must be and ODCoord
    col_set - collision set of node corresponding to col_set
    node    - For compatability purposes

    returns:
    coordinates of neighboring nodes
    """
    rob_dex = 0  # Keeps track of the robot to move in this step

    # split the coordinates into the start coordinate and the move
    # list if the node is standard,  doing this so variables are
    # initialized in  the preferred namespace,  which is probably not
    # necessary
    start_coord = coord.coord
    move_list = coord.move_list
    rob_dex = len(move_list)
    if ((len(col_set) > 0 and rob_dex in col_set[0])):
        # This robot is in the collision set,  so consider all
        # possible neighbors
        actions = self._sub_search[
            self.policy_keys[rob_dex]].get_neighbors(start_coord[rob_dex])
    else:
        actions = [self._sub_search[self.policy_keys[rob_dex]].get_step(
            start_coord[rob_dex])]
    # check if this is the last robot to be moved
    filled = (rob_dex == (self._num_bots - 1))

    if filled:
        return [ODCoord(move_list + (action, ), ()) for action in actions]
    return [ODCoord(start_coord, move_list + (action, )) for action in actions]


def epeastar_neighbors(self, coord, col_set, node):
    """Helper function for generating neighbors of a node using EPEA*

    Uses a two step process. First the incremental costs are
    computed, then the neighbors fitting those incremental costs.
    More directly matches what was done in the EPEA* paper.

    Does not do incremental collision checking to better fit the new
    design

    coord   - coord to generate neighbors of.  Must be and ODCoord
    col_set - collision set of node corresponding to col_set
    node    - passed to capture offset

    returns:
    coordinates of neighboring nodes
    """
    offset = node.offset
    adder = add_col_set
    if self._recursive:
        adder = add_col_set_recursive
    if len(col_set) == 0:
        # have empty collision set
        new_coord = tuple(
            self._sub_search[self.policy_keys[dex]].get_step(
                coord[dex]) for dex in xrange(self. num_bots))
        pass_col = self. pass_through(coord, new_coord)
        if pass_col:
            return [], pass_col
        col = self.col_checker.col_check(new_coord, self._recursive)
        if col:
            return [], col
        return [new_coord], []
    search_list = [(0, ())]
    assert len(col_set) == 1
    node_col = col_set[0]
    for rob_dex in xrange(self._num_bots):
        if rob_dex in node_col:
            offsets = self._sub_search[
                self.policy_keys[rob_dex]].get_offsets(coord[rob_dex])
        else:
            offsets = (0, )
        new_list = []
        for cost, pos in search_list:
            for off in offsets:
                if rob_dex < self._num_bots - 1:
                    if off + cost <= offset:
                        new_list.append((off + cost, pos + (off, )))
                elif off + cost == offset:
                    # For the last robot,  only want to keep costs which
                    # match perfectly
                    new_list.append((off + cost, pos + (off, )))
            search_list = new_list
    neighbors = []
    col_set = []
    for offset, costs in search_list:
        gen_list = [()]
        for dex, c in enumerate(costs):
            if dex in node_col:
                neib = (self._sub_search[
                        self.policy_keys[dex]].get_offset_neighbors(
                        coord[dex], c))
            else:
                neib = ((0, self._sub_search[
                    self.policy_keys[dex]].get_step(coord[dex])),)
            new_list = []
            for _, n in neib:
                for old in gen_list:
                    new_coord = old + (n, )
                    new_list.append(new_coord)
            gen_list = new_list
        neighbors.extend(gen_list)
    return neighbors


########################################################################
# transition_cost
########################################################################
def od_transition_cost(self, start_coord, next_coord, prev_cost):
    """Computes the transition cost for ODM*

    Can handle neighbors generated both in the recursive fashion and
    non-recursive fashion

    start_coord - coordinate of source node
    next_coord  - coordinate of target node
    prev_cost   - cost of path to start_coord

    returns:
    cost of path that reaches next_coord via start_coord
    """
    if next_coord.move_list:
        rob_dex = len(next_coord.move_list) - 1
        return (prev_cost +
                self._sub_search[self.policy_keys[rob_dex]].get_edge_cost(
                    start_coord.coord[rob_dex], next_coord.move_list[-1]))
    elif start_coord.move_list:
        # transition from a intermediate node to a standard node
        return (prev_cost +
                self._sub_search[self.policy_keys[-1]].get_edge_cost(
                    start_coord.coord[-1], next_coord.coord[-1]))
    # transition between standard nodes, i.e. a recursive M* transition
    for dex, (source, target) in enumerate(izip(start_coord.coord,
                                                next_coord.coord)):
        prev_cost += self._sub_search[self.policy_keys[dex]].get_edge_cost(
            source, target)
    return prev_cost


def transition_cost(self, start_coord, next_coord, prev_cost):
    """Computes the transition cost for ODM*

    start_coord - coordinate of source node
    next_coord  - coordinate of target node
    prev_cost   - cost of path to start_coord

    returns:
    cost of path that reaches next_coord via start_coord
    """
    # transition between standard nodes, i.e. a recursive M* transition
    for dex, (source, target) in enumerate(izip(start_coord, next_coord)):
        prev_cost += self._sub_search[self.policy_keys[dex]].get_edge_cost(
            source, target)
    return prev_cost


def create_sub_search(self, new_goals, rob_id):
    """Creates a new instance of a sub_search for recursive search

    new_goals - goals for the subset of the robots
    rob_ids   - ids of the robots involved in the sub_search

    returns:
    new class instance to perform search for the specified subset
    of robots
    """
    # Extract the information needed to fill out the constructor
    args, vargs, varkw, defaults = inspect.getargspec(self.__init__)
    overriden = set(['world', 'goals', 'rob_id'])
    arg_names = set(args[1:]).difference(overriden)

    # Grab any additional arguments that were passed in using kwargs
    try:
        arg_names.update(self._additional_fields)
    except AttributeError:
        pass

    kwargs = {}
    for key in arg_names:
        try:
            kwargs[key] = getattr(self, key)
        except AttributeError:
            kwargs[key] = getattr(self, '_' + key)
    kwargs['rob_id'] = rob_id
    return self.__class__(self._world, new_goals, **kwargs)


def get_subplanner_keys(self, col_set):
    """Returns keys to subplanners required for planning for some
    subset of robots.

    col_set - collision set to be solved

    returns:

    keys for the necessary subplanners in self._sub_search
    """
    # Convert the collision sets into the global indicies,  and
    # convert to tuples.  Assumes self._rob_id is sorted
    global_col = map(lambda y: tuple(map(lambda x: self._rob_id[x], y)),
                     col_set)
    # generate the sub planners,  if necessary
    for dex, gc in enumerate(global_col):
        if gc not in self._sub_search:
            t_goals = tuple([self._goals[k] for k in col_set[dex]])
            self._sub_search[gc] = self.create_sub_search(t_goals, gc)
    return global_col


def od_recursive_neighbors(self, coord, col_set, prev_col_set, node):
    """Get the neighbors of a node for recursive ODM*

    may fail if called an an intermediate node by ODM*

    node           - coordinate of node for which to find neighbors,
                     must be an ODCoord instance
    col_set        - collision set of source node
    col_set_memory - collision set that was used to generate the current
                     node.  Used to make more aggressive use of cached
                     paths
    node           - node to expand, to extract random information for
                     different planning approaches

    returns: neighbors, generating collision set
    neighbors          - list of coordinates for neighboring, reachable
                         nodes as ODCoords
    generating_col_set - Collision set used to generate rM* neighbors.
                         Empty tuple if delegated to
                         get_neighbors_nonrecursive.  Used to calculate
                         the collision set memory.
    """
    # Handle collision set memory if necessary
    if self._col_set_memory:
        col_set = effective_col_set(col_set, prev_col_set)
        effective_set = col_set
        # Sort the collision set,  which also converts them into
        # lists
        col_set = map(sorted, col_set)
    else:
        # Sort the collision set,  which also converts them into lists
        col_set = map(sorted, col_set)

    # If the collision set contains all of the robots, invoke the
    # non-recursive planner
    if len(col_set) == 1 and len(col_set[0]) == self._num_bots:
        # At base of recursion case
        return self.nonrecursive_neighbors(coord, col_set, node), ()

    start_coord = coord.coord
    # Generate subplanners for new coupled groups of robots and get
    # their sub_search keys
    coupled_keys = self.get_subplanner_keys(col_set)

    # Generate the individually optimal step
    new_coord = [self._sub_search[self.policy_keys[i]].get_step(
        start_coord[i]) for i in xrange(self._num_bots)]

    # Iterate over the colliding sets of robots,  and integrate the
    # results of the sup planning for each set
    for i in xrange(len(col_set)):
        try:
            new_step = self._sub_search[coupled_keys[i]].get_step(
                ODCoord(tuple([start_coord[j] for j in col_set[i]]), ()))
        except NoSolutionError:
            # Can't get to the goal from here
            return [], col_set
        # Copy the step into position
        for j in xrange(len(col_set[i])):
            new_coord[col_set[i][j]] = new_step.coord[j]
    return [ODCoord(tuple(new_coord), ())], col_set


def recursive_neighbors(self, coord, col_set, prev_col_set, node):
    """Get the neighbors of a node for recursive M*

    node           - coordinate of node for which to find neighbors
    col_set        - collision set of source node
    col_set_memory - collision set that was used to generate the current
                     node.  Used to make more aggressive use of cached
                     paths
    node           - node to expand, to extract random information for
                     different planning approaches

    returns: neighbors, generating collision set
    neighbors          - list of coordinates for neighboring, reachable
                         nodes as ODCoords
    generating_col_set - Collision set used to generate rM* neighbors.
                         Empty tuple if delegated to
                         get_neighbors_nonrecursive.  Used to calculate
                         the collision set memory.
    """
    # Handle collision set memory if necessary
    if self._col_set_memory:
        col_set = effective_col_set(col_set, prev_col_set)
        effective_set = col_set
        # Sort the collision set,  which also converts them into
        # lists
        col_set = map(sorted, col_set)
    else:
        # Sort the collision set,  which also converts them into lists
        col_set = map(sorted, col_set)

    # If the collision set contains all of the robots, invoke the
    # non-recursive planner
    if len(col_set) == 1 and len(col_set[0]) == self._num_bots:
        # At base of recursion case
        return self.nonrecursive_neighbors(coord, col_set, node), ()

    start_coord = coord
    # Generate subplanners for new coupled groups of robots and get
    # their sub_search keys
    coupled_keys = self.get_subplanner_keys(col_set)

    # Generate the individually optimal step
    new_coord = [self._sub_search[self.policy_keys[i]].get_step(
        start_coord[i]) for i in xrange(self._num_bots)]

    # Iterate over the colliding sets of robots,  and integrate the
    # results of the sup planning for each set
    for i in xrange(len(col_set)):
        try:
            new_step = self._sub_search[coupled_keys[i]].get_step(
                tuple([start_coord[j] for j in col_set[i]]))
        except NoSolutionError:
            # Can't get to the goal from here
            return [], col_set
        # Copy the step into position
        for j in xrange(len(col_set[i])):
            new_coord[col_set[i][j]] = new_step[j]
    return [tuple(new_coord)], col_set


# Setup methods
OD_MSTAR_DICT = {
    '__init__': __init__,
    'gen_policy_planners': gen_policy_planners,
    '_reset': _reset,
    'heuristic': od_heuristic,
    'pass_through': od_pass_through,
    'incremental_col_check': incremental_col_check,
    'get_node': get_od_node,
    'get_step': get_step,
    'gen_init_nodes': gen_init_nodes,
    'find_path': od_find_path,
    '_find_path': _find_path,
    'solution_condition': od_solution_condition,
    'expand': expand,
    'transition_cost': od_transition_cost,
    'nonrecursive_neighbors': od_mstar_neighbors,
    'create_sub_search': create_sub_search,
    'get_subplanner_keys': get_subplanner_keys,
    'recursive_neighbors': od_recursive_neighbors
}
# Setup default values
OD_MSTAR_DICT.update({
    '_default_col_checker': workspace_graph.Edge_Checker()
})
EPEMSTAR_DICT = copy.deepcopy(OD_MSTAR_DICT)
EPEMSTAR_DICT.update({
    '__init__': epea__init__,
    'heuristic': heuristic,
    'pass_through': pass_through,
    'get_node': get_epea_node,
    'find_path': find_path,
    'solution_condition': solution_condition,
    'transition_cost': transition_cost,
    'nonrecursive_neighbors': epeastar_neighbors,
    'recursive_neighbors': recursive_neighbors,
    'expand': epeastar_expand,
    '_expand': expand,
    '_offset_increment': 1
})


def gen_policy_planner_factory(policy_class):
    """Generates a gen_policy_planner function for the specified class
    of policy

    policy_class - callable(world, goal) that returns a policy
                   instance.  Can be a class instance or a function

    returns:
    method that populates the sub_search field of the calling class
    """
    def gen_policy_planners(self, sub_search, world, goals):
        """Creates the sub-planners and necessary policy keys.  This is
        because pretty much every sub-planner I've made requires
        adjusting the graph used to create the policies and passing
        around dummy sub_searches

        side effects to generate self._sub_search and self.policy_keys
        """
        self.policy_keys = tuple([(i, ) for i in self._rob_id])
        self._sub_search = sub_search
        if self._sub_search is None:
            self._sub_search = {key: policy_class(world, goals[dex])
                                for dex, key in enumerate(self.policy_keys)}
    return gen_policy_planners


def mstar_class_factory(name, function_dict, superclasses=(object, ),
                        source_dict=OD_MSTAR_DICT):
    """Factory to generate mstar classes

    the base functions come from source_dict

    name          - name of class to generate
    function_dict - dictionary of functions, will overwrite any
                    functions from source_dict
    superclasses  - tuple of superclasses.  By default contains object
    source_dict   - basic source of functions.  Its main purpose is to
                    change from being based on ODMstar to EPEA*
    """
    base_dict = copy.deepcopy(source_dict)
    for key, val in function_dict.iteritems():
        base_dict[key] = val
    return type(name, superclasses, base_dict)


def GridMstar(world, goals, recursive, inflation=1.0, mode='OD',
              end_time=10 ** 15, col_set_memory=False, connect_8=False,
              rotations=True):
    """Generates an Mstar instance for planning on a grid

    world         - obstacle map,  matrix with 0 for free,  1
                     for obstacle
    goals          - ((x1, y1), (x2, y2), ...) coordinates of the
                     goal, should be tuples
    recursive      - True for rM*, false for basic M*
    sub_search     - Sub planners, should be None for the full
                     configuration space
    rob_id         - maps local robot identity to full
                     configuration space identity,  should be
                     None for the full configuration space
                     instance
    inflation      - how much the metric should be inflated by
    mode           - in ['OD', 'EPEA'] what base graph search to use
    end_time       - when the search should be terminated
    col_set_memory - remember previous step collision set,
                     intended to provide more efficient cached
                     path utillization.  False by default
    connect_8      - 8 connected grid
    rotations      - If True, permit rotations (i.e. robots moving into
                     a position that was just vacated)

    returns:
    Mstar instance intended for planning on a grid graph
    """
    source_dict = {'OD': OD_MSTAR_DICT, 'EPEA': EPEMSTAR_DICT}[mode]
    policy_cls = workspace_graph.Astar_Policy
    if connect_8:
        def policy_func(world, goal):
            return policy_cls(
                world, lambda x: workspace_graph.GridGraphConn8WaitAtGoal(
                    x, goal, diagonal_cost=True, wait_cost=0.0),
                goal=goal,
                compute_heuristic=
                workspace_graph.compute_heuristic_conn_8_diagonal)
    else:
        def policy_func(world, goal):
            return policy_cls(
                world, lambda x: workspace_graph.GridGraphConn4WaitAtGoal(
                    x, goal, wait_cost=0.0), goal=goal,
                compute_heuristic=workspace_graph.compute_heuristic_conn_4)
    if rotations:
        col_checker = workspace_graph.Edge_Checker()
    else:
        col_checker = workspace_graph.NoRotationChecker()
    cls = mstar_class_factory(
        'GridMstar',
        {'gen_policy_planners': gen_policy_planner_factory(policy_func),
         '_default_col_checker': col_checker},
        (object, ), source_dict=source_dict)
    return cls(world, goals, recursive, inflation=inflation,
               end_time=end_time, col_set_memory=col_set_memory)

ODMstar = mstar_class_factory('ODMstar', {}, (object, ),
                              source_dict=OD_MSTAR_DICT)
EPEMstar = mstar_class_factory('EPEMstar', {}, (object, ),
                               source_dict=EPEMSTAR_DICT)


class ODNode(object):
    """Holds the data needed for a single node in operator decomposition

    coordinates are ODCoord instances
    """

    __slots__ = ['free', 'coord', 'updated', 'open', 'closed',
                 'h', 'cost', 'back_ptr', 'back_prop_set', 'col_set',
                 'recursive', 'forwards_ptr', 'prev_col_set', 'depth']

    def __init__(self, coord, free, recursive, back_ptr=None,
                 forwards_ptr=None):
        """Constructor for mstar_node

        Assumes the col_set is empty by default

        coord         - tuple giving coordinates,  may store partial
                        moves if not standard node
        free          - part of the free configuration space
        back_ptr      - pointer to best node to get to self
        forwards_ptr  - pointer along the best path to the goal
        """
        self.free = free
        self.coord = coord
        self.updated = -1
        # Whether already in the open list
        self.open = False

        # Whether this has been expanded.  Note that a node can be added
        # back to the open list after it has been expanded,  but will
        # still be marked as closed.  It cannot have its cost changed,
        # but it can add neighbors, but not be added as a neighbor
        self.closed = False
        # Heuristic cost to go,  None to ensure it will be properly
        # calculated
        self.h = None
        # Cost to reach
        self.cost = MAX_COST

        # Optimal way to reach this node.  Point to self to indicate the
        # initial position
        self.back_ptr = back_ptr
        self.back_prop_set = []  # Ways found to reach this node
        self.col_set = ()
        # store the collision set of back_ptr when the path from
        # back_ptr to self was first found.  Used for hopefully more
        # efficient cached path access
        self.prev_col_set = ()
        self.recursive = recursive

        # Keeps track of solutions that have already been found,
        # replace forwards_tree.  Denote the goal node by pointing
        # forwards_ptr
        # to itself
        self.forwards_ptr = forwards_ptr

    def reset(self, t):
        """Resets if t > last update time"""
        if t > self.updated:
            self.updated = t
            self.open = False
            self.closed = False
            self.cost = MAX_COST
            self.back_ptr = None
            self.back_prop_set = []

    def get_path(self):
        """Gets the path passing through path to the goal,  assumes that
        self is either the goal node,  or a node connected to the goal
        node through forwards_pointers
        """
        path = self.backtrack_path()
        return self.forwards_extend_path(path)

    def backtrack_path(self, path=None, prev=None):
        """Finds the path that leads up to this node,  updating
        forwards_ptr so that we can recover this path quickly,  only
        returns standard nodes

        path - current reconstructed path for use in recusion, must
               start as None
        prev - pointer to the last node visited by backtrack_path, used
               to update forwards_ptr to record the best paths to the
               goal
        """
        if path is None:
            path = []
        if prev is not None:
            self.forwards_ptr = prev
            if isinstance(self.h, tuple):
                # Needed for constrained od_mstar,  and don't feel like
                # coming up with a better solution for now
                self.h = (prev.h[0] + prev.cost[0] - self.cost[0], self.h[1])
            else:
                self.h = prev.h + (prev.cost - self.cost)
        if not self.coord.move_list:
            assert self.coord not in path
            path.insert(0, self.coord)
        if self.back_ptr == self:
            # Done so that it cannot terminate on a node that wasn't
            # properly initialized
            return path
        return self.back_ptr.backtrack_path(path, self)

    def forwards_extend_path(self, path):
        """Extends the path from self to the goal node,  following
        forwards pointers,  only includes standard nodes

        path - current path to extend towards the goal, as list of joint
               configuration space coordinates
        """
        if self.forwards_ptr == self:
            return path
        # if next is standard node
        if not self.forwards_ptr.coord.move_list:
            path.append(self.forwards_ptr.coord)
        return self.forwards_ptr.forwards_extend_path(path)

    def add_col_set(self, c):
        """Adds the contents of c to self.col_set.

        c - collision set to add to the current node's collision set

        returns:

        True if modifications were made, else False
        """
        if len(c) == 0:
            return False
        if self.recursive:
            temp = add_col_set_recursive(c, self.col_set)
        else:
            temp = add_col_set(c, self.col_set)
        modified = (temp != self.col_set)
        if modified:
            self.col_set = temp
            return True
        return False

    def back_prop_col_set(self, new_col, open_list):
        """Propagates the collision dependencies found by its children
        to the parent,  which adds any new dependencies to this col_set

        new_col   - the new collision set to add
        open_list - the open list to which nodes with changed collisoin
                    sets are added,  assumed to be a SortedCollection
        """
        further = self.add_col_set(new_col)
        if further:
            if not self.open:
                # assert self.closed
                self.open = True

                # Inserting to the left of any node with the same key
                # value,  to encourage exploring closer to the collison
                open_list.insert(self)
            for j in self.back_prop_set:
                j.back_prop_col_set(self.col_set, open_list)

    def get_step(self):
        """Returns the coordinate of the next standard node in the path,

        returns:

        None if no such thing
        """
        if self.forwards_ptr is None:
            return None
        if not self.forwards_ptr.coord.move_list:
            return self.forwards_ptr.coord
        else:
            return self.forwards_ptr.get_step()


class EPEANode(ODNode):
    """Holds the data needed for a single node in enhanced partial
    expansion A*
    """

    def __init__(self, coord, free, recursive, back_ptr=None,
                 forwards_ptr=None):
        """Assumes initial offset is 0

        Assumes the col_set is empty by default

        coord         - tuple giving coordinates,  may store partial
                        moves if not standard node
        free          - part of the free configuration space
        back_ptr      - pointer to best node to get to self
        forwards_ptr  - pointer along the best path to the goal
        """
        super(EPEANode, self).__init__(coord, free, recursive,
                                       back_ptr=back_ptr,
                                       forwards_ptr=forwards_ptr)
        self.offset = 0
        # Needs to be determined by the planner with access to the
        # individual policies
        self.max_offset = None

    def reset(self, t):
        """Resets if t > last update time"""
        if t > self.updated:
            self.updated = t
            self.open = False
            self.closed = False
            self.cost = MAX_COST
            self.back_ptr = None
            self.back_prop_set = []
            self.offset = 0

    def back_prop_col_set(self, new_col, open_list):
        """Propagates the collision dependencies found by its children
        to the parent,  which adds any new dependencies to this col_set

        new_col   - the new collision set to add
        open_list - the open list to which nodes with changed collisoin
                    sets are added,  assumed to be a SortedCollection
        """
        further = self.add_col_set(new_col)
        if further:
            if not self.open:
                # assert self.closed
                self.open = True

                # Inserting to the left of any node with the same key
                # value,  to encourage exploring closer to the collison
                open_list.insert(self)
            elif self.offset != 0:
                # Need to reset the offset and reinsert to allow a path
                # to be found even if the node is already in the open
                # list
                self.offset = 0
                # Inserting to the left of any node with the same key
                # value, to encourage exploring closer to the collison
                open_list.insert(self)
            for j in self.back_prop_set:
                j.back_prop_col_set(self.col_set, open_list)

    def backtrack_path(self, path=None, prev=None):
        """Finds the path that leads up to this node,  updating
        forwards_ptr so that we can recover this path quickly,  only
        returns standard nodes

        path - current reconstructed path for use in recusion, must
               start as None
        prev - pointer to the last node visited by backtrack_path, used
               to update forwards_ptr to record the best paths to the
               goal
        """
        if path is None:
            path = []
        if prev is not None:
            self.forwards_ptr = prev
            if isinstance(self.h, tuple):
                # Needed for constrained od_mstar,  and don't feel like
                # coming up with a better solution for now
                self.h = (prev.h[0] + prev.cost[0] - self.cost[0], self.h[1])
            else:
                self.h = prev.h + (prev.cost - self.cost)
        path.insert(0, self.coord)
        if self.back_ptr == self:
            # Done so that it cannot terminate on a node that wasn't
            # properly initialized
            return path
        return self.back_ptr.backtrack_path(path, self)

    def forwards_extend_path(self, path):
        """Extends the path from self to the goal node,  following
        forwards pointers,  only includes standard nodes

        path - current path to extend towards the goal, as list of joint
               configuration space coordinates
        """
        if self.forwards_ptr == self:
            return path
        # if next is standard node
        path.append(self.forwards_ptr.coord)
        return self.forwards_ptr.forwards_extend_path(path)

    def get_step(self):
        """Returns the coordinate of the next standard node in the path,

        returns:

        None if no such thing
        """
        if self.forwards_ptr is None:
            return None
        return self.forwards_ptr.coord
