'''Implementation of Od - Mstar that obeys stated constraints'''

import od_mstar
import cbs
import workspace_graph
import sys
import constrained_planner
import hashlib
import ipdb
MAX_COST = workspace_graph.MAX_COST

from od_mstar import POSITION, MOVE_TUPLE

class Constrained_Od_Mstar(od_mstar.Od_Mstar):
    '''M* and rM* using operator decomposition instead of basic
    M* as the base computation. Will obey arbitrary low level constraints,
    which are provided by calls to cbs.Constrained_Planner

    currently only implemented for 4 and 8 connected grids, but should be
    fairly simple to modify for use on arbitrary graphs a la prms'''
    def __init__(self,obs_map,heuristic_conf,goals,constraints,
                 recursive=True,sub_search=None,out_paths = None,
                 col_checker=None,inflation=1,end_time=10**15,
                 conn_8=False,astar=False,full_space=False, epeastar=False,
                 offset_increment=1.0):
        '''
        obs_map     - obstacle map, matrix with 0 for free, 1 for obstacle
        goals       - ((x1,y1),(x2,y2),...) coordinates of the goal, should be 
                       tuples
        constraints - cbs_constraints for the group of robots for which to plan
                       all robots must be included
        heuristic_conf - ((x1,y1),(x2,y2),...) initial position to be used
                          for the heuristic for the constrained policy 
                          generation code
        recursive   - True for rM*, false for basic M*
        sub_search  - Sub planners, should be None for the full configuration 
                       space
        out_paths   - [[[x1,y1],[x2,y2],..],[[x12,y12],[x22,y22],..],...] paths
                      of out of group robots to avoid
        col_checker - object to handle robot-robot collision checking.  Should
                       implement the same interface as 
                       workspace_graph.Edge_Checker
        rob_id      - maps local robot identity to full configuration space 
                       identity, should be None for the full configuration 
                       space instance
        inflation   - how much the metric should be inflated by
        end_time    - when the search should be terminated
        conn_8      - True for 8 connected graph, False for 4 connected graph
        astar       - use basic A* instead of operator decomposition
        full_space  - whether to perform a full configuration space search
        flood_fill_policy - compute policy with flood fill instead of 
                             resumable A*
        epeastar    - Use EPEA* instead of A* or OD to search the graph, default
                      False
        offset_increment  - amount to increment offsets by in EPEA* search

        '''
        self.out_paths = out_paths
        #Track max_t (maximum time in the graph) seperately from the maximum
        #time in a constraint, so we can handle out_paths correctly
        self.max_t = 0
        self.con_max_t = 0
        self.constraints = constraints
        if self.constraints != None:
            self.max_t = max(cbs.con_get_max_time(constraints) + 1, 0)
            if self.max_t == 1:
                # only possible if the constraint occurs at time 0,
                # which is meaningless
                self.max_t = 0
            self.con_max_t = self.max_t
        if self.out_paths != None:
            self.max_t = max(self.max_t, len(self.out_paths))
        self.rob_id = cbs.con_get_robots(self.constraints)
        self.heuristic_conf = heuristic_conf
        #Need to have goals in space_time for gen_policy_planner, but need to
        #pass in space-only goals to Od_Mstar
        self.tgoals = tuple(tuple(i) + (self.max_t, ) for i in goals)
        #Store a copy of goals without transforming into space-time, to be
        #used in generating sub planners
        self.non_time_goals = goals
        self.conn_8 = conn_8
        self.path_hash = hashlib.md5(str(self.out_paths)).hexdigest()
        #Need to delay calling the super constructor until all of the new
        #data fields are filled for use by gen_policy_planenrs
        od_mstar.Od_Mstar.__init__(
            self, obs_map, goals, recursive, sub_search=sub_search,
            col_checker=col_checker, rob_id=self.rob_id, inflation =inflation,
            end_time=end_time, connect_8=conn_8, astar=astar,
            full_space=full_space, epeastar=epeastar,
            offset_increment=offset_increment)
        self.open_list_key = lambda x:(-x.cost[0]-x.h[0]*self.inflation,
                                        -x.cost[1]-x.h[1])
        if self.epeastar:
            self.open_list_key = lambda x:(-x.cost[0]-x.h[0]*self.inflation-
                                            x.offset,  -x.cost[1]-x.h[1])
        #Now want to replace self.goals with 
        self.goals = self.tgoals

    def gen_policy_planners(self,sub_search,obs_map,goals):
        '''Creates the sub-planners and necessary policy keys.  This is
        because pretty much every sub-planner I've made requires adjusting
        the graph used to create the policies and passing around dummy
        sub_searches

        side effects to generate self.sub_search and self.policy_keys'''
        goals = self.tgoals
        self.policy_keys= tuple((cbs.con_subset_robots(self.constraints,(i,)),
                                 self.path_hash) for i in  self.rob_id)
        self.sub_search = sub_search
        if self.sub_search == None:
            self.sub_search = {}
        for dex,key in enumerate(self.policy_keys):
            if not key in self.sub_search:
                self.sub_search[key] = constrained_planner.Constrained_Planner(
                    obs_map, self.heuristic_conf[dex],self.goals[dex],key[0],
                    out_paths = self.out_paths,sub_search=self.sub_search,
                    conn_8=self.conn_8,inflation= self.inflation)


    def solution_condition(self,node):
        '''boolean function determining whether nod eis an acceptable 
        solution'''
        if node.forwards_ptr == node:
            #Have already found a path from here
            return True
        if not node.standard_node or node.coord[0][-1] < self.con_max_t:
            return False
        for bot in xrange(len(node.coord)):
            if (node.coord[bot][0] != self.goals[bot][0] or 
                node.coord[bot][1] != self.goals[bot][1]):
                return False
        #Note that we may never have generated this node a-priori, so need to
        #reset its forwards-pointer to be compatible with forwards_extend_path
        node.forwards_ptr = node
        return True

    def heuristic(self,coord,standard_node):
        '''Returns the heuristic value, takes in a time value to allow for     
        time dependent heuristics'''
        cost = 0
        out_col = 0
        if standard_node:
            for dex,key in enumerate(self.policy_keys):
                temp = self.sub_search[key].get_h_cost(coord[dex])
                cost+=temp[0]
                out_col+=temp[1]
            return (cost,out_col)
        else:
            #Compute heuristic properly for robots that have moved
            for dex,key in enumerate(self.policy_keys[:len(coord[1])]):
                temp = self.sub_search[key].get_h_cost(coord[1][dex])
                cost+=temp[0]
                out_col+=temp[1]
            #Handle robots that have not moved
            for dex,key in enumerate(self.policy_keys[len(coord[1]):]):
                dex+= len(coord[1])
                temp = self.sub_search[key].get_h_cost(coord[0][dex])
                cost+=temp[0]
                out_col+=temp[1]
        return (cost,out_col)

    def get_node(self,coord,standard_node=True):
        '''Returns the node at coord ((x1,y1),(x2,y2),...), at timestep t'''
        node = od_mstar.Od_Mstar.get_node(self,coord,
                                              standard_node=standard_node)
        if not isinstance(node.cost,tuple):
            node.cost = (MAX_COST,MAX_COST)
        return node

    def gen_init_nodes(self,init_pos):
        '''Generates the first node at which search starts.  Needed for 
        allowing changes in the structure of the cost function'''
        first_nodes = od_mstar.Od_Mstar.gen_init_nodes(self,init_pos)
        for node in first_nodes:
            node.cost = (0,0)
        return first_nodes

    def create_sub_search(self,new_goals,new_init,new_constraints):
        '''Creates a new instance of a subsearch for recursive search
        new_goals - ((x,y),(x,y),...) new goal configuration
        new_constraints - cbs constraint describing new robot subset'''
        return Constrained_Od_Mstar(
            self.obs_map,new_init,new_goals,new_constraints,self.recursive,
            sub_search=self.sub_search,out_paths=self.out_paths,
            col_checker=self.col_checker,inflation=self.inflation, 
            end_time=self.end_time,conn_8=self.conn_8,astar=self.astar,
            epeastar=self.epeastar,offset_increment=self.offset_increment)

    def get_subplanner_keys(self,col_set):
        '''Takes in a collision set.  Creates any new subplanners required,
        and returns the keys that are used to store them in self.sub_search
        col_set - collision set to be solved'''
        #Convert the collision sets into the global indicies, and convert to 
        #tuples.  Assumes self.rob_id is sorted
        global_col = map(lambda y:tuple(map(lambda x:self.rob_id[x],y)),
                         col_set)
        sub_keys = [(cbs.con_subset_robots(self.constraints,g),self.path_hash) 
                    for g in global_col]
        #generate the sub planners, if necessary
        for dex,key in enumerate(sub_keys):
            if not key in self.sub_search:
                t_goals = tuple([self.non_time_goals[k] for k in col_set[dex]])
                t_init = tuple([self.heuristic_conf[k] for k in col_set[dex]])
                self.sub_search[key] = self.create_sub_search(
                    t_goals,t_init,key[0])
        return sub_keys

    def epeastar_transition_cost(self,start_coord,prev_cost,new_coord):
        '''Computes the cost of a new node at the specified coordinates, 
        starting from the given position and cost

        '''
        cost = prev_cost[0]
        cols = prev_cost[1]
        t = new_coord[0][-1]
        for i in xrange(len(start_coord)):
            if not (start_coord[i][:-1]== self.goals[i][:-1] and 
                    start_coord[i][:-1] == new_coord[i][:-1]):
                    cost+=1
        if self.out_paths != None:
            for dex in range(len(new_coord)):
                if self.col_checker.single_bot_outpath_check(
                    new_coord[dex],start_coord[dex],t,self.out_paths):
                    cols+=1
        return (cost,cols)

    def od_mstar_transition_cost(self,start_coord,prev_cost,neighbor,rob_dex):
        '''Computes the transition cost for a single robot in od_mstar
        neighbor generation
        
        start_coord - base position of robots (prior to move assignment)
        prev_cost   - cost of base node
        neighbor    - proposed move assignmetn
        rob_dex     - robot move is assigned to'''
        cost = prev_cost[0]
        if not (start_coord[rob_dex][0] == self.goals[rob_dex][0] and 
                start_coord[rob_dex][1] == self.goals[rob_dex][1] and 
                neighbor[0] == self.goals[rob_dex][0] and
                neighbor[1] == self.goals[rob_dex][1]):
            cost+=od_mstar.PER_ROBOT_COST
        cols = prev_cost[1]
        if self.out_paths != None:
            if self.col_checker.single_bot_outpath_check(
                neighbor,start_coord[rob_dex],neighbor[-1],self.out_paths):
                cols+=1
        return (cost,cols)

    def od_rmstar_transition_cost(self,start_coord,prev_cost,new_coord):
        '''Computes the transition cost for a single robot in rmstar
        neighbor generation

        USED BY BOTH EPErM* and ODrM*
        
        start_coord - base position of robots (prior to move assignment)
        prev_cost   - cost of base node
        new_coord    - proposed move assignmetn'''
        cost = prev_cost[0]
        cols = prev_cost[1]
        t = new_coord[0][-1]
        for i in xrange(len(start_coord)):
            if not (start_coord[i][0]== self.goals[i][0] and 
                    start_coord[i][1]== self.goals[i][1] and 
                    start_coord[i][0] == new_coord[i][0] and
                    start_coord[i][1] == new_coord[i][1]):
                    cost+=1
        if self.out_paths != None:
            for dex in range(len(new_coord)):
                if self.col_checker.single_bot_outpath_check(
                    new_coord[dex],start_coord[dex],t,self.out_paths):
                    cols+=1
        return (cost,cols)

    def find_path(self,init_pos,time_limit=5*60):
        '''Finds a path from the specified configuration in space-time to the 
        goal
        init_pos - ((x,y,t),(x,y,t),...) space time initial configuration
        time_limit - maximum time for planning'''
        init_pos = tuple(tuple(i) for i in init_pos)
        path = od_mstar.Od_Mstar.find_path(self,init_pos,
                                               time_limit=time_limit)
        return tuple(tuple((bot_pos[:-1]) for bot_pos in coord) 
                     for coord in path)

    def find_path_time_pad(self,init_pos,time_limit=60*5):
        '''Finds a path from the given intial position to the goal.  Assumes
        the initial time is 0
        init_pos = ((x,y),(x,y),...)
        time_limit - maximum planning time'''
        init_pos = tuple( (coord[0],coord[1],0) for coord in init_pos)
        return self.find_path(init_pos,time_limit=time_limit)

    def get_path_cost(self):
        '''returns the cost of the best path found to the goal'''
        return self.graph[(self.goals)].cost[0]

def find_path(obs_map, init_pos, goals, constraints, recursive=True,
              inflation=1.0, time_limit=5 * 60.0, astar=False, get_obj=False,
              conn_8=False, full_space=False, flood_fill_policy=False,
              out_paths=None, epeastar=False, sum_of_costs=False):
    if sum_of_costs:
        o = SumOfCosts_Constrained_OD_rMstar(
            obs_map, init_pos, goals, constraints, recursive=recursive,
            inflation=inflation, astar=astar, conn_8=conn_8, epeastar=epeastar,
            full_space=full_space, out_paths=out_paths)
    else:
        o = Constrained_Od_Mstar(obs_map, init_pos, goals, constraints,
                                 recursive=recursive, inflation=inflation,
                                 astar=astar, conn_8=conn_8, epeastar=epeastar,
                                 full_space=full_space, out_paths=out_paths)
    #Need to make sure that the recursion limit is great enough to actually
    #construct the path
    longest = max([o.sub_search[o.policy_keys[i]].get_cost(
        (init_pos[i][0],init_pos[i][1],0)) 
                   for i in xrange(len(init_pos))])
    #Guess that the longest path will not be any longer than 5 times the 
    #longest individual robot path
    sys.setrecursionlimit(max(sys.getrecursionlimit(),longest*5*len(init_pos)))
    path = o.find_path_time_pad(init_pos,time_limit=time_limit)
    if get_obj:
        return path, o
    return path, o.get_path_cost()


class SumOfCostsNode(od_mstar.mstar_node):

    def __init__(self, coord, free, recursive, standard_node, back_ptr=None,
                 forwards_ptr=None):

        # Track the current time.  This differs from the time coordinate
        # of the coords, as that is capped to account for equivalence
        # classes in time (i.e. when the environment has ceased changing
        self.time = 0
        # Tracks the most recent time that each robot arrived at its,
        # goal, used to compute the cost of the node.
        
        # I apparently had the brilliant idea that standard and
        # intermediate nodes should have completely different coordinate
        # formats.  Stupid me.
        if standard_node:
            self.at_goal = [0 for i in xrange(len(coord))]
        else:
            self.at_goal = [0 for i in xrange(len(coord[POSITION]))]
        super(SumOfCostsNode, self).__init__(
            coord, free, recursive, standard_node, back_ptr=back_ptr,
            forwards_ptr=forwards_ptr)


class SumOfCosts_Constrained_OD_rMstar(Constrained_Od_Mstar):
    '''Variant of Constrained_Od_Mstar using the sum of costs heuristic

    sum of cost heuristic penalyzes the total time until the robot
    reaches its goal for the final time and waits there until the end of
    the plan

    I am doing some really, really ugly things here, as this is a dirty
    hack intended to get a paper out the door in really short time.
    IF THIS EVER BECOMES IMPORTANT, REDESIGN!
    '''

    def gen_policy_planners(self, sub_search, obs_map, goals):
        '''Creates the individual policies and associated reference keys

        sub_search - dictionary where individual policies will be stored
        obs_map    - matrix defining world occupancy grid.
                     0 is free, 1 is obstacle
        goals      - Here only to match the signature of
                     Od_Mstar.gen_policy_planners
        '''
        goals = self.tgoals
        self.policy_keys= tuple((cbs.con_subset_robots(self.constraints,(i,)),
                                 self.path_hash) for i in  self.rob_id)
        self.sub_search = sub_search
        if self.sub_search == None:
            self.sub_search = {}
        for dex,key in enumerate(self.policy_keys):
            if not key in self.sub_search:
                self.sub_search[key] = (
                    constrained_planner.SumOfCosts_Constrained_Planner(
                        obs_map, self.heuristic_conf[dex], self.goals[dex],
                        key[0], out_paths=self.out_paths,
                        sub_search=self.sub_search, conn_8=self.conn_8,
                        inflation=self.inflation))

    def get_node(self, coord, standard_node=True):
        '''Returns the node at coord ((x1,y1),(x2,y2),...), at time t'''
        if coord in self.graph:
            # Node already exists.  reset if necessary
            t_node = self.graph[coord]
            t_node.reset(self.updated)
            if not isinstance(t_node.cost, tuple):
                t_node.cost = (MAX_COST,MAX_COST)
            return t_node
        # Need to instantiate the node
        if standard_node:
            col = self.col_checker.col_check(coord, self.recursive)
        else:
            # Only check for collisions between robots whose move has
            # been determined
            col = self.col_checker.col_check(coord[MOVE_TUPLE], self.recursive)
        free = (len(col) == 0)
        t_node = SumOfCostsNode(coord, free, self.recursive, standard_node)
        # Cache the resultant col_set
        t_node.col_set = col
        t_node.updated = self.updated
        t_node.h = self.heuristic(coord, standard_node)
        t_node.cost = (MAX_COST,MAX_COST)
        
        # Add the node to the graph
        self.graph[coord] = t_node
        return t_node

    def create_sub_search(self,new_goals, new_init, new_constraints):
        '''Creates a new instance of a subsearch for recursive search
        new_goals       - ((x,y),(x,y),...) new goal configuration
        new_constraints - cbs constraint describing new robot subset'''
        return SumOfCosts_Constrained_OD_rMstar(
            self.obs_map, new_init, new_goals, new_constraints, self.recursive,
            sub_search=self.sub_search, out_paths=self.out_paths,
            col_checker=self.col_checker, inflation=self.inflation, 
            end_time=self.end_time, conn_8=self.conn_8, astar=self.astar,
            epeastar=self.epeastar, offset_increment=self.offset_increment)


    def epeastar_transition_cost(self, start_coord, prev_cost, new_coord):
        '''Computes the cost of a new node at the specified coordinates, 
        starting from the given position and cost

        '''
        start_node = self.get_node(start_coord)
        new_node = self.get_node(new_coord)
        cost = 0
        cols = prev_cost[1]
        t = new_coord[0][-1]
        # differs from t in that its not capped by when the environment
        # stops shifting
        full_time = start_node.time + 1
        at_goals = []
        for i in xrange(len(start_coord)):
            if (start_coord[i][:-1]== self.goals[i][:-1] and 
                start_coord[i][:-1] == new_coord[i][:-1]):
                # Have remained at the goal
                cost += start_node.at_goal[i]
                at_goals.append(start_node.at_goal[i])
            else:
                at_goals.append(full_time)
                cost += full_time
        if self.out_paths != None:
            for dex in range(len(new_coord)):
                if self.col_checker.single_bot_outpath_check(
                    new_coord[dex],start_coord[dex],t,self.out_paths):
                    cols += 1
        assert len(at_goals) == len(start_node.at_goal)
        # Update sum of costs related structure in new coord, if
        # appropriate
        if (cost, cols) < new_node.cost:
            new_node.at_goal = tuple(at_goals)
            new_node.time = full_time
        return (cost,cols)


    def od_rmstar_transition_cost(self,start_coord,prev_cost,new_coord):
        '''Computes the transition cost for a single robot in rmstar
        neighbor generation

        USED BY BOTH EPErM* and ODrM*
        
        start_coord - base position of robots (prior to move assignment)
        prev_cost   - cost of base node
        new_coord    - proposed move assignmetn'''
        start_node = self.get_node(start_coord)
        new_node = self.get_node(new_coord)
        cost = 0
        cols = prev_cost[1]
        t = new_coord[0][-1]
        # differs from t in that its not capped by when the environment
        # stops shifting
        full_time = start_node.time + 1
        at_goals = []
        for i in xrange(len(start_coord)):
            if (start_coord[i][:2] == self.goals[i][:2] and 
                new_coord[i][:2] == self.goals[i][:2]):
                cost += start_node.at_goal[i]
                at_goals.append(start_node.at_goal[i])
            else:
                at_goals.append(full_time)
                cost += full_time
        if self.out_paths != None:
            for dex in range(len(new_coord)):
                if self.col_checker.single_bot_outpath_check(
                    new_coord[dex],start_coord[dex],t,self.out_paths):
                    cols+=1
        # Update sum of costs related structure in new coord, if
        # appropriate
        if (cost, cols) < new_node.cost:
            new_node.at_goal = tuple(at_goals)
            new_node.time = full_time
        return (cost,cols)
