import interface
from col_set_addition import NoSolutionError

class KRobustPolicy(interface.Policy_Interface):
    """"Converts a policy for use with 'robust' planning

    All single robot planning can be done by just tacking history
    onto the action dictated by the individual policy.  Assumes cost
    is only incurred by the actual motion

    The purpose of robust planning is to ensure that no robot
    can pass through a vertex visited by another robot within
    k timesteps

    The state of the agent is now specified as a k-step history of
    previous positions (q1, .., qk), with q1 being the most
    recent position

    all coordinates should be k-tuples, or else backwards planning
    for policy computation won't work correctly
    """

    def __init__(self, policy, k):
        """
        policy - the policy for the single robot
        k      - number of time steps to use for robustness criteria
        """
        self._policy = policy
        self._k = k

    def get_cost(self, config):
        """Returns the cost of reaching the goal from config

        coord1 - (q1, ..., qk) start vertex

        returns:
        cost for the robot to reach the goal
        """
        return self._policy.get_cost(config[0])

    def get_edge_cost(self, coord1, coord2):
        """Returns edge_cost of going from coord1 to coord2

        only incur cost when a transition first occurs, i.e.
        at q1

        coord1 - (q1, ..., qk) source vertex
        coord2 - (q1, ..., qk) target vertex of edge
        """
        return self._policy.get_edge_cost(coord1[0], coord2[0])

    def get_step(self, config):
        """Returns an optimal successor of config

        coord1 - (q1, ..., qk) start vertex

        returns: (q1, ...., qk) defining an optimal neighbor
        """
        action = self._policy.get_step(config[0])
        return (action, ) + config[:(self._k - 1)]

    def get_neighbors(self, coord):
        """Returns the collision free neighbors of the specified coord

        coord - (q1, ..., qk) source vertex

        returns: list of successors of coord in (q1, ..., qk) form
        """
        targets = self._policy.get_neighbors(coord[0])
        return [(t, ) + coord[:(self._k - 1)] for t in targets]

    def get_graph_size(self, correct_for_size=True):
        self._policy.get_graph_size(correct_for_size=correct_for_size)

    def get_limited_offset_neighbors(self, coord, max_offset, min_offset=0):
        """Returns set of neighbors specified by the offsets

        More specifically, returns the set of neighbors for which the
        maximum difference in path cost if passed through is less than
        the specified value.

        (i.e. if you are forced to pass through coordinate x, instead of
        the optimal step, what is the difference in cost)?

        coord - coordinates of the node to find neighbors of
        max_offset - the maximum increase in path cost to encur in
                     choice of neighbors
        min_offset - minimum increae in path cost to encur in a neighbor

        returns:
        a list of tuples of the form (offset, coordinate)
        """
        action_pairs = self._policy.get_limited_offset_neighbors(
            coord[0], max_offset, min_offset=min_offset)
        return [(off, (t, ) + coord[:(self._k - 1)])
                for off, t in action_pairs]

    def get_offset_neighbors(self, coord, offset):
        """Generates offset neighbors for node specified by coord

        If no offset neighbors exist, they are created

        Only offset neighbors at a certain offset are returned

        coord - (q1, ..., qk) source vertex
        offset - value of offset determing which neighbors are
                 included in return value

        returns:
        list of tuples of form (offset, neighbor)
        """
        action_pairs = self._policy.get_offset_neighbors(coord[0], offset)
        return [(off, (t, ) + coord[:(self._k - 1)])
                for off, t in action_pairs]

    def get_offsets(self, config):
        """Returns the list of possible offsets of the given vertex"""
        return self._policy.get_offsets(config[0])


T = -1
    
class BackTimeRobustPolicy(interface.Policy_Interface):

    """"Converts a policy for use with 'robust' planning

    Plans backwards in time.  The goal of the policy is the initial
    configuration of the full problem, so step describes how to get
    there backwards in time
    """

    def __init__(self, policy, k):
        """
        note that the goal of policy must be the initial position of the
        robot in the full problem
        
        policy - the policy for the single robot ignoring time
        k      - number of time steps to use for robustness criteria
        """
        self._policy = policy
        self._k = k

    def get_cost(self, config, max_t):
        """Returns the cost of reaching the goal from config

        coord1 - ((q1, t), ..., (qk, t - (k - 1)) start vertex
        max_t  - maximum time.  Here just for compatibility

        returns:
        cost for the robot to reach the goal
        """
        return self._policy.get_cost(config[0][:T])

    # def get_edge_cost(self, coord1, coord2):
    #     """Returns edge_cost of going from coord1 to coord2

    #     only incur cost when a transition first occurs, i.e.
    #     at q1

    #     Direction is relative to forwards time

    #     coord1 - ((q1, t), ..., (qk, t - (k - 1)) source vertex
    #     coord2 - ((q1, t), ..., (qk, t - (k - 1)) target vertex of edge
    #     """
    #     return self._policy.get_edge_cost(coord1[0][:T], coord2[0][:T])

    def get_step(self, config, max_t):
        """Returns an optimal successor of config

        coord1 - ((q1, t), ..., (qk, t - (k - 1)) start vertex
        max_t  - maximum t

        returns: ((q0, t + 1), ...., (qk-1, t - (k - 2))
                   defining an optimal neighbor
        """
        t = config[-1][T]
        # Should never be called with t = 0 unless that's the maximum
        # time
        assert(t > 0 or max_t == 0)
        action = self._policy.get_step(config[-1][:T])
        return (config[1:self._k], (action, max(t - 1, max_t)))

    def get_neighbors(self, coord, max_t):
        """Returns the collision free neighbors of the specified coord
        backwards in time

        i.e. return the forwards-time predecessors of coord

        coord1 - ((q1, t), ..., (qk, t - (k - 1)) start vertex

        returns: list of successors of coord in (q1, ..., qk) form
        """
        if coord[-1][T] == 0 and max_t > 0:
            # Earliest possible time, so no 
            return []
        targets = self._policy.get_neighbors(coord[0])
        return [(t, ) + coord[:(self._k - 1)] for t in targets]

    def get_graph_size(self, correct_for_size=True):
        self._policy.get_graph_size(correct_for_size=correct_for_size)

    def get_limited_offset_neighbors(self, coord, max_offset, min_offset=0):
        """Returns set of neighbors specified by the offsets

        More specifically, returns the set of neighbors for which the
        maximum difference in path cost if passed through is less than
        the specified value.

        (i.e. if you are forced to pass through coordinate x, instead of
        the optimal step, what is the difference in cost)?

        coord - coordinates of the node to find neighbors of
        max_offset - the maximum increase in path cost to encur in
                     choice of neighbors
        min_offset - minimum increae in path cost to encur in a neighbor

        returns:
        a list of tuples of the form (offset, coordinate)
        """
        action_pairs = self._policy.get_limited_offset_neighbors(
            coord[0], max_offset, min_offset=min_offset)
        return [(off, (t, ) + coord[:(self._k - 1)])
                for off, t in action_pairs]

    def get_offset_neighbors(self, coord, offset):
        """Generates offset neighbors for node specified by coord

        If no offset neighbors exist, they are created

        Only offset neighbors at a certain offset are returned

        coord - (q1, ..., qk) source vertex
        offset - value of offset determing which neighbors are
                 included in return value

        returns:
        list of tuples of form (offset, neighbor)
        """
        action_pairs = self._policy.get_offset_neighbors(coord[0], offset)
        return [(off, (t, ) + coord[:(self._k - 1)])
                for off, t in action_pairs]

    def get_offsets(self, config):
        """Returns the list of possible offsets of the given vertex"""
        return self._policy.get_offsets(config[0])
