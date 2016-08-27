from libcpp cimport bool
from libcpp.vector cimport vector
from libcpp.pair cimport pair

from col_set_addition import OutOfTimeError, NoSolutionError

cdef extern from "grid_planning.hpp" namespace "mstar":
    vector[vector[pair[int, int]]] find_grid_path(
        const vector[vector[bool]] &obstacles,
        const vector[pair[int, int]] &init_pos,
        const vector[pair[int, int]] &goals,
        double inflation, int time_limit) except +

def find_path(world, init_pos, goals, inflation, time_limit):
    """Finds a path invoking C++ implementation

    Uses recursive ODrM* to explore a 4 connected grid

    world - matrix specifying obstacles, 1 for obstacle, 0 for free
    init_pos  - [[x, y], ...] specifying start position for each robot
    goals     - [[x, y], ...] specifying goal position for each robot
    inflation - inflation factor for heuristic
    time_limit - time until failure in seconds

    returns:
    [[[x1, y1], ...], [[x2, y2], ...], ...] path in the joint
    configuration space

    raises:
    NoSolutionError if problem has no solution
    OutOfTimeError if the planner ran out of time
    """
    # convert to boolean.  For some reason coercion doesn't seem to
    # work properly
    cdef vector[vector[bool]] obs
    cdef vector[bool] temp
    for row in world:
        temp = vector[bool]()
        for i in row:
            temp.push_back(i == 1)
        obs.push_back(temp)
    try:
        return find_grid_path(obs, init_pos, goals, inflation, time_limit)
    except Exception as e:
        if e.message == "Out of Time":
            raise OutOfTimeError()
        elif e.message == "No Solution":
            raise NoSolutionError()
        else:
            raise e
