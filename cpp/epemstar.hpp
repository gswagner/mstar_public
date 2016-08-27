#ifndef MSTAR_EPEA_MSTAR_H
#define MSTAR_EPEA_MSTAR_H

#include <unordered_map>
#include <functional>
#include <queue>
#include <memory>
#include <exception>

#include "mstar_type_defs.hpp"
#include "col_set.hpp"
#include "epea_vertex.hpp"
#include "col_checker.hpp"
#include "policy.hpp"
#include "common.hpp"

namespace mstar{

  class EPEMstar {    
  public:
    /**
     * Constructs a new, top level M* planner
     *
     * @param policies pointer to vector of policies.
     *                 EPEMstar does not take ownership
     * @param goals goal configuration of entire system
     * @param inflation inflation factor
     * @param end_time time at which M* will declare failure
     * @param increment how much to increment offset
     * @param checker collision checking object
     */
    EPEMstar(
      std::vector<std::shared_ptr<EPEMstar_Policy>> policies,
      Coord goals, double inflation, time_point end_time, double increment,
      std::shared_ptr<ColChecker> col_checker);

    /**
     * Creates a subplanner for a subsest of the robots
     *
     * robots is a collision set element in the frame of parent, not global
     * robot ids
     */
    EPEMstar(const ColSetElement &robots, EPEMstar &parent);

    ~EPEMstar();

    /**
     * Computes the optimal path to the goal from init_pos
     *
     * @param init_pos coordinate of the initial joint configuration
     *
     * @return the path in the joint configuration graph to the goal
     *
     * @throws OutOfTimeError ran out of planning time
     * @throws NoSolutionError no path to goal from init_pos
     */
    Path find_path(Coord init_pos);

  private:

    /*
     * Sort in decreasing order to give cheap access to the cheapest
     * elements
     */
    typedef std::priority_queue<EpeaVertex*, std::vector<EpeaVertex*>,
				greater_cost<EpeaVertex>> OpenList;

    /**TODO: fix
     * This is kind of horrifying, but I cannot store the EPEMstar objects
     * directly in the unordered map, as I get ungodly errors that look
     * like they come from an allocator.  Adding copy constructor and
     * assignment operator doesn't work, so its something involved about
     * STL.  Think this works, but annoying
     */
    std::unordered_map<ColSetElement, std::shared_ptr<EPEMstar>> *subplanners_;
    std::vector<std::shared_ptr<EPEMstar_Policy>> policies_;
    // ids of the robots this planner handles.  Assumed to be in ascending
    // order
    std::vector<int> ids_;
    Coord goals_;
    // holds the nodes in the joint configuration space
    std::unordered_map<Coord, EpeaVertex> graph_;
    time_point end_time_; // When planning will be halted
    double inflation_; // inflation factor for heuristic
    int planning_iter_; // current planning iteration
    int num_bots_;
    std::shared_ptr<ColChecker> col_checker_;
    bool top_level_; // tracks if the top level planner
    double increment_; // how much to increment the offset

    EPEMstar(const EPEMstar &that) = delete;

    /**
     * Resets planning for a new planning iteration.
     *
     * Does not reset forwards_ptrs, as those should be valid across
     * iterations
     */
    void reset();

    /**
     * Computes the heuristic value of a vertex at a given coordinate
     *
     * @param coord coordinate for which to compute a heuristic value
     *
     * @return the (inflated) heuristic value
     */
    double heuristic(const Coord &coord);

    /**
     * Returns a reference to the vertex at a given coordinate
     *
     * this->graph retains ownership of the vertex.  Will create the vertex
     * if it does not already exist.
     *
     * @param coord coordinate of the desired vertex
     *
     * @return pointer to the vertex at coord.
     */
    EpeaVertex* get_vertex(const Coord &coord);

    /**
     * Returns the optimal next step from init_pos
     *
     * Will compute the full path if necessary, but preferentially uses
     * cached results in forwards_ptrs.  Expected to only be called from
     * a standard coordinate, and to only return a standard coordinate
     *
     * @param init_pos coordinate to compute the optimal next step from
     *
     * @returns the coordinate of the optimal next step towards the goal
     */
    Coord get_step(const Coord &init_pos);

    /**
     * Generates the neighbors of vertex and add them to the open list
     *
     * @param vertex EpeaVertex to expand
     * @param open_list the sorted open list being used
     */
    void expand(EpeaVertex *vertex, OpenList &open_list);

    /**
     * Returns the limited neighbors of coord using recursive calculation
     *
     * @param vertex vertex to generate neighbors thereof
     * @param col_set collision set of vertex to generate neighbors
     *
     * @return list of limited neighbors
     */
    std::vector<Coord> get_neighbors(
      EpeaVertex *vertex, const ColSet &col_set);

    /**
     * Returns the limited neighbors of vertex using non-recursive computation
     *
     * Called when the collision set contains all of the robots, as a base
     * case for get_neighbors, thus always generate all possible neighbors
     *
     * @param vertex vertex to generate neighbors from
     *
     * @return list of limited neighbors
     */
     std::vector<Coord> get_all_neighbors(
       EpeaVertex *vertex);

    /**
     * Returns the cost of traversing a given edge
     *
     * @param source coordinate of the source vertex
     * @param target coordinate of the target vertex
     *
     * @return the cost of the edge
     */
    double edge_cost(const Coord &source, const Coord &target);

    /**
     * Returns the path through a vertex
     *
     * Assumes that back_ptr and forwards_ptr are set and non-none at vert
     * Identifies each end of the path by looking for a back_ptr/forwards_ptr
     * pointed at the holder
     *
     * @param vert the vertex to trace a path through
     *
     * @return the path passing through vert containing only standard vertices
     */
    Path trace_path(EpeaVertex *vert);

    /**
     * Generates the path to the specified vertex
     *
     * Sets forward_ptrs to cache the path, and updates the heuristic
     * values of the vertices on the path so we can end the moment a
     * vertex on a cached path is expanded.
     *
     * TODO: double check that making the heuristic inconsistent in this
     * fashion is OK.
     *
     * @param vert the vertex to trace the path to
     * @param successor the successor of vert on the path
     * @param path place to construct path
     */
    void back_trace_path(EpeaVertex *vert, EpeaVertex *successor, Path &path);

    /**
     * Genertes the path from the specified vertex to the goal
     *
     * Non-trivial only if vert lies on a previously cached path
     *
     * @param vert the vertex to trace the path from
     * @param path place to construct path
     */
    void forwards_trace_path(EpeaVertex *vert, Path &path);

    /**
     * Backpropagates collision set information to all predecessors of a
     * vertex.
     *
     * Adds vertices whose collision set changes back to the open list
     *
     * @param vertex pointer to the vertex to back propagate from
     * @param col_set the collision set that triggered backpropagation
     * @param open_list the current open list
     */
    void back_prop_col_set(EpeaVertex *vert, const ColSet &col_set,
			   OpenList &open_list);
  };  
};

#endif
