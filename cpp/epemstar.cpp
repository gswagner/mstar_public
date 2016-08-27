#include <chrono>
#include <cassert>
#include <algorithm>
#include <iostream>

#include "epemstar.hpp"

using namespace mstar;

EPEMstar::EPEMstar(std::vector<std::shared_ptr<EPEMstar_Policy>> policies,
		   Coord goals, double inflation, time_point end_time,
		   double increment,
		   std::shared_ptr<ColChecker> col_checker){
  subplanners_ = new std::unordered_map<ColSetElement,
					std::shared_ptr<EPEMstar>>();
  policies_ = policies;
  // top-level planner, so construct a set of all robot ids
  for (int i = 0; i < (int) goals.size(); ++i){
    ids_.push_back(i);
  }
  goals_ = goals;
  end_time_ = end_time;
  inflation_ = inflation;
  planning_iter_ = 0;
  num_bots_ = (int) ids_.size();
  col_checker_ = col_checker;
  top_level_ = true;
  increment_ = increment;
}

EPEMstar::EPEMstar(const ColSetElement &robots, EPEMstar &parent){
  subplanners_ = parent.subplanners_;
  policies_ = parent.policies_;
  for (int i: robots){
    ids_.push_back(parent.ids_[i]);
    goals_.push_back(parent.goals_[i]);
  }
  end_time_ = parent.end_time_;
  inflation_ = parent.inflation_;
  planning_iter_ = 0;
  num_bots_ = (int) ids_.size();
  col_checker_ = parent.col_checker_;
  top_level_ = false;
  increment_ = parent.increment_;
}

EPEMstar::~EPEMstar(){
  if (top_level_){
    delete subplanners_;
  }
}

Path EPEMstar::find_path(Coord init_pos){
  reset();

  // Configure the initial vertex
  // identified by setting the back_ptr to itself
  EpeaVertex *first = get_vertex(init_pos);
  first->back_ptr = first;
  first->cost = 0;
  first->open = true;

  EPEMstar::OpenList open_list;
  open_list.push(first);

  while (open_list.size() > 0){
    if (std::chrono::system_clock::now() > end_time_){
      throw OutOfTimeError();
    }

    EpeaVertex *vert = open_list.top();
    open_list.pop();
    vert->open = false;
    if (vert->closed){
      continue;
    }

    // check if this is the goal vertex
    if (std::equal(vert->coord.cbegin(), vert->coord.cend(),
		   goals_.cbegin())){
      vert->forwards_ptr = vert;
    }
    if (vert->forwards_ptr != nullptr){
      // Either the goal or on a previous found path to the goal
      return trace_path(vert);
    }

    expand(vert, open_list);
  }
  throw NoSolutionError();
}

void EPEMstar::reset(){
  planning_iter_++;
}

double EPEMstar::heuristic(const Coord &coord){
  // Heuristic is computed from the assigned move for elements of the
  // move tuple, and from the base coordinate for all others
  double h = 0;
  for (uint i = 0; i < coord.size(); i++){
    h += policies_[ids_[i]]->get_cost(coord[i]);
  }
  return h * inflation_;
}

EpeaVertex* EPEMstar::get_vertex(const Coord &coord){
  // returns a pair with the first element an interator to a <key, vertex>
  // pair and the second to a bool which is true if there was not a
  // preexisting value
  auto p = graph_.emplace(coord, coord);
  p.first->second.reset(planning_iter_);
  if (p.second){
    // new vertex, so need to set heuristic
    p.first->second.h = heuristic(coord);
    // set maximum offset so know when to terminate
    for (uint i = 0; i < coord.size(); i++){
      Offsets offs = policies_[ids_[i]]->get_offset_neighbors(coord[i]);
      if (offs.size() > 0){
	p.first->second.max_offset +=(*offs.rbegin()).first;
      }
    }
  }
  return &p.first->second;
}

Coord get_vertex_step(EpeaVertex * vert){
  assert(vert != nullptr);
  return vert->forwards_ptr->coord;
}

Coord EPEMstar::get_step(const Coord &init_pos){
  EpeaVertex* vert = EPEMstar::get_vertex(init_pos);
  if (vert->forwards_ptr != nullptr){
    return get_vertex_step(vert);
  }
  find_path(init_pos);
  return get_vertex_step(vert);
}

void EPEMstar::expand(EpeaVertex *vertex, EPEMstar::OpenList &open_list){
  vertex->closed = true;
  ColSet gen_set = col_set_to_expand(vertex->col_set, vertex->gen_set);
  bool recursive_expansion = true;
  if (gen_set.size() == 1 && (int) gen_set[0].size() == num_bots_){
    // the generating collision set contains all robots, so no caching
    // would be possible.  Therefore, don't use
    gen_set = vertex->col_set;
    recursive_expansion = false;
  }

  std::vector<Coord> neighbors = get_neighbors(vertex, gen_set);

  // accumulates the collision sets that occur while trying to move to
  // any of the neighbors
  ColSet col_set;
  for (Coord &new_coord: neighbors){
    ColSet new_col = col_checker_->check_edge(vertex->coord, new_coord, ids_);
    if (!new_col.empty()){
      // State not accessible due to collisions
      add_col_set_in_place(new_col, col_set);
      continue;
    }
    
    EpeaVertex *new_vert = get_vertex(new_coord);
    new_vert->back_prop_set.insert(vertex);
    // Always need to at the collision set of any vertex we can reach
    // to its successors, as otherwise we would need to wait for another
    // robot to collide downstream before triggering back propagation
    add_col_set_in_place(new_vert->col_set, col_set);

    if (new_vert->closed){
      continue;
    }

    double new_cost = vertex->cost + edge_cost(vertex->coord, new_coord);
    if (new_cost >= new_vert->cost){
      continue;
    }
    new_vert->cost = new_cost;
    new_vert->back_ptr = vertex;
    new_vert->open = true;
    new_vert->gen_set = gen_set;
    open_list.push(new_vert);
  }
  back_prop_col_set(vertex, col_set, open_list);
  if ( ! vertex->open && ! recursive_expansion){
    vertex->prev_offset = vertex->offset;
    vertex->offset += increment_;
    if (vertex->offset <= vertex->max_offset){
      vertex->open = true;
      vertex->closed = false;
      open_list.push(vertex);
    }
  }
}

std::vector<Coord> EPEMstar::get_neighbors(EpeaVertex *vertex,
					   const ColSet &col_set){
  const Coord &coord = vertex->coord;
  // If the collision set contains all robots, invoke the non-recursive
  // base case
  if (col_set.size() == 1 && (int) col_set[0].size() == num_bots_){
    return get_all_neighbors(vertex);
  }
  
  // Generate the step along the joint policy
  Coord policy_step;
  for (int i = 0; i < num_bots_; i++){
    policy_step.push_back(policies_[ids_[i]]->get_step(coord[i]));
  }

  // Iterate over colliding sets of robots, and integrate the results
  // of the sub planning for each set
  for (const ColSetElement &elem: col_set){
    // The collision set contains the local ids (relative to the robots in
    // this subplanner) of the robots in collision
    // To properly index child subplanners, need to convert to global robot
    // ids, so that the subplanners will be properly globally accessible
    ColSetElement global_col;
    for (auto &local_id: elem){
      global_col.insert(ids_[local_id]);
    }
    // Get, and if necessary construct, the appropriate subplanner.
    // returns a pair <p, bool> where bool is true if a new subplanner
    // was generated, and p is an iterator to a pair <key, val>
    if (subplanners_->find(global_col) == subplanners_->end()){
      subplanners_->insert(
	{global_col, std::shared_ptr<EPEMstar>(new EPEMstar(elem, *this))});
    }
    EPEMstar *planner = subplanners_->at(global_col).get();
    // create the query point
    Coord new_base;
    for (const int &i: elem){
      new_base.push_back(coord[i]);
    }

    Coord step;
    try{
      step = planner->get_step(new_base);
    } catch(NoSolutionError &e){
      // no solution for that subset of robots, so return no neighbors
      // only likely to be relevant on directed graphs
      return {};
    }

    int elem_dex = 0;
    // now need to copy into the relevant positions in policy_step
    for (auto i: elem){
      policy_step[i] = step[elem_dex];
      ++elem_dex; // could play with post appending, but don't want to
    }
  }
  return {policy_step};
}

struct MoveCandidate{
  double cost = 0;
  Coord partial_move = {};

  bool operator <(const MoveCandidate &other) const{
    return cost < other.cost;
  }
};

std::vector<Coord> EPEMstar::get_all_neighbors(EpeaVertex *vertex){
  const Coord &coord = vertex->coord;
  std::vector<Offsets> moves ;
  for (uint i = 0; i < coord.size(); i++){
    moves.push_back(policies_[ids_[i]]->get_offset_neighbors(
		      coord[i]));
  }

  MoveCandidate base;
  std::vector<MoveCandidate> partial_neibs = {base};
  for (uint i = 0; i < coord.size(); i++){
    std::vector<MoveCandidate> new_partials;
    for (auto &candidate: partial_neibs){
      for (auto &delta_moves: moves[i]){
	if (delta_moves.first + candidate.cost > vertex->offset){
	  break;
	}
	for (RobCoord &move: delta_moves.second){
	  MoveCandidate new_candidate;
	  new_candidate.cost = candidate.cost + delta_moves.first;
	  new_candidate.partial_move = candidate.partial_move;
	  new_candidate.partial_move.push_back(move);

	  new_partials.push_back(new_candidate);
	}
      }
    }
    std::swap(partial_neibs, new_partials);
  }
  std::vector<Coord> out;
  for (auto &candidate: partial_neibs){
    if (candidate.cost <= vertex->offset &&
	candidate.cost > vertex->prev_offset)
    out.push_back(candidate.partial_move);
  }
    
  return out;
}

double EPEMstar::edge_cost(const Coord &source, const Coord &target){
  // transition between standard vertex, so all robots are assigned moves and
  // incur costs
  double cost = 0;
  for (int i = 0; i < num_bots_; ++i){
    cost += policies_[ids_[i]]->get_edge_cost(source[i],
					      target[i]);
  }
  return cost;
}

Path EPEMstar::trace_path(EpeaVertex *vert){
  Path path;
  back_trace_path(vert, vert->forwards_ptr, path);
  forwards_trace_path(vert, path);
  return path;
}

void EPEMstar::back_trace_path(EpeaVertex *vert, EpeaVertex *successor,
			      Path &path){
  vert->forwards_ptr = successor;
  // check if this is the final, terminal state, which is not required
  // to have a zero-cost self loop, so could get problems
  if (vert != successor){
    vert->h = successor->h + edge_cost(vert->coord, successor->coord);
  } else{
    vert->h = 0;
  }
  path.insert(path.begin(), vert->coord);
  if (vert->back_ptr != vert){
    back_trace_path(vert->back_ptr, vert, path);
  }
}

void EPEMstar::forwards_trace_path(EpeaVertex *vert, Path &path){
  if (vert->forwards_ptr != vert){
    path.push_back(vert->forwards_ptr->coord);
    forwards_trace_path(vert->forwards_ptr, path);
  }
}

void EPEMstar::back_prop_col_set(EpeaVertex *vert, const ColSet &col_set,
				EPEMstar::OpenList &open_list){
  bool further = add_col_set_in_place(col_set, vert->col_set);
  if (further){
    vert->closed = false;
    // At the very least need to reset prev_offset.  Not sure if resetting
    // offset itself is worth it.  Probably not
    vert->offset = 0;
    vert->prev_offset = std::numeric_limits<double>::lowest();
    vert->open = true;
    // need to reinsert to handle new offsets
    open_list.push(vert);

    for(EpeaVertex *predecessor: vert->back_prop_set){
      back_prop_col_set(predecessor, vert->col_set, open_list);
    }
  }
}
