#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/reverse_graph.hpp>
#include <algorithm>

#include "policy.hpp"

using namespace mstar;


Policy::Policy(const Graph &g, const RobCoord goal){
  g_ = g;
  costs_ = std::vector<double>(num_vertices(g_));
  predecessors_.resize(boost::num_vertices(g_));

  boost::dijkstra_shortest_paths(
    boost::make_reverse_graph(g_), goal,
    boost::predecessor_map(&predecessors_[0]).distance_map(&costs_[0]));
  edge_weight_map_ = boost::get(boost::edge_weight_t(), g_);
}


double Policy::get_cost(RobCoord coord){
  return costs_[coord];
}


double Policy::get_edge_cost(RobCoord u, RobCoord v){
  // boost::edge returns pair<edge_descriptor, bool>
  return boost::get(edge_weight_map_, boost::edge(u, v, g_).first);
}


std::vector<RobCoord> Policy::get_out_neighbors(RobCoord coord){
  std::vector<RobCoord> out;
  for (auto adj_verts = boost::adjacent_vertices(coord, g_);
       adj_verts.first != adj_verts.second; adj_verts.first++){
    out.push_back(*(adj_verts.first));
  }
  return out;
}

RobCoord Policy::get_step(RobCoord coord){
  return predecessors_[coord];
}


Offsets EPEMstar_Policy::get_offset_neighbors(RobCoord coord){
  
  if (offset_neighbors_[coord].size() != 0){
    return offset_neighbors_[coord];
  }
  // offsets from this position have not been generated
  std::vector<RobCoord> neibs = get_out_neighbors(coord);
  if (neibs.size() == 0){
    return offset_neighbors_[coord];
  }
  std::vector<double> neib_costs;
  for (uint i = 0; i < neibs.size(); i++){
    neib_costs.push_back(get_cost(neibs[i]) +
			 get_edge_cost(coord, neibs[i]));
  }
  double min_cost = *min_element(neib_costs.begin(), neib_costs.end());
  for (uint i = 0; i < neibs.size(); i++){
    offset_neighbors_[coord][neib_costs[i] - min_cost].push_back(
      neibs[i]);
  }
  return offset_neighbors_[coord];
}
    
