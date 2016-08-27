#include "gtest/gtest.h"
#include <iostream>
#include <utility>
#include <vector>

#include "mstar_type_defs.hpp"
#include "policy.hpp"
#include "grid_policy.hpp"

// Anonymous namespaces effectively limit scope to the current file
namespace {
  class PolicyTest : public ::testing::Test{
  protected:
    mstar::Graph g;

    PolicyTest(){
      // construct a simple graph
      typedef std::pair<int,int> E;
      const int num_nodes = 5;
      std::vector<E> ed = { E(0, 0), E(0, 1),
		    E(1, 2),
		    E(2, 0), E(2, 1), E(2, 3),
		    E(3, 4),
		    E(4, 2)};
      std::vector<int> weights = {0, 1, 1, 1, 1, 1, 1, 1};
      g = mstar::Graph(ed.begin(), ed.end(), weights.begin(),
		       num_nodes);
    }
  };

  TEST_F(PolicyTest, TestDistance){
    mstar::Policy p(g, 0);
    ASSERT_EQ(p.get_cost(0), 0.);
    ASSERT_EQ(p.get_cost(1), 2.);
    ASSERT_EQ(p.get_cost(2), 1.);
    ASSERT_EQ(p.get_cost(3), 3.);
    ASSERT_EQ(p.get_cost(4), 2);
  }

  TEST_F(PolicyTest, TestEdgeCost){
    mstar::Policy p(g, 0);
    ASSERT_EQ(p.get_edge_cost(0, 0), 0.);
    ASSERT_EQ(p.get_edge_cost(0, 1), 1.);
  }

  TEST_F(PolicyTest, TestOutNeighbors){
    mstar::Policy p(g, 0);
    std::vector<int> val = {0, 1};
    ASSERT_EQ(p.get_out_neighbors(0), val);
    val = {2};
    ASSERT_EQ(p.get_out_neighbors(1), val);
    val = {0, 1, 3};
    ASSERT_EQ(p.get_out_neighbors(2), val);    
  }

  TEST_F(PolicyTest, TestGetStep){
    mstar::Policy p(g, 0);
    ASSERT_EQ(0, p.get_step(0));
    ASSERT_EQ(2, p.get_step(1));
    ASSERT_EQ(0, p.get_step(2));
    ASSERT_EQ(4, p.get_step(3));
    ASSERT_EQ(2, p.get_step(4));
  }

  TEST_F(PolicyTest, TestGetOffsetNeighbors){
    mstar::EPEMstar_Policy p(g, 0);
    auto offsets = p.get_offset_neighbors(2);
    ASSERT_EQ(offsets.size(), 3);
    std::vector<int> val = {0};
    ASSERT_EQ(offsets[0.], val);
    val = {1};
    ASSERT_EQ(offsets[2.], val);
    val = {3};
    ASSERT_EQ(offsets[3.], val);
  }

  class GridPolicyTest: public ::testing::Test{};

  TEST_F(GridPolicyTest, TestGridPolicy){
    mstar::Policy p = mstar::grid_policy({{0, 0}, {0, 0}}, {0, 0});

    ASSERT_EQ(0., p.get_cost(0));
    ASSERT_EQ(1., p.get_cost(1));
    ASSERT_EQ(1., p.get_cost(2));
    ASSERT_EQ(2., p.get_cost(3));

    p = mstar::grid_policy({{0, 1, 0}, {0, 0, 0}}, {0, 0});
    ASSERT_EQ(0., p.get_cost(0));
    ASSERT_EQ(1, p.get_cost(3));
    ASSERT_EQ(2, p.get_cost(4));
    ASSERT_EQ(3, p.get_cost(5));
    ASSERT_EQ(4., p.get_cost(2));

    ASSERT_EQ(0, p.get_step(0));
    ASSERT_EQ(0, p.get_step(3));
    ASSERT_EQ(3, p.get_step(4));
    ASSERT_EQ(4, p.get_step(5));
    ASSERT_EQ(5, p.get_step(2));

    p = mstar::grid_policy({{0, 1, 0}, {0, 0, 0}}, {0, 2});
    ASSERT_EQ(4., p.get_cost(0));
    ASSERT_EQ(3, p.get_cost(3));
    ASSERT_EQ(2, p.get_cost(4));
    ASSERT_EQ(1, p.get_cost(5));
    ASSERT_EQ(0., p.get_cost(2));
  }
}
