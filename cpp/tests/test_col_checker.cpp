#include "gtest/gtest.h"
#include <iostream>
#include <utility>
#include <vector>

#include "mstar_type_defs.hpp"
#include "col_checker.hpp"

namespace{

  class ODColCheckerTest : public ::testing::Test{
  };

  // helper function to compare collision sets, as order of elements
  // don't matter
  template<class T> bool compare_col_set(T c1, T c2){
    if (c1.size() != c2.size()){
      return false;
    }
    return std::is_permutation(c1.begin(), c1.end(), c2.begin());
  }

  TEST_F(ODColCheckerTest, TestSimpleGraphChecker){
    mstar::SimpleGraphODColCheck checker;
    ASSERT_TRUE(compare_col_set(
		  checker.check_edge(mstar::OdCoord({0, 1}, {}),
				     mstar::OdCoord({1, 2}, {}),
				     {0, 1}),
		  mstar::ColSet({})));
    ASSERT_TRUE(compare_col_set(
		  checker.check_edge(mstar::OdCoord({0, 1}, {}),
				     mstar::OdCoord({1, 0}, {}),
				     {0, 1}),
		  mstar::ColSet({{0, 1}})));
    ASSERT_TRUE(compare_col_set(
		  checker.check_edge(mstar::OdCoord({0, 2}, {}),
				     mstar::OdCoord({1, 1}, {}),
				     {0, 1}),
		  mstar::ColSet({{0, 1}})));
    ASSERT_TRUE(compare_col_set(
		  checker.check_edge(mstar::OdCoord({0, 2, 4, 5}, {}),
				     mstar::OdCoord({1, 1, 5, 6}, {}),
				     {0, 1, 2, 3}),
		  mstar::ColSet({{0, 1}})));
    ASSERT_TRUE(compare_col_set(
		  checker.check_edge(mstar::OdCoord({0, 2, 4, 5}, {}),
				     mstar::OdCoord({1, 1, 5, 4}, {}),
				     {0, 1, 2, 3}),
		  mstar::ColSet({{0, 1}, {2, 3}})));
    ASSERT_TRUE(compare_col_set(
		  checker.check_edge(mstar::OdCoord({0, 2, 4, 5}, {}),
				     mstar::OdCoord({1, 2, 5, 4}, {}),
				     {0, 1, 2, 3}),
		  mstar::ColSet({{2, 3}})));
  }
}
