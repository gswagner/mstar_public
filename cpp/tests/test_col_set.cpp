#include "gtest/gtest.h"
#include <iostream>
#include <utility>
#include <vector>

#include "mstar_type_defs.hpp"
#include "col_set.hpp"

namespace {
  class ColSetTest : public ::testing::Test{
  };


  TEST_F(ColSetTest, TestIsDisjoint){
    ASSERT_TRUE(mstar::is_disjoint(mstar::ColSetElement({0, 1}),
				   mstar::ColSetElement({2, 3})));
    ASSERT_TRUE(mstar::is_disjoint(mstar::ColSetElement({0, 1}),
				   mstar::ColSetElement({})));
    ASSERT_TRUE(mstar::is_disjoint(mstar::ColSetElement({0, 1, 6, 8}),
				   mstar::ColSetElement({2, 7})));

    ASSERT_FALSE(mstar::is_disjoint(mstar::ColSetElement({0, 1}),
				    mstar::ColSetElement({0, 3})));
    ASSERT_FALSE(mstar::is_disjoint(mstar::ColSetElement({0, 1}),
				    mstar::ColSetElement({1, 3})));
    ASSERT_FALSE(mstar::is_disjoint(mstar::ColSetElement({0, 1, 5, 87,100}),
				    mstar::ColSetElement({40, 87, 38})));
  }


  TEST_F(ColSetTest, TestIsSuperset){
    ASSERT_TRUE(mstar::is_superset(mstar::ColSetElement({0, 1, 2}),
				   mstar::ColSetElement({0, 1})));
    ASSERT_TRUE(mstar::is_superset(mstar::ColSetElement({0, 1, 2}),
				   mstar::ColSetElement({0, 1, 2})));
    ASSERT_TRUE(mstar::is_superset(mstar::ColSetElement({0, 1, 2}),
				   mstar::ColSetElement({0})));
    ASSERT_TRUE(mstar::is_superset(mstar::ColSetElement({0, 1, 2}),
				   mstar::ColSetElement({2})));

    ASSERT_FALSE(mstar::is_superset(mstar::ColSetElement({0, 1, 2}),
				   mstar::ColSetElement({3})));
    ASSERT_FALSE(mstar::is_superset(mstar::ColSetElement({1, 2}),
				    mstar::ColSetElement({0, 3})));
    ASSERT_FALSE(mstar::is_superset(mstar::ColSetElement({1, 5}),
				    mstar::ColSetElement({0, 3})));
    ASSERT_FALSE(
      mstar::is_superset(
	mstar::ColSetElement({3, 5, 7, 8, 23, 568, 2645}),
	mstar::ColSetElement({4, 6, 34, 567, 3452, 457465, 565675})));
  }


  TEST_F(ColSetTest, TestMerge){
    ASSERT_EQ(mstar::merge(mstar::ColSetElement({0, 1}),
			   mstar::ColSetElement({1, 2})),
		 mstar::ColSetElement({0, 1, 2}));
    ASSERT_EQ(mstar::merge(mstar::ColSetElement({0, 1, 5, 6}),
			   mstar::ColSetElement({1, 2})),
	      mstar::ColSetElement({0, 1, 2, 5, 6}));
  }

  TEST_F(ColSetTest, TestSetMerge){
    ASSERT_EQ(mstar::merge(mstar::ColSetElement({0, 1}),
			   mstar::ColSetElement({1, 2})),
	      mstar::ColSetElement({0, 1, 2}));
    ASSERT_EQ(mstar::merge(mstar::ColSetElement({0, 1, 5, 6}),
			   mstar::ColSetElement({1, 2})),
	      mstar::ColSetElement({0, 1, 2, 5, 6}));
  }

  // helper function to compare collision sets, as order of elements
  // don't matter
  template<class T> bool compare_col_set(T c1, T c2){
    if (c1.size() != c2.size()){
      return false;
    }
    return std::is_permutation(c1.begin(), c1.end(), c2.begin());
  }

  TEST_F(ColSetTest, TestSetAddColSet){
    ASSERT_TRUE(
      compare_col_set(
	mstar::add_col_set(std::vector<mstar::ColSetElement>({{0, 1}}),
			   std::vector<mstar::ColSetElement>({})),
		  std::vector<mstar::ColSetElement>({{0, 1}})));
    ASSERT_TRUE(
      compare_col_set(
	mstar::add_col_set(std::vector<mstar::ColSetElement>({{0, 1}}),
			   std::vector<mstar::ColSetElement>({{1, 2}})),
	std::vector<mstar::ColSetElement>({{0, 1, 2}})));
    ASSERT_TRUE(
      compare_col_set(
	mstar::add_col_set(std::vector<mstar::ColSetElement>({{1, 2}}),
			   std::vector<mstar::ColSetElement>({{0, 1}})),
	std::vector<mstar::ColSetElement>({{0, 1, 2}})));
    ASSERT_TRUE(
      compare_col_set(
	mstar::add_col_set(std::vector<mstar::ColSetElement>({{0, 1}}),
			   std::vector<mstar::ColSetElement>({{2, 3}})),
	std::vector<mstar::ColSetElement>({{0, 1}, {2, 3}})));
    ASSERT_TRUE(
      compare_col_set(
	mstar::add_col_set(
	  std::vector<mstar::ColSetElement>({{0, 1}, {3, 4}}),
	  std::vector<mstar::ColSetElement>({{2, 3}})),
	std::vector<mstar::ColSetElement>({{0, 1}, {2, 3, 4}})));
    ASSERT_TRUE(
      compare_col_set(
	mstar::add_col_set(
	  std::vector<mstar::ColSetElement>({{0, 3}, {2, 4}}),
	  std::vector<mstar::ColSetElement>({{2, 3}})),
	std::vector<mstar::ColSetElement>({{0, 2, 3, 4}})));
    ASSERT_TRUE(
      compare_col_set(
	mstar::add_col_set(
	  std::vector<mstar::ColSetElement>({{0, 3}, {1, 4}}),
	  std::vector<mstar::ColSetElement>({{2, 3}})),
	std::vector<mstar::ColSetElement>({{0, 2, 3}, {1, 4}})));
    ASSERT_TRUE(
      compare_col_set(
	mstar::add_col_set(
	  std::vector<mstar::ColSetElement>({{0, 1}, {3, 4}}),
	  std::vector<mstar::ColSetElement>({{2, 5}, {4, 8}})),
	std::vector<mstar::ColSetElement>({{0, 1}, {2, 5}, {3, 4, 8}})));
    ASSERT_TRUE(
      compare_col_set(
	mstar::add_col_set(
	  std::vector<mstar::ColSetElement>({{0, 1}, {3, 4}, {6, 7}}),
	  std::vector<mstar::ColSetElement>({{2, 5}, {4, 8}})),
	std::vector<mstar::ColSetElement>(
	  {{0, 1}, {2, 5}, {3, 4, 8}, {6, 7}})));
    // tests the comparison function
    ASSERT_FALSE(
      compare_col_set(
	mstar::add_col_set(std::vector<mstar::ColSetElement>({{0, 1}}),
			   std::vector<mstar::ColSetElement>({})),
	std::vector<mstar::ColSetElement>({{0, 2}})));
  }

  TEST_F(ColSetTest, TestAddColSetInPlace){
    std::vector<mstar::ColSetElement> input({});
    ASSERT_TRUE(mstar::add_col_set_in_place(
		  std::vector<mstar::ColSetElement>({{0, 1}}),
		  input));
    ASSERT_TRUE(
      compare_col_set(input, std::vector<mstar::ColSetElement>({{0, 1}})));
    ASSERT_FALSE(mstar::add_col_set_in_place(
		  std::vector<mstar::ColSetElement>({{0, 1}}),
		  input));
    ASSERT_TRUE(
      compare_col_set(input, std::vector<mstar::ColSetElement>({{0, 1}})));

    ASSERT_TRUE(mstar::add_col_set_in_place(
		  std::vector<mstar::ColSetElement>({{1, 2}}),
		  input));
    ASSERT_FALSE(mstar::add_col_set_in_place(
		  std::vector<mstar::ColSetElement>({{1, 2}}),
		  input));
    ASSERT_TRUE(
      compare_col_set(input, std::vector<mstar::ColSetElement>({{0, 1, 2}})));

    ASSERT_TRUE(mstar::add_col_set_in_place(
		  std::vector<mstar::ColSetElement>({{0, 5}, {1, 6}}), input));
    ASSERT_TRUE(
      compare_col_set(input,
		      std::vector<mstar::ColSetElement>({{0, 1, 2, 5, 6}})));
  }

  TEST_F(ColSetTest, TestColSetToExpand){
    ASSERT_TRUE(
      compare_col_set(
	mstar::col_set_to_expand(mstar::ColSet({}), mstar::ColSet({})),
	mstar::ColSet({})));
    ASSERT_TRUE(
      compare_col_set(
	mstar::col_set_to_expand(mstar::ColSet({}), mstar::ColSet({{1, 2}})),
	mstar::ColSet({{1, 2}})));
    ASSERT_TRUE(
      compare_col_set(
	mstar::col_set_to_expand(mstar::ColSet({{1, 2}}),
				 mstar::ColSet({{1, 2, 3}})),
	mstar::ColSet({{1, 2, 3}})));
    ASSERT_TRUE(
      compare_col_set(
	mstar::col_set_to_expand(mstar::ColSet({{1, 2}, {3, 4}}),
				 mstar::ColSet({{1, 2, 3}})),
	mstar::ColSet({{1, 2}, {3, 4}})));
    ASSERT_TRUE(
      compare_col_set(
	mstar::col_set_to_expand(mstar::ColSet({{1, 2}, {3, 4}}),
				 mstar::ColSet({{1, 2, 3}, {4, 5}})),
	mstar::ColSet({{1, 2}, {3, 4}})));
    ASSERT_TRUE(
      compare_col_set(
	mstar::col_set_to_expand(mstar::ColSet({{1, 2}, {4, 6}}),
				 mstar::ColSet({{1, 2, 3}, {4, 5, 6}})),
	mstar::ColSet({{1, 2, 3}, {4, 5, 6}})));
    ASSERT_TRUE(
      compare_col_set(
	mstar::col_set_to_expand(mstar::ColSet({{4, 5}, {1, 6}}),
				 mstar::ColSet({{1, 2, 3}, {4, 5, 6}})),
	mstar::ColSet({{4, 5}, {1, 6}})));
    ASSERT_TRUE(
      compare_col_set(
	mstar::col_set_to_expand(mstar::ColSet({{4, 5}, {8, 9}, {10, 11}}),
				 mstar::ColSet({{1, 2, 3}, {4, 5, 6},
							   {9, 10}})),
	mstar::ColSet({{1, 2, 3}, {4, 5, 6}, {8, 9}, {10, 11}})));
  }
}
