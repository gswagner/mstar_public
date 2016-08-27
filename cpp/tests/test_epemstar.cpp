#include "gtest/gtest.h"
#include <iostream>
#include <utility>
#include <vector>
#include <chrono>

#include "mstar_type_defs.hpp"
#include "epemstar.hpp"
#include "grid_policy.hpp"
#include "mstar_utils.hpp"
#include "grid_planning.hpp"

// Anonymous namespaces effectively limit scope to the current file
namespace {

  using namespace mstar;

  class EPEMstarTest : public ::testing::Test{
  };

  TEST_F(EPEMstarTest, TestConstructorDestructor){
    // simply checks if we can build and destroy the object successfully
    // TODO: Reject nullptrs
    EPEMstar m({std::shared_ptr<EPEMstar_Policy>(nullptr)},
	       {0, 1}, 1., std::chrono::system_clock::now(), 1.,
	       std::shared_ptr<ColChecker>(nullptr));
  }

  TEST_F(EPEMstarTest, TestSingleRobotPath){
    time_point t = std::chrono::system_clock::now();
    t += Clock::duration(std::chrono::seconds(10));
    EPEMstar m(
      {std::shared_ptr<EPEMstar_Policy>(
	  epem_grid_policy_ptr({{0, 0}, {0, 0}}, {0, 0}))},
      {0}, 1., t, 1.,
      std::shared_ptr<ColChecker>(new SimpleGraphColCheck()));
    Path path = m.find_path({0});
    Path expected = {{0}};
    ASSERT_EQ(expected, path);
    path = m.find_path({1});
    expected = {{1}, {0}};
    ASSERT_EQ(expected, path);
  }

  TEST_F(EPEMstarTest, TestSingleRobotPathWithObstacles){
    time_point t = std::chrono::system_clock::now();
    t += Clock::duration(std::chrono::seconds(10));
    EPEMstar m(
      {std::shared_ptr<EPEMstar_Policy>(epem_grid_policy_ptr(
				 {{0, 1, 0}, {0, 0, 0}}, {0, 0}))},
      {0}, 1., t, 1.,
      std::shared_ptr<ColChecker>(new SimpleGraphColCheck()));
    Path path = m.find_path({2});
    
    ASSERT_EQ(Path({{2}, {5}, {4},
	    {3}, {0}}), path);
    path = m.find_path({4});
    ASSERT_EQ(Path({{4}, {3}, {0}}),
	      path);
    path = m.find_path({2});
    ASSERT_EQ(Path({{2}, {5}, {4},
	    {3}, {0}}), path);
  }

  TEST_F(EPEMstarTest, TestSingleRobotPathWithObstaclesBigger){
    time_point t = std::chrono::system_clock::now();
    t += Clock::duration(std::chrono::seconds(10));
    EPEMstar m(
      {std::shared_ptr<EPEMstar_Policy>(
	  epem_grid_policy_ptr({{0, 1, 0}, {0, 0, 0}, {0, 0, 0}}, {0, 0}))},
      {0}, 1., t, 1.,
      std::shared_ptr<ColChecker>(new SimpleGraphColCheck()));
    Path path = m.find_path({2});
    ASSERT_EQ(Path({{2}, {5}, {4},
	    {3}, {0}}), path);
    path = m.find_path({4});
    ASSERT_EQ(Path({{4}, {3}, {0}}),
	      path);
    // Test we can find a solution which intersects a cached path
    path = m.find_path({8});
    ASSERT_EQ(5, path.size());
  }

  TEST_F(EPEMstarTest, TestTwoRobotsNoCollision){
    time_point t = std::chrono::system_clock::now();
    t += Clock::duration(std::chrono::seconds(10));
    EPEMstar m(
      {std::shared_ptr<EPEMstar_Policy>(epem_grid_policy_ptr(
				 {{0, 0, 0}, {0, 0, 0}}, {0, 0})),
       std::shared_ptr<EPEMstar_Policy>(epem_grid_policy_ptr(
				 {{0, 0, 0}, {0, 0, 0}}, {1, 0}))},
      {0, 3}, 1., t, 1.,
      std::shared_ptr<ColChecker>(new SimpleGraphColCheck()));
    Path path = m.find_path({2, 5});
    ASSERT_EQ(
      Path({{2, 5}, {1, 4}, {0, 3}}),
      path);
    path = m.find_path({1, 4});
    ASSERT_EQ(
      Path({{1, 4}, {0, 3}}),
      path);
    path = m.find_path({1, 5});
    ASSERT_EQ(
      Path({{1, 5}, {0, 4}, {0, 3}}),
      path);
  }

  TEST_F(EPEMstarTest, TestTwoRobotsSimpleCollision){
    time_point t = std::chrono::system_clock::now();
    t += Clock::duration(std::chrono::seconds(10));
    EPEMstar m(
      {std::shared_ptr<EPEMstar_Policy>(
	  epem_grid_policy_ptr({{0, 0, 0}, {0, 0, 0}}, {0, 0})),
       std::shared_ptr<EPEMstar_Policy>(
	 epem_grid_policy_ptr({{0, 0, 0}, {0, 0, 0}}, {0, 2}))},
      {0, 2}, 1., t, 1.,
      std::shared_ptr<ColChecker>(new SimpleGraphColCheck()));
    Path path = m.find_path({2, 0});
    ASSERT_EQ(5, path.size());
  }

  TEST_F(EPEMstarTest, TestThreeRobotsCollision){
    // Intended to force rM*
    time_point t = std::chrono::system_clock::now();
    t += Clock::duration(std::chrono::seconds(10));
    EPEMstar m(
      {std::shared_ptr<EPEMstar_Policy>(
	  epem_grid_policy_ptr({{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {0, 0})),
       std::shared_ptr<EPEMstar_Policy>(
	 epem_grid_policy_ptr({{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {0, 2})),
       std::shared_ptr<EPEMstar_Policy>(
	 epem_grid_policy_ptr({{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {2, 2}))},
      {0, 2, 8}, 1., t, 1.,
      std::shared_ptr<ColChecker>(new SimpleGraphColCheck()));
    Path path = m.find_path({2, 0, 6});
    ASSERT_EQ(5, path.size());
  }

  TEST_F(EPEMstarTest, TestFourRobotsCollision){
    // Intended to force rM*
    time_point t = std::chrono::system_clock::now();
    t += Clock::duration(std::chrono::seconds(10));
    EPEMstar m(
      {std::shared_ptr<EPEMstar_Policy>(
	  epem_grid_policy_ptr({{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {0, 0})),
       std::shared_ptr<EPEMstar_Policy>(
	 epem_grid_policy_ptr({{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {0, 2})),
       std::shared_ptr<EPEMstar_Policy>(
	 epem_grid_policy_ptr({{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {2, 0})),
       std::shared_ptr<EPEMstar_Policy>(
	 epem_grid_policy_ptr({{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {2, 2}))},
      {0, 2, 6, 8}, 1., t, 1.,
      std::shared_ptr<ColChecker>(new SimpleGraphColCheck()));
    Path path = m.find_path({2, 0, 8, 6});
    ASSERT_EQ(6, path.size());
  }

  TEST_F(EPEMstarTest, TestFiveRobotsCollision){
    // Intended to force rM*
    time_point t = std::chrono::system_clock::now();
    t += Clock::duration(std::chrono::seconds(10));
    EPEMstar m(
      {std::shared_ptr<EPEMstar_Policy>(
	  epem_grid_policy_ptr({{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {0, 0})),
       std::shared_ptr<EPEMstar_Policy>(
	 epem_grid_policy_ptr({{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {0, 2})),
       std::shared_ptr<EPEMstar_Policy>(
	 epem_grid_policy_ptr({{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {2, 0})),
       std::shared_ptr<EPEMstar_Policy>(
	 epem_grid_policy_ptr({{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {2, 2})),
       std::shared_ptr<EPEMstar_Policy>(
	 epem_grid_policy_ptr({{0, 0, 0}, {0, 0, 0}, {0, 0, 0}}, {1, 1}))},
      {0, 2, 6, 8, 4}, 1., t, 1., 
      std::shared_ptr<ColChecker>(new SimpleGraphColCheck()));
    Path path = m.find_path({2, 0, 8, 6, 4});
    ASSERT_EQ(6, path.size());
  }

  TEST_F(EPEMstarTest, TestFindGridPath){
    typedef std::vector<std::vector<std::pair<int, int>>> Path;
    Path path = find_grid_epemstar_path(
      std::vector<std::vector<bool>>(10, std::vector<bool>(10, false)),
      {{1, 0}}, {{0, 0}}, 1, 10, 1.);
    Path correct = {{{1, 0}}, {{0, 0}}};
    ASSERT_EQ(correct, path);

    std::vector<std::vector<bool>> world(10, std::vector<bool>(10, false));
    world[1][0] = true;
    path = find_grid_epemstar_path(
      world, {{2, 0}}, {{0, 0}}, 1, 10, 1.);
    correct = {{{2, 0}}, {{2, 1}}, {{1, 1}}, {{0, 1}}, {{0, 0}}};
    ASSERT_EQ(correct, path);

    path = find_grid_epemstar_path(
      std::vector<std::vector<bool>>(10, std::vector<bool>(10, false)),
      {{1, 0}, {0, 0}}, {{0, 0}, {1, 0}}, 1, 10, 1.);
    ASSERT_EQ(4, path.size());
  }

  TEST_F(EPEMstarTest, DISABLED_TestLargeProblem){
    typedef std::vector<std::vector<std::pair<int, int>>> Path;
    std::vector<std::vector<bool>> world(10, std::vector<bool>(10, false));
    Path path = find_grid_epemstar_path(
      world,
      {{9, 0}, {0, 9}, {0, 0}, {9, 9}, {4, 0}, {4, 9}, {9, 4}},
      {{0, 9}, {9, 0}, {9, 9}, {0, 0}, {4, 9}, {4, 0}, {0, 4}},
      1, 300, 1.);
  }

  TEST_F(EPEMstarTest, TestInflation){
    typedef std::vector<std::vector<std::pair<int, int>>> Path;
    std::vector<std::vector<bool>> world(10, std::vector<bool>(10, false));
    Path path = find_grid_epemstar_path(
      world,
      {{0, 0}, {9, 9}, {9, 0}, {0, 9}, {4, 0}, {4, 9}, {9, 4}, {0, 4}},
      {{9, 9}, {0, 0}, {0, 9}, {9, 0}, {4, 9}, {4, 0}, {0, 4}, {9, 4}},
      1.1, 1, 1.);
  }

  TEST_F(EPEMstarTest, DISABLED_TestLarge){
    typedef std::vector<std::vector<std::pair<int, int>>> Path;
    std::vector<std::vector<bool>> world = {
{0,1,0,0,0,0,1,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0},
{1,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,1,1,0,0,0,1,1,0},
{0,1,0,0,1,0,0,0,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1},
{0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
{0,1,0,1,1,0,0,0,0,0,1,0,0,0,0,0,1,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0},
{0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
{0,0,0,1,0,0,0,0,1,1,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,1,0,0,0,0,0,0},
{0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,1,0,0,1,0,0,1,0,1,0,1,0,1,0,0,0},
{0,0,0,0,1,0,1,1,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,1,1,0,0,0,0,0,0,0},
{1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,1,0,0,0,1,0,0,1,1},
{0,0,1,1,1,0,0,0,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,0,0,0,1},
{0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,1,1,1,0,0,1,0,0,0,1,0,0,0,0,0,0,0},
{0,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,1,0,0,0,0,0,1,1,0,0,0,0},
{0,0,0,0,0,0,0,0,0,1,0,1,0,0,0,0,1,1,0,1,0,0,0,0,0,0,0,0,0,0,0,1},
{0,0,0,1,0,1,0,0,0,0,0,1,0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,1,0,0,1},
{0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,1,0,0,0,1,0,0,0,0,1,0,0,0,0,0,1,0},
{0,0,1,0,0,1,1,1,0,1,1,0,0,1,1,0,0,1,0,0,0,1,0,0,0,1,0,1,0,0,0,1},
{1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,0,0,0,0,0,0,1,1,0,0,0,0,0,1,0,1},
{0,0,0,1,0,0,1,0,0,1,0,1,1,0,1,0,0,0,0,0,0,0,1,1,0,0,0,0,0,1,0,0},
{1,0,0,0,0,0,0,0,0,1,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0},
{0,0,1,1,0,0,0,0,0,0,1,0,1,0,1,0,1,1,1,0,0,1,0,0,0,1,1,0,0,1,1,1},
{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,1,0},
{0,0,0,0,0,0,1,1,0,1,0,0,1,0,0,0,0,1,0,0,0,0,1,0,0,0,1,1,0,0,0,0},
{0,0,1,0,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,0},
{1,0,1,0,0,1,1,0,0,0,1,0,0,0,0,0,1,0,0,0,1,0,1,1,0,0,0,0,0,0,0,0},
{0,0,0,0,1,0,1,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0},
{0,0,0,1,0,0,0,1,0,0,0,0,0,0,0,1,0,0,0,0,0,0,1,0,0,0,0,1,0,1,0,1},
{0,0,1,0,0,1,0,0,0,0,0,0,1,1,0,0,1,1,0,0,0,0,0,0,1,0,1,0,0,0,0,0},
{0,0,0,1,0,0,1,0,0,1,1,0,1,0,0,0,1,0,1,0,1,0,0,0,0,0,0,0,1,0,0,0},
{0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,1,0,1,0,1,0,0},
{0,0,0,0,0,0,1,1,0,0,1,0,1,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0},
{1,0,0,0,0,0,0,0,0,0,0,1,0,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0}
    };

    std::vector<std::pair<int, int>> init_pos = {
      { 26,26},{21,3},{27,6},{17,28},{4,18},{23,3},{24,26},{4,20},{14,15},{27,27},{31,20},{22,31},{30,20},{4,28},{10,19},{5,12},{14,1},{31,17},{20,0},{12,24},{21,28},{24,1},{1,28},{15,8},{5,26},{29,21},{2,25},{13,28},{30,30},{27,7},{28,23},{26,1},{13,8},{15,22},{31,15},{11,18},{12,13},{6,5},{2,27},{29,17},{9,4},{4,2},{24,28},{3,6},{10,22},{13,21},{14,25},{18,16},{17,8},{17,19},{22,15},{12,14},{31,29},{10,5},{2,5},{7,7},{16,4},{3,14},{19,28},{3,4},{12,4},{30,11},{26,13},{15,18},{17,16},{13,14},{30,25},{17,26},{11,8},{24,27},{2,21},{22,19},{8,26},{31,18},{23,26},{9,16},{3,2},{31,27},{6,11},{31,7},{22,28},{3,1},{25,2},{23,20},{23,17},{17,4},{17,7},{11,7},{8,12},{8,3}};
    std::vector<std::pair<int, int>> goals = {
      {29,0},{7,25},{27,11},{17,3},{9,3},{22,16},{5,7},{19,15},{1,7},{2,21},{15,17},{3,8},{14,10},{8,16},{18,21},{9,2},{10,27},{12,6},{23,12},{11,19},{23,10},{12,15},{23,16},{8,28},{14,8},{7,13},{12,12},{0,20},{20,24},{19,18},{7,2},{10,26},{31,9},{7,11},{20,13},{16,8},{21,19},{4,5},{26,11},{9,5},{25,1},{11,26},{6,14},{6,24},{27,21},{27,14},{18,19},{4,14},{20,9},{25,2},{9,10},{4,24},{15,26},{8,14},{26,21},{18,7},{25,0},{22,10},{4,31},{3,12},{27,6},{31,7},{15,5},{20,15},{11,14},{19,3},{23,24},{30,16},{23,7},{17,7},{26,23},{22,23},{10,22},{15,16},{6,21},{20,7},{18,30},{16,30},{21,17},{16,15},{15,27},{29,19},{6,16},{27,8},{0,15},{15,23},{5,25},{9,4},{14,27},{23,14}};

    Path path = find_grid_epemstar_path(
      world, init_pos, goals,3, 60, 1.);
  }



}
