// Copyright 2019 kvedder@umass.edu
// College of Information and Computer Sciences,
// University of Massachusetts Amherst
//
//
// This software is free: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License Version 3,
// as published by the Free Software Foundation.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// Version 3 in the file COPYING that came with this distribution.
// If not, see <http://www.gnu.org/licenses/>.
// ========================================================================

#include <signal.h>
#include <chrono>
#include <iostream>
#include <utility>
#include <vector>
#include <fstream>

#include "grid_planning.hpp"
#include "grid_policy.hpp"
#include "mstar_type_defs.hpp"
#include "mstar_utils.hpp"
#include "od_mstar.hpp"

#include <boost/functional/hash.hpp>
#include <boost/program_options.hpp>

#include <yaml-cpp/yaml.h>

using mstar::Clock;
using mstar::find_grid_path;
using mstar::grid_policy_ptr;
using mstar::ODColChecker;
using mstar::OdCoord;
using mstar::OdMstar;
using mstar::OdPath;
using mstar::Policy;
using mstar::RobCoord;
using mstar::SimpleGraphODColCheck;
using mstar::time_point;
using Path = std::vector<std::vector<std::pair<int, int>>>;

class Timer {
public:
  Timer()
    : start_(std::chrono::high_resolution_clock::now()),
      end_(std::chrono::high_resolution_clock::now()) {}

  void reset() { start_ = std::chrono::high_resolution_clock::now(); }

  void stop() { end_ = std::chrono::high_resolution_clock::now(); }

  double elapsedSeconds() const {
    auto timeSpan = std::chrono::duration_cast<std::chrono::duration<double>>(
                                                                              end_ - start_);
    return timeSpan.count();
  }

private:
  std::chrono::high_resolution_clock::time_point start_;
  std::chrono::high_resolution_clock::time_point end_;
};

class ScopedTimer : public Timer {
public:
  ScopedTimer() {}

  ~ScopedTimer() {
    stop();
    std::cout << "Elapsed: " << elapsedSeconds() << " s" << std::endl;
  }
};

void FatalSignalHandler(const int signo) {
  fprintf(stderr, "Received fatal signal %s, firing custom stack trace:\n",
          strsignal(signo));
  fflush(stderr);
  exit(1);
}

void Init(char* argv0) {

  if (signal(SIGINT, FatalSignalHandler) == SIG_ERR) {
    std::cerr << "Cannot trap SIGINT\n";
  }

  if (signal(SIGSEGV, FatalSignalHandler) == SIG_ERR) {
    std::cerr << "Cannot trap SIGSEGV\n";
  }

  if (signal(SIGABRT, FatalSignalHandler) == SIG_ERR) {
    std::cerr << "Cannot trap SIGABRT\n";
  }
}

struct Location {
  Location(int x, int y) : x(x), y(y) {}
  int x;
  int y;

  bool operator<(const Location& other) const {
    return std::tie(x, y) < std::tie(other.x, other.y);
  }

  bool operator==(const Location& other) const {
    return std::tie(x, y) == std::tie(other.x, other.y);
  }

  friend std::ostream& operator<<(std::ostream& os, const Location& c) {
    return os << "(" << c.x << "," << c.y << ")";
  }
};

namespace std {
  template <>
  struct hash<Location> {
    size_t operator()(const Location& s) const {
      size_t seed = 0;
      boost::hash_combine(seed, s.x);
      boost::hash_combine(seed, s.y);
      return seed;
    }
  };
}  // namespace std

std::vector<std::pair<int, int>> LocationToPair(const std::vector<Location>& v) {
  std::vector<std::pair<int, int>> r;
  for (const Location& l : v) {
    r.push_back(std::make_pair(l.x, l.y));
  }
  return r;
}

int main(int argc, char* argv[]) {
  namespace po = boost::program_options;
  // Declare the supported options.
  po::options_description desc("Allowed options");
  std::string inputFile;
  std::string outputFile;
  desc.add_options()("help", "produce help message")(
      "input,i", po::value<std::string>(&inputFile)->required(),
      "input file (YAML)")("output,o",
                           po::value<std::string>(&outputFile)->required(),
                           "output file (YAML)");

  try {
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help") != 0u) {
      std::cout << desc << "\n";
      return 0;
    }
  } catch (po::error& e) {
    std::cerr << e.what() << std::endl << std::endl;
    std::cerr << desc << std::endl;
    return 1;
  }

  YAML::Node config = YAML::LoadFile(inputFile);

  std::unordered_set<Location> obstacles;
  std::vector<Location> goals;
  std::vector<Location> starts;

  const auto& dim = config["map"]["dimensions"];
  int dimx = dim[0].as<int>();
  int dimy = dim[1].as<int>();

  for (const auto& node : config["map"]["obstacles"]) {
    obstacles.insert(Location(node[0].as<int>(), node[1].as<int>()));
  }

  for (const auto& node : config["agents"]) {
    const auto& start = node["start"];
    const auto& goal = node["goal"];
    starts.emplace_back(Location(start[0].as<int>(), start[1].as<int>()));
    goals.emplace_back(Location(goal[0].as<int>(), goal[1].as<int>()));
  }

  std::vector<std::vector<bool>> world_map;
  for (int x = 0; x < dimx; ++x) {
    world_map.push_back({});
    for (int y = 0; y < dimy; ++y) {
      const Location l(x, y);
      const bool is_obstacle = (obstacles.find(l) != obstacles.end());
      world_map.back().push_back(is_obstacle);
    }
  }

  Timer timer;
  const Path path =
    find_grid_path(world_map, LocationToPair(starts), LocationToPair(goals), 1.0, 100000);
  timer.stop();

  std::ofstream out(outputFile);
  out << "runtime: " << timer.elapsedSeconds() << std::endl;
  out.close();
  return 0;
}

