#include "mstar_utils.hpp"
#include <iostream>

using namespace mstar;

void print_od_path(const OdPath &path){
  for (const OdCoord &pos: path){
    std::cout << "{";
    for (const RobCoord &i: pos.coord){
      std::cout << i << " ";
    }
    std::cout << "}" << std::endl;
  }
}

void print_path(const std::vector<std::vector<std::pair<int, int>>> &path){
  for (const auto &coord: path){
    std::cout << "{";
    for (const auto &c: coord){
      std::cout << "(" << c.first << ", " << c.second << ") ";
    }
    std::cout << "}" << std::endl;
  }
}
