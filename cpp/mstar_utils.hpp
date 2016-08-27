#ifndef MSTAR_UTILS_H
#define MSTAR_UTILS_H

/**
 * Defines convinence functions for testing or other purposes not directly
 * related to the actual planning
 */

#include "mstar_type_defs.hpp"

namespace mstar{
  void print_od_path(const OdPath &path);

  void print_path(const std::vector<std::vector<std::pair<int, int>>> &path);
}

#endif
