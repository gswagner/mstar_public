#ifndef MSTAR_COMMON_H
#define MSTAR_COMMON_H

#include <boost/functional/hash_fwd.hpp>

namespace std{
  template <> struct hash<mstar::OdCoord>{
    size_t operator()(const mstar::OdCoord &val) const{
      size_t hash = boost::hash_range(val.coord.cbegin(),
				      val.coord.cend());
      boost::hash_combine<size_t>(
	hash,
	boost::hash_range(val.move_tuple.cbegin(), val.move_tuple.cend()));
      return hash;
    }
  };

  template <> struct hash<std::vector<int>>{
    size_t operator()(const std::vector<int> &val) const{
      return boost::hash_range(val.cbegin(), val.cend());
    }
  };

  template <> struct hash<mstar::ColSetElement>{
    size_t operator()(const mstar::ColSetElement &val) const{
      return boost::hash_range(val.cbegin(), val.cend());
    }
  };
}

namespace mstar{
  template <class Vertex>
  struct greater_cost{
    bool operator()(const Vertex *x, const Vertex *y) const{
      if (x == nullptr || y == nullptr){
	return true;
      }
      return *x > *y;
    }
  };


  struct OutOfTimeError : public std::exception{
    const char * what () const throw(){
      return "Out of Time";
    }
  };

  struct NoSolutionError : public std::exception{
    const char * what () const throw(){
      return "No Solution";
    }
  };

}

#endif
