#ifndef MSTAR_EPEA_VERTEX_H
#define MSTAR_EPEA_VERTEX_H

namespace mstar{

  struct EpeaVertex{
    Coord coord;
    ColSet col_set, gen_set; // Collision set and generating collision set
    int updated; // last planning iteration used
    bool closed, open;
    double cost, h, offset, prev_offset, max_offset;
    EpeaVertex* back_ptr; // optimal way to reach this
    std::set<EpeaVertex*> back_prop_set; // all explored ways to reach this
    EpeaVertex* forwards_ptr; // way to goal from this

    EpeaVertex(Coord coord):
      coord(coord), col_set(), updated(0), closed(false), open(false),
      cost(std::numeric_limits<double>::max()), h(),
      offset(), prev_offset(std::numeric_limits<double>::lowest()),
      max_offset(), back_ptr(nullptr), back_prop_set(),
      forwards_ptr(nullptr)
      {};

    bool operator>=(const EpeaVertex &other) const{
      return cost + h + offset >= other.cost + other.h + other.offset;
    }

    bool operator>(const EpeaVertex &other) const{
      return cost + h + offset > other.cost + other.h + other.offset;
    }

    bool operator<=(const EpeaVertex &other) const{
      return cost + h + offset <= other.cost + other.h + other.offset;
    }

    bool operator<(const EpeaVertex &other) const{
      return cost + h + offset < other.cost + other.h + other.offset;
    }

    /**
     * Resets a vertex used in a previous planning iteration
     *
     * @param t Current planning iteration
     */
    void reset(int t){
      if (t > updated){
	updated = t;
	open = false;
	closed = false;
	cost = std::numeric_limits<double>::max();
	offset = 0;
	prev_offset = std::numeric_limits<double>::lowest();
	back_ptr = nullptr;
	back_prop_set = std::set<EpeaVertex *>();
      }
    }
  };

}

#endif
