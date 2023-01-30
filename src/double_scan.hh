#pragma once

#include <assert.h>
#include <vector>
#include <queue>
#include <set>

#include "timetable.hh"


typedef timetable::ST ST; // stations
typedef timetable::S S; // stops
typedef timetable::R R; // routes
typedef timetable::T T; // time
    
typedef int TR; // trips

struct t_edge {
    TR trip; // for debugging, cannot use it to vary minimum waiting time in double_scan \fb{actually finally we said we can using prev_in_trip, but need to check carefully}
    ST from, to;
    T dep, arr;
    int index_in_trip;
    t_edge(TR tr, ST u, ST v, T d, T a, int i)
        : trip(tr), from(u), to(v), dep(d), arr(a), index_in_trip(i) {}
};


struct cost_empty { // EAT queries do not use costs

    cost_empty() {
    }
    
    cost_empty(const t_edge &edg) {
    }
    
    friend bool operator<(const cost_empty &e, const cost_empty &f) {
        return false;
    }
    
    friend cost_empty operator+(cost_empty e, cost_empty f) {
        return e;
    }
};



template<class C = cost_empty> // cost type, must override < and + operators, and have a constructor from a t_edge as well as a default constructor

class double_scan {

private:
    const timetable &ttbl;
   
    std::vector<t_edge> e_dep; // t_edges ordered by from_station and dep_time
    typedef int E; // index of a t_edge in e_dep
    std::vector<E> e_arr; // t_edges ordered by arrival time stored through indexes of temporal edges stored in e_dep

    std::vector<std::pair<R, int> > trip_route;
 

    TR n_tr; //number of trips
    E n_edg; // number of temporal edges
    
    std::vector<C> edge_cost;
    std::vector<C> best_cost;
    std::vector<E> parent;
    std::vector<C> min_cost;
    // TODO: std::vector<E> prev_in_trip; // next t_edge in the trip

    struct e_dep_indexes {
        E first, last; // first/last edge from a station. I would like them to be const but it makes it "ugly" to properly initialize st_indexes (since I would like to handle the in which there exists stops with no outgoing edges)
        E left, right; // a suitable interval of edges from a station v that are reachable from the last scanned edge arriving at v
        e_dep_indexes (int fi, int la, int le, int ri)
            :   first(fi), last(la), left(le), right(ri) {}
    };  
    std::vector<e_dep_indexes> st_indexes; // Indexes of edges from in e_dep a given station 

    struct interval {
        E left, right;
        C cost;
        E parent;
    };
    std::vector<std::vector<interval> > st_intervals;
    

    static constexpr E not_a_t_edge = -1;

public:
    double_scan(const timetable &tt)
        : ttbl(tt), n_tr(0), n_edg(0) {
        // create e_dep:
        for (R r = 0; r < ttbl.n_r; ++r) {
            n_tr += tt.trips_of[r].size();
            for (int i = 0; i < tt.trips_of[r].size(); ++i) {
                n_edg += tt.trips_of[r][i].size() - 1;
            }
        }
        e_dep.reserve(n_edg);
        trip_route.reserve(n_tr);
        int i_tr = 0;
        for (R r = 0; r < ttbl.n_r; ++r) {
            const std::vector<S> &stops = tt.route_stops[r];
            for (int i = 0; i < tt.trips_of[r].size(); ++i) {
                for (int j = 1; j < tt.trips_of[r][i].size(); ++j) {
                    e_dep.emplace_back(i_tr,
                                       tt.stop_station[stops[j-1]],
                                       tt.stop_station[stops[j]],
                                       tt.trips_of[r][i][j-1].second,
                                       tt.trips_of[r][i][j].first,
                                       j); // 1 if first edge of a trip
                }
                trip_route.emplace_back(r, i);
                ++i_tr;
            }
        }
        std::sort(e_dep.begin(), e_dep.end(),
                  [](const t_edge &c, const t_edge &d) {
                      if (c.from != d.from) return c.from < d.from; // lexicographical order
                      return c.dep < d.dep;
                  });
        

        // create e_arr:
        e_arr.reserve(n_edg);
        for (E i = 0; i < n_edg; ++i) { e_arr.push_back(i); }
        std::sort(e_arr.begin(), e_arr.end(),
                  [this](E i, E j) {
                      const t_edge &c = e_dep[i];
                      const t_edge &d = e_dep[j];
                      if (c.arr != d.arr) return c.arr < d.arr;
                      // c.arr = d.arr
                      // Be careful to 0 delay t_edges:
                      // Heuristic for zero-acyclicity (not sufficient):
                      if (c.trip == d.trip) return c.index_in_trip < d.index_in_trip;
                      // 0 delay t_edge after:
                      if (c.dep != c.arr) return true;
                      if (d.dep != d.arr) return false;
                      return c.trip < d.trip;
                  });
        
        // costs and parent associated to edges:
        edge_cost.reserve(n_edg);
        best_cost.reserve(n_edg);
        parent.reserve(n_edg);
        for (t_edge edg : e_dep) {
            edge_cost.emplace_back(edg);
            best_cost.emplace_back();
            parent.push_back(not_a_t_edge);
        }
        
        // initialize st_indexes:
        st_indexes.reserve(ttbl.n_st);
        for (int i = 0; i < ttbl.n_st; ++i) { st_indexes.emplace_back(-1,-1,-1,-1); } // this is to handle the case there are stations with no departing edges!
        ST tmp_st = e_dep[0].from;
        E tmp_first = 0;
        for (int i = 1; i < n_edg; ++i) {
            if (e_dep[i].from != tmp_st){
                st_indexes[tmp_st] = e_dep_indexes(tmp_first,i-1,tmp_first,tmp_first-1);
                tmp_st = e_dep[i].from;
                tmp_first = i;
            }
        }
        st_indexes[tmp_st] = e_dep_indexes(tmp_first,n_edg-1,tmp_first,tmp_first-1); // initialize the indexes for the last station
        
    }

    // This function is similar to the one in "SAND paper". The index 'right' here, plays the role of p_v in the paper.
    T earliest_arrival_time(const ST src, const ST dst, const T t_dep,
                            const T min_waiting_time,
                            const T max_waiting_time) {
        assert(0 <= t_dep && t_dep <= 3600*48);
        assert(max_waiting_time > 0);

        // initialize parents
        for (E i = 0; i < n_edg; ++i) { parent[i] = not_a_t_edge; }

        // initialize left and right indexes
        for (E i = 0; i < n_edg; i++) 
        {
            st_indexes[i].left = st_indexes[i].first;
            st_indexes[i].right = st_indexes[i].first-1;
        }       
        
        
        // scan t_edges by non-decreasing arrival times:
        for (E i : e_arr) {
            const t_edge &e = e_dep[i];
            if (e.from == src || parent[i] != not_a_t_edge) // check if e can start or extend a temporal walk
            {
                // compute an interval of edges [l,r] departing from e.to that extend e
                E l = st_indexes[e.to].right; // l is the first index of an edge with departure time >= e.arr + min_waiting_time
                E last = st_indexes[e.to].last; 

                while (e_dep[l].dep < e.arr + min_waiting_time && l <= last)
                {
                    l++;
                }
                l++;
            
                E r =l; // r is the last index of an edge with departure time <= e.arr + max_waiting_time

                while (e_dep[r].dep < e.arr + max_waiting_time && r <= last)
                {
                    parent[r] =i;
                    r++;
                }

                st_indexes[e.to].right = r; // update right to the last edge has been processed

                if (e.to == dst) return e.arr; // the destination has been reached
            }
            
        }
        return ttbl.t_max; // what is the best value to return if the destination was not reachable?
    }

    C get_best_cost(E i) {
        assert(parent[i] != not_a_t_edge);
        return best_cost[i];
    }

    C has_best_cost(E i) {
        return parent[i] != not_a_t_edge;
    }
};


// Standard costs:
    
struct cost_eat { // earliest arrival time

private:
    
    T arr;

public:

    typedef T cost; // Intended cost

    cost intended_cost(const t_edge &last_edge) {
        assert(last_edge.arr == arr);
        return arr;
    }
    
    cost_eat() : arr(std::numeric_limits<T>::max()) {
    }
    
    cost_eat(const t_edge &edg) : arr(edg.arr) {
    }
    
    friend bool operator<(const cost_eat &e, const cost_eat &f) {
        return e.arr < f.arr;
    }
    
    friend cost_eat operator+(cost_eat e, cost_eat f) {
        return f;
    }
};

struct cost_dur { // duration

private:
    
    T dep;

public:

    typedef T cost; // Intended cost

    cost intended_cost(const t_edge &last_edge) {
        return last_edge.arr - dep;
    }
    
    cost_dur() : dep(std::numeric_limits<T>::min()) {
    }
    
    cost_dur(const t_edge &edg) : dep(edg.dep) {
    }
    
    friend bool operator<(const cost_dur &e, const cost_dur &f) {
        return e.dep > f.dep; // later departure leads to shorter duration (for same arrival time)
    }
    
    friend cost_dur operator+(cost_dur e, cost_dur f) {
        return e;
    }
};
