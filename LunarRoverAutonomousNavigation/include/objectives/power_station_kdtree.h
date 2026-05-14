#ifndef POWER_STATION_KDTREE_H
#define POWER_STATION_KDTREE_H

#include "structs.h"

#ifdef __cplusplus
#include <vector>
#include <nanoflann.hpp>

// Adapter for nanoflann KD-tree
class PowerStationKDTree {
private:
    std::vector<struct PowerStation*> power_stations;
    std::vector<PowerStation*> temp_blacklist;
    
public:
    inline size_t kdtree_get_point_count() const { return power_stations.size(); }
    
    inline float kdtree_distance(const float* p1, const size_t idx_p2, size_t /*size*/) const {
        const float d0 = p1[0] - power_stations[idx_p2]->location.x_dim;
        const float d1 = p1[1] - power_stations[idx_p2]->location.y_dim;
        return d0*d0 + d1*d1;
    }

float kdtree_get_pt(const size_t idx, const size_t dim) const;

template <class BBOX>
bool kdtree_get_bbox(BBOX& /* bb */) const;
    
    void add_power_station(PowerStation* station);
    void clear();
    PowerStation* find_nearest(double x, double y);
    PowerStation* find_nearest_excluding_blacklist(double x, double y);
    void add_to_temp_blacklist(PowerStation* station);
    void clear_temp_blacklist();
    void build_tree();
    
private:
    typedef nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, PowerStationKDTree>,
        PowerStationKDTree,
        2 /* dimension */
    > KDTree;
    
    KDTree* index;
};


extern "C" {
#endif

// C interface functions
void power_station_kdtree_init(void);
void power_station_kdtree_add(struct PowerStation* station);
void power_station_kdtree_build(void);
struct PowerStation* power_station_kdtree_find_nearest(double x, double y);
struct PowerStation* power_station_kdtree_find_nearest_excluding_blacklist(double x, double y);
void power_station_kdtree_add_to_temp_blacklist(struct PowerStation* station);
void power_station_kdtree_clear_temp_blacklist(void);
void power_station_kdtree_cleanup(void);


#ifdef __cplusplus
}
#endif

#endif
