#include <vector>
#include <memory>

#include <nanoflann.hpp>

#include "objectives/power_station_kdtree.h"


static PowerStationKDTree* kdtree_instance = nullptr;

// PowerStationKDTree implementation
void PowerStationKDTree::add_power_station(PowerStation* station) {
    power_stations.push_back(station);
}

void PowerStationKDTree::clear() {
    power_stations.clear();
    if (index) {
        delete index;
        index = nullptr;
    }
}

float PowerStationKDTree::kdtree_get_pt(const size_t idx, const size_t dim) const {
    if (dim == 0) return (float)power_stations[idx]->location.x_dim;
    return (float)power_stations[idx]->location.y_dim;
}

template <class BBOX>
bool PowerStationKDTree::kdtree_get_bbox(BBOX& /* bb */) const {
    return false;
}

void PowerStationKDTree::build_tree() {
    if (index) {
        delete index;
    }
    index = new KDTree(2 /* dimension */, *this, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    index->buildIndex();
}

PowerStation* PowerStationKDTree::find_nearest(double x, double y) {
    if (!index || power_stations.empty()) {
        return nullptr;
    }
    
    float query_point[2] = { (float)x, (float)y };
    size_t ret_index;
    float out_dist_sqr;
    
    // Find nearest neighbor
    nanoflann::KNNResultSet<float> resultSet(1);
    resultSet.init(&ret_index, &out_dist_sqr);
    nanoflann::SearchParams search_params;
    index->findNeighbors(resultSet, query_point, search_params);
    
    if (ret_index < power_stations.size()) {
        return power_stations[ret_index];
    }
    
    return nullptr;
}

PowerStation* PowerStationKDTree::find_nearest_excluding_blacklist(double x, double y) {
    if (!index || power_stations.empty()) {
        return nullptr;
    }
    
    float query_point[2] = { (float)x, (float)y };
    const size_t k = temp_blacklist.size() + 1; // Get enough neighbors to skip blacklisted ones
    std::vector<size_t> ret_indices(k);
    std::vector<float> out_dist_sqr(k);
    
    // Find k nearest neighbors
    nanoflann::KNNResultSet<float> resultSet(k);
    resultSet.init(&ret_indices[0], &out_dist_sqr[0]);
    nanoflann::SearchParams search_params;
    index->findNeighbors(resultSet, query_point, search_params);
    
    // Return the first non-blacklisted station
    for (size_t i = 0; i < k; i++) {
        if (ret_indices[i] < power_stations.size()) {
            PowerStation* station = power_stations[ret_indices[i]];
            // Check if station is in blacklist
            bool is_blacklisted = false;
            for (size_t j = 0; j < temp_blacklist.size(); j++) {
                if (temp_blacklist[j] == station) {
                    is_blacklisted = true;
                    break;
                }
            }
            if (!is_blacklisted) {
                return station;
            }
        }
    }
    
    return nullptr;
}

void PowerStationKDTree::add_to_temp_blacklist(PowerStation* station) {
    if (station) {
        temp_blacklist.push_back(station);
    }
}

void PowerStationKDTree::clear_temp_blacklist() {
    temp_blacklist.clear();
}

// C interface implementation
extern "C" {

void power_station_kdtree_init(void) {
    if (!kdtree_instance) {
        kdtree_instance = new PowerStationKDTree();
    }
}

void power_station_kdtree_add(PowerStation* station) {
    if (kdtree_instance) {
        kdtree_instance->add_power_station(station);
    }
}

void power_station_kdtree_build(void) {
    if (kdtree_instance) {
        kdtree_instance->build_tree();
    }
}

PowerStation* power_station_kdtree_find_nearest(double x, double y) {
    if (kdtree_instance) {
        return kdtree_instance->find_nearest(x, y);
    }
    return nullptr;
}

PowerStation* power_station_kdtree_find_nearest_excluding_blacklist(double x, double y) {
    if (kdtree_instance) {
        return kdtree_instance->find_nearest_excluding_blacklist(x, y);
    }
    return nullptr;
}

void power_station_kdtree_add_to_temp_blacklist(PowerStation* station) {
    if (kdtree_instance) {
        kdtree_instance->add_to_temp_blacklist(station);
    }
}

void power_station_kdtree_clear_temp_blacklist(void) {
    if (kdtree_instance) {
        kdtree_instance->clear_temp_blacklist();
    }
}

void power_station_kdtree_cleanup(void) {
    if (kdtree_instance) {
        delete kdtree_instance;
        kdtree_instance = nullptr;
    }
}

}
