#include <vector>
#include <nanoflann.hpp>

#include "constants/arrsize_global.h"

#include "objectives/objective_kdtree.h"

// Adapter for nanoflann KD-tree
class ObjectiveKDTreeImpl {
private:
    std::vector<Objective*> objectives;
    
public:
    // Must return the number of data points
    inline size_t kdtree_get_point_count() const { return objectives.size(); }
    
    // Must return the dim'th component of the idx'th point in "class PointRep"
    inline float kdtree_distance(const float* p1, const size_t idx_p2, size_t /*size*/) const {
        const float d0 = p1[0] - objectives[idx_p2]->location.x_dim;
        const float d1 = p1[1] - objectives[idx_p2]->location.y_dim;
        return d0*d0 + d1*d1;
    }

        // Must return the dim'th component of the idx'th point
    inline float kdtree_get_pt(const size_t idx, int dim) const {
        if (dim == 0) return objectives[idx]->location.x_dim;
        if (dim == 1) return objectives[idx]->location.y_dim;
        return 0.0f;
    }

    
    // Optional bounding-box computation
    template <class BBOX>
    bool kdtree_get_bbox(BBOX& /* bb */) const { return false; }
    
    // Management functions
    void add_objective(Objective* objective) {
        objectives.push_back(objective);
    }
    
    void clear() {
        objectives.clear();
        if (index) {
            delete index;
            index = nullptr;
        }
    }
    
    Objective* find_nearest(double x, double y) {
        if (!index || objectives.empty()) {
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
        
        if (ret_index < objectives.size()) {
            return objectives[ret_index];
        }
        return nullptr;
    }
    
    void remove_objective(Objective* objective) {
        for (auto it = objectives.begin(); it != objectives.end(); ++it) {
            if (*it == objective) {
                objectives.erase(it);
                break;
            }
        }
    }
    
    void build_tree() {
        if (index) {
            delete index;
        }
        index = new KDTree(2 /* dimension */, *this, nanoflann::KDTreeSingleIndexAdaptorParams(10));
        index->buildIndex();
    }
    
    size_t size() const { return objectives.size(); }
    
    Objective* get_objective_by_index(size_t index) {
        if (index < objectives.size()) {
            return objectives[index];
        }
        return nullptr;
    }
    
    Objective* find_next_nearest(double x, double y, Objective* exclude_obj) {
        if (!index || objectives.empty()) {
            return nullptr;
        }
        
        Objective* best_obj = nullptr;
        double best_dist = -1.0;
        
        // Brute-force search through all objectives
        for (size_t i = 0; i < objectives.size(); i++) {
            Objective* obj = objectives[i];
            if (obj == exclude_obj) continue;
            
            double dx = x - obj->location.x_dim;
            double dy = y - obj->location.y_dim;
            double dist = dx*dx + dy*dy;
            
            if (best_obj == nullptr || dist < best_dist) {
                best_dist = dist;
                best_obj = obj;
            }
        }
        
        return best_obj;
    }
    
private:
    typedef nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<float, ObjectiveKDTreeImpl>,
        ObjectiveKDTreeImpl,
        2 /* dimension */
    > KDTree;
    
    KDTree* index;
};

// Static instances for each priority level
static ObjectiveKDTreeImpl* objective_kdtrees[MAX_OBJECTIVE_PRIORITY_LEVELS];

// Reserved KD-tree for consumed objectives
static ObjectiveKDTreeImpl* reserved_kdtree = nullptr;


void objective_kdtree_add(struct ObjectiveKDTree* tree, struct Objective* objective) {
    // tree parameter is ignored, we use static instances
    if (objective && objective->priority >= 0 && objective->priority < MAX_OBJECTIVE_PRIORITY_LEVELS) {
        if (!objective_kdtrees[objective->priority]) {
            objective_kdtrees[objective->priority] = new ObjectiveKDTreeImpl();
        }
        objective_kdtrees[objective->priority]->add_objective(objective);
    }
}

void objective_kdtree_build(struct ObjectiveKDTree* tree) {
    // Initialize reserved tree if not exists
    if (!reserved_kdtree) {
        reserved_kdtree = new ObjectiveKDTreeImpl();
    }
    
    // Build all trees
    for (int i = 0; i < MAX_OBJECTIVE_PRIORITY_LEVELS; i++) {
        if (objective_kdtrees[i]) {
            objective_kdtrees[i]->build_tree();
        }
    }
    
    // Build reserved tree
    reserved_kdtree->build_tree();
}

void objective_kdtree_remove(struct ObjectiveKDTree* tree, struct Objective* objective) {
    if (objective && objective->priority >= 0 && objective->priority < MAX_OBJECTIVE_PRIORITY_LEVELS) {
        if (objective_kdtrees[objective->priority]) {
            // Remove from priority tree
            objective_kdtrees[objective->priority]->remove_objective(objective);
            
            // Add to reserved tree
            if (reserved_kdtree) {
                reserved_kdtree->add_objective(objective);
            }
        }
    }
}

// Additional function to find nearest in specific priority level
struct Objective* objective_kdtree_find_nearest_in_priority(int priority, struct Location location) {
    if (priority >= 0 && priority < MAX_OBJECTIVE_PRIORITY_LEVELS && 
        objective_kdtrees[priority] && objective_kdtrees[priority]->size() > 0) {
        return objective_kdtrees[priority]->find_nearest(location.x_dim, location.y_dim);
    }
    return nullptr;
}

// Function to find next-nearest objective excluding a specific one
struct Objective* objective_kdtree_find_next_nearest_in_priority(int priority, struct Location location, struct Objective* exclude_obj) {
    if (priority >= 0 && priority < MAX_OBJECTIVE_PRIORITY_LEVELS && 
        objective_kdtrees[priority] && objective_kdtrees[priority]->size() > 0) {
        
        // If tree has only 1 objective and it's the excluded one, return null
        if (objective_kdtrees[priority]->size() == 1) {
            Objective* only_obj = objective_kdtrees[priority]->find_nearest(location.x_dim, location.y_dim);
            if (only_obj == exclude_obj) {
                return nullptr;
            }
            return only_obj;
        }
        
        // Find the nearest, if it's not the excluded one, return it
        Objective* nearest = objective_kdtrees[priority]->find_nearest(location.x_dim, location.y_dim);
        if (nearest != exclude_obj) {
            return nearest;
        }
        
        // If the nearest is the excluded one, find the next nearest
        return objective_kdtrees[priority]->find_next_nearest(location.x_dim, location.y_dim, exclude_obj);
    }
    return nullptr;
}

// Push function to pull from reserved tree and add back to priority tree
void objective_kdtree_push(struct Objective* obj) {
    if (!reserved_kdtree || !obj) return;
    
    // Remove from reserved tree
    reserved_kdtree->remove_objective(obj);
    
    // Add back to priority tree
    if (obj->priority >= 0 && obj->priority < MAX_OBJECTIVE_PRIORITY_LEVELS) {
        if (!objective_kdtrees[obj->priority]) {
            objective_kdtrees[obj->priority] = new ObjectiveKDTreeImpl();
        }
        objective_kdtrees[obj->priority]->add_objective(obj);
        objective_kdtrees[obj->priority]->build_tree();
    }
}

// Cleanup function
void objective_kdtree_cleanup(void) {
    for (int i = 0; i < MAX_OBJECTIVE_PRIORITY_LEVELS; i++) {
        if (objective_kdtrees[i]) {
            delete objective_kdtrees[i];
            objective_kdtrees[i] = nullptr;
        }
    }
    
    if (reserved_kdtree) {
        delete reserved_kdtree;
        reserved_kdtree = nullptr;
    }
}
