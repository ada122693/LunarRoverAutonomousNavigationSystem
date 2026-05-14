#include <cmath>
#include <algorithm>

#include <opencv2/opencv.hpp>

#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_sdf/SignedDistanceField.hpp>
#include <grid_map_core/grid_map_core.hpp>

#include "constants/constants.h"
#include "constants/config.h"


grid_map::GridMap map;

//returns slope in rads
extern "C" double get_slope(double x, double y) {
    grid_map::Position position(x, y);

    if (!map.isInside(position)) {
        return 1.5708; // 90 degrees
    }
    
    grid_map::Index index;
    if (!map.getIndex(position, index)) {
        return 1.5708;
    }

    //if (!map.isValid(position, "slope")) {
    //    return 1.5708; // 90 degrees
    //}
    double slope_value = map.at("slope", index);
    if (!std::isfinite(slope_value)) {
        return 1.5708; // 90 degrees
    }
    
    return slope_value;
}

//checks for collisions, returns true if there's a collision
extern "C" bool check_collision(double x, double y) {
        int n = 8;
    for (int i = 0; i < n; i++) {
        double angle = i * (2.0 * M_PI / n);
        double cx = x + ROVER_RADIUS * cos(angle);
        double cy = y + ROVER_RADIUS * sin(angle);
        grid_map::Position pos(cx, cy);
        if (!map.isInside(pos)) return true;
        grid_map::Index idx;
        if (!map.getIndex(pos, idx)) return true;
        double elev = map.at("elevation", idx);
        if (!std::isfinite(elev)) return true;
        if (get_slope(cx, cy) > MAX_ALLOWED_SLOPE) return true;
        if (map.exists("roughness") && map.at("roughness", idx) > MAX_ALLOWED_ROUGHNESS) return true;
    }

    grid_map::Position center(x, y);
    if (!map.isInside(center)) return true;
    grid_map::Index cidx;
    if (!map.getIndex(center, cidx)) return true;
    double elev = map.at("elevation", cidx);
    if (!std::isfinite(elev)) return true;
    if (get_slope(x, y) > MAX_ALLOWED_SLOPE) return true;
    return false;
}

// THIS IS FOR PNG FOR GAZEBO AND SIMILAR SIMS
extern "C" void init_gridmap() {
    fprintf(stderr, "[GRIDMAP] Starting gridmap initialization\n");
    // Load PNG heightmap using Gazebo Classic conversion formula
    cv::Mat heightmap = cv::imread(GRID_FILE_PATH, cv::IMREAD_GRAYSCALE);
    if (heightmap.empty()) {
        fprintf(stderr, "[GRIDMAP] ERROR: Failed to load heightmap from %s\n", GRID_FILE_PATH);
        return;
    }
    
    
    map.add("elevation");
    map.add("slope");
    map.add("roughness");
    map.setGeometry(grid_map::Length(X_DIM_SIZE, Y_DIM_SIZE), PRIM_LENGTH);
    map.setFrameId("map");
    map.setPosition(grid_map::Position(0.0, 0.0));
    
    // grayscale (0-255) → elevation (0 to MAX_ELEVATION)
    // black = 0m, white = MAX_ELEVATION meters from config.c
    fprintf(stderr, "[GRIDMAP] Converting your png into a map...\n");
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
    grid_map::Position pos;
    map.getPosition(*it, pos);
    
    int img_x = static_cast<int>((pos.x() + X_DIM_SIZE/2.0) / X_DIM_SIZE * heightmap.cols);
    int img_y = static_cast<int>((-pos.y() + Y_DIM_SIZE/2.0) / Y_DIM_SIZE * heightmap.rows);
    
    img_x = std::max(0, std::min(img_x, heightmap.cols - 1));
    img_y = std::max(0, std::min(img_y, heightmap.rows - 1));
    
    double grayscale = heightmap.at<uchar>(img_y, img_x);
    map.at("elevation", *it) = (grayscale / 255.0) * MAX_ELEVATION;
}
    
    fprintf(stderr, "[GRIDMAP] Calculating features (this may take a while)...\n");
    // slope and roughness
    grid_map::GridMapIterator it2(map);
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
        grid_map::Index index(*it);
        
        // calculate gradient using differences
        double dx = 0.0, dy = 0.0;
        grid_map::Index neighbor;
        
        // X gradient (central difference)
        if (index.x() > 0 && index.x() < map.getSize().x() - 1) {
            neighbor = index;

            neighbor.x() = index.x() - 1;
            double h_left = map.at("elevation", neighbor);
            neighbor.x() = index.x() + 1;
            double h_right = map.at("elevation", neighbor);
            dx = (h_right - h_left) / (2.0 * map.getResolution());
        }
        
        // Y gradient (central difference)
        if (index.y() > 0 && index.y() < map.getSize().y() - 1) {
            neighbor = index;
            neighbor.y() = index.y() - 1;
            double h_down = map.at("elevation", neighbor);
            neighbor.y() = index.y() + 1;
            double h_up = map.at("elevation", neighbor);
            dy = (h_up - h_down) / (2.0 * map.getResolution());
        }
        
        // slope = atan(sqrt(dx^2 + dy^2))
        double gradient_magnitude = std::sqrt(dx*dx + dy*dy);
        map.at("slope", *it) = std::atan(gradient_magnitude);
        // forcing a max slope above 1 cause gazebo is acting weird
	if (map.at("elevation", *it) > 1.0) {
    		map.at("slope", *it) = 1.5708;
	}
        
        
        // roughness
        grid_map::Position pos;
        map.getPosition(*it, pos);
        float sum = 0, sq_sum = 0; int count = 0;
        for (grid_map::CircleIterator cIt(map, pos, 0.2); !cIt.isPastEnd(); ++cIt) {
            float h = map.at("elevation", *cIt);
            if (std::isfinite(h)) { 
                sum += h;
                sq_sum += h * h;
                count++;
            }
        }
        if (count > 0) {
            double mean = sum/count;
            map.at("roughness", *it) = std::sqrt(std::max(0.0, (sq_sum/count) - (mean*mean)));
        } else {
        map.at("roughness", *it) = 0.0;
        }
    }
    
            grid_map::Position p1(3.0, 2.0);
grid_map::Position p2(1.0, -2.0);
grid_map::Index i1, i2;
if (map.getIndex(p1, i1)) {
    fprintf(stderr, "elevation at (3,2): %.2f\n", map.at("elevation", i1)); }
if (map.getIndex(p2, i2)) {
    fprintf(stderr, "elevation at (1,-2): %.2f\n", map.at("elevation", i2)); }
    fprintf(stderr, "collision at (3,2): %d\n", check_collision(3.0, 2.0));
fprintf(stderr, "collision at (1,-2): %d\n", check_collision(1.0, -2.0));
        
    
    fprintf(stderr, "[GRIDMAP] Map Created successfully\n");
    
}

//FOR PGMS / YAMLS
//JUST USING THE CARTOGRAPHER DEFAULT FOR NOW BUT YOU CAN READ THE YAML FOR CUSTOM
//VALUES
/*
extern "C" void init_gridmap() {
    cv::Mat heightmap = cv::imread(GRID_FILE_PATH, cv::IMREAD_GRAYSCALE);
    if (heightmap.empty()) {
        fprintf(stderr, "[GRIDMAP] ERROR: Failed to load pgm from %s\n", GRID_FILE_PATH);
        return;
    }

    const double origin_x = -2.58;
    const double origin_y = -2.12;
    const double resolution = 0.05;
    

    map.add("elevation");
    map.add("slope");
    map.add("roughness");
    // Set map geometry to match PGM's actual world dimensions
    double map_width_m = heightmap.cols * resolution;
    double map_height_m = heightmap.rows * resolution;
    map.setGeometry(grid_map::Length(map_width_m, map_height_m), PRIM_LENGTH);
    map.setFrameId("map");
    // Position map so its bottom-left corner is at the PGM's world origin
    map.setGeometry(grid_map::Length(X_DIM_SIZE, Y_DIM_SIZE), PRIM_LENGTH);
map.setFrameId("map");
map.setPosition(grid_map::Position(0.0, 0.0));
    //map.setPosition(grid_map::Position(origin_x + map_width_m / 2.0, origin_y + map_height_m / 2.0));

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
        grid_map::Position pos;
        map.getPosition(*it, pos);  // X, Y world

        int img_x = static_cast<int>((pos.x() - origin_x) / resolution);
        int img_y = heightmap.rows - static_cast<int>((pos.y() - origin_y) / resolution) - 1;

        // Bounds check
        if (img_x < 0 || img_x >= heightmap.cols || img_y < 0 || img_y >= heightmap.rows) {
            map.at("slope", *it) = 1.5708;
            map.at("elevation", *it) = MAX_ELEVATION;
            map.at("roughness", *it) = MAX_ALLOWED_ROUGHNESS + 1.0;
            continue;
        }

        uint8_t pixel = heightmap.at<uchar>(img_y, img_x);

        if (pixel < 165) {
            map.at("slope", *it) = 1.5708;
            map.at("elevation", *it) = MAX_ELEVATION;
            map.at("roughness", *it) = MAX_ALLOWED_ROUGHNESS + 1.0;
        } else {
            map.at("slope", *it) = 0.0;
            map.at("elevation", *it) = 0.0;
            map.at("roughness", *it) = 0.0;
        }
    }

    fprintf(stderr, "[GRIDMAP] Map created successfully\n");
}*/
