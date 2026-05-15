#pragma once

#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <utility>
#include <vector>

namespace click_obstacles_layer {

class RegionKeepoutLayer: public nav2_costmap_2d::Layer {
public:
    RegionKeepoutLayer() = default;
    ~RegionKeepoutLayer() override = default;

    void onInitialize() override;
    void updateBounds(
        double origin_x,
        double origin_y,
        double origin_yaw,
        double* min_x,
        double* min_y,
        double* max_x,
        double* max_y
    ) override;
    void
    updateCosts(nav2_costmap_2d::Costmap2D& master_grid, int min_i, int min_j, int max_i, int max_j)
        override;
    void reset() override;

    bool isClearable() override {
        return false;
    }

private:
    struct Bounds {
        double min_x { 0.0 };
        double max_x { 0.0 };
        double min_y { 0.0 };
        double max_y { 0.0 };
        bool valid { false };
    };

    bool pointInPolygon(double x, double y, const std::vector<std::pair<double, double>>& polygon)
        const;
    bool transformPolygon(std::vector<std::pair<double, double>>& polygon_out) const;
    Bounds computeBounds(const std::vector<std::pair<double, double>>& polygon) const;
    bool outsideCenterCorridor(double x, double y, const Bounds& bounds) const;

    bool enabled_ { true };
    std::string polygon_frame_ { "map" };
    std::vector<std::pair<double, double>> polygon_;
    double centerline_width_ { 0.36 };
    unsigned char keepout_cost_ { nav2_costmap_2d::LETHAL_OBSTACLE };
};

} // namespace click_obstacles_layer
