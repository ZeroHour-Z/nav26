#pragma once

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mutex>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav2_costmap_2d/costmap_layer.hpp>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <string>
#include <vector>

namespace click_obstacles_layer {

enum class RegionRole {
    OneWayFallback,
    PreferredFallback,
};

struct ForbiddenRegion {
    std::string name;
    RegionRole role { RegionRole::OneWayFallback };
    std::vector<std::pair<double, double>> polygon;
    double center_y { 0.0 };
    double min_x { 0.0 };
    double min_y { 0.0 };
    double max_x { 0.0 };
    double max_y { 0.0 };
    bool blocked { false };
};

class DirectionalForbiddenLayer: public nav2_costmap_2d::CostmapLayer {
public:
    DirectionalForbiddenLayer();
    ~DirectionalForbiddenLayer() override = default;

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

    void activate() override;
    void deactivate() override;

private:
    void onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void onPlan(const nav_msgs::msg::Path::SharedPtr msg);
    bool loadRegion(const std::string& param_name, const std::string& name);
    bool pathHasForbiddenTraversal(
        const nav_msgs::msg::Path& path,
        const ForbiddenRegion& region
    ) const;
    static bool isPointInPolygon(
        double x,
        double y,
        const std::vector<std::pair<double, double>>& polygon
    );

    bool enabled_ { true };
    double min_region_delta_y_ { 0.10 };
    std::string goal_topic_ { "/goal_pose" };
    std::string plan_topic_ { "/plan" };
    std::string route_failed_topic_ { "/fluctuate_route_failed" };
    double dwell_timeout_sec_ { 6.0 };
    double dwell_near_distance_ { 0.35 };
    double fallback_clear_timeout_sec_ { 3.0 };
    bool fallback_active_ { false };
    bool route_failed_ { false };
    bool path_through_one_way_ { false };
    bool path_through_preferred_ { false };

    enum class DwellTarget {
        None,
        PreferredFallback,
        OneWayFallback,
    };

    DwellTarget dwell_target_ { DwellTarget::None };
    rclcpp::Time dwell_started_at_;
    bool dwell_timer_active_ { false };
    rclcpp::Time clear_started_at_;
    bool clear_timer_active_ { false };

    mutable std::mutex mutex_;
    std::vector<ForbiddenRegion> regions_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr route_failed_pub_;

    void resetRouteStateLocked();
    void updatePathStateLocked(const nav_msgs::msg::Path& path);
    void updateDwellStateLocked(double robot_x, double robot_y);
    void setFallbackActiveLocked(bool active);
    void setRouteFailedLocked(bool failed);
    bool isNearAnyRegion(double x, double y, RegionRole role) const;
    static double distanceToPolygon(
        double x,
        double y,
        const std::vector<std::pair<double, double>>& polygon
    );
};

} // namespace click_obstacles_layer
