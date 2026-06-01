#include "click_obstacles_layer/directional_forbidden_layer.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <pluginlib/class_list_macros.hpp>

namespace click_obstacles_layer {

DirectionalForbiddenLayer::DirectionalForbiddenLayer() {
    enabled_ = true;
}

void DirectionalForbiddenLayer::onInitialize() {
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("goal_topic", rclcpp::ParameterValue(std::string("/goal_pose")));
    declareParameter("plan_topic", rclcpp::ParameterValue(std::string("/plan")));
    declareParameter("min_region_delta_y", rclcpp::ParameterValue(0.10));
    declareParameter("dwell_timeout_sec", rclcpp::ParameterValue(6.0));
    declareParameter("dwell_near_distance", rclcpp::ParameterValue(0.35));
    declareParameter("fallback_clear_timeout_sec", rclcpp::ParameterValue(3.0));
    declareParameter("route_failed_topic", rclcpp::ParameterValue(std::string("/fluctuate_route_failed")));
    declareParameter("fluctuate_region_1", rclcpp::ParameterValue(std::vector<double> {}));
    declareParameter("fluctuate_region_2", rclcpp::ParameterValue(std::vector<double> {}));
    declareParameter("fluctuate_region_3", rclcpp::ParameterValue(std::vector<double> {}));
    declareParameter("fluctuate_region_4", rclcpp::ParameterValue(std::vector<double> {}));

    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error("DirectionalForbiddenLayer: node expired");
    }

    (void)node->get_parameter(name_ + ".enabled", enabled_);
    (void)node->get_parameter(name_ + ".goal_topic", goal_topic_);
    (void)node->get_parameter(name_ + ".plan_topic", plan_topic_);
    (void)node->get_parameter(name_ + ".min_region_delta_y", min_region_delta_y_);
    (void)node->get_parameter(name_ + ".dwell_timeout_sec", dwell_timeout_sec_);
    (void)node->get_parameter(name_ + ".dwell_near_distance", dwell_near_distance_);
    (void)node->get_parameter(name_ + ".fallback_clear_timeout_sec", fallback_clear_timeout_sec_);
    (void)node->get_parameter(name_ + ".route_failed_topic", route_failed_topic_);

    regions_.clear();
    if (loadRegion("fluctuate_region_1", "fluctuate_region_1")) {
        regions_.back().role = RegionRole::OneWayFallback;
    }
    if (loadRegion("fluctuate_region_3", "fluctuate_region_3")) {
        regions_.back().role = RegionRole::OneWayFallback;
    }
    if (loadRegion("fluctuate_region_2", "fluctuate_region_2")) {
        regions_.back().role = RegionRole::PreferredFallback;
    }
    if (loadRegion("fluctuate_region_4", "fluctuate_region_4")) {
        regions_.back().role = RegionRole::PreferredFallback;
    }

    goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic_,
        10,
        std::bind(&DirectionalForbiddenLayer::onGoal, this, std::placeholders::_1)
    );
    plan_sub_ = node->create_subscription<nav_msgs::msg::Path>(
        plan_topic_,
        10,
        std::bind(&DirectionalForbiddenLayer::onPlan, this, std::placeholders::_1)
    );
    route_failed_pub_ = node->create_publisher<std_msgs::msg::Bool>(
        route_failed_topic_,
        rclcpp::QoS(1).transient_local().reliable()
    );
    setRouteFailedLocked(false);

    current_ = true;
}

void DirectionalForbiddenLayer::activate() {}
void DirectionalForbiddenLayer::deactivate() {}
void DirectionalForbiddenLayer::reset() {}

void DirectionalForbiddenLayer::onGoal(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    (void)msg;
    std::lock_guard<std::mutex> lk(mutex_);
    resetRouteStateLocked();
}

void DirectionalForbiddenLayer::onPlan(const nav_msgs::msg::Path::SharedPtr msg) {
    auto node = node_.lock();
    std::lock_guard<std::mutex> lk(mutex_);
    updatePathStateLocked(*msg);

    for (auto& region: regions_) {
        if (region.role != RegionRole::OneWayFallback || region.blocked || fallback_active_
            || !pathHasForbiddenTraversal(*msg, region))
        {
            continue;
        }

        region.blocked = true;
        addExtraBounds(region.min_x, region.min_y, region.max_x, region.max_y);
        if (node) {
            RCLCPP_WARN(
                node->get_logger(),
                "DirectionalForbiddenLayer: %s blocked because /plan traverses it with increasing |y|",
                region.name.c_str()
            );
        }
    }
}

void DirectionalForbiddenLayer::resetRouteStateLocked() {
    for (auto& region: regions_) {
        region.blocked = false;
        addExtraBounds(region.min_x, region.min_y, region.max_x, region.max_y);
    }
    fallback_active_ = false;
    dwell_target_ = DwellTarget::None;
    dwell_timer_active_ = false;
    clear_timer_active_ = false;
    path_through_one_way_ = false;
    path_through_preferred_ = false;
    setRouteFailedLocked(false);
}

void DirectionalForbiddenLayer::updateDwellStateLocked(double robot_x, double robot_y) {
    auto node = node_.lock();
    if (!node) {
        return;
    }

    const bool near_preferred = isNearAnyRegion(robot_x, robot_y, RegionRole::PreferredFallback);
    const bool near_one_way = isNearAnyRegion(robot_x, robot_y, RegionRole::OneWayFallback);

    DwellTarget target = DwellTarget::None;
    if (fallback_active_) {
        if (path_through_one_way_ && near_one_way) {
            target = DwellTarget::OneWayFallback;
        }
    } else if (path_through_preferred_ && near_preferred) {
        target = DwellTarget::PreferredFallback;
    }

    const rclcpp::Time now = node->now();
    if (target == DwellTarget::None) {
        dwell_target_ = DwellTarget::None;
        dwell_timer_active_ = false;
    } else if (!dwell_timer_active_ || target != dwell_target_) {
        dwell_target_ = target;
        dwell_started_at_ = now;
        dwell_timer_active_ = true;
    } else if ((now - dwell_started_at_).seconds() >= dwell_timeout_sec_) {
        if (target == DwellTarget::PreferredFallback) {
            setFallbackActiveLocked(true);
            dwell_timer_active_ = false;
        } else if (target == DwellTarget::OneWayFallback) {
            setRouteFailedLocked(true);
        }
    }

    if (fallback_active_ && !path_through_one_way_ && !path_through_preferred_) {
        if (!clear_timer_active_) {
            clear_started_at_ = now;
            clear_timer_active_ = true;
        } else if ((now - clear_started_at_).seconds() >= fallback_clear_timeout_sec_) {
            setFallbackActiveLocked(false);
            setRouteFailedLocked(false);
            clear_timer_active_ = false;
        }
    } else {
        clear_timer_active_ = false;
    }
}

void DirectionalForbiddenLayer::updatePathStateLocked(const nav_msgs::msg::Path& path) {
    path_through_one_way_ = false;
    path_through_preferred_ = false;

    for (const auto& region: regions_) {
        for (const auto& pose: path.poses) {
            if (!isPointInPolygon(
                    pose.pose.position.x,
                    pose.pose.position.y,
                    region.polygon
                ))
            {
                continue;
            }

            if (region.role == RegionRole::OneWayFallback) {
                path_through_one_way_ = true;
            } else {
                path_through_preferred_ = true;
            }
            break;
        }
    }
}

void DirectionalForbiddenLayer::setFallbackActiveLocked(bool active) {
    auto node = node_.lock();
    if (fallback_active_ == active) {
        return;
    }

    fallback_active_ = active;
    for (auto& region: regions_) {
        if (region.role == RegionRole::OneWayFallback) {
            region.blocked = false;
        } else if (region.role == RegionRole::PreferredFallback) {
            region.blocked = active;
        }
        addExtraBounds(region.min_x, region.min_y, region.max_x, region.max_y);
    }

    if (node) {
        RCLCPP_WARN(
            node->get_logger(),
            "DirectionalForbiddenLayer: fallback route %s; fluctuate2/4=%s, fluctuate1/3 one-way=%s",
            active ? "enabled" : "cleared",
            active ? "blocked" : "open",
            active ? "disabled" : "restored"
        );
    }
}

void DirectionalForbiddenLayer::setRouteFailedLocked(bool failed) {
    if (route_failed_ == failed && route_failed_pub_) {
        return;
    }
    route_failed_ = failed;
    if (route_failed_pub_) {
        std_msgs::msg::Bool msg;
        msg.data = failed;
        route_failed_pub_->publish(msg);
    }
}

bool DirectionalForbiddenLayer::loadRegion(const std::string& param_name, const std::string& name) {
    auto node = node_.lock();
    if (!node) {
        return false;
    }

    std::vector<double> data;
    (void)node->get_parameter(name_ + "." + param_name, data);
    if (data.size() < 6 || data.size() % 2 != 0) {
        RCLCPP_WARN(
            node->get_logger(),
            "DirectionalForbiddenLayer: invalid %s, need even size >= 6",
            param_name.c_str()
        );
        return false;
    }

    ForbiddenRegion region;
    region.name = name;
    region.min_x = std::numeric_limits<double>::max();
    region.min_y = std::numeric_limits<double>::max();
    region.max_x = std::numeric_limits<double>::lowest();
    region.max_y = std::numeric_limits<double>::lowest();

    double sum_y = 0.0;
    for (size_t i = 0; i < data.size(); i += 2) {
        const double x = data[i];
        const double y = data[i + 1];
        region.polygon.push_back({ x, y });
        sum_y += y;
        region.min_x = std::min(region.min_x, x);
        region.min_y = std::min(region.min_y, y);
        region.max_x = std::max(region.max_x, x);
        region.max_y = std::max(region.max_y, y);
    }
    region.center_y = sum_y / static_cast<double>(region.polygon.size());
    regions_.push_back(region);

    RCLCPP_INFO(
        node->get_logger(),
        "DirectionalForbiddenLayer: loaded %s with %zu vertices, center_y=%.2f",
        name.c_str(),
        region.polygon.size(),
        region.center_y
    );
    return true;
}

bool DirectionalForbiddenLayer::pathHasForbiddenTraversal(
    const nav_msgs::msg::Path& path,
    const ForbiddenRegion& region
) const {
    bool found = false;
    double first_y = 0.0;
    double last_y = 0.0;

    for (const auto& pose: path.poses) {
        const double x = pose.pose.position.x;
        const double y = pose.pose.position.y;
        if (!isPointInPolygon(x, y, region.polygon)) {
            continue;
        }

        if (!found) {
            first_y = y;
            found = true;
        }
        last_y = y;
    }

    if (!found) {
        return false;
    }

    const double side = (region.center_y >= 0.0) ? 1.0 : -1.0;
    return side * (last_y - first_y) > min_region_delta_y_;
}

void DirectionalForbiddenLayer::updateBounds(
    double origin_x,
    double origin_y,
    double /*origin_yaw*/,
    double* min_x,
    double* min_y,
    double* max_x,
    double* max_y
) {
    if (!enabled_) {
        return;
    }

    std::lock_guard<std::mutex> lk(mutex_);
    updateDwellStateLocked(origin_x, origin_y);
    useExtraBounds(min_x, min_y, max_x, max_y);
    for (const auto& region: regions_) {
        if (!region.blocked) {
            continue;
        }
        *min_x = std::min(*min_x, region.min_x);
        *min_y = std::min(*min_y, region.min_y);
        *max_x = std::max(*max_x, region.max_x);
        *max_y = std::max(*max_y, region.max_y);
    }
}

void DirectionalForbiddenLayer::updateCosts(
    nav2_costmap_2d::Costmap2D& master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j
) {
    if (!enabled_) {
        return;
    }

    std::lock_guard<std::mutex> lk(mutex_);
    for (const auto& region: regions_) {
        if (!region.blocked) {
            continue;
        }

        unsigned int min_mx = 0, min_my = 0, max_mx = 0, max_my = 0;
        if (!master_grid.worldToMap(region.min_x, region.min_y, min_mx, min_my) ||
            !master_grid.worldToMap(region.max_x, region.max_y, max_mx, max_my)) {
            continue;
        }

        const int i0 = std::max(min_i, static_cast<int>(std::min(min_mx, max_mx)));
        const int j0 = std::max(min_j, static_cast<int>(std::min(min_my, max_my)));
        const int i1 = std::min(max_i, static_cast<int>(std::max(min_mx, max_mx)) + 1);
        const int j1 = std::min(max_j, static_cast<int>(std::max(min_my, max_my)) + 1);

        for (int j = j0; j < j1; ++j) {
            for (int i = i0; i < i1; ++i) {
                double wx = 0.0, wy = 0.0;
                master_grid.mapToWorld(static_cast<unsigned int>(i), static_cast<unsigned int>(j), wx, wy);
                if (isPointInPolygon(wx, wy, region.polygon)) {
                    master_grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE);
                }
            }
        }
    }
}

bool DirectionalForbiddenLayer::isPointInPolygon(
    double x,
    double y,
    const std::vector<std::pair<double, double>>& polygon
) {
    bool inside = false;
    const size_t n = polygon.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const double xi = polygon[i].first;
        const double yi = polygon[i].second;
        const double xj = polygon[j].first;
        const double yj = polygon[j].second;
        if (((yi > y) != (yj > y)) &&
            (x < (xj - xi) * (y - yi) / (yj - yi) + xi)) {
            inside = !inside;
        }
    }
    return inside;
}

bool DirectionalForbiddenLayer::isNearAnyRegion(double x, double y, RegionRole role) const {
    for (const auto& region: regions_) {
        if (region.role != role) {
            continue;
        }
        if (isPointInPolygon(x, y, region.polygon)
            || distanceToPolygon(x, y, region.polygon) <= dwell_near_distance_)
        {
            return true;
        }
    }
    return false;
}

double DirectionalForbiddenLayer::distanceToPolygon(
    double x,
    double y,
    const std::vector<std::pair<double, double>>& polygon
) {
    if (polygon.size() < 3) {
        return std::numeric_limits<double>::max();
    }

    double min_dist = std::numeric_limits<double>::max();
    const size_t n = polygon.size();
    for (size_t i = 0; i < n; ++i) {
        const size_t j = (i + 1) % n;
        const double x1 = polygon[i].first;
        const double y1 = polygon[i].second;
        const double x2 = polygon[j].first;
        const double y2 = polygon[j].second;
        const double dx = x2 - x1;
        const double dy = y2 - y1;
        const double denom = dx * dx + dy * dy;
        const double t = denom > 1e-9
            ? std::max(0.0, std::min(1.0, ((x - x1) * dx + (y - y1) * dy) / denom))
            : 0.0;
        const double closest_x = x1 + t * dx;
        const double closest_y = y1 + t * dy;
        min_dist = std::min(min_dist, std::hypot(x - closest_x, y - closest_y));
    }
    return min_dist;
}

} // namespace click_obstacles_layer

PLUGINLIB_EXPORT_CLASS(
    click_obstacles_layer::DirectionalForbiddenLayer,
    nav2_costmap_2d::Layer
)
