#include "click_obstacles_layer/region_keepout_layer.hpp"

#include <algorithm>
#include <cmath>
#include <pluginlib/class_list_macros.hpp>
#include <stdexcept>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace click_obstacles_layer {

void RegionKeepoutLayer::onInitialize() {
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("polygon_frame", rclcpp::ParameterValue(std::string("map")));
    declareParameter("polygon", rclcpp::ParameterValue(std::vector<double> {}));
    declareParameter("centerline_width", rclcpp::ParameterValue(0.36));
    declareParameter("keepout_cost", rclcpp::ParameterValue(254));

    auto node = node_.lock();
    if (!node) {
        throw std::runtime_error("RegionKeepoutLayer: node expired");
    }

    (void)node->get_parameter("enabled", enabled_);
    (void)node->get_parameter("polygon_frame", polygon_frame_);
    (void)node->get_parameter("centerline_width", centerline_width_);

    int keepout_cost = static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE);
    (void)node->get_parameter("keepout_cost", keepout_cost);
    keepout_cost_ = static_cast<unsigned char>(
        std::clamp(keepout_cost, 0, static_cast<int>(nav2_costmap_2d::LETHAL_OBSTACLE)));

    std::vector<double> polygon_data;
    (void)node->get_parameter("polygon", polygon_data);
    polygon_.clear();
    if (polygon_data.size() >= 6 && polygon_data.size() % 2 == 0) {
        for (size_t i = 0; i < polygon_data.size(); i += 2) {
            polygon_.push_back({ polygon_data[i], polygon_data[i + 1] });
        }
    } else if (enabled_) {
        RCLCPP_WARN(
            node->get_logger(),
            "RegionKeepoutLayer polygon invalid: size=%zu, need even size >= 6",
            polygon_data.size());
    }

    current_ = true;
}

void RegionKeepoutLayer::reset() {
    current_ = true;
}

bool RegionKeepoutLayer::pointInPolygon(
    double x,
    double y,
    const std::vector<std::pair<double, double>>& polygon) const
{
    bool inside = false;
    const size_t n = polygon.size();
    for (size_t i = 0, j = n - 1; i < n; j = i++) {
        const double xi = polygon[i].first;
        const double yi = polygon[i].second;
        const double xj = polygon[j].first;
        const double yj = polygon[j].second;
        const double denom = yj - yi;
        if (std::fabs(denom) > 1e-12 && ((yi > y) != (yj > y)) &&
            (x < (xj - xi) * (y - yi) / denom + xi)) {
            inside = !inside;
        }
    }
    return inside;
}

bool RegionKeepoutLayer::transformPolygon(
    std::vector<std::pair<double, double>>& polygon_out) const
{
    polygon_out.clear();
    if (polygon_.size() < 3) {
        return false;
    }

    const std::string target_frame = layered_costmap_->getGlobalFrameID();
    if (polygon_frame_ == target_frame || polygon_frame_.empty()) {
        polygon_out = polygon_;
        return true;
    }

    geometry_msgs::msg::TransformStamped transform;
    try {
        transform = tf_->lookupTransform(
            target_frame,
            polygon_frame_,
            tf2::TimePointZero,
            tf2::durationFromSec(0.2));
    } catch (const tf2::TransformException& ex) {
        auto node = node_.lock();
        if (node) {
            RCLCPP_WARN_THROTTLE(
                node->get_logger(),
                *node->get_clock(),
                2000,
                "RegionKeepoutLayer failed to transform %s to %s: %s",
                polygon_frame_.c_str(),
                target_frame.c_str(),
                ex.what());
        }
        return false;
    }

    polygon_out.reserve(polygon_.size());
    for (const auto& point: polygon_) {
        geometry_msgs::msg::PointStamped source;
        source.header.frame_id = polygon_frame_;
        source.point.x = point.first;
        source.point.y = point.second;
        source.point.z = 0.0;

        geometry_msgs::msg::PointStamped target;
        tf2::doTransform(source, target, transform);
        polygon_out.push_back({ target.point.x, target.point.y });
    }
    return true;
}

RegionKeepoutLayer::Bounds RegionKeepoutLayer::computeBounds(
    const std::vector<std::pair<double, double>>& polygon) const
{
    Bounds bounds;
    if (polygon.empty()) {
        return bounds;
    }

    bounds.min_x = bounds.max_x = polygon.front().first;
    bounds.min_y = bounds.max_y = polygon.front().second;
    for (const auto& point: polygon) {
        bounds.min_x = std::min(bounds.min_x, point.first);
        bounds.max_x = std::max(bounds.max_x, point.first);
        bounds.min_y = std::min(bounds.min_y, point.second);
        bounds.max_y = std::max(bounds.max_y, point.second);
    }
    bounds.valid = true;
    return bounds;
}

bool RegionKeepoutLayer::outsideCenterCorridor(double x, double y, const Bounds& bounds) const {
    if (!bounds.valid) {
        return false;
    }

    const double half_width = std::max(0.01, centerline_width_ * 0.5);
    const double span_x = bounds.max_x - bounds.min_x;
    const double span_y = bounds.max_y - bounds.min_y;

    if (span_y >= span_x) {
        const double center_x = 0.5 * (bounds.min_x + bounds.max_x);
        return std::fabs(x - center_x) > half_width;
    }

    const double center_y = 0.5 * (bounds.min_y + bounds.max_y);
    return std::fabs(y - center_y) > half_width;
}

void RegionKeepoutLayer::updateBounds(
    double /*origin_x*/,
    double /*origin_y*/,
    double /*origin_yaw*/,
    double* min_x,
    double* min_y,
    double* max_x,
    double* max_y)
{
    if (!enabled_) {
        return;
    }

    std::vector<std::pair<double, double>> polygon;
    if (!transformPolygon(polygon)) {
        return;
    }

    const auto bounds = computeBounds(polygon);
    if (!bounds.valid) {
        return;
    }

    *min_x = std::min(*min_x, bounds.min_x);
    *min_y = std::min(*min_y, bounds.min_y);
    *max_x = std::max(*max_x, bounds.max_x);
    *max_y = std::max(*max_y, bounds.max_y);
}

void RegionKeepoutLayer::updateCosts(
    nav2_costmap_2d::Costmap2D& master_grid,
    int min_i,
    int min_j,
    int max_i,
    int max_j)
{
    if (!enabled_) {
        return;
    }

    std::vector<std::pair<double, double>> polygon;
    if (!transformPolygon(polygon)) {
        return;
    }

    const auto bounds = computeBounds(polygon);
    if (!bounds.valid) {
        return;
    }

    int min_mx = 0;
    int min_my = 0;
    int max_mx = 0;
    int max_my = 0;
    master_grid.worldToMapEnforceBounds(bounds.min_x, bounds.min_y, min_mx, min_my);
    master_grid.worldToMapEnforceBounds(bounds.max_x, bounds.max_y, max_mx, max_my);

    int start_x = std::max(min_i, std::min(min_mx, max_mx));
    int end_x = std::min(max_i, std::max(min_mx, max_mx) + 1);
    int start_y = std::max(min_j, std::min(min_my, max_my));
    int end_y = std::min(max_j, std::max(min_my, max_my) + 1);

    for (int y = start_y; y < end_y; ++y) {
        for (int x = start_x; x < end_x; ++x) {
            double wx = 0.0;
            double wy = 0.0;
            master_grid.mapToWorld(x, y, wx, wy);
            if (pointInPolygon(wx, wy, polygon) && outsideCenterCorridor(wx, wy, bounds)) {
                master_grid.setCost(x, y, keepout_cost_);
            }
        }
    }
}

} // namespace click_obstacles_layer

PLUGINLIB_EXPORT_CLASS(click_obstacles_layer::RegionKeepoutLayer, nav2_costmap_2d::Layer)
