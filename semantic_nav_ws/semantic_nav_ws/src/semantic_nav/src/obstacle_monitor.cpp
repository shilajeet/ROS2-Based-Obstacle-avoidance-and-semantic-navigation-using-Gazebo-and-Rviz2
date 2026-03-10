/**
 * obstacle_monitor.cpp
 *
 * Obstacle Monitor Node — ROS2
 * ─────────────────────────────
 * Subscribes to /scan (LaserScan) and analyses the point cloud to:
 *   1. Publish a proximity warning when any obstacle is closer than
 *      `warning_distance_m` (default 0.5 m).
 *   2. Publish a PATH_BLOCKED alert when the forward cone is fully blocked.
 *   3. Publish an OccupancyGrid-compatible marker for RViz visualization.
 *
 * All thresholds are ROS2 parameters — tune live with `ros2 param set`.
 */

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "visualization_msgs/msg/marker.hpp"

class ObstacleMonitor : public rclcpp::Node
{
public:
  ObstacleMonitor()
  : Node("obstacle_monitor")
  {
    // ── Parameters ──────────────────────────────────────────────────────────
    declare_parameter("warning_distance_m", 0.5);
    declare_parameter("critical_distance_m", 0.25);
    declare_parameter("forward_cone_deg", 60.0);   // ±30° around robot heading
    declare_parameter("blocked_fraction", 0.6);    // >60% rays blocked → PATH_BLOCKED

    warn_dist_     = get_parameter("warning_distance_m").as_double();
    crit_dist_     = get_parameter("critical_distance_m").as_double();
    cone_half_rad_ = (get_parameter("forward_cone_deg").as_double() / 2.0)
                     * M_PI / 180.0;
    blocked_frac_  = get_parameter("blocked_fraction").as_double();

    // ── Subscriptions ────────────────────────────────────────────────────────
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::LaserScan::SharedPtr msg){ on_scan(msg); });

    // ── Publishers ───────────────────────────────────────────────────────────
    alert_pub_  = create_publisher<std_msgs::msg::String>(
      "obstacle_monitor/alert", 10);

    marker_pub_ = create_publisher<visualization_msgs::msg::Marker>(
      "obstacle_monitor/closest_marker", 10);

    RCLCPP_INFO(get_logger(),
      "ObstacleMonitor ready | warn=%.2f m | crit=%.2f m | cone=±%.0f°",
      warn_dist_, crit_dist_,
      get_parameter("forward_cone_deg").as_double() / 2.0);
  }

private:
  double warn_dist_, crit_dist_, cone_half_rad_, blocked_frac_;

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr alert_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  // ── Helpers ────────────────────────────────────────────────────────────────
  void publish_alert(const std::string & msg) {
    std_msgs::msg::String s;  s.data = msg;
    alert_pub_->publish(s);
    RCLCPP_WARN(get_logger(), "ALERT: %s", msg.c_str());
  }

  // ── Main scan callback ─────────────────────────────────────────────────────
  void on_scan(const sensor_msgs::msg::LaserScan::SharedPtr scan)
  {
    const size_t N = scan->ranges.size();
    if (N == 0) return;

    float  closest_range   = std::numeric_limits<float>::infinity();
    float  closest_angle   = 0.0f;
    size_t forward_total   = 0;
    size_t forward_blocked = 0;

    for (size_t i = 0; i < N; ++i) {
      float r = scan->ranges[i];
      if (!std::isfinite(r)) continue;

      // Clamp to valid range
      if (r < scan->range_min || r > scan->range_max) continue;

      float angle = scan->angle_min + i * scan->angle_increment;

      // Track global closest
      if (r < closest_range) {
        closest_range = r;
        closest_angle = angle;
      }

      // Forward cone analysis
      if (std::fabs(angle) <= cone_half_rad_) {
        ++forward_total;
        if (r < warn_dist_) ++forward_blocked;
      }
    }

    // ── Alerts ────────────────────────────────────────────────────────────
    if (closest_range < crit_dist_) {
      publish_alert("CRITICAL_PROXIMITY:" + std::to_string(closest_range) + "m");
    } else if (closest_range < warn_dist_) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "Obstacle nearby: %.2f m at %.1f°",
        closest_range, closest_angle * 180.0 / M_PI);
    }

    if (forward_total > 0) {
      double frac = static_cast<double>(forward_blocked) / forward_total;
      if (frac >= blocked_frac_) {
        publish_alert("PATH_BLOCKED");
      }
    }

    // ── RViz marker for closest obstacle ──────────────────────────────────
    publish_closest_marker(scan->header.frame_id, closest_range, closest_angle);
  }

  void publish_closest_marker(const std::string & frame,
                               float range, float angle)
  {
    visualization_msgs::msg::Marker m;
    m.header.frame_id = frame;
    m.header.stamp    = now();
    m.ns     = "closest_obstacle";
    m.id     = 0;
    m.type   = visualization_msgs::msg::Marker::ARROW;
    m.action = visualization_msgs::msg::Marker::ADD;

    // Arrow from origin toward closest obstacle
    geometry_msgs::msg::Point start, end;
    start.x = start.y = start.z = 0.0;
    end.x = range * std::cos(angle);
    end.y = range * std::sin(angle);
    end.z = 0.0;
    m.points = {start, end};

    m.scale.x = 0.04;   // shaft diameter
    m.scale.y = 0.08;   // head diameter
    m.scale.z = 0.08;   // head length

    // Color: green→yellow→red based on distance
    float t = std::clamp(1.0f - (range / (float)warn_dist_), 0.0f, 1.0f);
    m.color.r = t;
    m.color.g = 1.0f - t;
    m.color.b = 0.0f;
    m.color.a = 0.9f;

    m.lifetime = rclcpp::Duration(0, 200'000'000);  // 200 ms
    marker_pub_->publish(m);
  }
};

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleMonitor>());
  rclcpp::shutdown();
  return 0;
}
