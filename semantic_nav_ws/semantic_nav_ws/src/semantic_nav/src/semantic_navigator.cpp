/**
 * semantic_navigator.cpp
 *
 * Semantic Navigation Node — ROS2 / Nav2
 * ----------------------------------------
 * Maintains a registry of named semantic waypoints (e.g. "kitchen",
 * "office_desk", "charging_dock") and drives the robot through them
 * sequentially using the Nav2 NavigateToPose action.
 *
 * Key features demonstrated:
 *  - Nav2 action client with full feedback / result handling
 *  - RViz marker publishing for semantic waypoint visualization
 *  - Goal cancellation and re-planning on obstacle detection
 *  - TF2 transform usage for pose queries
 */

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/string.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

using namespace std::chrono_literals;
using NavigateToPose    = nav2_msgs::action::NavigateToPose;
using GoalHandleNav     = rclcpp_action::ClientGoalHandle<NavigateToPose>;

// ── Semantic waypoint descriptor ─────────────────────────────────────────────
struct SemanticWaypoint {
  std::string label;
  double x, y, yaw;          // map-frame pose
  std::string description;
};

// ── Node ─────────────────────────────────────────────────────────────────────
class SemanticNavigator : public rclcpp::Node
{
public:
  SemanticNavigator()
  : Node("semantic_navigator"),
    current_goal_idx_(0),
    mission_active_(false)
  {
    // ── Parameters ──────────────────────────────────────────────────────────
    declare_parameter("loop_mission", false);
    declare_parameter("goal_tolerance_m", 0.25);

    loop_mission_    = get_parameter("loop_mission").as_bool();
    goal_tolerance_  = get_parameter("goal_tolerance_m").as_double();

    // ── Populate semantic map ────────────────────────────────────────────────
    // Coordinates tuned for turtlebot3_world (3 × 3 m arena).
    // Swap these out for your custom Gazebo world.
    register_waypoint({"entrance",     -1.5,  1.5,  0.0,  "Main entrance door"});
    register_waypoint({"office_desk",   1.5,  1.5,  1.57, "Office workstation"});
    register_waypoint({"storage_room",  1.5, -1.5, -1.57, "Storage / supply room"});
    register_waypoint({"charging_dock",-1.5, -1.5,  3.14, "Robot charging station"});

    // ── TF2 ─────────────────────────────────────────────────────────────────
    tf_buffer_   = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // ── Nav2 action client ──────────────────────────────────────────────────
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(
      this, "navigate_to_pose");

    // ── Publishers ──────────────────────────────────────────────────────────
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "semantic_waypoints", rclcpp::QoS(10).transient_local());

    status_pub_ = create_publisher<std_msgs::msg::String>(
      "semantic_nav/status", 10);

    // ── Subscriber: external command (e.g. "go_to:kitchen") ─────────────────
    cmd_sub_ = create_subscription<std_msgs::msg::String>(
      "semantic_nav/command", 10,
      [this](const std_msgs::msg::String::SharedPtr msg){ on_command(msg); });

    // ── Obstacle-blocked signal from obstacle_monitor ───────────────────────
    obstacle_sub_ = create_subscription<std_msgs::msg::String>(
      "obstacle_monitor/alert", 10,
      [this](const std_msgs::msg::String::SharedPtr msg){ on_obstacle_alert(msg); });

    // ── Publish static RViz markers ──────────────────────────────────────────
    publish_waypoint_markers();

    // ── Wait for Nav2 then start mission ────────────────────────────────────
    startup_timer_ = create_wall_timer(2s, [this](){
      startup_timer_->cancel();
      wait_for_nav2_and_start();
    });

    RCLCPP_INFO(get_logger(), "SemanticNavigator ready. %zu waypoints registered.",
      waypoints_.size());
  }

private:
  // ── State ──────────────────────────────────────────────────────────────────
  std::vector<SemanticWaypoint> waypoints_;
  std::unordered_map<std::string, size_t> label_index_;
  size_t  current_goal_idx_;
  bool    mission_active_;
  bool    loop_mission_;
  double  goal_tolerance_;

  // ── ROS handles ────────────────────────────────────────────────────────────
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr obstacle_sub_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  GoalHandleNav::SharedPtr current_goal_handle_;

  // ── Helpers ────────────────────────────────────────────────────────────────
  void register_waypoint(SemanticWaypoint wp) {
    label_index_[wp.label] = waypoints_.size();
    waypoints_.push_back(std::move(wp));
  }

  geometry_msgs::msg::PoseStamped make_pose(const SemanticWaypoint & wp) {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.stamp    = now();
    ps.header.frame_id = "map";
    ps.pose.position.x = wp.x;
    ps.pose.position.y = wp.y;
    ps.pose.position.z = 0.0;
    // Convert yaw to quaternion (simplified: only z,w)
    ps.pose.orientation.z = std::sin(wp.yaw * 0.5);
    ps.pose.orientation.w = std::cos(wp.yaw * 0.5);
    return ps;
  }

  void publish_status(const std::string & msg) {
    std_msgs::msg::String s;
    s.data = msg;
    status_pub_->publish(s);
    RCLCPP_INFO(get_logger(), "[STATUS] %s", msg.c_str());
  }

  // ── Nav2 startup ───────────────────────────────────────────────────────────
  void wait_for_nav2_and_start() {
    if (!nav_client_->wait_for_action_server(5s)) {
      RCLCPP_WARN(get_logger(), "Nav2 not yet available — retrying in 2 s.");
      startup_timer_ = create_wall_timer(2s, [this](){
        startup_timer_->cancel();
        wait_for_nav2_and_start();
      });
      return;
    }
    RCLCPP_INFO(get_logger(), "Nav2 action server found. Starting mission.");
    mission_active_ = true;
    send_next_goal();
  }

  // ── Mission sequencer ─────────────────────────────────────────────────────
  void send_next_goal() {
    if (!mission_active_ || current_goal_idx_ >= waypoints_.size()) {
      if (loop_mission_) {
        current_goal_idx_ = 0;
        RCLCPP_INFO(get_logger(), "Mission loop restarting.");
        send_next_goal();
      } else {
        publish_status("MISSION_COMPLETE");
      }
      return;
    }

    const auto & wp = waypoints_[current_goal_idx_];
    publish_status("NAVIGATING_TO:" + wp.label + " — " + wp.description);

    auto goal_msg = NavigateToPose::Goal();
    goal_msg.pose = make_pose(wp);

    auto send_opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();

    send_opts.goal_response_callback =
      [this, label = wp.label](const GoalHandleNav::SharedPtr & gh) {
        if (!gh) {
          RCLCPP_ERROR(get_logger(), "Goal to '%s' rejected!", label.c_str());
          advance_to_next_goal();
        } else {
          current_goal_handle_ = gh;
          RCLCPP_INFO(get_logger(), "Goal accepted: '%s'", label.c_str());
        }
      };

    send_opts.feedback_callback =
      [this](GoalHandleNav::SharedPtr,
             const std::shared_ptr<const NavigateToPose::Feedback> fb) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
          "Distance remaining: %.2f m", fb->distance_remaining);
      };

    send_opts.result_callback =
      [this](const GoalHandleNav::WrappedResult & result) {
        switch (result.code) {
          case rclcpp_action::ResultCode::SUCCEEDED:
            publish_status("REACHED:" + waypoints_[current_goal_idx_].label);
            advance_to_next_goal();
            break;
          case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_WARN(get_logger(), "Goal aborted — skipping waypoint.");
            advance_to_next_goal();
            break;
          case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_INFO(get_logger(), "Goal canceled.");
            break;
          default:
            RCLCPP_ERROR(get_logger(), "Unknown result code.");
            break;
        }
      };

    nav_client_->async_send_goal(goal_msg, send_opts);
  }

  void advance_to_next_goal() {
    ++current_goal_idx_;
    // Brief pause so the user can see the waypoint reached in demo
    auto t = create_wall_timer(1500ms, [this](){ startup_timer_->cancel(); send_next_goal(); });
    (void)t;
  }

  // ── External command handler ───────────────────────────────────────────────
  void on_command(const std_msgs::msg::String::SharedPtr msg) {
    const std::string & cmd = msg->data;

    if (cmd.rfind("go_to:", 0) == 0) {
      std::string target = cmd.substr(6);
      auto it = label_index_.find(target);
      if (it == label_index_.end()) {
        RCLCPP_WARN(get_logger(), "Unknown semantic label: '%s'", target.c_str());
        return;
      }
      // Cancel current goal and jump
      if (current_goal_handle_) nav_client_->async_cancel_goal(current_goal_handle_);
      current_goal_idx_ = it->second;
      mission_active_   = true;
      send_next_goal();

    } else if (cmd == "pause") {
      mission_active_ = false;
      if (current_goal_handle_) nav_client_->async_cancel_goal(current_goal_handle_);
      publish_status("MISSION_PAUSED");

    } else if (cmd == "resume") {
      mission_active_ = true;
      send_next_goal();

    } else {
      RCLCPP_WARN(get_logger(), "Unknown command: '%s'", cmd.c_str());
    }
  }

  // ── Obstacle alert handler ─────────────────────────────────────────────────
  void on_obstacle_alert(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_WARN(get_logger(), "Obstacle alert: %s", msg->data.c_str());
    publish_status("OBSTACLE_DETECTED — Nav2 replanning...");
    // Nav2's DWB / MPPI controller handles low-level avoidance.
    // For a hard-blocked path, cancel and re-send the goal to force replanning.
    if (current_goal_handle_ && msg->data == "PATH_BLOCKED") {
      nav_client_->async_cancel_goal(current_goal_handle_);
      auto t = create_wall_timer(2s, [this](){ startup_timer_->cancel(); send_next_goal(); });
      (void)t;
    }
  }

  // ── RViz Markers ──────────────────────────────────────────────────────────
  void publish_waypoint_markers() {
    visualization_msgs::msg::MarkerArray arr;
    int id = 0;

    for (const auto & wp : waypoints_) {
      // Sphere marker
      visualization_msgs::msg::Marker sphere;
      sphere.header.frame_id = "map";
      sphere.header.stamp    = now();
      sphere.ns              = "semantic_waypoints";
      sphere.id              = id++;
      sphere.type            = visualization_msgs::msg::Marker::SPHERE;
      sphere.action          = visualization_msgs::msg::Marker::ADD;
      sphere.pose.position.x = wp.x;
      sphere.pose.position.y = wp.y;
      sphere.pose.position.z = 0.1;
      sphere.pose.orientation.w = 1.0;
      sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.2;
      sphere.color.r = 0.2f; sphere.color.g = 0.8f;
      sphere.color.b = 0.2f; sphere.color.a = 0.9f;
      sphere.lifetime = rclcpp::Duration(0, 0);
      arr.markers.push_back(sphere);

      // Text label
      visualization_msgs::msg::Marker text;
      text.header  = sphere.header;
      text.ns      = "semantic_labels";
      text.id      = id++;
      text.type    = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action  = visualization_msgs::msg::Marker::ADD;
      text.pose.position.x = wp.x;
      text.pose.position.y = wp.y;
      text.pose.position.z = 0.45;
      text.pose.orientation.w = 1.0;
      text.scale.z = 0.18;
      text.color.r = text.color.g = text.color.b = 1.0f;
      text.color.a = 1.0f;
      text.text    = wp.label;
      text.lifetime = rclcpp::Duration(0, 0);
      arr.markers.push_back(text);
    }
    marker_pub_->publish(arr);
  }
};

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SemanticNavigator>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
