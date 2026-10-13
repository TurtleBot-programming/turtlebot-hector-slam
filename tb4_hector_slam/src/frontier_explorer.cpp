// Frontier-based autonomous exploration on top of Hector SLAM + Nav2.
//
// Hector SLAM publishes /map and map -> odom. This node finds the nearest
// frontier (a known-free cell next to unknown space), reachable through free
// space from the robot, and sends it to Nav2 as a NavigateToPose goal.
// It repeats until no frontier is left, i.e. the map is closed.

#include <algorithm>
#include <cmath>
#include <deque>
#include <memory>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

struct Point2D { double x, y; };

class FrontierExplorer : public rclcpp::Node
{
public:
  FrontierExplorer()
  : Node("frontier_explorer"), tf_buffer_(get_clock()), tf_listener_(tf_buffer_)
  {
    map_frame_ = declare_parameter("map_frame", "map");
    base_frame_ = declare_parameter("base_frame", "base_link");
    min_frontier_cells_ = declare_parameter("min_frontier_cells", 8);
    clearance_ = declare_parameter("goal_clearance", 0.3);
    blacklist_radius_ = declare_parameter("blacklist_radius", 0.5);
    min_goal_dist_ = declare_parameter("min_goal_distance", 0.5);
    goal_timeout_ = declare_parameter("goal_timeout", 60.0);
    empty_limit_ = declare_parameter("empty_checks_to_finish", 5);

    // Hector publishes the map transient_local, so late joiners still get it.
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) {map_ = m;});

    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {step();});
    RCLCPP_INFO(get_logger(), "Frontier explorer started; waiting for /map and Nav2.");
  }

private:
  bool robotPose(Point2D & p)
  {
    try {
      auto t = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      p = {t.transform.translation.x, t.transform.translation.y};
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  bool blacklisted(const Point2D & g) const
  {
    for (const auto & b : blacklist_) {
      if (std::hypot(b.x - g.x, b.y - g.y) < blacklist_radius_) {return true;}
    }
    return false;
  }

  // Goal must be free and at least `clearance_` from any occupied cell.
  bool hasClearance(int cx, int cy) const
  {
    const auto & m = *map_;
    int r = static_cast<int>(std::ceil(clearance_ / m.info.resolution));
    int w = m.info.width, h = m.info.height;
    for (int dy = -r; dy <= r; ++dy) {
      for (int dx = -r; dx <= r; ++dx) {
        int x = cx + dx, y = cy + dy;
        if (x < 0 || y < 0 || x >= w || y >= h) {continue;}
        if (m.data[y * w + x] >= 50) {return false;}
      }
    }
    return true;
  }

  // Returns the best frontier goal, or false if none.
  bool findFrontier(const Point2D & robot, Point2D & goal)
  {
    const auto & m = *map_;
    const int w = m.info.width, h = m.info.height;
    const double res = m.info.resolution;
    const double ox = m.info.origin.position.x, oy = m.info.origin.position.y;
    auto idx = [w](int x, int y) {return y * w + x;};

    int sx = static_cast<int>((robot.x - ox) / res);
    int sy = static_cast<int>((robot.y - oy) / res);
    if (sx < 0 || sy < 0 || sx >= w || sy >= h) {return false;}

    // BFS through free cells from the robot: only reachable frontiers count.
    std::vector<uint8_t> seen(w * h, 0), is_frontier(w * h, 0);
    std::deque<std::pair<int, int>> q;
    q.push_back({sx, sy});
    seen[idx(sx, sy)] = 1;
    const int d4[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
    std::vector<std::pair<int, int>> frontier_cells;
    while (!q.empty()) {
      auto [x, y] = q.front();
      q.pop_front();
      bool near_unknown = false;
      for (auto & d : d4) {
        int nx = x + d[0], ny = y + d[1];
        if (nx < 0 || ny < 0 || nx >= w || ny >= h) {continue;}
        int8_t v = m.data[idx(nx, ny)];
        if (v < 0) {near_unknown = true;}
        if (!seen[idx(nx, ny)] && v >= 0 && v < 50) {
          seen[idx(nx, ny)] = 1;
          q.push_back({nx, ny});
        }
      }
      if (near_unknown && m.data[idx(x, y)] >= 0) {
        is_frontier[idx(x, y)] = 1;
        frontier_cells.push_back({x, y});
      }
    }

    // Cluster frontier cells (8-connected) and score each cluster.
    std::vector<uint8_t> used(w * h, 0);
    double best = 1e18;
    bool found = false;
    for (auto & start : frontier_cells) {
      if (used[idx(start.first, start.second)]) {continue;}
      std::vector<std::pair<int, int>> cluster;
      std::deque<std::pair<int, int>> cq;
      cq.push_back(start);
      used[idx(start.first, start.second)] = 1;
      while (!cq.empty()) {
        auto c = cq.front();
        cq.pop_front();
        cluster.push_back(c);
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            int nx = c.first + dx, ny = c.second + dy;
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) {continue;}
            if (is_frontier[idx(nx, ny)] && !used[idx(nx, ny)]) {
              used[idx(nx, ny)] = 1;
              cq.push_back({nx, ny});
            }
          }
        }
      }
      if (static_cast<int>(cluster.size()) < min_frontier_cells_) {continue;}

      // Goal = cluster cell closest to the cluster centroid.
      double mx = 0, my = 0;
      for (auto & c : cluster) {mx += c.first; my += c.second;}
      mx /= cluster.size(); my /= cluster.size();
      std::pair<int, int> pick = cluster.front();
      double pd = 1e18;
      for (auto & c : cluster) {
        double d = std::hypot(c.first - mx, c.second - my);
        if (d < pd && hasClearance(c.first, c.second)) {pd = d; pick = c;}
      }
      if (pd >= 1e18) {continue;}
      Point2D g{ox + (pick.first + 0.5) * res, oy + (pick.second + 0.5) * res};
      double dist = std::hypot(g.x - robot.x, g.y - robot.y);
      if (dist < min_goal_dist_ || blacklisted(g)) {continue;}
      if (dist < best) {best = dist; goal = g; found = true;}
    }
    return found;
  }

  void step()
  {
    if (!map_) {return;}
    if (goal_active_) {
      if ((now() - goal_sent_).seconds() > goal_timeout_) {
        RCLCPP_WARN(get_logger(), "Goal timed out; cancelling and blacklisting it.");
        blacklist_.push_back(current_goal_);
        client_->async_cancel_all_goals();
        goal_active_ = false;
      }
      return;
    }
    if (!client_->action_server_is_ready()) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for navigate_to_pose...");
      return;
    }
    Point2D robot;
    if (!robotPose(robot)) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for %s -> %s TF...",
        map_frame_.c_str(), base_frame_.c_str());
      return;
    }
    Point2D goal;
    if (!findFrontier(robot, goal)) {
      if (++empty_count_ >= empty_limit_) {
        RCLCPP_INFO(get_logger(),
          "No reachable frontiers left: exploration complete. Save the map with map_saver_cli.");
        timer_->cancel();
      }
      return;
    }
    empty_count_ = 0;
    send(robot, goal);
  }

  void send(const Point2D & robot, const Point2D & goal)
  {
    NavigateToPose::Goal g;
    g.pose.header.frame_id = map_frame_;
    g.pose.header.stamp = now();
    g.pose.pose.position.x = goal.x;
    g.pose.pose.position.y = goal.y;
    tf2::Quaternion qt;
    qt.setRPY(0, 0, std::atan2(goal.y - robot.y, goal.x - robot.x));
    g.pose.pose.orientation.x = qt.x();
    g.pose.pose.orientation.y = qt.y();
    g.pose.pose.orientation.z = qt.z();
    g.pose.pose.orientation.w = qt.w();

    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    opts.goal_response_callback = [this](GoalHandle::SharedPtr h) {
        if (!h) {
          RCLCPP_WARN(get_logger(), "Goal rejected; blacklisting.");
          blacklist_.push_back(current_goal_);
          goal_active_ = false;
        }
      };
    opts.result_callback = [this](const GoalHandle::WrappedResult & r) {
        if (r.code != rclcpp_action::ResultCode::SUCCEEDED) {
          RCLCPP_WARN(get_logger(), "Goal not reached (code %d); blacklisting.",
            static_cast<int>(r.code));
          blacklist_.push_back(current_goal_);
        }
        goal_active_ = false;
      };
    current_goal_ = goal;
    goal_active_ = true;
    goal_sent_ = now();
    RCLCPP_INFO(get_logger(), "Exploring towards frontier (%.2f, %.2f)", goal.x, goal.y);
    client_->async_send_goal(g, opts);
  }

  std::string map_frame_, base_frame_;
  int min_frontier_cells_, empty_limit_, empty_count_{0};
  double clearance_, blacklist_radius_, min_goal_dist_, goal_timeout_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  std::vector<Point2D> blacklist_;
  Point2D current_goal_{0, 0};
  bool goal_active_{false};
  rclcpp::Time goal_sent_{0, 0, RCL_ROS_TIME};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontierExplorer>());
  rclcpp::shutdown();
  return 0;
}
