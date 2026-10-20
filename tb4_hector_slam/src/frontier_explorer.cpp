// Frontier-based autonomous exploration on top of Hector SLAM + Nav2.
//
// Hector SLAM publishes /map and map -> odom. This node finds the nearest
// frontier (a known-free cell next to unknown space), reachable through free
// space from the robot, and sends it to Nav2 as a NavigateToPose goal.
// It repeats until no frontier is left, i.e. the map is closed.
//
// Multi-floor (ramp_* parameters set, floor_manager running): /map is the
// current floor's map and /floor says which floor that is. When a floor has
// no frontiers left, the robot drives to the entry of the nearest ramp that
// leads to an unexplored floor (Nav2), lines up with it, and climbs or
// descends it with a simple heading + cross-track controller on /cmd_vel_unstamped:
// to a horizontal 2D lidar a ramp looks like a wall, so Nav2 cannot plan
// onto it. It then explores that floor, and so on until every floor is done.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPose>;

struct Point2D { double x, y; };

// One direction of one ramp: drive from the entry pose along `yaw` for
// `length` metres to get from floor `from` to floor `to`.
struct RampTransition
{
  std::string name;
  int from, to;
  double x, y, yaw, length;
};

static double wrapAngle(double a) {return std::atan2(std::sin(a), std::cos(a));}

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
    climb_speed_ = declare_parameter("climb_speed", 0.2);
    ramp_width_ = declare_parameter("ramp_width", 1.4);
    loadRamps();

    // Hector publishes the map transient_local, so late joiners still get it.
    map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      "map", rclcpp::QoS(1).transient_local().reliable(),
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr m) {map_ = m;});

    client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
    timer_ = create_wall_timer(std::chrono::seconds(1), [this]() {step();});

    if (!ramps_.empty()) {
      auto latched = rclcpp::QoS(1).transient_local().reliable();
      floor_sub_ = create_subscription<std_msgs::msg::Int32>(
        "floor", latched, [this](std_msgs::msg::Int32::SharedPtr m) {
          if (m->data != floor_) {
            RCLCPP_INFO(get_logger(), "Now on floor %d.", m->data);
            floor_ = m->data;
            empty_count_ = 0;
          }
        });
      ramp_sub_ = create_subscription<std_msgs::msg::Bool>(
        "on_ramp", latched, [this](std_msgs::msg::Bool::SharedPtr m) {
          if (m->data && !on_ramp_) {on_ramp_since_ = now();}
          on_ramp_ = m->data;
        });
      cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel_unstamped", 10);
      control_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {control();});
    }
    RCLCPP_INFO(get_logger(), "Frontier explorer started (%zu ramp transitions); "
      "waiting for /map and Nav2.", ramps_.size());
  }

private:
  enum class State {Explore, ToRamp, Align, Climb, BackOff};

  // Inside the footprint of any ramp (entry pad to exit pad)? From the lower
  // floor the bottom of a ramp is below the scan plane and looks like free
  // floor, so frontiers there must be ignored or Nav2 drives onto the ramp.
  bool inRamp(const Point2D & p) const
  {
    for (const auto & r : ramps_) {
      const double ux = std::cos(r.yaw), uy = std::sin(r.yaw);
      const double along = (p.x - r.x) * ux + (p.y - r.y) * uy;
      const double cross = -(p.x - r.x) * uy + (p.y - r.y) * ux;
      if (along > -0.3 && along < r.length + 0.3 && std::fabs(cross) < ramp_width_ / 2 + 0.3) {
        return true;
      }
    }
    return false;
  }

  void loadRamps()
  {
    auto names = declare_parameter("ramp_names", std::vector<std::string>{});
    auto from = declare_parameter("ramp_from", std::vector<int64_t>{});
    auto to = declare_parameter("ramp_to", std::vector<int64_t>{});
    auto x = declare_parameter("ramp_x", std::vector<double>{});
    auto y = declare_parameter("ramp_y", std::vector<double>{});
    auto yaw = declare_parameter("ramp_yaw", std::vector<double>{});
    auto len = declare_parameter("ramp_length", std::vector<double>{});
    const size_t n = names.size();
    if (from.size() != n || to.size() != n || x.size() != n || y.size() != n ||
      yaw.size() != n || len.size() != n)
    {
      RCLCPP_ERROR(get_logger(), "ramp_* parameters differ in length; ignoring ramps.");
      return;
    }
    for (size_t i = 0; i < n; ++i) {
      ramps_.push_back({names[i], static_cast<int>(from[i]), static_cast<int>(to[i]),
          x[i], y[i], yaw[i], len[i]});
      floors_.insert(static_cast<int>(from[i]));
      floors_.insert(static_cast<int>(to[i]));
    }
  }

  bool robotPose(Point2D & p) {double yaw; return robotPose(p, yaw);}

  bool robotPose(Point2D & p, double & yaw)
  {
    try {
      auto t = tf_buffer_.lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
      p = {t.transform.translation.x, t.transform.translation.y};
      const auto & q = t.transform.rotation;
      yaw = std::atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z));
      return true;
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  // Floors share the map frame, so each floor keeps its own blacklist.
  std::vector<Point2D> & blacklist() {return blacklist_[floor_];}

  bool blacklisted(const Point2D & g)
  {
    for (const auto & b : blacklist()) {
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
      if (dist < min_goal_dist_ || blacklisted(g) || inRamp(g)) {continue;}
      if (dist < best) {best = dist; goal = g; found = true;}
    }
    return found;
  }

  void step()
  {
    if (!map_ || state_ == State::Align || state_ == State::Climb ||
      state_ == State::BackOff) {return;}
    if (goal_active_) {
      if ((now() - goal_sent_).seconds() > goal_timeout_) {
        client_->async_cancel_all_goals();
        ++goal_seq_;  // ignore the CANCELED result of this goal
        goal_active_ = false;
        if (state_ == State::ToRamp) {
          RCLCPP_WARN(get_logger(), "Could not reach ramp %s in time; skipping it.",
            ramps_[ramp_].name.c_str());
          failed_ramps_.insert(ramp_);
          state_ = State::Explore;
        } else {
          RCLCPP_WARN(get_logger(), "Goal timed out; cancelling and blacklisting it.");
          blacklist().push_back(current_goal_);
        }
      }
      return;
    }
    if (on_ramp_) {
      // On a ramp without a planned traversal (pushed or driven there by
      // mistake): back off it rather than wait forever.
      if (!ramps_.empty() && (now() - on_ramp_since_).seconds() > 5.0) {
        RCLCPP_WARN(get_logger(), "On a ramp outside a planned traversal; backing off it.");
        state_ = State::BackOff;
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
    if (!robotCellKnown(robot)) {
      // e.g. just arrived on a floor whose map has not been published yet
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for the map around the robot...");
      return;
    }
    Point2D goal;
    if (!findFrontier(robot, goal)) {
      if (++empty_count_ >= empty_limit_) {
        empty_count_ = 0;
        if (ramps_.empty()) {
          RCLCPP_INFO(get_logger(),
            "No reachable frontiers left: exploration complete. Save the map with map_saver_cli.");
          timer_->cancel();
          return;
        }
        if (!done_floors_.count(floor_)) {
          RCLCPP_INFO(get_logger(), "Floor %d explored.", floor_);
          done_floors_.insert(floor_);
        }
        if (!goToRamp(robot)) {
          bool all = std::all_of(floors_.begin(), floors_.end(),
              [this](int f) {return done_floors_.count(f) > 0;});
          RCLCPP_INFO(get_logger(), all ?
            "All floors explored: exploration complete. Save each floor's map with "
            "map_saver_cli -t /floor<N>/map." :
            "No usable ramp to the remaining floors: exploration stopped.");
          timer_->cancel();
        }
      }
      return;
    }
    empty_count_ = 0;
    send(robot, goal);
  }

  bool robotCellKnown(const Point2D & robot) const
  {
    const auto & m = *map_;
    int x = static_cast<int>((robot.x - m.info.origin.position.x) / m.info.resolution);
    int y = static_cast<int>((robot.y - m.info.origin.position.y) / m.info.resolution);
    if (x < 0 || y < 0 || x >= static_cast<int>(m.info.width) ||
      y >= static_cast<int>(m.info.height)) {return false;}
    return m.data[y * m.info.width + x] >= 0;
  }

  // Nearest ramp from this floor to a floor not explored yet -> Nav2 goal at
  // its entry pose.
  bool goToRamp(const Point2D & robot)
  {
    int best = -1;
    double best_d = 1e18;
    for (size_t i = 0; i < ramps_.size(); ++i) {
      const auto & r = ramps_[i];
      if (r.from != floor_ || done_floors_.count(r.to) || failed_ramps_.count(i)) {continue;}
      double d = std::hypot(r.x - robot.x, r.y - robot.y);
      if (d < best_d) {best_d = d; best = static_cast<int>(i);}
    }
    if (best < 0) {return false;}
    ramp_ = best;
    const auto & r = ramps_[ramp_];
    RCLCPP_INFO(get_logger(), "Heading to ramp %s (floor %d -> %d), entry (%.2f, %.2f).",
      r.name.c_str(), r.from, r.to, r.x, r.y);
    state_ = State::ToRamp;
    sendPose({r.x, r.y}, r.yaw);
    return true;
  }

  // Ramp traversal: turn to the ramp heading, then drive along the ramp
  // centre line until the floor manager reports the target floor.
  void control()
  {
    if (state_ == State::BackOff) {
      geometry_msgs::msg::Twist cmd;
      if (on_ramp_) {
        cmd.linear.x = -0.15;
      } else {
        state_ = State::Explore;
        empty_count_ = 0;
      }
      cmd_pub_->publish(cmd);
      return;
    }
    if (state_ != State::Align && state_ != State::Climb) {return;}
    const auto & r = ramps_[ramp_];
    Point2D p;
    double yaw;
    if (!robotPose(p, yaw)) {return;}
    geometry_msgs::msg::Twist cmd;
    if (state_ == State::Align) {
      double err = wrapAngle(r.yaw - yaw);
      if (std::fabs(err) < 0.03) {
        RCLCPP_INFO(get_logger(), "Driving ramp %s.", r.name.c_str());
        state_ = State::Climb;
        climb_start_ = now();
      } else {
        cmd.angular.z = std::clamp(1.5 * err, -0.5, 0.5);
      }
      cmd_pub_->publish(cmd);
      return;
    }
    const double ux = std::cos(r.yaw), uy = std::sin(r.yaw);
    const double along = (p.x - r.x) * ux + (p.y - r.y) * uy;
    const double cross = -(p.x - r.x) * uy + (p.y - r.y) * ux;   // + = left of the line
    const bool arrived = floor_ == r.to && !on_ramp_;
    const double limit = 3.0 * r.length / climb_speed_;
    if ((along >= r.length && arrived) || along >= r.length + 1.0 ||
      (now() - climb_start_).seconds() > limit)
    {
      cmd_pub_->publish(geometry_msgs::msg::Twist());
      state_ = State::Explore;
      empty_count_ = 0;
      if (arrived) {
        RCLCPP_INFO(get_logger(), "Ramp %s done; exploring floor %d.", r.name.c_str(), floor_);
      } else {
        RCLCPP_WARN(get_logger(), "Ramp %s: still on floor %d after %.1f m; skipping it.",
          r.name.c_str(), floor_, along);
        failed_ramps_.insert(ramp_);
      }
      return;
    }
    double heading = r.yaw - std::atan(1.5 * cross);
    cmd.linear.x = climb_speed_;
    cmd.angular.z = std::clamp(2.0 * wrapAngle(heading - yaw), -0.5, 0.5);
    cmd_pub_->publish(cmd);
  }

  void send(const Point2D & robot, const Point2D & goal)
  {
    RCLCPP_INFO(get_logger(), "Exploring towards frontier (%.2f, %.2f)", goal.x, goal.y);
    sendPose(goal, std::atan2(goal.y - robot.y, goal.x - robot.x));
  }

  void sendPose(const Point2D & goal, double yaw)
  {
    NavigateToPose::Goal g;
    g.pose.header.frame_id = map_frame_;
    g.pose.header.stamp = now();
    g.pose.pose.position.x = goal.x;
    g.pose.pose.position.y = goal.y;
    tf2::Quaternion qt;
    qt.setRPY(0, 0, yaw);
    g.pose.pose.orientation.x = qt.x();
    g.pose.pose.orientation.y = qt.y();
    g.pose.pose.orientation.z = qt.z();
    g.pose.pose.orientation.w = qt.w();

    // Callbacks of a goal we already gave up on (timeout) must not touch the
    // state of the goal sent after it.
    const uint64_t seq = ++goal_seq_;
    auto opts = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    opts.goal_response_callback = [this, seq](GoalHandle::SharedPtr h) {
        if (!h && seq == goal_seq_) {
          // bt_navigator advertises the action before it is activated and
          // rejects goals until then; that says nothing about the frontier.
          RCLCPP_WARN(get_logger(), "Goal rejected (Nav2 not active yet?); retrying.");
          goal_active_ = false;
          if (state_ == State::ToRamp) {state_ = State::Explore;}
        }
      };
    opts.result_callback = [this, seq](const GoalHandle::WrappedResult & r) {
        if (seq != goal_seq_) {return;}
        goal_active_ = false;
        const bool ok = r.code == rclcpp_action::ResultCode::SUCCEEDED;
        if (state_ == State::ToRamp) {
          if (ok) {
            state_ = State::Align;
          } else {
            RCLCPP_WARN(get_logger(), "Could not reach ramp %s (code %d); skipping it.",
              ramps_[ramp_].name.c_str(), static_cast<int>(r.code));
            failed_ramps_.insert(ramp_);
            state_ = State::Explore;
          }
        } else if (!ok) {
          RCLCPP_WARN(get_logger(), "Goal not reached (code %d); blacklisting.",
            static_cast<int>(r.code));
          blacklist().push_back(current_goal_);
        }
      };
    current_goal_ = goal;
    goal_active_ = true;
    goal_sent_ = now();
    client_->async_send_goal(g, opts);
  }

  std::string map_frame_, base_frame_;
  int min_frontier_cells_, empty_limit_, empty_count_{0};
  uint64_t goal_seq_{0};
  double clearance_, blacklist_radius_, min_goal_dist_, goal_timeout_, climb_speed_,
    ramp_width_;
  nav_msgs::msg::OccupancyGrid::SharedPtr map_;
  std::map<int, std::vector<Point2D>> blacklist_;   // per floor

  std::vector<RampTransition> ramps_;
  std::set<int> floors_{1}, done_floors_;
  std::set<size_t> failed_ramps_;
  int floor_{1};
  bool on_ramp_{false};
  State state_{State::Explore};
  size_t ramp_{0};
  rclcpp::Time climb_start_{0, 0, RCL_ROS_TIME}, on_ramp_since_{0, 0, RCL_ROS_TIME};
  Point2D current_goal_{0, 0};
  bool goal_active_{false};
  rclcpp::Time goal_sent_{0, 0, RCL_ROS_TIME};
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_, control_timer_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr floor_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ramp_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FrontierExplorer>());
  rclcpp::shutdown();
  return 0;
}
