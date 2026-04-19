/**
 * Scan Grid Mapper — builds a 2D occupancy grid from LaserScan data.
 *
 * Subscribes to LaserScan + TF, maintains a probabilistic log-odds grid,
 * and publishes nav_msgs/OccupancyGrid for Nav2 map_saver to capture.
 *
 * This is the missing piece in the mapping commissioning pipeline:
 *   pointcloud_to_laserscan -> /agv/scan -> scan_grid_mapper -> /agv/live_map
 *   -> map_saver_cli saves .pgm/.yaml -> map_server loads for navigation.
 */

#include <cmath>
#include <cstring>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
// Manual yaw extraction (avoids tf2::getYaw linker issues)
inline double yaw_from_quat(const geometry_msgs::msg::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

class ScanGridMapperNode : public rclcpp::Node
{
public:
  ScanGridMapperNode() : Node("scan_grid_mapper")
  {
    // Parameters
    declare_parameter("resolution", 0.05);
    declare_parameter("initial_width", 200);
    declare_parameter("initial_height", 200);
    declare_parameter("publish_rate_hz", 1.0);
    declare_parameter("map_frame", "map");
    declare_parameter("l_occupied", 0.85);
    declare_parameter("l_free", -0.4);
    declare_parameter("l_min", -5.0);
    declare_parameter("l_max", 5.0);
    declare_parameter("occupied_threshold", 0.65);
    declare_parameter("free_threshold", 0.35);
    declare_parameter("max_range", 8.0);
    declare_parameter("min_range", 0.3);
    declare_parameter("expand_margin_cells", 40);
    declare_parameter("ray_subsample", 2);
    declare_parameter("min_travel_distance", 0.025);
    declare_parameter("warmup_seconds", 5.0);
    declare_parameter("warmup_min_scans", 30);

    res_        = static_cast<float>(get_parameter("resolution").as_double());
    width_      = get_parameter("initial_width").as_int();
    height_     = get_parameter("initial_height").as_int();
    map_frame_  = get_parameter("map_frame").as_string();
    l_occ_      = static_cast<float>(get_parameter("l_occupied").as_double());
    l_free_     = static_cast<float>(get_parameter("l_free").as_double());
    l_min_      = static_cast<float>(get_parameter("l_min").as_double());
    l_max_      = static_cast<float>(get_parameter("l_max").as_double());
    occ_thresh_ = static_cast<float>(get_parameter("occupied_threshold").as_double());
    free_thresh_= static_cast<float>(get_parameter("free_threshold").as_double());
    max_range_  = static_cast<float>(get_parameter("max_range").as_double());
    min_range_  = static_cast<float>(get_parameter("min_range").as_double());
    expand_margin_     = get_parameter("expand_margin_cells").as_int();
    ray_subsample_     = std::max(1, static_cast<int>(get_parameter("ray_subsample").as_int()));
    min_travel_dist_sq_= get_parameter("min_travel_distance").as_double();
    min_travel_dist_sq_ *= min_travel_dist_sq_; // store squared to avoid sqrt
    warmup_seconds_    = get_parameter("warmup_seconds").as_double();
    warmup_min_scans_  = static_cast<int>(get_parameter("warmup_min_scans").as_int());

    initial_width_  = width_;
    initial_height_ = height_;

    // Dynamic origin: center initial grid on (0, 0)
    origin_x_ = -(width_ * res_) / 2.0;
    origin_y_ = -(height_ * res_) / 2.0;

    // Initialize grid (log-odds, all zeros = unknown)
    grid_.resize(static_cast<size_t>(width_) * height_, 0.0f);

    // Persistent publish buffer
    pub_data_.resize(grid_.size(), -1);
    full_rebuild_needed_ = true;
    reset_dirty_rect();

    // TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscriber
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", rclcpp::SensorDataQoS(),
      std::bind(&ScanGridMapperNode::scan_cb, this, std::placeholders::_1));

    // Publisher (transient local so late subscribers get the latest map)
    auto qos = rclcpp::QoS(1).transient_local().reliable();
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("live_map", qos);

    // Clear service (resets the grid centered on current robot position)
    clear_srv_ = create_service<std_srvs::srv::Empty>(
      "clear_map_srv",
      [this](const std::shared_ptr<std_srvs::srv::Empty::Request>,
             std::shared_ptr<std_srvs::srv::Empty::Response>) {
        auto center = get_robot_position();
        reset_grid(center.first, center.second);
        RCLCPP_INFO(get_logger(), "Map cleared (service), centered on (%.2f, %.2f)",
                    center.first, center.second);
      });

    // Clear via topic (used by dashboard backend)
    clear_sub_ = create_subscription<std_msgs::msg::Bool>(
      "clear_map", 10,
      [this](const std_msgs::msg::Bool::SharedPtr /*msg*/) {
        auto center = get_robot_position();
        reset_grid(center.first, center.second);
        RCLCPP_INFO(get_logger(), "Map cleared (topic), centered on (%.2f, %.2f)",
                    center.first, center.second);
      });

    // Publish timer
    double rate = get_parameter("publish_rate_hz").as_double();
    auto period = std::chrono::duration<double>(1.0 / rate);
    pub_timer_ = create_wall_timer(period, [this]() { publish_map(); });

    RCLCPP_INFO(get_logger(),
      "Scan grid mapper: %dx%d @ %.2fm (auto-expand), publish %.1fHz, "
      "ray_skip=%d, min_travel=%.3fm, frame=%s",
      width_, height_, res_, rate, ray_subsample_,
      std::sqrt(min_travel_dist_sq_), map_frame_.c_str());
  }

private:
  int scan_total_{0};
  int scan_processed_{0};
  int initial_width_, initial_height_;

  // Last processed pose for travel-distance gating
  double last_rx_{std::nan("")};
  double last_ry_{std::nan("")};

  // Dirty region tracking for incremental publish
  int dirty_min_x_{0}, dirty_max_x_{0};
  int dirty_min_y_{0}, dirty_max_y_{0};
  bool full_rebuild_needed_{true};

  void reset_dirty_rect()
  {
    dirty_min_x_ = width_;
    dirty_max_x_ = 0;
    dirty_min_y_ = height_;
    dirty_max_y_ = 0;
  }

  void mark_dirty(int gx, int gy)
  {
    dirty_min_x_ = std::min(dirty_min_x_, gx);
    dirty_max_x_ = std::max(dirty_max_x_, gx);
    dirty_min_y_ = std::min(dirty_min_y_, gy);
    dirty_max_y_ = std::max(dirty_max_y_, gy);
  }

  /// Get current robot position in map frame (falls back to 0,0 if TF unavailable).
  std::pair<double, double> get_robot_position()
  {
    try {
      auto tf = tf_buffer_->lookupTransform(map_frame_, "base_link",
        rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.1));
      return {tf.transform.translation.x, tf.transform.translation.y};
    } catch (...) {
      return {0.0, 0.0};
    }
  }

  /// Reset grid centered on (center_x, center_y) in map frame.
  void reset_grid(double center_x = 0.0, double center_y = 0.0)
  {
    width_ = initial_width_;
    height_ = initial_height_;
    origin_x_ = center_x - (width_ * res_) / 2.0;
    origin_y_ = center_y - (height_ * res_) / 2.0;
    grid_.assign(static_cast<size_t>(width_) * height_, 0.0f);
    pub_data_.assign(grid_.size(), -1);
    full_rebuild_needed_ = true;
    reset_dirty_rect();
    has_data_ = false;
    last_rx_ = std::nan("");
    last_ry_ = std::nan("");

    // Force-publish the empty map immediately so the transient_local cache
    // is replaced. Without this, late subscribers (or dashboard reconnects)
    // see the stale pre-clear map.
    publish_map_force();
  }

  /// Publish map unconditionally (bypasses has_data_ check).
  void publish_map_force()
  {
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = map_frame_;
    msg.info.resolution = res_;
    msg.info.width = width_;
    msg.info.height = height_;
    msg.info.origin.position.x = origin_x_;
    msg.info.origin.position.y = origin_y_;
    msg.info.origin.orientation.w = 1.0;
    msg.data = pub_data_;  // all -1 (unknown) after reset
    map_pub_->publish(msg);
  }

  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    scan_total_++;

    // Get robot pose — use latest TF (TimePointZero) instead of exact stamp.
    // Exact stamp often fails with sim_time jitter over USB, dropping >90% of scans.
    // For mapping, slight pose lag is acceptable.
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(map_frame_, msg->header.frame_id,
        rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration::from_seconds(0.5));
    } catch (...) {
      if (scan_total_ % 50 == 0) {
        RCLCPP_WARN(get_logger(), "TF not available — %d/%d scans processed", scan_processed_, scan_total_);
      }
      return;
    }

    // Warm-up gate: skip early scans while cuVSLAM stabilizes (noisy initial poses)
    if (!warmup_complete_) {
      if (!first_tf_received_) {
        first_tf_received_ = true;
        first_tf_time_ = this->now();
        RCLCPP_INFO(get_logger(), "First TF received, warm-up started (%.1fs)", warmup_seconds_);
      }
      warmup_scans_skipped_++;
      double elapsed = (this->now() - first_tf_time_).seconds();
      if (elapsed >= warmup_seconds_ && warmup_scans_skipped_ >= warmup_min_scans_) {
        warmup_complete_ = true;
        double rx0 = tf.transform.translation.x;
        double ry0 = tf.transform.translation.y;
        reset_grid(rx0, ry0);
        RCLCPP_INFO(get_logger(),
                    "Warm-up complete (%.1fs, %d scans skipped), grid centered on (%.2f, %.2f)",
                    elapsed, warmup_scans_skipped_, rx0, ry0);
      } else {
        return;
      }
    }

    double rx = tf.transform.translation.x;
    double ry = tf.transform.translation.y;

    // Travel-distance gate: skip if robot hasn't moved enough since last processed scan
    if (std::isfinite(last_rx_)) {
      double ddx = rx - last_rx_;
      double ddy = ry - last_ry_;
      if (ddx * ddx + ddy * ddy < min_travel_dist_sq_) {
        return;
      }
    }
    last_rx_ = rx;
    last_ry_ = ry;

    scan_processed_++;
    if (scan_total_ % 100 == 0) {
      RCLCPP_INFO(get_logger(), "Scans: %d/%d processed (%.0f%%), grid %dx%d",
        scan_processed_, scan_total_, scan_processed_ * 100.0 / scan_total_,
        width_, height_);
    }

    double ryaw = yaw_from_quat(tf.transform.rotation);

    // Compute world-space bounding box of this scan for potential grid expansion
    // (iterate ALL rays, not subsampled, to ensure correct expansion)
    double wx_min = rx, wx_max = rx, wy_min = ry, wy_max = ry;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      float r = msg->ranges[i];
      if (!std::isfinite(r) || r < min_range_ || r > max_range_) continue;
      double beam_angle = ryaw + msg->angle_min + i * msg->angle_increment;
      double ex = rx + r * std::cos(beam_angle);
      double ey = ry + r * std::sin(beam_angle);
      wx_min = std::min(wx_min, ex);
      wx_max = std::max(wx_max, ex);
      wy_min = std::min(wy_min, ey);
      wy_max = std::max(wy_max, ey);
    }

    // Expand grid if any scan endpoint falls outside current bounds
    ensure_bounds(wx_min, wy_min, wx_max, wy_max);

    // Robot grid position
    int gx0 = world_to_grid_x(rx);
    int gy0 = world_to_grid_y(ry);

    // Process rays (subsampled)
    for (size_t i = 0; i < msg->ranges.size(); i += ray_subsample_) {
      float r = msg->ranges[i];
      if (!std::isfinite(r) || r < min_range_ || r > max_range_) continue;

      // Endpoint in world frame
      double beam_angle = ryaw + msg->angle_min + i * msg->angle_increment;
      double ex = rx + r * std::cos(beam_angle);
      double ey = ry + r * std::sin(beam_angle);
      int gx1 = world_to_grid_x(ex);
      int gy1 = world_to_grid_y(ey);

      // Bresenham raycast: mark free along ray, occupied at endpoint
      bresenham_free(gx0, gy0, gx1, gy1);

      // Mark endpoint as occupied
      if (in_bounds(gx1, gy1)) {
        auto& cell = grid_[gy1 * width_ + gx1];
        cell = std::min(l_max_, cell + l_occ_);
        mark_dirty(gx1, gy1);
      }
    }

    has_data_ = true;
  }

  /// Expand the grid so that world rectangle [wx_min,wy_min]-[wx_max,wy_max] fits.
  void ensure_bounds(double wx_min, double wy_min, double wx_max, double wy_max)
  {
    // Current world-space extent of the grid
    double cur_wx_min = origin_x_;
    double cur_wy_min = origin_y_;
    double cur_wx_max = origin_x_ + width_ * res_;
    double cur_wy_max = origin_y_ + height_ * res_;

    if (wx_min >= cur_wx_min && wx_max <= cur_wx_max &&
        wy_min >= cur_wy_min && wy_max <= cur_wy_max) {
      return;  // everything fits
    }

    // New world extent with margin
    double margin = expand_margin_ * res_;
    double new_wx_min = std::min(cur_wx_min, wx_min - margin);
    double new_wy_min = std::min(cur_wy_min, wy_min - margin);
    double new_wx_max = std::max(cur_wx_max, wx_max + margin);
    double new_wy_max = std::max(cur_wy_max, wy_max + margin);

    int new_width  = static_cast<int>(std::ceil((new_wx_max - new_wx_min) / res_));
    int new_height = static_cast<int>(std::ceil((new_wy_max - new_wy_min) / res_));

    // Offset of old origin in the new grid
    int dx = static_cast<int>(std::round((origin_x_ - new_wx_min) / res_));
    int dy = static_cast<int>(std::round((origin_y_ - new_wy_min) / res_));

    // Copy old grid into new grid — clamp from destination side to prevent OOB
    std::vector<float> new_grid(static_cast<size_t>(new_width) * new_height, 0.0f);
    for (int y = 0; y < height_; ++y) {
      int ny = y + dy;
      if (ny < 0 || ny >= new_height) continue;
      int dst_x_start = std::max(0, dx);
      int dst_x_end   = std::min(new_width, dx + width_);
      if (dst_x_start >= dst_x_end) continue;
      int copy_len    = dst_x_end - dst_x_start;
      int src_x_start = dst_x_start - dx;
      std::memcpy(
        &new_grid[static_cast<size_t>(ny) * new_width + dst_x_start],
        &grid_[static_cast<size_t>(y) * width_ + src_x_start],
        static_cast<size_t>(copy_len) * sizeof(float));
    }

    if (dx + width_ > new_width + 1 || dy + height_ > new_height + 1) {
      RCLCPP_ERROR(get_logger(),
        "Grid expansion offset anomaly: dx=%d dy=%d old=%dx%d new=%dx%d — copy clamped safely",
        dx, dy, width_, height_, new_width, new_height);
    }

    RCLCPP_INFO(get_logger(),
      "Grid expanded: %dx%d -> %dx%d (%.1fm x %.1fm), origin (%.1f, %.1f)",
      width_, height_, new_width, new_height,
      new_width * res_, new_height * res_, new_wx_min, new_wy_min);

    grid_ = std::move(new_grid);
    width_ = new_width;
    height_ = new_height;
    origin_x_ = new_wx_min;
    origin_y_ = new_wy_min;

    // Resize publish buffer and force full rebuild
    pub_data_.assign(grid_.size(), -1);
    full_rebuild_needed_ = true;
    reset_dirty_rect();
  }

  void bresenham_free(int x0, int y0, int x1, int y1)
  {
    int dx = std::abs(x1 - x0);
    int dy = std::abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1;
    int sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;

    // Don't mark the endpoint (that's occupied)
    while (!(x0 == x1 && y0 == y1)) {
      if (in_bounds(x0, y0)) {
        auto& cell = grid_[y0 * width_ + x0];
        cell = std::max(l_min_, cell + l_free_);
        mark_dirty(x0, y0);
      }
      int e2 = 2 * err;
      if (e2 > -dy) { err -= dy; x0 += sx; }
      if (e2 < dx)  { err += dx; y0 += sy; }
    }
  }

  /// Convert a single log-odds cell to OccupancyGrid int8 value
  int8_t logodds_to_occ(float l) const
  {
    if (std::abs(l) < 0.01f) return -1; // unknown
    float p = 1.0f - 1.0f / (1.0f + std::exp(l));
    if (p >= occ_thresh_) return 100;
    if (p <= free_thresh_) return 0;
    return static_cast<int8_t>(std::round(p * 100.0f));
  }

  void publish_map()
  {
    if (!has_data_) return;

    // Incremental update of pub_data_
    if (full_rebuild_needed_) {
      pub_data_.resize(grid_.size());
      for (size_t i = 0; i < grid_.size(); ++i) {
        pub_data_[i] = logodds_to_occ(grid_[i]);
      }
      full_rebuild_needed_ = false;
    } else if (dirty_min_x_ <= dirty_max_x_ && dirty_min_y_ <= dirty_max_y_) {
      // Only reconvert dirty region
      int x0 = std::max(0, dirty_min_x_);
      int x1 = std::min(width_ - 1, dirty_max_x_);
      int y0 = std::max(0, dirty_min_y_);
      int y1 = std::min(height_ - 1, dirty_max_y_);
      for (int y = y0; y <= y1; ++y) {
        for (int x = x0; x <= x1; ++x) {
          size_t idx = static_cast<size_t>(y) * width_ + x;
          pub_data_[idx] = logodds_to_occ(grid_[idx]);
        }
      }
    }
    reset_dirty_rect();

    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = map_frame_;
    msg.info.resolution = res_;
    msg.info.width = width_;
    msg.info.height = height_;
    msg.info.origin.position.x = origin_x_;
    msg.info.origin.position.y = origin_y_;
    msg.info.origin.orientation.w = 1.0;
    msg.data = pub_data_;

    map_pub_->publish(msg);
  }

  // Coordinate conversion
  int world_to_grid_x(double wx) const { return static_cast<int>((wx - origin_x_) / res_); }
  int world_to_grid_y(double wy) const { return static_cast<int>((wy - origin_y_) / res_); }
  bool in_bounds(int gx, int gy) const { return gx >= 0 && gx < width_ && gy >= 0 && gy < height_; }

  // Warm-up state (skip early noisy scans while cuVSLAM stabilizes)
  bool warmup_complete_{false};
  bool first_tf_received_{false};
  rclcpp::Time first_tf_time_;
  int warmup_scans_skipped_{0};
  double warmup_seconds_;
  int warmup_min_scans_;

  // Parameters
  float res_;
  double origin_x_, origin_y_;
  int width_, height_;
  std::string map_frame_;
  float l_occ_, l_free_, l_min_, l_max_;
  float occ_thresh_, free_thresh_;
  float max_range_, min_range_;
  int expand_margin_;
  int ray_subsample_;
  double min_travel_dist_sq_;

  // Grid (log-odds, float32)
  std::vector<float> grid_;
  bool has_data_{false};

  // Persistent publish buffer (avoids per-publish allocation)
  std::vector<int8_t> pub_data_;

  // ROS
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr clear_srv_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr clear_sub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanGridMapperNode>());
  rclcpp::shutdown();
  return 0;
}
