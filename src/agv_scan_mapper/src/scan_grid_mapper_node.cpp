/**
 * Scan Grid Mapper — builds a 2D occupancy grid from LaserScan data.
 *
 * Subscribes to LaserScan + TF, maintains a probabilistic log-odds grid,
 * and publishes nav_msgs/OccupancyGrid for Nav2 map_saver to capture.
 *
 * This is the missing piece in the mapping commissioning pipeline:
 *   pointcloud_to_laserscan → /agv/scan → scan_grid_mapper → /agv/live_map
 *   → map_saver_cli saves .pgm/.yaml → map_server loads for navigation.
 */

#include <cmath>
#include <vector>
#include <algorithm>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
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
    declare_parameter("width", 400);
    declare_parameter("height", 400);
    declare_parameter("origin_x", -10.0);
    declare_parameter("origin_y", -10.0);
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

    res_ = get_parameter("resolution").as_double();
    width_ = get_parameter("width").as_int();
    height_ = get_parameter("height").as_int();
    origin_x_ = get_parameter("origin_x").as_double();
    origin_y_ = get_parameter("origin_y").as_double();
    map_frame_ = get_parameter("map_frame").as_string();
    l_occ_ = get_parameter("l_occupied").as_double();
    l_free_ = get_parameter("l_free").as_double();
    l_min_ = get_parameter("l_min").as_double();
    l_max_ = get_parameter("l_max").as_double();
    occ_thresh_ = get_parameter("occupied_threshold").as_double();
    free_thresh_ = get_parameter("free_threshold").as_double();
    max_range_ = get_parameter("max_range").as_double();
    min_range_ = get_parameter("min_range").as_double();

    // Initialize grid (log-odds, all zeros = unknown)
    grid_.resize(static_cast<size_t>(width_) * height_, 0.0);

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

    // Publish timer
    double rate = get_parameter("publish_rate_hz").as_double();
    auto period = std::chrono::duration<double>(1.0 / rate);
    pub_timer_ = create_wall_timer(period, [this]() { publish_map(); });

    RCLCPP_INFO(get_logger(),
      "Scan grid mapper: %dx%d @ %.2fm, publish %.1fHz, frame=%s",
      width_, height_, res_, rate, map_frame_.c_str());
  }

private:
  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Get robot pose in map frame via TF
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform(map_frame_, msg->header.frame_id,
        msg->header.stamp, rclcpp::Duration::from_seconds(0.2));
    } catch (...) {
      return; // TF not available yet
    }

    double rx = tf.transform.translation.x;
    double ry = tf.transform.translation.y;
    double ryaw = yaw_from_quat(tf.transform.rotation);

    // Robot grid position
    int gx0 = world_to_grid_x(rx);
    int gy0 = world_to_grid_y(ry);

    // Process each ray
    double angle = msg->angle_min;
    for (size_t i = 0; i < msg->ranges.size(); ++i) {
      double r = msg->ranges[i];
      angle = msg->angle_min + i * msg->angle_increment;

      if (!std::isfinite(r) || r < min_range_ || r > max_range_) continue;

      // Endpoint in world frame
      double beam_angle = ryaw + angle;
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
      }
    }

    has_data_ = true;
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
      }
      int e2 = 2 * err;
      if (e2 > -dy) { err -= dy; x0 += sx; }
      if (e2 < dx)  { err += dx; y0 += sy; }
    }
  }

  void publish_map()
  {
    if (!has_data_) return;

    nav_msgs::msg::OccupancyGrid msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = map_frame_;
    msg.info.resolution = res_;
    msg.info.width = width_;
    msg.info.height = height_;
    msg.info.origin.position.x = origin_x_;
    msg.info.origin.position.y = origin_y_;
    msg.info.origin.orientation.w = 1.0;

    msg.data.resize(grid_.size());
    for (size_t i = 0; i < grid_.size(); ++i) {
      double l = grid_[i];
      if (std::abs(l) < 0.01) {
        msg.data[i] = -1; // unknown
      } else {
        double p = 1.0 - 1.0 / (1.0 + std::exp(l));
        if (p >= occ_thresh_) {
          msg.data[i] = 100;
        } else if (p <= free_thresh_) {
          msg.data[i] = 0;
        } else {
          msg.data[i] = static_cast<int8_t>(std::round(p * 100.0));
        }
      }
    }

    map_pub_->publish(msg);
  }

  // Coordinate conversion
  int world_to_grid_x(double wx) const { return static_cast<int>((wx - origin_x_) / res_); }
  int world_to_grid_y(double wy) const { return static_cast<int>((wy - origin_y_) / res_); }
  bool in_bounds(int gx, int gy) const { return gx >= 0 && gx < width_ && gy >= 0 && gy < height_; }

  // Parameters
  double res_, origin_x_, origin_y_;
  int width_, height_;
  std::string map_frame_;
  double l_occ_, l_free_, l_min_, l_max_;
  double occ_thresh_, free_thresh_;
  double max_range_, min_range_;

  // Grid (log-odds)
  std::vector<double> grid_;
  bool has_data_{false};

  // ROS
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanGridMapperNode>());
  rclcpp::shutdown();
  return 0;
}
