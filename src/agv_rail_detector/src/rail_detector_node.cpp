// rail_detector_node — ZED depth → BEV ground slice → RANSAC rail pair.
//
// Subscribes:
//   /agv/zed/depth/depth_registered (sensor_msgs/Image, 32FC1)
//   /agv/zed/left/camera_info       (sensor_msgs/CameraInfo)
//
// Publishes:
//   /agv/rail_detections (geometry_msgs/PoseArray) — 2 poses per tick
//     when confidence > threshold; both in base_link frame with
//     position at the rail's nearest point to the robot and orientation
//     pointing along the rail.
//
// The BEV reprojector is simple: for each depth pixel, back-project to
// camera optical frame via the pinhole model, then transform to base_link
// with the static camera offset read from params. Only points within
// `z_slice_{min,max}` are kept (ground-plane slice). The resulting 2-D
// points go to the RANSAC detector.
//
// Runs at `publish_rate_hz` (default 5 Hz) — processes at most one depth
// frame per tick.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "std_msgs/msg/string.hpp"

#include "agv_rail_detector/rail_ransac.hpp"

using std::placeholders::_1;

namespace agv_rail_detector {

class RailDetectorNode : public rclcpp::Node {
 public:
  RailDetectorNode() : rclcpp::Node("rail_detector") {
    declare_parameter<std::string>("depth_topic",
                                   "/agv/zed/depth/depth_registered");
    declare_parameter<std::string>("camera_info_topic",
                                   "/agv/zed/left/camera_info");
    declare_parameter<std::string>("detections_topic",
                                   "/agv/rail_detections");
    declare_parameter<std::string>("state_topic",
                                   "/agv/rail_detector/state");
    declare_parameter<double>("publish_rate_hz", 5.0);
    // Camera offset from base_link (X forward, Z up). Matches the real
    // greenhouse AGV as re-measured 2026-04-18: ZED at (0.700, 0.0, 0.21)
    // above ground. The optical frame rotation is applied internally.
    declare_parameter<double>("camera_x", 0.700);
    declare_parameter<double>("camera_y", 0.0);
    declare_parameter<double>("camera_z", 0.21);
    // Ground-plane slice thickness relative to z=0 in base_link (robot
    // floor). Rails sit within ±2 cm of the floor.
    declare_parameter<double>("z_slice_min", -0.02);
    declare_parameter<double>("z_slice_max", 0.08);
    // BEV forward window: only process depth points whose base_link X is
    // inside [x_min, x_max]. 0.3-3 m by default keeps RANSAC fast and
    // matches rail tube visibility.
    declare_parameter<double>("x_min", 0.3);
    declare_parameter<double>("x_max", 3.0);
    declare_parameter<double>("y_half_width", 1.0);
    // Depth sub-sampling stride in pixels.
    declare_parameter<int>("depth_stride", 8);
    // Reject detections below this confidence.
    declare_parameter<double>("min_confidence", 0.3);

    pub_detections_ = create_publisher<geometry_msgs::msg::PoseArray>(
        get_parameter("detections_topic").as_string(), rclcpp::QoS{5});
    pub_state_ = create_publisher<std_msgs::msg::String>(
        get_parameter("state_topic").as_string(), rclcpp::QoS{5});

    sub_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        get_parameter("camera_info_topic").as_string(),
        rclcpp::SensorDataQoS(),
        std::bind(&RailDetectorNode::on_camera_info, this, _1));
    sub_depth_ = create_subscription<sensor_msgs::msg::Image>(
        get_parameter("depth_topic").as_string(),
        rclcpp::SensorDataQoS(),
        std::bind(&RailDetectorNode::on_depth, this, _1));

    const double rate = get_parameter("publish_rate_hz").as_double();
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1.0 / rate)),
        std::bind(&RailDetectorNode::on_tick, this));

    RCLCPP_INFO(get_logger(),
                "rail_detector up: depth=%s info=%s → %s @ %.1f Hz",
                get_parameter("depth_topic").as_string().c_str(),
                get_parameter("camera_info_topic").as_string().c_str(),
                get_parameter("detections_topic").as_string().c_str(),
                rate);
  }

 private:
  void on_camera_info(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
    if (camera_info_received_) return;
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];
    if (fx_ > 1.0 && fy_ > 1.0) {
      camera_info_received_ = true;
      RCLCPP_INFO(get_logger(),
                  "camera intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
                  fx_, fy_, cx_, cy_);
    }
  }

  void on_depth(sensor_msgs::msg::Image::ConstSharedPtr msg) {
    last_depth_ = msg;
  }

  // Reproject the latest depth frame to base_link and keep only ground-
  // slice points. Does not use tf2 to avoid the heavy runtime dep — the
  // static camera offset is read from params (the URDF-correct value).
  std::vector<Point2D> reproject_to_bev() {
    std::vector<Point2D> pts;
    if (!last_depth_ || !camera_info_received_) return pts;
    const auto &d = *last_depth_;
    if (d.encoding != "32FC1") return pts;  // expect float depth in metres
    const int stride = std::max(1, static_cast<int>(
        get_parameter("depth_stride").as_int()));
    const double cam_x = get_parameter("camera_x").as_double();
    const double cam_z = get_parameter("camera_z").as_double();
    const double z_min = get_parameter("z_slice_min").as_double();
    const double z_max = get_parameter("z_slice_max").as_double();
    const double x_min = get_parameter("x_min").as_double();
    const double x_max = get_parameter("x_max").as_double();
    const double y_half = get_parameter("y_half_width").as_double();

    // Depth payload as float32 little-endian.
    const float *data = reinterpret_cast<const float *>(d.data.data());
    const size_t row_floats = d.step / sizeof(float);
    for (int v = 0; v < static_cast<int>(d.height); v += stride) {
      for (int u = 0; u < static_cast<int>(d.width); u += stride) {
        const float z_cam = data[v * row_floats + u];
        if (!std::isfinite(z_cam) || z_cam <= 0.1f || z_cam > 6.0f) continue;
        // Optical frame: X right, Y down, Z forward. Pinhole back-projection.
        const double x_opt = (u - cx_) / fx_ * z_cam;
        const double y_opt = (v - cy_) / fy_ * z_cam;
        const double z_opt = z_cam;
        // Rotate optical → base_link (camera facing +X, no tilt): body_X =
        // Z_opt, body_Y = -X_opt, body_Z = -Y_opt. Apply camera mount
        // offset.
        const double body_x = z_opt + cam_x;
        const double body_y = -x_opt;
        const double body_z = -y_opt + cam_z;
        if (body_z < z_min || body_z > z_max) continue;
        if (body_x < x_min || body_x > x_max) continue;
        if (std::abs(body_y) > y_half) continue;
        pts.push_back({body_x, body_y});
      }
    }
    return pts;
  }

  void on_tick() {
    const auto pts = reproject_to_bev();
    RansacParams p;
    p.min_confidence = get_parameter("min_confidence").as_double();
    const auto det = detect_rails(pts, p);

    // Publish state regardless — consumers can read confidence=0 as "no".
    publish_state(det, pts.size());

    if (det.confidence <= 0.0) return;

    // Translate each line into a PoseStamped-like entry in a PoseArray.
    // Pose position = the line's nearest point to the origin (base_link);
    // orientation yaw = angle of the line's DIRECTION (not normal).
    geometry_msgs::msg::PoseArray arr;
    arr.header.stamp = get_clock()->now();
    arr.header.frame_id = "base_link";
    arr.poses.push_back(pose_from_line(det.line_left));
    arr.poses.push_back(pose_from_line(det.line_right));
    pub_detections_->publish(arr);
  }

  static geometry_msgs::msg::Pose pose_from_line(const Line2D &l) {
    geometry_msgs::msg::Pose p;
    // Nearest point to origin: x = -a*c, y = -b*c (since a²+b² = 1).
    p.position.x = -l.a * l.c;
    p.position.y = -l.b * l.c;
    p.position.z = 0.0;
    // Direction yaw = atan2(direction), where direction is
    // (b, -a) (perpendicular to the normal (a, b)).
    const double yaw = std::atan2(-l.a, l.b);
    p.orientation.x = 0.0;
    p.orientation.y = 0.0;
    p.orientation.z = std::sin(yaw * 0.5);
    p.orientation.w = std::cos(yaw * 0.5);
    return p;
  }

  void publish_state(const RailDetection &det, size_t n_points) {
    std_msgs::msg::String msg;
    std::ostringstream os;
    os << "{\"confidence\":" << det.confidence
       << ",\"inliers_left\":" << det.inliers_left
       << ",\"inliers_right\":" << det.inliers_right
       << ",\"n_points\":" << n_points
       << ",\"has_detection\":" << (det.confidence > 0.0 ? "true" : "false")
       << "}";
    msg.data = os.str();
    pub_state_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub_detections_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::Image::ConstSharedPtr last_depth_;
  double fx_ = 0.0, fy_ = 0.0, cx_ = 0.0, cy_ = 0.0;
  bool camera_info_received_ = false;
};

}  // namespace agv_rail_detector

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<agv_rail_detector::RailDetectorNode>());
  rclcpp::shutdown();
  return 0;
}
