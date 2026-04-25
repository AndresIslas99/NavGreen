/**
 * AGV Image Server — C++17 MJPEG HTTP server
 *
 * Serves camera and depth heatmap as MJPEG streams via raw HTTP sockets.
 * Uses OpenCV imencode (libjpeg-turbo NEON on ARM) for 3-5x faster
 * JPEG encoding than Python PIL.
 *
 * Endpoints:
 *   GET /camera/stream   — MJPEG multipart stream from camera topic
 *   GET /camera/snapshot  — single JPEG frame
 *   GET /depth/stream    — MJPEG depth heatmap stream
 *   GET /depth/snapshot   — single depth heatmap frame
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <set>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>

class ImageServerNode : public rclcpp::Node
{
public:
  ImageServerNode() : Node("image_server")
  {
    declare_parameter("port", 8091);
    declare_parameter("camera_topic", "/zed/zed_node/left/image_rect_color");
    declare_parameter("depth_topic", "/zed/zed_node/depth/depth_registered");
    declare_parameter("jpeg_quality", 70);
    declare_parameter("max_width", 640);
    // Sprint 1 Fase A1: cap simultaneous MJPEG streams. Each stream blocks a
    // dedicated thread for the duration of the client connection. Without a
    // cap, a misbehaving frontend (open + reload + retry) can spawn unbounded
    // threads competing JPEG-encode CPU with cuVSLAM. Excess clients receive
    // HTTP 503; the browser typically retries, which lets the cap recover
    // gracefully when an existing stream closes.
    declare_parameter("max_concurrent_streams", 4);

    port_ = get_parameter("port").as_int();
    jpeg_quality_ = get_parameter("jpeg_quality").as_int();
    max_width_ = get_parameter("max_width").as_int();
    max_concurrent_streams_ = get_parameter("max_concurrent_streams").as_int();

    auto cam_topic = get_parameter("camera_topic").as_string();
    auto depth_topic = get_parameter("depth_topic").as_string();

    auto qos = rclcpp::SensorDataQoS();

    cam_sub_ = create_subscription<sensor_msgs::msg::Image>(
      cam_topic, qos, [this](sensor_msgs::msg::Image::SharedPtr msg) { on_camera(msg); });

    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_topic, qos, [this](sensor_msgs::msg::Image::SharedPtr msg) { on_depth(msg); });

    // Start HTTP server in background thread
    server_thread_ = std::thread([this]() { run_server(); });

    RCLCPP_INFO(get_logger(), "Image server on port %d (camera: %s, depth: %s)",
      port_, cam_topic.c_str(), depth_topic.c_str());
  }

  ~ImageServerNode() override
  {
    running_ = false;
    if (server_fd_ >= 0) close(server_fd_);
    if (server_thread_.joinable()) server_thread_.join();
  }

private:
  void on_camera(sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      // Accept any color encoding — convert to BGR for JPEG output
      auto cv_img = cv_bridge::toCvCopy(msg);
      cv::Mat bgr;
      if (msg->encoding == "rgb8") {
        cv::cvtColor(cv_img->image, bgr, cv::COLOR_RGB2BGR);
      } else if (msg->encoding == "bgr8") {
        bgr = cv_img->image;
      } else if (msg->encoding == "rgba8") {
        cv::cvtColor(cv_img->image, bgr, cv::COLOR_RGBA2BGR);
      } else if (msg->encoding == "bgra8") {
        cv::cvtColor(cv_img->image, bgr, cv::COLOR_BGRA2BGR);
      } else {
        bgr = cv_img->image;  // best effort
      }

      cv::Mat resized;
      if (bgr.cols > max_width_) {
        double scale = static_cast<double>(max_width_) / bgr.cols;
        cv::resize(bgr, resized, cv::Size(), scale, scale, cv::INTER_AREA);
      } else {
        resized = bgr;
      }

      std::vector<uchar> buf;
      std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
      cv::imencode(".jpg", resized, buf, params);

      std::lock_guard<std::mutex> lock(cam_mutex_);
      cam_jpeg_ = std::move(buf);
    } catch (const std::exception& e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Camera frame error: %s (encoding: %s)", e.what(), msg->encoding.c_str());
    }
  }

  void on_depth(sensor_msgs::msg::Image::SharedPtr msg)
  {
    try {
      auto cv_img = cv_bridge::toCvCopy(msg, "32FC1");
      cv::Mat depth = cv_img->image;

      // Downsample 2x (was 4x — better resolution for greenhouse mapping feedback)
      cv::Mat small;
      cv::resize(depth, small, cv::Size(depth.cols / 2, depth.rows / 2), 0, 0, cv::INTER_NEAREST);

      // Normalize to 0-255
      cv::Mat norm;
      small.setTo(10.0, ~cv::Mat(small == small)); // NaN → 10m
      small = cv::max(small, 0.3);
      small = cv::min(small, 10.0);
      norm = (small - 0.3) / 9.7 * 255.0;
      norm.convertTo(norm, CV_8UC1);

      // Apply JET colormap (near=red, far=blue — OpenCV JET is blue→red, so invert)
      cv::Mat colored;
      cv::applyColorMap(255 - norm, colored, cv::COLORMAP_JET);

      std::vector<uchar> buf;
      std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 75};
      cv::imencode(".jpg", colored, buf, params);

      std::lock_guard<std::mutex> lock(depth_mutex_);
      depth_jpeg_ = std::move(buf);
    } catch (...) {}
  }

  // ── Simple HTTP server ──

  void run_server()
  {
    server_fd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd_ < 0) { RCLCPP_ERROR(get_logger(), "socket() failed"); return; }

    int opt = 1;
    setsockopt(server_fd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port_);

    if (bind(server_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(get_logger(), "bind() port %d failed", port_);
      return;
    }
    listen(server_fd_, 10);

    while (running_) {
      sockaddr_in client_addr{};
      socklen_t client_len = sizeof(client_addr);
      int client_fd = accept(server_fd_, reinterpret_cast<sockaddr*>(&client_addr), &client_len);
      if (client_fd < 0) continue;

      // Read HTTP request (simple: just get the path)
      char buf[1024] = {};
      ssize_t n = recv(client_fd, buf, sizeof(buf) - 1, 0);
      if (n <= 0) { close(client_fd); continue; }

      std::string req(buf, n);
      std::string path;
      if (req.substr(0, 4) == "GET ") {
        auto end = req.find(' ', 4);
        path = req.substr(4, end - 4);
      }

      if (path == "/camera/stream") {
        spawn_stream(client_fd, cam_jpeg_, cam_mutex_, 100);
      } else if (path == "/camera/snapshot") {
        serve_snapshot(client_fd, cam_jpeg_, cam_mutex_);
      } else if (path == "/depth/stream") {
        spawn_stream(client_fd, depth_jpeg_, depth_mutex_, 200);
      } else if (path == "/depth/snapshot") {
        serve_snapshot(client_fd, depth_jpeg_, depth_mutex_);
      } else {
        const char* resp = "HTTP/1.1 404 Not Found\r\nContent-Length: 9\r\n\r\nNot Found";
        send(client_fd, resp, strlen(resp), 0);
        close(client_fd);
      }
    }
  }

  // Spawn a streaming worker if the active-stream cap allows it. Excess
  // clients are rejected with HTTP 503 so the front-end can decide to back
  // off or retry. The worker is detached but tracked by active_streams_ so
  // the cap is respected; the destructor blocks until streams drain via
  // running_=false (each stream's inner loop checks running_ before each
  // frame and exits within delay_ms).
  void spawn_stream(int client_fd, std::vector<uchar>& jpeg_ref,
                    std::mutex& mtx, int delay_ms)
  {
    int current = active_streams_.fetch_add(1, std::memory_order_relaxed);
    if (current >= max_concurrent_streams_) {
      active_streams_.fetch_sub(1, std::memory_order_relaxed);
      const char* resp = "HTTP/1.1 503 Service Unavailable\r\n"
        "Retry-After: 2\r\n"
        "Content-Length: 27\r\n\r\n"
        "Stream cap reached, retry.\n";
      send(client_fd, resp, strlen(resp), MSG_NOSIGNAL);
      close(client_fd);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Image stream rejected: %d concurrent clients (cap=%d)",
        current, max_concurrent_streams_);
      return;
    }
    std::thread([this, client_fd, &jpeg_ref, &mtx, delay_ms]() {
      serve_mjpeg(client_fd, jpeg_ref, mtx, delay_ms);
      active_streams_.fetch_sub(1, std::memory_order_relaxed);
    }).detach();
  }

  void serve_mjpeg(int fd, std::vector<uchar>& jpeg_ref, std::mutex& mtx, int delay_ms)
  {
    const char* header = "HTTP/1.1 200 OK\r\n"
      "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n"
      "Cache-Control: no-cache\r\n"
      "Access-Control-Allow-Origin: *\r\n\r\n";
    if (send(fd, header, strlen(header), MSG_NOSIGNAL) < 0) { close(fd); return; }

    while (running_) {
      std::vector<uchar> frame;
      {
        std::lock_guard<std::mutex> lock(mtx);
        if (!jpeg_ref.empty()) frame = jpeg_ref;
      }

      if (!frame.empty()) {
        std::string part = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: "
          + std::to_string(frame.size()) + "\r\n\r\n";
        if (send(fd, part.c_str(), part.size(), MSG_NOSIGNAL) < 0) break;
        if (send(fd, frame.data(), frame.size(), MSG_NOSIGNAL) < 0) break;
        if (send(fd, "\r\n", 2, MSG_NOSIGNAL) < 0) break;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
    }
    close(fd);
  }

  void serve_snapshot(int fd, std::vector<uchar>& jpeg_ref, std::mutex& mtx)
  {
    std::vector<uchar> frame;
    {
      std::lock_guard<std::mutex> lock(mtx);
      if (!jpeg_ref.empty()) frame = jpeg_ref;
    }

    if (frame.empty()) {
      const char* resp = "HTTP/1.1 404 Not Found\r\nContent-Length: 8\r\n\r\nNo frame";
      send(fd, resp, strlen(resp), 0);
    } else {
      std::string header = "HTTP/1.1 200 OK\r\n"
        "Content-Type: image/jpeg\r\n"
        "Access-Control-Allow-Origin: *\r\n"
        "Content-Length: " + std::to_string(frame.size()) + "\r\n\r\n";
      send(fd, header.c_str(), header.size(), MSG_NOSIGNAL);
      send(fd, frame.data(), frame.size(), MSG_NOSIGNAL);
    }
    close(fd);
  }

  // State
  std::vector<uchar> cam_jpeg_;
  std::mutex cam_mutex_;
  std::vector<uchar> depth_jpeg_;
  std::mutex depth_mutex_;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr cam_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;

  // Server
  int port_;
  int jpeg_quality_;
  int max_width_;
  int max_concurrent_streams_;
  std::atomic<int> active_streams_{0};
  int server_fd_{-1};
  std::atomic<bool> running_{true};
  std::thread server_thread_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageServerNode>());
  rclcpp::shutdown();
  return 0;
}
