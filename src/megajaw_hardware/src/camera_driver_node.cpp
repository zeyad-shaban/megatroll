#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

using namespace std::chrono_literals;

class CameraDriverNode : public rclcpp::Node {
public:
  CameraDriverNode() : Node("camera_driver_node") {
    // Declare parameters with defaults matching user-provided URLs
    this->declare_parameter<std::vector<std::string>>("camera_urls",
      {"http://192.168.1.11:8080/video", "http://10.152.247.225:8080/video"});
    this->declare_parameter<std::string>("frame_id", "camera_link_optical");
    this->declare_parameter<int>("target_fps", 30);
    this->declare_parameter<int>("width", 640);
    this->declare_parameter<int>("height", 480);

    // Retrieve parameters
    this->get_parameter("camera_urls", camera_urls_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("target_fps", target_fps_);
    this->get_parameter("width", width_);
    this->get_parameter("height", height_);

    // Publisher for compressed images (JPEG)
    pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
      "/camera/image/compressed", 10);

    // Try to open a viable stream
    if (!open_camera()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open any camera URL");
      rclcpp::shutdown();
      return;
    }

    // Timer to capture and publish frames at the desired rate
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(1000 / target_fps_),
      std::bind(&CameraDriverNode::capture_and_publish, this));
  }

private:
  bool open_camera() {
    bool opened = false;
    for (const auto &url : camera_urls_) {
      RCLCPP_INFO(this->get_logger(), "Attempting to open camera URL: %s", url.c_str());
      cap_.open(url, cv::CAP_FFMPEG);
      if (cap_.isOpened()) {
        RCLCPP_INFO(this->get_logger(), "Successfully connected to: %s", url.c_str());
        opened = true;
        break;
      }
    }
    if (!opened) {
      RCLCPP_ERROR(this->get_logger(), "Could not open any provided camera streams");
      return false;
    }
    // Reduce latency as per user comment
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    // Apply desired resolution if set
    if (width_ > 0 && height_ > 0) {
      cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
      cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    }
    return true;
  }

  void capture_and_publish() {
    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
        "Failed to read frame from camera");
      return;
    }

    std::vector<uchar> buf;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
    if (!cv::imencode(".jpg", frame, buf, params)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to encode frame to JPEG");
      return;
    }

    auto msg = sensor_msgs::msg::CompressedImage();
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    msg.format = "jpeg";
    msg.data = std::move(buf);
    pub_->publish(msg);
  }

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  cv::VideoCapture cap_;
  std::vector<std::string> camera_urls_;
  std::string frame_id_;
  int target_fps_ = 30;
  int width_ = 640;
  int height_ = 480;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
