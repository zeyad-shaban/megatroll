#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>
#include <cmath>

using namespace std::chrono_literals;

class CameraDriverNode : public rclcpp::Node
{
public:
  enum class ConnectionState
  {
    DISCONNECTED, // Not connected, not trying
    CONNECTING,   // Attempting to connect
    CONNECTED,    // Successfully connected
    FAILED        // Max retries exhausted
  };

  CameraDriverNode() : Node("camera_driver_node")
  {
    // Declare parameters with defaults matching user-provided URLs
    this->declare_parameter<std::vector<std::string>>("camera_urls",
                                                      {"http://192.168.1.11:8080/video", "http://192.168.43.1:8080/video"});
    this->declare_parameter<std::string>("frame_id", "camera_link_optical");
    this->declare_parameter<int>("target_fps", 30);
    this->declare_parameter<int>("width", 640);
    this->declare_parameter<int>("height", 480);
    this->declare_parameter<int>("initial_retry_delay_ms", 1000);
    this->declare_parameter<int>("max_retry_delay_ms", 30000);
    this->declare_parameter<double>("backoff_multiplier", 1.5);
    this->declare_parameter<int>("max_consecutive_failures", 10);

    // Retrieve parameters
    this->get_parameter("camera_urls", camera_urls_);
    this->get_parameter("frame_id", frame_id_);
    this->get_parameter("target_fps", target_fps_);
    this->get_parameter("width", width_);
    this->get_parameter("height", height_);
    this->get_parameter("initial_retry_delay_ms", initial_retry_delay_ms_);
    this->get_parameter("max_retry_delay_ms", max_retry_delay_ms_);
    this->get_parameter("backoff_multiplier", backoff_multiplier_);
    this->get_parameter("max_consecutive_failures", max_consecutive_failures_);

    // Publisher for compressed images (JPEG)
    pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
        "/camera/image/compressed", 10);

    RCLCPP_INFO(this->get_logger(), "Camera Driver Node initialized");
    RCLCPP_INFO(this->get_logger(), "Retry config: initial=%dms, max=%dms, backoff=%.1fx, max_failures=%d",
                initial_retry_delay_ms_, max_retry_delay_ms_, backoff_multiplier_, max_consecutive_failures_);

    // Start initial connection attempt
    set_state(ConnectionState::CONNECTING);
    current_retry_delay_ms_ = initial_retry_delay_ms_;

    // Timer to attempt connection (starts immediately)
    reconnect_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&CameraDriverNode::try_connect, this));

    // Timer for frame capture (will only run when connected)
    capture_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / target_fps_),
        std::bind(&CameraDriverNode::capture_and_publish, this),
        nullptr);

    // Pause capture timer until connected
    capture_timer_->cancel();
  }

private:
  void set_state(ConnectionState new_state)
  {
    if (connection_state_ != new_state)
    {
      connection_state_ = new_state;
      std::string state_str;
      switch (new_state)
      {
      case ConnectionState::DISCONNECTED:
        state_str = "DISCONNECTED";
        break;
      case ConnectionState::CONNECTING:
        state_str = "CONNECTING";
        break;
      case ConnectionState::CONNECTED:
        state_str = "CONNECTED";
        break;
      case ConnectionState::FAILED:
        state_str = "FAILED";
        break;
      }
      RCLCPP_INFO(this->get_logger(), "Connection state changed to: %s", state_str.c_str());
    }
  }

  void try_connect()
  {
    if (connection_state_ == ConnectionState::CONNECTED)
    {
      reconnect_timer_->cancel();
      return;
    }

    if (connection_state_ == ConnectionState::FAILED)
    {
      return; // Stop attempting
    }

    if (!attempt_open_camera())
    {
      consecutive_failures_++;

      if (consecutive_failures_ >= max_consecutive_failures_)
      {
        set_state(ConnectionState::FAILED);
        RCLCPP_ERROR(this->get_logger(),
                     "Max consecutive failures (%d) reached. Stopping reconnection attempts.",
                     max_consecutive_failures_);
        reconnect_timer_->cancel();
        return;
      }

      // Exponential backoff
      current_retry_delay_ms_ = std::min(
          (int)(current_retry_delay_ms_ * backoff_multiplier_),
          max_retry_delay_ms_);

      RCLCPP_WARN(this->get_logger(),
                  "Connection attempt failed. Next attempt in %dms (attempt %d/%d)",
                  current_retry_delay_ms_, consecutive_failures_, max_consecutive_failures_);

      // Update timer delay
      reconnect_timer_->cancel();
      reconnect_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(current_retry_delay_ms_),
          std::bind(&CameraDriverNode::try_connect, this));
    }
  }

  bool attempt_open_camera()
  {
    for (const auto &url : camera_urls_)
    {
      RCLCPP_INFO(this->get_logger(), "Attempting to open camera URL: %s", url.c_str());

      cv::VideoCapture test_cap;
      test_cap.open(url, cv::CAP_FFMPEG);

      if (test_cap.isOpened())
      {
        // Verify we can actually read a frame
        cv::Mat test_frame;
        if (test_cap.read(test_frame) && !test_frame.empty())
        {
          // Connection successful
          cap_ = std::move(test_cap);
          configure_camera();
          set_state(ConnectionState::CONNECTED);
          consecutive_failures_ = 0;
          current_retry_delay_ms_ = initial_retry_delay_ms_;

          RCLCPP_INFO(this->get_logger(), "✓ Successfully connected to: %s", url.c_str());

          // Resume capture timer
          if (capture_timer_)
          {
            capture_timer_->reset();
          }
          return true;
        }
        test_cap.release();
      }
    }

    return false;
  }

  void configure_camera()
  {
    // Reduce latency
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);
    // Apply desired resolution if set
    if (width_ > 0 && height_ > 0)
    {
      cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
      cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    }
  }

  void capture_and_publish()
  {
    if (connection_state_ != ConnectionState::CONNECTED)
    {
      return;
    }

    cv::Mat frame;
    if (!cap_.isOpened() || !cap_.read(frame) || frame.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Camera connection lost. Initiating reconnection...");
      set_state(ConnectionState::CONNECTING);
      consecutive_failures_ = 0;
      current_retry_delay_ms_ = initial_retry_delay_ms_;

      // Pause capture and start reconnection
      capture_timer_->cancel();

      reconnect_timer_->cancel();
      reconnect_timer_ = this->create_wall_timer(
          std::chrono::milliseconds(100),
          std::bind(&CameraDriverNode::try_connect, this));
      return;
    }

    std::vector<uchar> buf;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
    if (!cv::imencode(".jpg", frame, buf, params))
    {
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

  // State management
  ConnectionState connection_state_ = ConnectionState::DISCONNECTED;
  int consecutive_failures_ = 0;
  int current_retry_delay_ms_ = 1000;

  // Reconnection parameters
  int initial_retry_delay_ms_ = 1000;
  int max_retry_delay_ms_ = 30000;
  double backoff_multiplier_ = 1.5;
  int max_consecutive_failures_ = 10;

  // ROS2 components
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr capture_timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;

  // Camera state
  cv::VideoCapture cap_;
  std::vector<std::string> camera_urls_;
  std::string frame_id_;
  int target_fps_ = 30;
  int width_ = 640;
  int height_ = 480;
};
}
;

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
