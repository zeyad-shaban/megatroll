#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <string>

using namespace std::chrono_literals;

class CameraDriverNode : public rclcpp::Node
{
private:
  // Configuration constants
  static constexpr int DEFAULT_FPS = 10;
  static constexpr int DEFAULT_WIDTH = 352;
  static constexpr int DEFAULT_HEIGHT = 288;
  static constexpr int DEFAULT_JPEG_QUALITY = 80;
  static constexpr int DEFAULT_BUFFER_SIZE = 1;
  static constexpr int DEFAULT_INITIAL_RETRY_MS = 1000;
  static constexpr int DEFAULT_MAX_RETRY_MS = 5000;
  static constexpr double DEFAULT_BACKOFF = 1.1;
  static constexpr int QUICK_RETRY_MS = 100;

  enum class State
  {
    DISCONNECTED,
    CONNECTING,
    CONNECTED,
    FAILED
  };

  // ROS2 components
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;
  rclcpp::TimerBase::SharedPtr capture_timer_;

  // Camera and state
  cv::VideoCapture cap_;
  State state_ = State::DISCONNECTED;
  int consecutive_failures_ = 0;
  int current_retry_ms_ = DEFAULT_INITIAL_RETRY_MS;

  // Parameters
  std::vector<std::string> urls_;
  std::string frame_id_;
  int target_fps_;
  int width_;
  int height_;
  int initial_retry_ms_;
  int max_retry_ms_;
  double backoff_;
  int max_failures_;

public:
  CameraDriverNode() : Node("camera_driver_node")
  {
    // Declare and retrieve all parameters in one step
    load_parameters();

    // Setup publisher
    pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/camera/image/compressed", 10);

    RCLCPP_INFO(this->get_logger(),
                "Camera Driver initialized | FPS: %d | Retry: %dms->%dms (%.1fx backoff)",
                target_fps_, initial_retry_ms_, max_retry_ms_, backoff_);

    // Start connection attempt
    start_connection_attempt();

    // Setup capture timer (paused until connected)
    capture_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / target_fps_),
        [this]()
        { capture_and_publish(); });
    capture_timer_->cancel();
  }

private:
  void load_parameters()
  {
    urls_ = this->declare_and_get<std::vector<std::string>>(
        "camera_urls", {"http://192.168.1.11:8080/video", "http://192.168.43.1:8080/video"});
    frame_id_ = this->declare_and_get<std::string>("frame_id", "camera_link_optical");
    target_fps_ = this->declare_and_get<int>("target_fps", DEFAULT_FPS);
    width_ = this->declare_and_get<int>("width", DEFAULT_WIDTH);
    height_ = this->declare_and_get<int>("height", DEFAULT_HEIGHT);
    initial_retry_ms_ = this->declare_and_get<int>("initial_retry_delay_ms", DEFAULT_INITIAL_RETRY_MS);
    max_retry_ms_ = this->declare_and_get<int>("max_retry_delay_ms", DEFAULT_MAX_RETRY_MS);
    backoff_ = this->declare_and_get<double>("backoff_multiplier", DEFAULT_BACKOFF);
    max_failures_ = this->declare_and_get<int>("max_consecutive_failures", 999999999);
  }

  template <typename T>
  T declare_and_get(const std::string &name, const T &default_value)
  {
    this->declare_parameter<T>(name, default_value);
    return this->get_parameter(name).get_value<T>();
  }

  void set_state(State new_state)
  {
    if (state_ == new_state)
      return;

    state_ = new_state;
    const char *state_names[] = {"DISCONNECTED", "CONNECTING", "CONNECTED", "FAILED"};
    RCLCPP_INFO(this->get_logger(), "State → %s", state_names[static_cast<int>(new_state)]);
  }

  void start_connection_attempt()
  {
    set_state(State::CONNECTING);
    if (reconnect_timer_)
      reconnect_timer_->cancel();

    reconnect_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(QUICK_RETRY_MS),
        [this]()
        { try_connect(); });
  }

  void try_connect()
  {
    if (state_ == State::CONNECTED || state_ == State::FAILED)
      return;

    if (attempt_connect_to_any_camera())
    {
      reconnect_timer_->cancel();
      if (capture_timer_)
        capture_timer_->reset();
      return;
    }

    consecutive_failures_++;
    if (consecutive_failures_ >= max_failures_)
    {
      set_state(State::FAILED);
      RCLCPP_ERROR(this->get_logger(), "Max failures (%d) reached. Stopping.", max_failures_);
      reconnect_timer_->cancel();
      return;
    }

    // Exponential backoff
    current_retry_ms_ = std::min(static_cast<int>(current_retry_ms_ * backoff_), max_retry_ms_);
    RCLCPP_WARN(this->get_logger(), "Connection failed. Retry %d/%d in %dms",
                consecutive_failures_, max_failures_, current_retry_ms_);

    // Reschedule with new delay
    reconnect_timer_->cancel();
    reconnect_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(current_retry_ms_),
        [this]()
        { try_connect(); });
  }

  bool attempt_connect_to_any_camera()
  {
    for (const auto &url : urls_)
    {
      if (try_open_camera(url))
      {
        set_state(State::CONNECTED);
        consecutive_failures_ = 0;
        current_retry_ms_ = initial_retry_ms_;
        RCLCPP_INFO(this->get_logger(), "✓ Connected to: %s", url.c_str());
        return true;
      }
    }
    return false;
  }

  bool try_open_camera(const std::string &url)
  {
    cv::VideoCapture test_cap(url, cv::CAP_FFMPEG);
    if (!test_cap.isOpened())
      return false;

    cv::Mat frame;
    if (!test_cap.read(frame) || frame.empty())
    {
      test_cap.release();
      return false;
    }

    cap_ = std::move(test_cap);
    configure_camera();
    return true;
  }

  void configure_camera()
  {
    cap_.set(cv::CAP_PROP_BUFFERSIZE, DEFAULT_BUFFER_SIZE);
    if (width_ > 0 && height_ > 0)
    {
      cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
      cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    }
  }

  void capture_and_publish()
  {
    if (state_ != State::CONNECTED)
      return;

    cv::Mat frame;
    if (!cap_.isOpened() || !cap_.read(frame) || frame.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Camera lost. Reconnecting...");
      start_connection_attempt();
      return;
    }

    std::vector<uchar> buf;
    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, DEFAULT_JPEG_QUALITY};
    if (!cv::imencode(".jpg", frame, buf, params))
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to encode frame");
      return;
    }

    auto msg = sensor_msgs::msg::CompressedImage();
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;
    msg.format = "jpeg";
    msg.data = std::move(buf);
    pub_->publish(msg);
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CameraDriverNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
