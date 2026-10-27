#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <limits>
#include <string>
#include <thread>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"

class LoadCellNode : public rclcpp::Node
{
public:
  LoadCellNode()
  : Node("load_cell")
  {
    // ---------------------------------------------------------
    // Parameters
    // ---------------------------------------------------------

    // Relative topic names:
    // ROS 2 automatically adds the namespace from the launch file.
    //
    // Example:
    // namespace = eagle11
    // load_cell/grams -> /eagle11/load_cell/grams

    topic_ = declare_parameter<std::string>(
      "topic", "fluidpressure");

    grams_topic_ = declare_parameter<std::string>(
      "grams_topic", "load_cell/grams");

    tension_topic_ = declare_parameter<std::string>(
      "tension_topic", "load_cell/tension");

    port_ = declare_parameter<std::string>(
      "port", "/dev/ttyACM0");

    frame_id_ = declare_parameter<std::string>(
      "frame_id", "load_cell");

    variance_ = declare_parameter<double>(
      "variance",
      std::numeric_limits<double>::quiet_NaN());

    read_rate_hz_ = declare_parameter<double>(
      "read_rate_hz", 500.0);

    calibration_zero_delay_sec_ = declare_parameter<double>(
      "calibration_zero_delay_sec", 2.0);

    calibration_weight_delay_sec_ = declare_parameter<double>(
      "calibration_weight_delay_sec", 5.0);

    // ---------------------------------------------------------
    // Publishers
    // ---------------------------------------------------------

    // Legacy publisher (value is tension in N, not pressure in Pa)
    pressure_pub_ =
      create_publisher<sensor_msgs::msg::FluidPressure>(
        topic_, 10);

    // Weight in grams
    grams_pub_ =
      create_publisher<std_msgs::msg::Float64>(
        grams_topic_, 10);

    // Tension in Newtons
    tension_pub_ =
      create_publisher<std_msgs::msg::Float64>(
        tension_topic_, 10);

    // ---------------------------------------------------------
    // Subscribers
    // ---------------------------------------------------------

    // Tare command
    tare_sub_ =
      create_subscription<std_msgs::msg::Empty>(
        "load_cell/tare",
        10,
        std::bind(
          &LoadCellNode::tareCallback,
          this,
          std::placeholders::_1));

    // Calibration command
    calibrate_sub_ =
      create_subscription<std_msgs::msg::Float64>(
        "load_cell/calibrate",
        10,
        std::bind(
          &LoadCellNode::calibrateCallback,
          this,
          std::placeholders::_1));

    // ---------------------------------------------------------
    // Serial port
    // ---------------------------------------------------------

    openSerialPort();

    // ---------------------------------------------------------
    // Serial reading timer
    // ---------------------------------------------------------

    if (read_rate_hz_ <= 0.0) {
      RCLCPP_WARN(
        get_logger(),
        "Invalid read_rate_hz. Using 500 Hz.");

      read_rate_hz_ = 500.0;
    }

    const auto period =
      std::chrono::duration<double>(1.0 / read_rate_hz_);

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LoadCellNode::readSerial, this));

    // ---------------------------------------------------------
    // Logging
    // ---------------------------------------------------------

    RCLCPP_INFO(
      get_logger(),
      "Load cell node started");

    RCLCPP_INFO(
      get_logger(),
      "Namespace: %s",
      get_namespace());

    RCLCPP_INFO(
      get_logger(),
      "Serial port: %s",
      port_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Grams topic: %s",
      grams_pub_->get_topic_name());

    RCLCPP_INFO(
      get_logger(),
      "Tension topic: %s",
      tension_pub_->get_topic_name());

    RCLCPP_INFO(
      get_logger(),
      "Legacy fluidpressure topic: %s",
      pressure_pub_->get_topic_name());

    RCLCPP_INFO(
      get_logger(),
      "Tare topic: %s",
      tare_sub_->get_topic_name());

    RCLCPP_INFO(
      get_logger(),
      "Calibration topic: %s",
      calibrate_sub_->get_topic_name());
  }
  
  ~LoadCellNode()
  {
    if (serial_port_ >= 0) {
      close(serial_port_);
    }
  }

private:
  void openSerialPort()
  {
    serial_port_ = open(
      port_.c_str(),
      O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (serial_port_ < 0) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to open %s: %s",
        port_.c_str(),
        strerror(errno));

      return;
    }

    termios tty{};

    if (tcgetattr(serial_port_, &tty) != 0) {
      RCLCPP_ERROR(
        get_logger(),
        "tcgetattr failed: %s",
        strerror(errno));

      close(serial_port_);
      serial_port_ = -1;
      return;
    }

    // Baud rate
    cfsetispeed(&tty, B921600);
    cfsetospeed(&tty, B921600);

    // 8 data bits, no parity, 1 stop bit
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    // Raw serial mode
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag = 0;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(
        get_logger(),
        "tcsetattr failed: %s",
        strerror(errno));

      close(serial_port_);
      serial_port_ = -1;
      return;
    }

    tcflush(serial_port_, TCIOFLUSH);

    RCLCPP_INFO(
      get_logger(),
      "Serial port opened successfully: %s",
      port_.c_str());
  }

  void readSerial()
  {
    if (serial_port_ < 0) {
      return;
    }

    char serial_data[256];

    while (true) {
      const ssize_t bytes_read = read(
        serial_port_,
        serial_data,
        sizeof(serial_data));

      if (bytes_read < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK &&
            errno != EINTR) {
          RCLCPP_ERROR(
            get_logger(),
            "Serial read failed: %s",
            strerror(errno));
        }
        return;
      }

      if (bytes_read == 0) {
        return;
      }

      for (ssize_t i = 0; i < bytes_read; ++i) {
        const char c = serial_data[i];

        if (c == '\n') {
          buffer_[buffer_index_] = '\0';

          handleLine(std::string(buffer_));

          buffer_index_ = 0;
        }
        else if (
          c != '\r' &&
          buffer_index_ <
            static_cast<int>(sizeof(buffer_) - 1))
        {
          buffer_[buffer_index_++] = c;
        }
      }
    }
  }

  void handleLine(const std::string & line)
  {
    if (line.empty()) {
      return;
    }

    char * end = nullptr;

    errno = 0;

    const double grams =
      std::strtod(line.c_str(), &end);

    // Not a numeric reading: treat as Teensy status text.
    if (end == line.c_str()) {
      RCLCPP_INFO(
        get_logger(),
        "Teensy: %s",
        line.c_str());

      return;
    }

    // Reject partially numeric or invalid readings.
    while (*end == ' ' || *end == '\t') {
      ++end;
    }

    if (*end != '\0' ||
        errno == ERANGE ||
        !std::isfinite(grams)) {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring invalid serial reading: %s",
        line.c_str());

      return;
    }

    // Convert equivalent grams to Newtons.
    const double tension_N =
      grams * 9.80665 / 1000.0;

    publishGrams(grams);
    publishTension(tension_N);
    publishPressure(tension_N);
  }

  void publishGrams(double grams)
  {
    std_msgs::msg::Float64 msg;

    msg.data = grams;

    grams_pub_->publish(msg);
  }

  void publishTension(double tension_N)
  {
    std_msgs::msg::Float64 msg;

    msg.data = tension_N;

    tension_pub_->publish(msg);
  }

  void publishPressure(double tension_N)
  {
    sensor_msgs::msg::FluidPressure msg;

    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = frame_id_;

    // Legacy compatibility only:
    // This field normally expects Pascals.
    // Here it contains tension in Newtons.
    msg.fluid_pressure = tension_N;

    msg.variance = variance_;

    pressure_pub_->publish(msg);
  }

  void tareCallback(
    const std_msgs::msg::Empty::SharedPtr)
  {
    if (sendSerial("t\n")) {
      RCLCPP_INFO(
        get_logger(),
        "Sent tare command to Teensy");
    }
  }

  void calibrateCallback(
    const std_msgs::msg::Float64::SharedPtr msg)
  {
    const double known_weight_g = msg->data;

    if (!std::isfinite(known_weight_g) ||
        known_weight_g <= 0.0) {
      RCLCPP_ERROR(
        get_logger(),
        "Calibration weight must be finite and greater than 0 grams");

      return;
    }

    if (serial_port_ < 0) {
      RCLCPP_ERROR(
        get_logger(),
        "Cannot calibrate: serial port not open");

      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Starting calibration with %.2f grams",
      known_weight_g);

    // Tell Teensy to start calibration.
    if (!sendSerial("c\n")) {
      return;
    }

    // Wait for unloaded calibration step.
    std::this_thread::sleep_for(
      std::chrono::duration<double>(
        calibration_zero_delay_sec_));

    // Confirm unloaded scale.
    if (!sendSerial("\n")) {
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Place %.2f g on the load cell now",
      known_weight_g);

    // Allow time to place the calibration mass.
    std::this_thread::sleep_for(
      std::chrono::duration<double>(
        calibration_weight_delay_sec_));

    // Confirm that the calibration mass is in place.
    if (!sendSerial("\n")) {
      return;
    }

    std::this_thread::sleep_for(
      std::chrono::milliseconds(200));

    // Send known mass in grams.
    if (!sendSerial(
        std::to_string(known_weight_g) + "\n")) {
      return;
    }

    RCLCPP_INFO(
      get_logger(),
      "Calibration command sequence sent");
  }

  bool sendSerial(const std::string & data)
  {
    if (serial_port_ < 0) {
      RCLCPP_ERROR(
        get_logger(),
        "Serial port not open");

      return false;
    }

    size_t bytes_sent = 0;

    while (bytes_sent < data.size()) {
      const ssize_t result = write(
        serial_port_,
        data.data() + bytes_sent,
        data.size() - bytes_sent);

      if (result > 0) {
        bytes_sent += static_cast<size_t>(result);
      }
      else if (result < 0 &&
               errno == EINTR) {
        continue;
      }
      else if (result < 0 &&
               (errno == EAGAIN ||
                errno == EWOULDBLOCK)) {
        std::this_thread::sleep_for(
          std::chrono::milliseconds(1));
      }
      else {
        RCLCPP_ERROR(
          get_logger(),
          "Serial write failed: %s",
          strerror(errno));

        return false;
      }
    }

    return true;
  }

  std::string topic_;
  std::string grams_topic_;
  std::string tension_topic_;
  std::string port_;
  std::string frame_id_;

  double variance_;
  double read_rate_hz_;

  double calibration_zero_delay_sec_;
  double calibration_weight_delay_sec_;

  int serial_port_{-1};

  char buffer_[256]{};
  int buffer_index_{0};

  // Publishers
  rclcpp::Publisher<
    sensor_msgs::msg::FluidPressure>::SharedPtr pressure_pub_;

  rclcpp::Publisher<
    std_msgs::msg::Float64>::SharedPtr grams_pub_;

  rclcpp::Publisher<
    std_msgs::msg::Float64>::SharedPtr tension_pub_;

  // Subscribers
  rclcpp::Subscription<
    std_msgs::msg::Empty>::SharedPtr tare_sub_;

  rclcpp::Subscription<
    std_msgs::msg::Float64>::SharedPtr calibrate_sub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(
    std::make_shared<LoadCellNode>());

  rclcpp::shutdown();

  return 0;
}