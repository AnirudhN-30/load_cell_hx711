#include <chrono>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"

using namespace std::chrono_literals;

float adcToGrams(long adc)
{
  return (adc + 1367000) * 121.0 / (-154000.0);
}

class LoadCellNode : public rclcpp::Node
{
public:
  LoadCellNode()
  : Node("load_cell")
  {
    topic_ = this->declare_parameter<std::string>("topic", "/eagle10/fluidpressure");
    port_ = this->declare_parameter<std::string>("port", "/dev/ttyACM0");
    frame_id_ = this->declare_parameter<std::string>("frame_id", "load_cell");
    variance_ = this->declare_parameter<double>(
      "variance", std::numeric_limits<double>::quiet_NaN());
    const double read_rate_hz = this->declare_parameter<double>("read_rate_hz", 500.0);
    calibration_duration_ = this->declare_parameter<double>("calibration_duration", 5.0);

    publisher_ = this->create_publisher<sensor_msgs::msg::FluidPressure>(topic_, 10);

    openSerialPort();
    calibration_start_time_ = std::chrono::steady_clock::now();

    const auto period = std::chrono::duration<double>(1.0 / read_rate_hz);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&LoadCellNode::readSerial, this));

    RCLCPP_INFO(this->get_logger(), "Publishing FluidPressure on %s", topic_.c_str());
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
    serial_port_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (serial_port_ < 0) {
      RCLCPP_ERROR(
        this->get_logger(), "Failed to open port %s: %s", port_.c_str(), strerror(errno));
      return;
    }

    struct termios tty;
    memset(&tty, 0, sizeof tty);

    if (tcgetattr(serial_port_, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcgetattr failed: %s", strerror(errno));
      close(serial_port_);
      serial_port_ = -1;
      return;
    }

    cfsetispeed(&tty, B921600);
    cfsetospeed(&tty, B921600);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_iflag = 0;

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(this->get_logger(), "tcsetattr failed: %s", strerror(errno));
      close(serial_port_);
      serial_port_ = -1;
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Reading load cell data from %s", port_.c_str());
  }

  void readSerial()
  {
    if (serial_port_ < 0) {
      return;
    }

    char serial_data[256];
    while (true) {
      const int bytes_read = read(serial_port_, serial_data, sizeof(serial_data));

      if (bytes_read <= 0) {
        return;
      }

      for (int i = 0; i < bytes_read; ++i) {
        const char c = serial_data[i];

        if (c == '\n') {
          buffer_[buffer_index_] = '\0';
          const long sensor_value = std::atol(buffer_);

          if (sensor_value == 0 && has_last_sensor_value_) {
            handleSensorValue(last_sensor_value_);
          } else if (sensor_value != 0) {
            handleSensorValue(sensor_value);
          }

          buffer_index_ = 0;
        } else if (c != '\r' && buffer_index_ < static_cast<int>(sizeof(buffer_) - 1)) {
          buffer_[buffer_index_++] = c;
        }
      }
    }
  }

  void handleSensorValue(const long sensor_value)
  {
    last_sensor_value_ = sensor_value;
    has_last_sensor_value_ = true;

    if (!calibrated_) {
      calibration_sum_ += sensor_value;
      ++calibration_samples_;

      const auto elapsed = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - calibration_start_time_);

      if (elapsed.count() >= calibration_duration_) {
        start_raw_adc_ = static_cast<double>(calibration_sum_) /
          static_cast<double>(calibration_samples_);
        calibrated_ = true;
        RCLCPP_INFO(
          this->get_logger(), "Load cell start raw average: %.3f from %ld samples",
          start_raw_adc_, calibration_samples_);
      }

      return;
    }

    const double relative_grams =
      static_cast<double>(adcToGrams(sensor_value)) - static_cast<double>(adcToGrams(start_raw_adc_));
    publishPressure(relative_grams);
  }

  void publishPressure(const double fluid_pressure_pa)
  {
    this->get_parameter("variance", variance_);

    sensor_msgs::msg::FluidPressure msg;
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = frame_id_;
    msg.fluid_pressure = fluid_pressure_pa;
    msg.variance = variance_;

    publisher_->publish(msg);
  }

  std::string topic_;
  std::string port_;
  std::string frame_id_;
  double variance_;
  int serial_port_{-1};
  char buffer_[256]{};
  int buffer_index_{0};
  double calibration_duration_{5.0};
  std::chrono::steady_clock::time_point calibration_start_time_;
  std::int64_t calibration_sum_{0};
  long calibration_samples_{0};
  double start_raw_adc_{0.0};
  long last_sensor_value_{0};
  bool has_last_sensor_value_{false};
  bool calibrated_{false};
  rclcpp::Publisher<sensor_msgs::msg::FluidPressure>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LoadCellNode>());
  rclcpp::shutdown();
  return 0;
}
