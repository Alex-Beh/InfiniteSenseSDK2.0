#include <fstream>
#include <string>
#include <sstream>
#include <iomanip>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/time_reference.hpp>

class GprmcPublisher : public rclcpp::Node {
 public:
  GprmcPublisher() : Node("gprmc_publisher") {
    this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    this->declare_parameter<int>("baud", 115200);

    std::string port;
    int baud;
    this->get_parameter("port", port);
    this->get_parameter("baud", baud);

    publisher_ = this->create_publisher<std_msgs::msg::String>("/gps/gprmc", 10);
    time_pub_ = this->create_publisher<sensor_msgs::msg::TimeReference>("/gps/time_reference", 10);

    fd_ = open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open port: %s", port.c_str());
      rclcpp::shutdown();
      return;
    }

    configure_serial(fd_, baud);
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&GprmcPublisher::read_and_publish, this));

    RCLCPP_INFO(this->get_logger(), "Opened %s @ %d baud", port.c_str(), baud);
  }

  ~GprmcPublisher() {
    if (fd_ >= 0) close(fd_);
  }

 private:
  void configure_serial(int fd, int baudrate) {
    struct termios tty;
    tcgetattr(fd, &tty);
    cfsetospeed(&tty, baudrate_to_constant(baudrate));
    cfsetispeed(&tty, baudrate_to_constant(baudrate));
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag = IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 10;
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    tcsetattr(fd, TCSANOW, &tty);
  }

  speed_t baudrate_to_constant(int baud) {
    switch (baud) {
      case 9600:
        return B9600;
      case 19200:
        return B19200;
      case 38400:
        return B38400;
      case 57600:
        return B57600;
      case 115200:
        return B115200;
      default:
        RCLCPP_FATAL(this->get_logger(), "Unsupported baudrate: %d", baud);
        rclcpp::shutdown();
        return B9600;
    }
  }

  void read_and_publish() {
    char buf[256];
    ssize_t n = read(fd_, buf, sizeof(buf));
    if (n <= 0) return;

    for (ssize_t i = 0; i < n; ++i) {
      if (buf[i] == '$') {
        line_.clear();
      }
      line_ += buf[i];
      if (buf[i] == '\n') {
        if (is_valid_checksum(line_)) {
          // ---> Publish GPRMC as String
          std_msgs::msg::String msg;
          msg.data = line_;
          std::cout << "Data: " << line_ << std::endl;
          publisher_->publish(msg);

          // ---> Publish GPRMC as TimeReference
          auto maybe_time = parse_gprmc_utc(line_);
          if (maybe_time.has_value()) {
            sensor_msgs::msg::TimeReference tref;
            tref.header.stamp = this->now();
            tref.header.frame_id = "infinite_sense";
            tref.source = "infinite_sense";
            tref.time_ref = maybe_time.value();
            time_pub_->publish(tref);
          }
        }
        line_.clear();
      }
    }
  }

  bool is_valid_checksum(const std::string &line) {
    auto asterisk = line.find('*');
    if (asterisk == std::string::npos || asterisk + 2 >= line.size()) return false;

    uint8_t checksum = 0;
    for (size_t i = 1; i < asterisk; ++i) {
      checksum ^= line[i];
    }

    std::istringstream iss(line.substr(asterisk + 1, 2));
    int sent_checksum;
    iss >> std::hex >> sent_checksum;
    return checksum == sent_checksum;
  }

  std::optional<builtin_interfaces::msg::Time> parse_gprmc_utc(const std::string &nmea_line) {
    std::vector<std::string> fields;
    std::istringstream ss(nmea_line);
    std::string token;
    while (std::getline(ss, token, ',')) {
      fields.push_back(token);
    }

    if (fields.size() > 9 && fields[1].length() >= 6 && fields[9].length() >= 6) {
      try {
        std::string hh = fields[1].substr(0, 2);
        std::string mm = fields[1].substr(2, 2);
        std::string ss = fields[1].substr(4, 2);

        std::string dd = fields[9].substr(0, 2);
        std::string mo = fields[9].substr(2, 2);
        std::string yy = fields[9].substr(4, 2);

        std::tm timeinfo = {};
        timeinfo.tm_year = std::stoi(yy) + 100;
        timeinfo.tm_mon = std::stoi(mo) - 1;
        timeinfo.tm_mday = std::stoi(dd);
        timeinfo.tm_hour = std::stoi(hh);
        timeinfo.tm_min = std::stoi(mm);
        timeinfo.tm_sec = std::stoi(ss);
        timeinfo.tm_isdst = 0;

        time_t utc_sec = timegm(&timeinfo);
        if (utc_sec == -1) return std::nullopt;

        builtin_interfaces::msg::Time t;
        t.sec = static_cast<int32_t>(utc_sec);
        t.nanosec = 0;
        return t;
      } catch (...) {
        return std::nullopt;
      }
    }

    return std::nullopt;
  }

  int fd_;
  std::string line_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<sensor_msgs::msg::TimeReference>::SharedPtr time_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GprmcPublisher>());
  rclcpp::shutdown();
  return 0;
}
