#include "device.hpp"
#include "logger.hpp"
#include "piezo_serial.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include <chrono>
#include <cstdint>
#include <memory>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <std_msgs/msg/detail/u_int8_multi_array__struct.hpp>
#include <sys/types.h>

using namespace std::chrono_literals;

class SerialReaderNode : public rclcpp::Node {
public:
  SerialReaderNode(std::string devName, std::string outputDir)
      : Node("serial_reader"), piezoSerial(devName), logger(outputDir) {

    publisher_ = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
        "sensor/raw", 10);

    piezoSerial.addCallback(
        [&](PacketStruct packet) -> void { logger.appendData(packet); });

    piezoSerial.addCallback([&](PacketStruct packet) -> void {
      RCLCPP_INFO(this->get_logger(), "Publishing: '%u'", packet.tick);

      std_msgs::msg::UInt8MultiArray message = std_msgs::msg::UInt8MultiArray();
      message.data.resize(sizeof(PacketStruct));
      memcpy(message.data.data(), &packet, sizeof(PacketStruct));
      publisher_->publish(message);

      // auto message = std_msgs::msg::String();
      // message.data.resize(sizeof(PacketStruct));
      // memcpy(message.data.data(), &packet, sizeof(PacketStruct));
      // publisher_->publish(message);

      // auto message = std_msgs::msg::String();

      // std::stringstream ss;

      // for (int i = 0; i < 32; i++) {
      //   ss << packet.mic1[i] << " ";
      // }
      // for (int i = 0; i < 32; i++) {
      //   ss << packet.mic2[i] << " ";
      // }

      // std::string result = ss.str();
      // message.data = result;

      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'",
      // message.data.c_str()); publisher_->publish(message);
    });

    logger.start();
    piezoSerial.start();
  }
  ~SerialReaderNode() {
    piezoSerial.stop();
    logger.stop();
  }

private:
  PiezoSerial piezoSerial;
  Logger logger;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialReaderNode>("/dev/ttyUSB0", "./output/"));
  rclcpp::shutdown();
  return 0;
}
