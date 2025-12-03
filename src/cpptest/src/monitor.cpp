#include "logger.hpp"
#include "piezo_serial.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_spectrogram_widget.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include <cstdint>
#include <memory>
#include <qapplication.h>
#include <rclcpp/executors/single_threaded_executor.hpp>

class PacketListener : public rclcpp::Node {
public:
  PacketListener(std::shared_ptr<RealtimeSpectrogramWidget> widget)
      : Node("packet_listener"), widget(widget) {
    sub_ = this->create_subscription<std_msgs::msg::UInt8MultiArray>(
        "/sensor/raw", 10,
        std::bind(&PacketListener::onPacketReceived, this,
                  std::placeholders::_1));
  }

private:
  void onPacketReceived(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    if (msg->data.size() != sizeof(PacketStruct)) {
      RCLCPP_WARN(this->get_logger(), "Invalid packet size received.");
      return;
    }

    PacketStruct packet;
    memcpy(&packet, msg->data.data(), sizeof(PacketStruct));

    QMetaObject::invokeMethod(
        widget.get(), [this, packet]() { widget->appendData(packet); },
        Qt::QueuedConnection);
  }
  std::shared_ptr<RealtimeSpectrogramWidget> widget;
  rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub_;
};

int main(int argc, char *argv[]) {
  QApplication app(argc, argv);

  std::shared_ptr<RealtimeSpectrogramWidget> widget =
      std::make_shared<RealtimeSpectrogramWidget>();
  widget->resize(2000, 300);
  widget->show();

  rclcpp::init(argc, argv);
  std::shared_ptr<PacketListener> listener =
      std::make_shared<PacketListener>(widget);

  std::thread ros_thread([listener]() {
    // rclcpp::executors::MultiThreadedExecutor exec;
    rclcpp::executors::SingleThreadedExecutor exec;
    exec.add_node(listener);
    exec.spin();
  });

  int ret = app.exec();

  rclcpp::shutdown();
  // ros_thread.join();

  return ret;

  // QApplication app(argc, argv);

  // std::shared_ptr<RealtimeSpectrogramWidget> widget =
  //     std::make_shared<RealtimeSpectrogramWidget>();
  // widget->resize(2000, 300);
  // widget->show();

  // rclcpp::init(argc, argv);

  // auto listener = std::make_shared<PacketListener>(widget);

  // // ROS spin 별도 스레드
  // std::thread ros_thread([listener]() {
  //   rclcpp::executors::MultiThreadedExecutor exec;
  //   exec.add_node(listener);
  //   exec.spin();
  // });

  // auto listener = std::make_shared<PacketListener>(widget);

  // // ROS -> Qt 안전한 스레드 업데이트
  // listener->setCallback([widget](const PacketStruct &packet) {
  //   QMetaObject::invokeMethod(
  //       widget.get(), [widget, packet]() { widget->appendData(packet); },
  //       Qt::QueuedConnection);
  // });

  // // ROS spin 별도 스레드
  // std::thread ros_thread([listener]() {
  //   rclcpp::executors::MultiThreadedExecutor exec;
  //   exec.add_node(listener);
  //   exec.spin();
  // });

  // int ret = app.exec();

  // rclcpp::shutdown();
  // ros_thread.join();

  // return ret;
  // return 0;

  // QApplication app(argc, argv);

  // RealtimeSpectrogramWidget widget;
  // widget.resize(2000, 300);
  // widget.show();

  // rclcpp::init(argc, argv);
  // rclcpp::spin(std::make_shared<PacketListener>());

  // auto listener = std::make_shared<PacketListener>();

  // listener->addCallback([&widget](PacketStruct packet) {
  //   QMetaObject::invokeMethod(
  //       z & widget, [packet, &widget]() { widget.appendData(packet); });
  // });

  // std::thread ros_thread([listener]() {
  //   rclcpp::executors::MultiThreadedExecutor exec;
  //   exec.add_node(listener);
  //   exec.spin();
  // });

  // int ret = app.exec();
  // rclcpp::shutdown();

  // return ret;
  // return 0;
  // PiezoSerial piezoSerial;
  // if (argc < 2) {
  //   std::string port = "/dev/ttyUSB0";
  //   piezoSerial.setPort(port);
  // } else {
  //   piezoSerial.setPort(argv[1]);
  // }

  // Logger logger("./logs/");

  // piezoSerial.addCallback(
  //     [](PacketStruct packet) -> void { std::cout << packet << std::endl; });
  // piezoSerial.addCallback(
  //     [&logger](PacketStruct packet) -> void { logger.appendData(packet); });
  // piezoSerial.addCallback(
  //     [&widget](PacketStruct packet) -> void { widget.appendData(packet); });

  // logger.start();
  // piezoSerial.start();

  // // sleep(10);

  // piezoSerial.stop();
  // logger.stop();
}
