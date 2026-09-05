#ifndef SPR_REMOTE_CONTROL__SERIAL_DRIVER_HPP
#define SPR_REMOTE_CONTROL__SERIAL_DRIVER_HPP

#include <cstdint>
#include <string>

// Linux termios2 串口驱动（移植自参考 dt7_receiver/serial_driver_v2）。
// 用 ioctl(TCGETS2/TCSETS2) + BOTHER 支持任意波特率（如 DR16 DBUS 100000bps），
// 比 tcsetattr 的 B1000000 表驱动方式更通用、更可靠。

namespace spr_remote_control
{

class SerialDriver
{
public:
  SerialDriver();
  ~SerialDriver();
  SerialDriver(const SerialDriver &) = delete;
  SerialDriver & operator=(const SerialDriver &) = delete;

  /// @brief 打开串口（8 数据位 / 1 停止位 / 无流控 / 原始模式）
  /// @param device   设备路径，如 "/dev/ttyTHS1"
  /// @param baudrate 波特率，支持任意值（如 100000）
  /// @param parity   校验位：'N'=无, 'E'=偶, 'O'=奇
  bool open(const std::string & device, int baudrate, char parity = 'N');

  void close();
  bool isOpen() const { return fd_ >= 0; }
  int fd() const { return fd_; }

  ssize_t send(const uint8_t * data, size_t len);
  /// @brief 阻塞读（VMIN/VTIME=0，实际近乎非阻塞，见实现说明）
  ssize_t recv(uint8_t * data, size_t len);
  /// @brief 带超时的读（poll 实现，timeout_ms<=0 表示不等待）
  ssize_t recv_timeout(uint8_t * data, size_t len, int timeout_ms);
  void drain();
  /// @param queue 同 tcflush：TCIFLUSH/ TCOFLUSH/ TCIOFLUSH
  void flush(int queue);

private:
  int fd_;
};

}  // namespace spr_remote_control

#endif  // SPR_REMOTE_CONTROL__SERIAL_DRIVER_HPP
