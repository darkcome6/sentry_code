#include "spr_remote_control/serial_driver.hpp"

#include <fcntl.h>
#include <unistd.h>
#include <asm/termbits.h>   // 唯一串口头文件：termios2 + 全部常量（不需要 <termios.h>）
#include <cerrno>
#include <cstring>
#include <iostream>
#include <poll.h>
#include <sys/ioctl.h>

namespace spr_remote_control
{

SerialDriver::SerialDriver() : fd_(-1) {}
SerialDriver::~SerialDriver() { close(); }

bool SerialDriver::open(const std::string & device, int baudrate, char parity)
{
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    std::cerr << "[SerialDriver] 无法打开 " << device << ": "
              << strerror(errno) << std::endl;
    return false;
  }

  // ────── 全程用 termios2 + ioctl ──────
  struct termios2 tty;
  if (ioctl(fd_, TCGETS2, &tty) != 0) {
    std::cerr << "[SerialDriver] TCGETS2 失败" << std::endl;
    close();
    return false;
  }

  // 波特率（BOTHER 自定义任意值）
  tty.c_cflag &= ~CBAUD;
  tty.c_cflag |= BOTHER;
  tty.c_ospeed = baudrate;
  tty.c_ispeed = baudrate;

  // 数据位：8
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;

  // 校验位
  switch (parity) {
    case 'E':
    case 'e':
      tty.c_cflag |= PARENB;
      tty.c_cflag &= ~PARODD;
      break;
    case 'O':
    case 'o':
      tty.c_cflag |= PARENB;
      tty.c_cflag |= PARODD;
      break;
    default:
      tty.c_cflag &= ~PARENB;
      break;
  }

  // 停止位 / 流控 / 接收
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cflag |= CREAD | CLOCAL;

  // 原始模式
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK
                   | ISTRIP | INLCR | IGNCR | ICRNL);
  tty.c_oflag &= ~(OPOST | ONLCR);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  if (ioctl(fd_, TCSETS2, &tty) != 0) {
    std::cerr << "[SerialDriver] TCSETS2 失败: " << strerror(errno) << std::endl;
    close();
    return false;
  }

  // 清除残留
  ioctl(fd_, TCFLSH, TCIOFLUSH);

  // 去掉 O_NONBLOCK（配合 VMIN/VTIME=0：read 若无可读字节立即返回 0，
  // 不会阻塞——结合上层的 poll/recv_timeout 使用）
  {
    int flags = fcntl(fd_, F_GETFL, 0);
    if (flags != -1) {
      fcntl(fd_, F_SETFL, flags & ~O_NONBLOCK);
    }
  }

  std::cout << "[SerialDriver] " << device << " 已打开, " << baudrate
            << " bps, 8" << parity << "1" << std::endl;
  return true;
}

void SerialDriver::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

void SerialDriver::drain()
{
  if (fd_ >= 0) ioctl(fd_, TCSBRK, 1);   // ioctl 版 tcdrain
}

void SerialDriver::flush(int queue)
{
  if (fd_ >= 0) ioctl(fd_, TCFLSH, queue);   
  // ioctl 代tcflush
}

ssize_t SerialDriver::send(const uint8_t * data, size_t len)
{
  return fd_ < 0 ? -1 : ::write(fd_, data, len);
}

ssize_t SerialDriver::recv(uint8_t * data, size_t len)
{
  return fd_ < 0 ? -1 : ::read(fd_, data, len);
}

ssize_t SerialDriver::recv_timeout(uint8_t * data, size_t len, int timeout_ms)
{
  if (fd_ < 0) return -1; // 串口没打开 → 报错
  struct pollfd pfd{fd_, POLLIN, 0};// 声明要监听：fd_ 上是否有 POLLIN(可读)
  int ret = poll(&pfd, 1, timeout_ms);// 阻塞等待最多 timeout_ms
  if (ret < 0) return -1; // poll 出错(被信号打断等) → 报错
  if (ret == 0 || !(pfd.revents & POLLIN)) return 0;// 超时/没可读 → 返回 0
  return ::read(fd_, data, len);// 有数据了 → 真正读走，返回实际字节数
}

}  // namespace spr_remote_control
