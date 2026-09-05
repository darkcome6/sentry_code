// 哨兵串口遥控接收节点（C++，统一包 spr_remote_control 的一部分）。
// 用法示例（真机）:
//   ros2 run spr_remote_control rc_serial_remote_cpp --ros-args
//     -p device:=/dev/ttyTHS1 -p baudrate:=100000 -p parity:=even
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <spr_msgs/msg/gimbal_cmd.hpp>

#include "spr_remote_control/serial_driver.hpp"
#include "spr_remote_control/dbus_parser.hpp"

namespace spr_remote_control
{
namespace
{
// 云台模式（与 spr_sentry_controllers/gimbal_controller 约定一致）
constexpr uint8_t GIMBAL_MODE_HOLD = 0;    // 保持
constexpr uint8_t GIMBAL_MODE_SCAN = 1;    // 扫描
constexpr uint8_t GIMBAL_MODE_AIM = 2;     // 自瞄
constexpr uint8_t GIMBAL_MODE_REMOTE = 3;  // 遥控

// 缓冲上限（约 4 帧），防串口粘包/异常涨满
constexpr size_t RX_BUF_MAX = DBUS_FRAME_LEN * 4;

// 右拨杆 s[1] 位置 → 云台模式（下=保持 中=遥控 上=自瞄）
// 返回 0xFF 表示非法/过渡档，调用方保持上一模式。
uint8_t mode_from_switch(uint8_t sw)
{
  switch (sw) {
    case SW_DOWN: return GIMBAL_MODE_HOLD;
    case SW_MID: return GIMBAL_MODE_REMOTE;
    case SW_UP: return GIMBAL_MODE_AIM;
    default: return 0xFF;
  }
}
}  // namespace

class RcSerialRemoteNode : public rclcpp::Node
{
public:
  RcSerialRemoteNode()
  : Node("rc_serial_remote_cpp")
  {
    declare_params_();
    read_params_();

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    gimbal_pub_ = create_publisher<spr_msgs::msg::GimbalCmd>(
      "gimbal_controller/gimbal_cmd", 10);
    // 内部状态实时导出(≈Ozone 运行时看全局变量)：PlotJuggler / ros2 topic echo
    debug_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
      "remote_debug", 10);

    // 发布/映射节拍：publish_hz 与 DBUS 100Hz 对齐
    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_hz_),
      [this]() { on_timer(); });

    running_ = true;
    read_thread_ = std::thread(&RcSerialRemoteNode::read_loop_, this);

    RCLCPP_INFO(get_logger(),
      "DR16 串口遥控(C++)就绪: %s @ %d bps %s, publish=%.0fHz",
      device_.c_str(), baudrate_, parity_str_.c_str(), publish_hz_);
  }

  ~RcSerialRemoteNode() override
  {
    running_ = false;
    if (read_thread_.joinable()) {
      read_thread_.join();
    }
    serial_.close();
  }

private:
  // ---------------- 参数 ----------------
  void declare_params_()
  {
    declare_parameter<std::string>("device", "/dev/ttyUSB0");
    declare_parameter<int>("baudrate", 100000);
    declare_parameter<std::string>("parity", "even");   // none/even/odd
    declare_parameter<double>("publish_hz", 100.0);
    // 控制映射
    declare_parameter<double>("deadzone", 0.08);
    declare_parameter<double>("max_vx", 1.0);      // m/s
    declare_parameter<double>("max_vy", 1.0);      // m/s
    declare_parameter<double>("max_wz", 2.0);      // rad/s
    declare_parameter<double>("pitch_rate", 1.0);  // rad/s @满杆
    // 摇杆方向符号（真机校准：前推/左推为正则填 +1，反了填 -1）
    declare_parameter<double>("vx_sign", 1.0);     // 左杆竖直→vx
    declare_parameter<double>("vy_sign", 1.0);     // 左杆水平→vy
    declare_parameter<double>("wz_sign", 1.0);     // 右杆水平→wz
    declare_parameter<double>("pitch_sign", 1.0);  // 右杆竖直→pitch
    // 失联
    declare_parameter<double>("lost_timeout", 0.2);  // s
  }

  void read_params_()
  {
    auto g = [this](const std::string & n) { return get_parameter(n); };
    device_ = g("device").as_string();
    baudrate_ = g("baudrate").as_int();
    parity_str_ = g("parity").as_string();
    publish_hz_ = g("publish_hz").as_double();
    deadzone_ = g("deadzone").as_double();
    max_vx_ = g("max_vx").as_double();
    max_vy_ = g("max_vy").as_double();
    max_wz_ = g("max_wz").as_double();
    pitch_rate_ = g("pitch_rate").as_double();
    vx_sign_ = g("vx_sign").as_double();
    vy_sign_ = g("vy_sign").as_double();
    wz_sign_ = g("wz_sign").as_double();
    pitch_sign_ = g("pitch_sign").as_double();
    lost_timeout_ = g("lost_timeout").as_double();

    parity_char_ = 'N';
    if (parity_str_ == "even" || parity_str_ == "e") {
      parity_char_ = 'E';
    } else if (parity_str_ == "odd" || parity_str_ == "o") {
      parity_char_ = 'O';
    }
  }

  // ---------------- 读线程 ----------------
  void read_loop_()
  {
    uint8_t tmp[64];
    bool reopen_warned = false;

    while (running_ && rclcpp::ok()) {
      if (!serial_.isOpen()) {
        // 打开失败时每 ~1s 重试一次（支持接收机热插拔）
        if (!serial_.open(device_, baudrate_, parity_char_)) {
          if (!reopen_warned) {
            RCLCPP_WARN(get_logger(), "打开串口失败 %s（将每 1s 自动重试）",
              device_.c_str());
            reopen_warned = true;
          }
          std::this_thread::sleep_for(std::chrono::seconds(1));
          continue;
        }
        reopen_warned = false;
        RCLCPP_INFO(get_logger(), "已打开串口 %s", device_.c_str());
      }

      ssize_t n = serial_.recv_timeout(tmp, sizeof(tmp), 5);  // 5ms 超时轮询
      if (n > 0) {
        on_rx_bytes_(tmp, static_cast<size_t>(n));
      }
    }
  }

  // 读到的字节进入缓冲并滑动解码（满 18B 就试；非法窗口滑 1 字节重同步）
  void on_rx_bytes_(const uint8_t * data, size_t n)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    rx_buf_.insert(rx_buf_.end(), data, data + n);
    if (rx_buf_.size() > RX_BUF_MAX) {
      rx_buf_.erase(rx_buf_.begin(), rx_buf_.end() - RX_BUF_MAX);
    }

    while (rx_buf_.size() >= DBUS_FRAME_LEN) {
      RCData rc;
      if (dbus_parse_frame(rx_buf_.data(), rc)) {
        rx_buf_.erase(rx_buf_.begin(), rx_buf_.begin() + DBUS_FRAME_LEN);
        on_valid_frame_(rc);
      } else {
        rx_buf_.erase(rx_buf_.begin());   // 滑动 1 字节重同步
      }
    }
  }

  // 有效帧：更新共享快照（调用方已持 mutex_）
  void on_valid_frame_(const RCData & rc)
  {
    const auto now = std::chrono::steady_clock::now();
    if (!got_frame_) {
      got_frame_ = true;
      RCLCPP_INFO(get_logger(), "已收到首帧遥控数据");
    }
    if (lost_) {
      lost_ = false;
      RCLCPP_WARN(get_logger(), "遥控恢复");
    }
    snap_ = rc;
    last_frame_time_ = now;
    ++frame_count_;
  }

  // ---------------- 定时发布 / 映射 ----------------
  void on_timer()
  {
    const auto now = std::chrono::steady_clock::now();

    // 时间步长（pitch 增量积分用）；首拍或异常时退回 1/publish_hz
    double dt = 1.0 / publish_hz_;
    if (last_tick_.time_since_epoch().count() != 0) {
      const double elapsed =
        std::chrono::duration<double>(now - last_tick_).count();
      if (elapsed > 0.0 && elapsed < 0.5) {
        dt = elapsed;
      }
    }
    last_tick_ = now;

    // ── 取共享快照 + 失联判定（锁内一次完成，避免与读线程竞争）──
    RCData rc;
    bool have_frame = false;
    double age = 0.0;
    bool stop = false;
    uint64_t fc = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      have_frame = got_frame_;
      fc = frame_count_;
      if (have_frame) {
        rc = snap_;
        age = std::chrono::duration<double>(now - last_frame_time_).count();
      }
      // 失联检测：超时无有效帧 → 底盘停车（云台保持最后模式/角度）
      if (have_frame && age > lost_timeout_) {
        if (!lost_) {
          RCLCPP_WARN(get_logger(), "遥控失联（无帧>%.0fms）→ 底盘停车",
            age * 1000.0);
        }
        lost_ = true;
        stop = true;
      }
    }
    if (stop) {
      publish_(0.0, 0.0, 0.0, have_frame);
      publish_dbg_(rc, have_frame, 0.0, 0.0, 0.0);  // 失联停车这拍也发
      return;
    }
    // 更新模式（可能来自本拍前的帧，直接沿用最新快照 rc）
    uint8_t mode = mode_from_switch(rc.s[1]);
    if (mode != 0xFF) {
      mode_ = mode;
    }
    if (have_frame && mode_ == GIMBAL_MODE_REMOTE) {
      // 遥控模式：右杆竖直 → pitch 绝对角增量积分（松杆即停保持角度）
      pitch_angle_ +=
        norm_stick(rc.ch[1], deadzone_) * pitch_sign_ * pitch_rate_ * dt;
    }

    // ── 底盘速度映射（左拨杆 s0 非"下"档才允许）──
    double vx = 0.0, vy = 0.0, wz = 0.0;
    if (have_frame && rc.s[0] != SW_DOWN) {
      vx = norm_stick(rc.ch[2], deadzone_) * vx_sign_ * max_vx_;
      vy = norm_stick(rc.ch[3], deadzone_) * vy_sign_ * max_vy_;
      wz = norm_stick(rc.ch[0], deadzone_) * wz_sign_ * max_wz_;
    }

    publish_(vx, vy, wz, have_frame);
    publish_dbg_(rc, have_frame, vx, vy, wz);  // 实时导出内部状态到 /remote_debug

    // 节流日志（每 20 帧 ≈ 5Hz）
    if (have_frame && (fc % 20 == 0)) {
      RCLCPP_INFO(get_logger(),
        "CH %.2f %.2f %.2f %.2f | SW %u %u | mode=%u vx=%.2f vy=%.2f wz=%.2f "
        "pitch=%+.2f",
        norm_stick(rc.ch[0]), norm_stick(rc.ch[1]),
        norm_stick(rc.ch[2]), norm_stick(rc.ch[3]),
        rc.s[0], rc.s[1], mode_, vx, vy, wz, pitch_angle_);
    }
  }

  void publish_(double vx, double vy, double wz, bool enable_gimbal)
  {
    auto twist = std::make_unique<geometry_msgs::msg::Twist>();
    twist->linear.x = vx;
    twist->linear.y = vy;
    twist->angular.z = wz;
    cmd_vel_pub_->publish(std::move(twist));

    // 未收到首帧前不发布 gimbal_cmd（避免抢占云台其它模式）
    if (!enable_gimbal) {
      return;
    }
    auto cmd = std::make_unique<spr_msgs::msg::GimbalCmd>();
    cmd->mode = mode_;
    cmd->pitch_angle = pitch_angle_;
    cmd->small_yaw_angle = small_yaw_angle_;
    cmd->big_yaw_angle = big_yaw_angle_;
    gimbal_pub_->publish(std::move(cmd));
  }

  // ---------------- 内部状态实时导出（/remote_debug）----------------
  // 每拍把内部状态发成 std_msgs/Float64MultiArray，PlotJuggler 或
  // `ros2 topic echo /remote_debug` 可运行中实时查看（≈MCU 调试器 Live Watch）。
  // data 布局：0 mode | 1 vx 2 vy 3 wz | 4 pitch 5 syaw 6 byaw |
  //            7..10 ch_norm[0..3] | 11 s0 12 s1 |
  //            13 frame_count | 14 got_frame | 15 lost
  void publish_dbg_(const RCData & rc, bool have_frame, double vx, double vy,
                    double wz)
  {
    if (!debug_pub_) {
      return;
    }
    double chn[4] = {0.0, 0.0, 0.0, 0.0};
    if (have_frame) {
      for (int k = 0; k < 4; ++k) {
        chn[k] = norm_stick(rc.ch[k], deadzone_);
      }
    }
    auto msg = std::make_unique<std_msgs::msg::Float64MultiArray>();
    msg->data = {
      static_cast<double>(mode_),
      vx, vy, wz,
      pitch_angle_, small_yaw_angle_, big_yaw_angle_,
      chn[0], chn[1], chn[2], chn[3],
      static_cast<double>(rc.s[0]),
      static_cast<double>(rc.s[1]),
      static_cast<double>(frame_count_),
      have_frame ? 1.0 : 0.0,
      lost_ ? 1.0 : 0.0};
    debug_pub_->publish(std::move(msg));
  }

  // ---------------- 成员 ----------------
  // 参数
  std::string device_;
  int baudrate_{100000};
  std::string parity_str_{"even"};
  char parity_char_{'E'};
  double publish_hz_{100.0};
  double deadzone_{0.08};
  double max_vx_{1.0};
  double max_vy_{1.0};
  double max_wz_{2.0};
  double pitch_rate_{1.0};
  double vx_sign_{1.0};
  double vy_sign_{1.0};
  double wz_sign_{1.0};
  double pitch_sign_{1.0};
  double lost_timeout_{0.2};

  SerialDriver serial_;
  std::thread read_thread_;
  std::atomic<bool> running_{false};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Publisher<spr_msgs::msg::GimbalCmd>::SharedPtr gimbal_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_pub_;

  // 读线程与定时器共享（mutex_ 保护）
  std::mutex mutex_;
  std::vector<uint8_t> rx_buf_;
  bool got_frame_{false};     // 是否收到过有效帧
  bool lost_{false};          // 是否处于失联停车状态
  RCData snap_{};
  std::chrono::steady_clock::time_point last_frame_time_{};
  uint64_t frame_count_{0};

  // 发布侧持续状态（仅定时器线程访问）
  uint8_t mode_{GIMBAL_MODE_REMOTE};
  double pitch_angle_{0.0};
  double small_yaw_angle_{0.0};
  double big_yaw_angle_{0.0};
  std::chrono::steady_clock::time_point last_tick_{};
};

}  // namespace spr_remote_control

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<spr_remote_control::RcSerialRemoteNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
