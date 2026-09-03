#include "spr_motor.hpp"
#include <cmath>

namespace spr_hw_interface
{
DJI_Motor::DJI_Motor(const Motor_Config_t& config)
{
  config_ = config;
  last_time_ = rclcpp::Clock().now();
  last_comm_time_ = last_time_;
}
//解包电机信号
void DJI_Motor::decode_feedback()
{
  measure.last_ecd = measure.ecd;
  measure.ecd = rx_buff[0] << 8 | rx_buff[1];
//低通滤波     y[n]=(1−α)⋅y[n−1]+α⋅x[n]
  // 反馈转速为电机转子 rpm → act2vel 得转子 rad/s；除以减速比折成输出轴 rad/s（与逆运动学目标对齐）
  measure.speed_aps =
      ((1.0f - SPEED_SMOOTH_COEF) * measure.speed_aps +
       act2vel * SPEED_SMOOTH_COEF * (double)((int16_t)(rx_buff[2] << 8 | rx_buff[3]))) /
      config_.reduction;
  measure.real_current = (1.0f - CURRENT_SMOOTH_COEF) * measure.real_current +
                         CURRENT_SMOOTH_COEF * (double)((int16_t)(rx_buff[4] << 8 | rx_buff[5]));
  
  measure.temperature = rx_buff[6];
//编码值跳变
  if (measure.ecd - measure.last_ecd > 4096)
    measure.total_round--;
  else if (measure.ecd - measure.last_ecd < -4096)
    measure.total_round++;

  measure.total_angle = (measure.total_round * 8191 + measure.ecd - config_.offset) * act2pos;

  double normalized_angle = measure.total_angle;

  while (normalized_angle > M_PI)
  {
    normalized_angle -= M_PI * 2;
  }
  while (normalized_angle <= -M_PI)
  {
    normalized_angle += M_PI * 2;
  }

  angle_current = normalized_angle;
}
//检查连接
bool DJI_Motor::check_connection(const rclcpp::Time& current_time)
{
  if (config_.motor_type == VIRTUAL_JOINT)
  {
    status = MOTOR_ACTIVE;
    return true;
  }

  double current_seconds = current_time.seconds();
  double last_comm_seconds = last_comm_time_.seconds();
  double time_diff = current_seconds - last_comm_seconds;

  if (time_diff > MOTOR_WATCHDOG_TIMEOUT)
  {
    status = MOTOR_LOST;
    return false;
  }

  return (status != MOTOR_LOST);
}

// ========== 达妙 MIT 协议 ==========
// ⚠️ scale 必须用 (1<<bits)-1（MIT 官方 / 电机固件刻度）。
//    旧实现误用 (1<<(bits-1))-1：t=0 会打包成 1023 而非中点 2048，
//    电机端按 4095 刻度解出 -50% 满量程力矩 → 电机 t=0 仍被持续驱动自转
// 浮点 -> 定点（MIT float_to_uint）
int DJI_Motor::float_to_uint(float x, float x_min, float x_max, int bits)
{
  const float span = x_max - x_min;
  const float offset = x_min;
  const float scale = static_cast<float>((1 << bits) - 1);
  return static_cast<int>((x - offset) * scale / span);
}
// 定点 -> 浮点（MIT uint_to_float）
float DJI_Motor::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  const float span = x_max - x_min;
  const float offset = x_min;
  const float scale = static_cast<float>((1 << bits) - 1);
  return static_cast<float>(x_int) * span / scale + offset;
}

// 组装达妙 MIT 控制帧（8字节，帧ID=CAN ID）
// 布局：p_des(16) + v_des(12) + Kp(12) + Kd(12) + t_ff(12)
//       D0=p[15:8] D1=p[7:0] D2=v[11:4] D3=v[3:0]|kp[11:8]
//       D4=kp[7:0] D5=kd[11:4] D6=kd[3:0]|t[11:8] D7=t[7:0]
void DJI_Motor::encode_mit_frame(std::array<uint8_t, 8>& frame,
                                 float p, float v, float kp, float kd, float t,
                                 const Motor_Config_t& cfg)
{
  constexpr float kp_max = 500.0f;  // Kp 范围 [0, 500]（调试助手默认）
  constexpr float kd_max = 5.0f;    // Kd 范围 [0, 5]
  const uint16_t p_int = static_cast<uint16_t>(
    float_to_uint(p, -cfg.pos_max, cfg.pos_max, 16));
  const uint16_t v_int = static_cast<uint16_t>(
    float_to_uint(v, -cfg.vel_max, cfg.vel_max, 12));
  const uint16_t kp_int = static_cast<uint16_t>(
    float_to_uint(kp, 0.0f, kp_max, 12));
  const uint16_t kd_int = static_cast<uint16_t>(
    float_to_uint(kd, 0.0f, kd_max, 12));
  const uint16_t t_int = static_cast<uint16_t>(
    float_to_uint(t, -cfg.tor_max, cfg.tor_max, 12));

  frame[0] = (p_int >> 8) & 0xFF;
  frame[1] = p_int & 0xFF;
  frame[2] = (v_int >> 4) & 0xFF;
  frame[3] = ((v_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F);
  frame[4] = kp_int & 0xFF;
  frame[5] = (kd_int >> 4) & 0xFF;
  frame[6] = ((kd_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F);
  frame[7] = t_int & 0xFF;
}

// 解包达妙 MIT 回传帧（8字节）
// ⚠️ 真机核对修正(2026-09-02)：布局以用户下位机 get_da_motor_measure 为准
//   D0:      [7:4]=err  [3:0]=id(从站号)
//   D1D2:    位置 16bit 有符号(D1高8/D2低8)，中心 0 对应 0 rad → ±pos_max
//   D3D4[7:4]: 速度 12bit → ±vel_max
//   D4[3:0]D5: 力矩 12bit → ±tor_max
//   D6: MOS温度   D7: 线圈温度
//   旧实现把回传当命令帧布局解 → 整体错位一字节 → 位置/速度乱跳(假 error_dot)
void DJI_Motor::decode_dm_feedback()
{
  dm_id_ = rx_buff[0] & 0x0F;
  dm_err_ = rx_buff[0] >> 4;

  const int16_t ecd = static_cast<int16_t>((rx_buff[1] << 8) | rx_buff[2]);
  // 软件零点偏移（config_.offset，单位 rad）：把机械中值对应的电机原始位置读到 0。
  // ⚠️ 曾缺失：DM 解码路径没用 offset → xacro 里 offset=2.728 无效，软件 0 仍对电机 3.14
  // 软件零点偏移(rad)：位置 = 原始 - offset。DM 单圈 ±π 时 +π/-π 是同一物理点(wrap)，
  // 减 offset 后可能越过 ±π 边界(如 -3.14-3.14=-6.28)，必须 wrap 回 (-π, π]，
  // 否则位置环看到超界假位置 → 猛打红灯。
  dm_position_ = static_cast<double>(ecd) * config_.pos_max / 32768.0 - config_.offset;
  while (dm_position_ > M_PI) dm_position_ -= 2.0 * M_PI;
  while (dm_position_ <= -M_PI) dm_position_ += 2.0 * M_PI;

  const uint16_t v_int = static_cast<uint16_t>((rx_buff[3] << 4) | (rx_buff[4] >> 4));
  dm_velocity_ = uint_to_float(v_int, -config_.vel_max, config_.vel_max, 12);

  const uint16_t t_int = static_cast<uint16_t>(((rx_buff[4] & 0x0F) << 8) | rx_buff[5]);
  dm_torque_ = uint_to_float(t_int, -config_.tor_max, config_.tor_max, 12);

  measure.temperature = rx_buff[6];  // MOS 温度

  // 同步到统一的状态变量（state 接口直接读取）
  measure.total_angle = dm_position_;
  measure.speed_aps = dm_velocity_;
  measure.real_current = static_cast<int16_t>(dm_torque_);
  angle_current = dm_position_;
}


}  // namespace spr_hw_interface
