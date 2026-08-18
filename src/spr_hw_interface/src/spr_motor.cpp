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
  measure.speed_aps =
      (1.0f - SPEED_SMOOTH_COEF) * measure.speed_aps +
      act2vel * SPEED_SMOOTH_COEF * (double)((int16_t)(rx_buff[2] << 8 | rx_buff[3]));
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
// 浮点 -> 定点（文档 float_to_uint）
int DJI_Motor::float_to_uint(float x, float x_min, float x_max, int bits)
{
  const float span = x_max - x_min;
  const float offset = x_min;
  const float scale = static_cast<float>((1 << (bits - 1)) - 1);
  return static_cast<int>((x - offset) * scale / span);
}
// 定点 -> 浮点（文档 uint_to_float）
float DJI_Motor::uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  const float span = x_max - x_min;
  const float offset = x_min;
  const float scale = static_cast<float>((1 << (bits - 1)) - 1);
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
// 布局：D0=MST_ID D1=ID<<4|ERR D2=POS[15:8] D3=POS[7:0]
//       D4=VEL[11:4] D5=VEL[3:0]|T[11:8] D6=T[7:0] D7=T_MOS
void DJI_Motor::decode_dm_feedback()
{
  dm_err_ = rx_buff[1] & 0x0F;  // 低4位为状态
  const uint16_t pos_int = static_cast<uint16_t>((rx_buff[2] << 8) | rx_buff[3]);
  const uint16_t vel_int = static_cast<uint16_t>(
    (rx_buff[4] << 4) | ((rx_buff[5] >> 4) & 0x0F));
  const uint16_t tor_int = static_cast<uint16_t>(
    ((rx_buff[5] & 0x0F) << 8) | rx_buff[6]);

  dm_position_ = uint_to_float(pos_int, -config_.pos_max, config_.pos_max, 16);
  dm_velocity_ = uint_to_float(vel_int, -config_.vel_max, config_.vel_max, 12);
  dm_torque_ = uint_to_float(tor_int, -config_.tor_max, config_.tor_max, 12);

  // 同步到统一的状态变量（state 接口直接读取）
  measure.total_angle = dm_position_;
  measure.speed_aps = dm_velocity_;
  measure.real_current = static_cast<int16_t>(dm_torque_);
  angle_current = dm_position_;
}


}  // namespace spr_hw_interface
