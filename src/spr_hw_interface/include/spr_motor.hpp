#ifndef TIDE_MOTOR_HPP_
#define TIDE_MOTOR_HPP_
#include <rclcpp/rclcpp.hpp>
#include <array>
#include <string>
#include <cmath>//M_PI常量

namespace spr_hw_interface
{
 constexpr double SPEED_SMOOTH_COEF = 0.85f;  //速度平滑系数
 constexpr double CURRENT_SMOOTH_COEF = 0.9f;//电流平滑系数
 constexpr double act2pos = 0.0007670840;        // 2PI/8192  编码值转弧度
 constexpr double act2vel = 0.1047197551;        // 2PI/60 rpm转弧度
 constexpr double MOTOR_WATCHDOG_TIMEOUT = 1.0;  // 电机通信超时时间，单位秒

typedef enum
{
  MOTOR_TYPE_NONE = 0,
  GM6020,
  M3508,
  M2006,
  VIRTUAL_JOINT,
} Motor_Type_e;

typedef enum
{
  MOTOR_LOST = 0,
  MOTOR_OK,
} Motor_Status_e;

typedef enum
{
  OPEN_LOOP = 0,
  SPEED_LOOP,
  POSITION_LOOP,
} Motor_CloseMode_e;

typedef struct
{
  std::string motor_name;
  std::string can_bus;
  uint32_t tx_id;
  uint32_t rx_id;
  uint32_t identifier;
  uint16_t offset;
  Motor_Type_e motor_type;
} Motor_Config_t;

class DJI_Motor
{
public:
  struct Measure
  {
    uint16_t last_ecd{ 0 };
    uint16_t ecd{ 0 };
    double speed_aps{ 0.0 };
    int16_t real_current{ 0 };
    uint8_t temperature{ 0 };
    double total_angle{ 0.0 };
    int32_t total_round{ 0 };
    Measure() = default;
  };
  Motor_Status_e status = MOTOR_LOST;
  Motor_Config_t config_;
  Measure measure;
  std::array<uint8_t, 8> rx_buff = { 0 };
  int16_t output = 0;
  double angle_current = 0.0;

  DJI_Motor(const Motor_Config_t& config);
  void decode_feedback();
  void stop() { output = 0; }

  bool check_connection(const rclcpp::Time& current_time);
  void update_timestamp(const rclcpp::Time& time) { last_comm_time_ = time; }

private:
  rclcpp::Time last_time_{ 0, 0, RCL_ROS_TIME };
  rclcpp::Time last_comm_time_{ 0, 0, RCL_ROS_TIME };
};
}  // namespace spr_hw_interface
#endif
