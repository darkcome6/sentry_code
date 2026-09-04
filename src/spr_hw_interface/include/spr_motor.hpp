#ifndef TIDE_MOTOR_HPP_
#define TIDE_MOTOR_HPP_
#include <rclcpp/rclcpp.hpp>
#include <array>
#include <string>
#include <cmath>//M_PI常量

namespace spr_hw_interface
{
 constexpr double SPEED_SMOOTH_COEF = 0.5f;  //速度平滑系数
 constexpr double CURRENT_SMOOTH_COEF = 0.9f;//电流平滑系数
 constexpr double act2pos = 0.0007670840;        // 2PI/8192  编码值转弧度
 constexpr double act2vel = 0.1047197551;        // 2PI/60 rpm转弧度
 constexpr double MOTOR_WATCHDOG_TIMEOUT = 1.0;  // 电机通信超时时间，单位秒

typedef enum
{
  MOTOR_TYPE_NONE = 0,
  DM6006,
  DM4310,
  GM6020,
  M3508,
  M2006,
  VIRTUAL_JOINT,
} Motor_Type_e;

typedef enum
{
  MOTOR_LOST = 0,
  MOTOR_UNACTIVE,
  MOTOR_ACTIVE,
} Motor_Status_e;

typedef struct
{
  std::string motor_name;
  std::string can_bus;
  uint32_t tx_id;
  uint32_t rx_id;
  uint32_t identifier;//报文标识符
  double offset{ 0.0 };  // 软件零点偏移：达妙(DM) 为 rad（位置反馈直接减，须支持小数）；DJI 为编码器 ticks(整数)
  Motor_Type_e motor_type;
  double reduction{ 1.0 };  // 减速比（电机轴/输出轴）：M3508≈19.2(3591/187)、M2006=36，无减速箱为 1
  double direction{ 1.0 };  // 转向系数：电机镜像安装时配 -1（命令/反馈同时取反），左右对称底盘右侧轮通常为 -1
  // 达妙电机 MIT 线性映射满量程（由达妙调试助手设定，发送/接收必须一致）
  float pos_max{ 3.14f };   // 位置 ±rad（DM 单圈默认 ±π，多圈需按调试助手 PMAX 覆写）
  float vel_max{ 30.0f };   // 速度 ±rad/s
  float tor_max{ 10.0f };   // 扭矩 ±N·m
} Motor_Config_t;

class DJI_Motor
{
public:
  struct Measure
  {
    uint16_t last_ecd{ 0 };//上一次的ECD
    uint16_t ecd{ 0 };//这一次的ECD
    double speed_aps{ 0.0 };//速度的
    int16_t real_current{ 0 };//电流大小
    uint8_t temperature{ 0 };//温度
    double total_angle{ 0.0 };//总角度
    int32_t total_round{ 0 };//总圈数
    Measure() = default;
  };
  Motor_Status_e status = MOTOR_LOST;//电机的上线状态
  Motor_Config_t config_;//电机的属性配置
  Measure measure;//电机的数值参数
  std::array<uint8_t, 8> rx_buff = { 0 };//电机的can缓存区
  int16_t output = 0;//输出
  double angle_current = 0.0;//当前角度

  // 达妙 MIT 反馈（经线性映射还原的物理量）
  double dm_position_{ 0.0 };  // rad
  double dm_velocity_{ 0.0 };  // rad/s
  double dm_torque_{ 0.0 };    // N·m
  uint8_t dm_id_{ 0 };         // 从站 ID（回传 D0 低4位）
  uint8_t dm_err_{ 0 };        // 状态(0失能/1使能/3制动/...)
  rclcpp::Time last_enable_sent_{ 0, 0, RCL_ROS_TIME };  // write() 周期性补发 0xFC 的上次时间

  DJI_Motor(const Motor_Config_t& config);//构造函数
  void decode_feedback();       // 大疆编码器格式解包
  void decode_dm_feedback();    // 达妙 MIT 回传帧解包
  void stop() { output = 0; }

  // 达妙 MIT 浮点<->定点线性映射（与调试助手设定一致）
  static int float_to_uint(float x, float x_min, float x_max, int bits);
  static float uint_to_float(int x_int, float x_min, float x_max, int bits);
  // 组装达妙 MIT 控制帧（8字节：p16/v12/kp12/kd12/t12）
  static void encode_mit_frame(std::array<uint8_t, 8>& frame,
                               float p, float v, float kp, float kd, float t,
                               const Motor_Config_t& cfg);
  //检查电机的连接状态
  bool check_connection(const rclcpp::Time& current_time);
  //更新时间
  void update_timestamp(const rclcpp::Time& time) { last_comm_time_ = time; }

private:
  rclcpp::Time last_time_{ 0, 0, RCL_ROS_TIME };
  rclcpp::Time last_comm_time_{ 0, 0, RCL_ROS_TIME };
};
}  // namespace spr_hw_interface
#endif
