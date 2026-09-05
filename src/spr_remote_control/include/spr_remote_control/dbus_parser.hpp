#ifndef SPR_REMOTE_CONTROL__DBUS_PARSER_HPP
#define SPR_REMOTE_CONTROL__DBUS_PARSER_HPP

#include <cstdint>
#include <cstddef>

// DJI / RoboMaster DR16 接收机 DBUS（裸 18 字节帧, ~100Hz）解码。
// 位运算移植自 DJI 官方 remote_control.c 的 sbus_to_rc（PT_link_en=0 配置），
// 与参考实现 dt7_receiver/dbus_parser 一致。DBUS 帧无帧头无校验，
// 判帧依赖通道/拨杆合法性（供上层滑动重同步）。

namespace spr_remote_control
{

constexpr size_t DBUS_FRAME_LEN = 18;
constexpr int16_t RC_CH_VALUE_OFFSET = 1024;   // 摇杆原始值中位偏移
constexpr int16_t RC_CH_VALID_ABS = 700;       // 去偏后合法幅值上限
constexpr int16_t RC_CH_STICK_RANGE = 660;     // 去偏后满量程幅值

// 拨杆取值（PT_link_en=0 映射：上=1 中=3 下=2；0=非法/过渡）
constexpr uint8_t SW_UP = 1;
constexpr uint8_t SW_MID = 3;
constexpr uint8_t SW_DOWN = 2;

/// @brief 单帧解码结果。ch[0..4] 去偏后有符号值；s[0]=左拨杆 s[1]=右拨杆。
struct RCData
{
  int16_t ch[5]{0, 0, 0, 0, 0};   // ch0 右杆水平, ch1 右杆竖直, ch2 左杆竖直,
                                  // ch3 左杆水平, ch4 第 5 通道(空/波轮)
  uint8_t s[2]{0, 0};             // 拨杆（1=上 2=下 3=中）
  int16_t mouse_x{0};
  int16_t mouse_y{0};
  int16_t mouse_z{0};
  uint8_t mouse_press_l{0};
  uint8_t mouse_press_r{0};
  uint16_t key{0};                // 键盘(16 位)
  bool valid{false};
};

/// @brief 解码一帧 18 字节 DBUS。帧非法返回 false（out 保持默认值）。
bool dbus_parse_frame(const uint8_t * data, RCData & out);

/// @brief 摇杆去偏值 → [-1, 1]：死区内回 0，死区外重新线性化。
double norm_stick(double raw, double deadzone = 0.08);

}  // namespace spr_remote_control

#endif  // SPR_REMOTE_CONTROL__DBUS_PARSER_HPP
