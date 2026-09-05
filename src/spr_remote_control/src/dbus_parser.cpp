#include "spr_remote_control/dbus_parser.hpp"

#include <cmath>

namespace spr_remote_control
{

bool dbus_parse_frame(const uint8_t * data, RCData & out)
{
  if (data == nullptr) {
    return false;
  }

  RCData rc;

  // ────── 4 个摇杆通道 + ch4（11-bit）──────
  rc.ch[0] = static_cast<int16_t>(
    ((data[0] | (data[1] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET);
  rc.ch[1] = static_cast<int16_t>(
    (((data[1] >> 3) | (data[2] << 5)) & 0x07FF) - RC_CH_VALUE_OFFSET);
  rc.ch[2] = static_cast<int16_t>(
    (((data[2] >> 6) | (data[3] << 2) | (data[4] << 10)) & 0x07FF)
    - RC_CH_VALUE_OFFSET);
  rc.ch[3] = static_cast<int16_t>(
    (((data[4] >> 1) | (data[5] << 7)) & 0x07FF) - RC_CH_VALUE_OFFSET);
  rc.ch[4] = static_cast<int16_t>(
    ((data[16] | (data[17] << 8)) & 0x07FF) - RC_CH_VALUE_OFFSET);

  // ────── 拨杆：s[0]=左(bit4-5), s[1]=右(bit6-7) ──────
  rc.s[0] = (data[5] >> 4) & 0x03;
  rc.s[1] = (data[5] >> 6) & 0x03;

  // ────── 合法性判帧：拨杆非 0 且 4 主通道幅值不越界 ──────
  if (rc.s[0] == 0 || rc.s[1] == 0) {
    return false;
  }
  for (int i = 0; i < 4; ++i) {
    if (std::abs(rc.ch[i]) > RC_CH_VALID_ABS) {
      return false;
    }
  }

  // ────── 鼠标 / 键盘 ──────
  rc.mouse_x = static_cast<int16_t>(data[6] | (data[7] << 8));
  rc.mouse_y = static_cast<int16_t>(data[8] | (data[9] << 8));
  rc.mouse_z = static_cast<int16_t>(data[10] | (data[11] << 8));
  rc.mouse_press_l = data[12];
  rc.mouse_press_r = data[13];
  rc.key = static_cast<uint16_t>(data[14] | (data[15] << 8));

  rc.valid = true;
  out = rc;
  return true;
}

double norm_stick(double raw, double deadzone)
{
  double v = std::max(-1.0, std::min(1.0, raw / RC_CH_STICK_RANGE));
  if (std::abs(v) < deadzone) {
    return 0.0;
  }
  v = (v - std::copysign(deadzone, v)) / (1.0 - deadzone);
  return std::max(-1.0, std::min(1.0, v));
}

}  // namespace spr_remote_control
