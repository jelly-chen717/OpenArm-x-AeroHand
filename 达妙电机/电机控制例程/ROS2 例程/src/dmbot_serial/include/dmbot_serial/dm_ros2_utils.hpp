#pragma once

#include "dmbot_serial/protocol/damiao.h"

#include <algorithm>
#include <cctype>
#include <stdexcept>
#include <string>

namespace dmbot_serial
{

inline std::string normalize_token(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });

  value.erase(std::remove_if(value.begin(), value.end(), [](char c) {
    return c == '-' || c == '_' || c == ' ';
  }), value.end());

  return value;
}

inline damiao::Can_control_Mode parse_can_mode(const std::string& value)
{
  const auto token = normalize_token(value);
  if (token == "can" || token == "classiccan") {
    return damiao::can;
  }
  if (token == "canfd" || token == "fd") {
    return damiao::canfd;
  }

  throw std::invalid_argument("can_mode 仅支持 can 或 canfd");
}

inline damiao::Control_Mode parse_control_mode(const std::string& value)
{
  const auto token = normalize_token(value);
  if (token == "mit" || token == "mitmode") {
    return damiao::MIT_MODE;
  }
  if (token == "posvel" || token == "positionvelocity" || token == "position") {
    return damiao::POS_VEL_MODE;
  }
  if (token == "vel" || token == "velocity") {
    return damiao::VEL_MODE;
  }
  if (token == "posforce" || token == "positionforce" || token == "force") {
    return damiao::POS_FORCE_MODE;
  }

  throw std::invalid_argument("control_mode 仅支持 mit、pos_vel、vel、pos_force");
}

inline damiao::DM_Motor_Type parse_motor_type(const std::string& value)
{
  const auto token = normalize_token(value);
  if (token == "dm3507") return damiao::DM3507;
  if (token == "dm4310") return damiao::DM4310;
  if (token == "dm431048v") return damiao::DM4310_48V;
  if (token == "dm4340") return damiao::DM4340;
  if (token == "dm434048v") return damiao::DM4340_48V;
  if (token == "dm6006") return damiao::DM6006;
  if (token == "dm6248p") return damiao::DM6248P;
  if (token == "dm8006") return damiao::DM8006;
  if (token == "dm8009") return damiao::DM8009;
  if (token == "dm10010l") return damiao::DM10010L;
  if (token == "dm10010") return damiao::DM10010;
  if (token == "dmh3510") return damiao::DMH3510;
  if (token == "dmh6215") return damiao::DMH6215;
  if (token == "dms3519") return damiao::DMS3519;
  if (token == "dmg6220") return damiao::DMG6220;

  throw std::invalid_argument("未知电机型号: " + value);
}

}  // namespace dmbot_serial
