#include "sl_tools.hpp"
namespace sl_tools
{

rclcpp::Time slTime2Ros(uint64_t t_nsec, rcl_clock_type_t clock_type)
{
  uint32_t sec = static_cast<uint32_t>(t_nsec / 1000000000);
  uint32_t nsec = static_cast<uint32_t>(t_nsec % 1000000000);
  return rclcpp::Time(sec, nsec, clock_type);
}

} // namespace sl_tools
