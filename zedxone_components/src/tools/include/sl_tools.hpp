#ifndef SL_TOOLS_HPP
#define SL_TOOLS_HPP

#include <rclcpp/clock.hpp>

namespace sl_tools
{

/*! \brief Convert StereoLabs timestamp to ROS timestamp
*  \param t : timestamp to be converted
*  \param t : ROS2 clock type
*/
rclcpp::Time slTime2Ros(uint64_t t_nsec, rcl_clock_type_t clock_type = RCL_ROS_TIME);

} // namespace sl_tools

#endif // SL_TOOLS_HPP
