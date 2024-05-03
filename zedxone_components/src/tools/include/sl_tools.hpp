#ifndef SL_TOOLS_HPP
#define SL_TOOLS_HPP

#include <rclcpp/clock.hpp>
#include "sl_win_avg.hpp"

namespace sl_tools
{

/*! \brief Convert StereoLabs timestamp to ROS timestamp
*  \param t : timestamp to be converted
*  \param t : ROS2 clock type
*/
rclcpp::Time slTime2Ros(uint64_t t_nsec, rcl_clock_type_t clock_type = RCL_ROS_TIME);

/**
 * @brief Stop Timer used to measure time intervals
 *
 */
class StopWatch
{
public:
  explicit StopWatch(rclcpp::Clock::SharedPtr clock);
  ~StopWatch() {}

  void tic();    //!< Set the reference time point to the current time
  double toc(std::string func_name = std::string() );  //!< Returns the seconds elapsed from the last tic in ROS clock reference (it works also in simulation)

private:
  rclcpp::Time mStartTime;  // Reference time point
  rclcpp::Clock::SharedPtr mClockPtr;  // Node clock interface
};

} // namespace sl_tools

#endif // SL_TOOLS_HPP
