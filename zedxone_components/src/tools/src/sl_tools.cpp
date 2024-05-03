#include "sl_tools.hpp"
#include <iostream>
namespace sl_tools
{

rclcpp::Time slTime2Ros(uint64_t t_nsec, rcl_clock_type_t clock_type)
{
  uint32_t sec = static_cast<uint32_t>(t_nsec / 1000000000);
  uint32_t nsec = static_cast<uint32_t>(t_nsec % 1000000000);
  return rclcpp::Time(sec, nsec, clock_type);
}

StopWatch::StopWatch(rclcpp::Clock::SharedPtr clock)
: mStartTime(0, 0, RCL_ROS_TIME),
  mClockPtr(clock)
{
  tic();  // Start the timer at creation
}

void StopWatch::tic()
{
  mStartTime = mClockPtr->now();  // Reset the start time point
}

double StopWatch::toc(std::string func_name)
{
  auto now = mClockPtr->now();

  double elapsed_nsec = (now - mStartTime).nanoseconds();
  if (!func_name.empty()) {
    std::cerr << func_name << " -> toc elapsed_sec: " << elapsed_nsec / 1e9 << std::endl <<
      std::flush;
  }

  return elapsed_nsec / 1e9;  // Returns elapsed time in seconds
}

} // namespace sl_tools
