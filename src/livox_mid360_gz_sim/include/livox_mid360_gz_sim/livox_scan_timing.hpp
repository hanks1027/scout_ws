// [260716] Keep simulated Livox point timing consistent with the interpolated scan pose.

#ifndef LIVOX_MID360_GZ_SIM__LIVOX_SCAN_TIMING_HPP_
#define LIVOX_MID360_GZ_SIM__LIVOX_SCAN_TIMING_HPP_

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>

namespace livox_mid360_gz_sim
{

inline double ScanPointFraction(const std::size_t index, const std::size_t pointCount)
{
  if (pointCount <= 1U) {
    return 1.0;
  }
  return static_cast<double>(index) / static_cast<double>(pointCount - 1U);
}

inline uint32_t ScanPointOffsetNs(
  const std::size_t index,
  const std::size_t pointCount,
  const int64_t scanDurationNs)
{
  const int64_t boundedDuration = std::clamp<int64_t>(
    scanDurationNs, 0, static_cast<int64_t>(std::numeric_limits<uint32_t>::max()));
  return static_cast<uint32_t>(
    ScanPointFraction(index, pointCount) * static_cast<double>(boundedDuration));
}

}  // namespace livox_mid360_gz_sim

#endif  // LIVOX_MID360_GZ_SIM__LIVOX_SCAN_TIMING_HPP_
