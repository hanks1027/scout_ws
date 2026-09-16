// [260716] Regression coverage for scan-begin timestamps and per-point offsets.

#include <cstdint>
#include <iostream>

#include "livox_mid360_gz_sim/livox_scan_timing.hpp"

namespace
{
bool Check(const bool condition, const char * expression)
{
  if (!condition) {
    std::cerr << "check failed: " << expression << '\n';
  }
  return condition;
}
}  // namespace

int main()
{
  constexpr int64_t scanDurationNs = 100000000LL;
  constexpr std::size_t pointCount = 20000U;

  bool ok = true;
  ok &= Check(
    livox_mid360_gz_sim::ScanPointFraction(0U, pointCount) == 0.0,
    "first fraction");
  ok &= Check(
    livox_mid360_gz_sim::ScanPointFraction(pointCount - 1U, pointCount) == 1.0,
    "last fraction");
  ok &= Check(
    livox_mid360_gz_sim::ScanPointOffsetNs(0U, pointCount, scanDurationNs) == 0U,
    "first offset");
  ok &= Check(
    livox_mid360_gz_sim::ScanPointOffsetNs(pointCount - 1U, pointCount, scanDurationNs) ==
    static_cast<uint32_t>(scanDurationNs), "last offset");
  ok &= Check(livox_mid360_gz_sim::ScanPointFraction(0U, 1U) == 1.0, "single fraction");
  ok &= Check(
    livox_mid360_gz_sim::ScanPointOffsetNs(0U, 1U, scanDurationNs) == scanDurationNs,
    "single offset");
  ok &= Check(
    livox_mid360_gz_sim::ScanPointOffsetNs(0U, pointCount, -1) == 0U,
    "negative duration");
  return ok ? 0 : 1;
}
