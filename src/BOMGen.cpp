#include <iostream>
#include <ortools/base/commandlineflags.h>

static const char kUsage[] =
  "Generate BOM adjacency list";
/**
 * simple solution to simulate a BOM
 * output format: adjacency list like
 * product id -> [dependent product id] (a list)
 * Used to build BOM matrix
 */
DEFINE_int32(
  numType, 100, "how many product types");
int main(int argc, char **argv) {
  gflags::SetUsageMessage(kUsage);
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  const auto numType = FLAGS_numType;
  for (int i = 0; i < numType; ++ i) {
    std::cout << i << ' ' << 2 << ' '
              << 1 << ' ' << numType + i << ' '
              << 1 << ' ' << 2 * numType + i << '\n';
  }
  for (int i = 0; i < 2 * numType; ++ i) {
    std::cout << i << ' ' << 0 << '\n';
  }
  return 0;
}
