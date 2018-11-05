#include <iostream>
#include <ortools/base/commandlineflags.h>

static const char kUsage[] =
  "Usage: see flags.\nThis program runs a simple job shop optimization "
  "output besides the debug LOGs of the solver.";
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
    std::cout << i << ' '
              << numType + i << ' '
              << 2 * numType + i << '\n';
  }
  return 0;
}
