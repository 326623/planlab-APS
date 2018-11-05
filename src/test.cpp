// #include <iostream>
// #include <Eigen/Dense>

// using Eigen::MatrixXd;

// int main()
// {
//   MatrixXd m(2, 2);
//   m(0, 0) = 3;
//   m(1, 0) = 2.5;
//   m(0, 1) = -1;
//   m(1, 1) = m(1, 0) + m(0, 1);
//   std::cout << m.inverse() << '\n';
//   std::cout << (m.array() > 0.0) << '\n';
//   std::cout << m * m.inverse() << '\n';
// }
#include "factoryWorld.hpp"
#include <ortools/base/commandlineflags.h>

static const char kUsage[] =
  "Usage: example program to load data";

static bool ValidateFilename(const char *flagname, const string &value) {
  return value.size() > 0;
}

DEFINE_string(
  factory_world,
  "", // no default
  "the file containing information about the world of factory");
DEFINE_validator(factory_world, &ValidateFilename);
int main(int argc, char **argv) {
  gflags::SetUsageMessage(kUsage);
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  using namespace FactoryWorld;
  Factory myFactory;
  myFactory.load(FLAGS_factory_world);
}
