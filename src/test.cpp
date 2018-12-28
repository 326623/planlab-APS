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

static bool ValidateLambda(const char *flagname, const double lambda) {
  return lambda >= 0.0;
}

DEFINE_string(
  factory_world,
  "", // no default
  "the file containing information about the world of factory");
DEFINE_double(lambda,
              0.01,
              "The weight of l1 norm");
DEFINE_double(time_limit, 10, "Time limit for the solver(seconds)");
DEFINE_string(
  output_file,
  "output_schedule",
  "The scheduler's output");
DEFINE_validator(factory_world, &ValidateFilename);
DEFINE_validator(output_file, &ValidateFilename);
DEFINE_validator(lambda, &ValidateLambda);
int main(int argc, char **argv) {
  std::ios::sync_with_stdio(false);
  gflags::SetUsageMessage(kUsage);
  gflags::ParseCommandLineFlags(&argc, &argv, true);

  using namespace FactoryWorld;
  std::shared_ptr<Factory> myFactory = std::make_shared<Factory>();
  std::ofstream outputStream(FLAGS_output_file);
  myFactory->load(FLAGS_factory_world);
  Scheduler planner;
  planner.factoryScheduler(myFactory,
                           operations_research::MPSolver::CBC_MIXED_INTEGER_PROGRAMMING, //GLOP_LINEAR_PROGRAMMING);
                           FLAGS_lambda, FLAGS_time_limit * 1000, outputStream);
  // planner.factoryScheduler(myFactory,
  //   operations_research::MPSolver::BOP_INTEGER_PROGRAMMING);
  // std::cout << myFactory.getBOM().getBOM() << '\n';

  // for (const auto &machine : myFactory.getMachines()) {
  //   for (const auto cap : machine.getCapability()) {
  //     std::cout << cap << ' ';
  //   }
  //   std::cout << '\n';
  // }

  // for (const auto & order : myFactory.getOrders()) {
  //   std::cout << order << '\n';
  // }
}
