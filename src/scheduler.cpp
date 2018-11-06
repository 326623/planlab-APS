#include "factoryWorld.hpp"

namespace FactoryWorld {
  void Scheduler::computeTimeNeeded() {
    const auto &bom = factory->getBOM();
    const auto &orders = factory__->getOrders();
    const auto &machines = factory->getMachines();

    timeNeeded__.resize(orders.size());
  }

  void Scheduler::factoryScheduler(const Factory &factory,
    MPSolver::OptimizationProblemType optimization_problem_type)
  {
    using namespace operations_research;
    MPSolver solver("FactorySolver", optimization_problem_type);
    auto orderSize = factory.getOrders().size();
    const double infinity = solver.infinity();
    std::vector<MPVariable const*> completionTimes;

    solver.MakeNumVarArray(orderSize, 0.0, infinity,
                           "CompletionTime", &completionTimes);
    MPVariable const *makeSpan = solver.MakeNumVar(0.0, infinity, "MakeSpan");
    std::vector<MPConstraint const*> constraints_1;
    for (const auto &completionTime: completionTimes) {
      constraints_1.emplace_back(
        solver.MakeRowConstraint(
          -infinity, 0.0, "CompletionTime shoule precede MakeSpan"));
    }

    for (auto i = 0ul; i < constraints_1.size(); ++ i) {
      constraints_1[i]->SetCoefficient(completionTimes[i], 1.0);
      constraints_1[i]->SetCoefficient(makeSpan, -1.0);
    }


  }
}
