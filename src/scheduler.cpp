#include "factoryWorld.hpp"
#include <limits>

namespace FactoryWorld {
  void Scheduler::computeTimeNeeded() {
    constexpr double infinity = std::numeric_limits<double>::infinity();
    const auto &bom = factory__->getBOM();
    const auto &orders = factory__->getOrders();
    const auto &machines = factory__->getMachines();
    productionTime__.resize(orders.size());

    // per order
    for (auto i = 0ul; i < productionTime__.size(); ++ i) {
      auto &currentOrder = productionTime__[i];
      const auto &order = orders[i];
      const auto &productQuan = order.getProductQuan();
      const auto &productType = order.getProductType();
      // how many products type
      const auto typeSize = productQuan.size();
      currentOrder.resize(typeSize);
      // productQuan.size() == productType.size() assumed
      // per product in order
      for (auto j = 0ul; j < typeSize; ++ j) {
        const auto &numProduct = productQuan[j];
        const auto &typeIndex = productType[j];
        // product j of order i
        auto &currentProduct = currentOrder[j];
        currentProduct.resize(machines.size());
        // per machine
        for (auto k = 0ul; k < machines.size(); ++ k) {
          const auto &machine = machines[i];
          if (machine.capable(typeIndex)) {
            currentProduct[k] = infinity;
          }
          else {
            currentProduct[k] =
              machine.produceTime(typeIndex, numProduct);
          }
        }
      }
    }
  }

  void Scheduler::factoryScheduler(const Factory &factory,
    MPSolver::OptimizationProblemType optimization_problem_type)
  {
    using namespace operations_research;
    MPSolver solver("FactorySolver", optimization_problem_type);
    auto orderSize = factory.getOrders().size();
    const double infinity = solver.infinity();
    std::vector<MPVariable *> completionTimes;

    solver.MakeNumVarArray(orderSize, 0.0, infinity,
                           "CompletionTime", &completionTimes);
    MPVariable const *makeSpan = solver.MakeNumVar(0.0, infinity, "MakeSpan");
    std::vector<MPConstraint *> constraints_1;
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
