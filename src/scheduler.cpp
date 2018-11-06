#include "factoryWorld.hpp"
#include <limits>
#include <cassert>

namespace FactoryWorld {
  Scheduler::OrderWithDep::
  OrderWithDep(Order noDepOrder,
               const RelationOfProducts &bom)
    : Order(noDepOrder)
  {
    const auto &inAndDirectMask = bom.getInAndDirectMask();

    // dependent type, excluding themselves
    const auto dependentMask = inAndDirectMask -
      MatrixB::Identity(inAndDirectMask.rows(),
                        inAndDirectMask.cols());

  }

  void Scheduler::computeTimeNeeded() {
    const auto &bom = factory__->getBOM();
    const auto &orders = factory__->getOrders();
    const auto &machines = factory__->getMachines();

    // predecessor matrix, each row corresponds to dependent product number
    const auto &requiredMask = factory__->getBOM().getDirectMask();
    const auto &predcessor = factory__->getBOM().getPredecessor();

    // extend preductionTime
    // productionTime__.resize(orders.size());

    // need to expand product by including
    // the dependent products on each order

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

  inline void Scheduler::
  addConstraints_1(std::vector<MPVariable *> completionTimes,
                   MPVariable const *makeSpan, MPSolver &solver,
                   const std::string &purposeMessage)
  {
    std::vector<MPConstraint *> constraints;
    for (auto i = 0ul; i < completionTimes.size(); ++ i) {
      constraints.emplace_back(
        solver.MakeRowConstraint(
          -infinity, 0.0, purposeMessage + "_" + std::to_string(i)));
    }

    for (auto i = 0ul; i < constraints.size(); ++ i) {
      constraints[i]->SetCoefficient(completionTimes[i], 1.0);
      constraints[i]->SetCoefficient(makeSpan, -1.0);
    }

    LOG(INFO) << purposeMessage;
  }

  inline void Scheduler::
  addConstraints_2(const std::vector<std::vector<MPVariable *>> &startTime,
                   const std::vector<MPVariable *> &completionTimes,
                   const Var3D &onMachine, const Factory &factory,
                   MPSolver &solver, const std::string &purposeMessage)
  {
    std::vector<MPConstraint *> constraints;
    const auto &machines = factory.getMachines();
    const auto &orders = factory.getOrders();
    const auto &productionTime = productionTime__;
    // here startTime one to one mapps to orders
    // with index, referring to underly products

    assert(orders.size() == startTime.size());
    // per order, per startTime. same thing
    for (auto i = 0ul; i < startTime.size(); ++ i) {
      const auto &prodStartTimes = startTime[i];
      const auto &currentOrder = orders[i];
      const auto &productType = currentOrder.getProductType();

      assert(productType.size() == prodStartTime.size());
      // assume currentOrder.size() == prodStartTime.size()
      // per product of order
      for (auto j = 0ul; j < prodStartTimes.size(); ++ j) {
        const auto &typeIndex = productType[j];
        const auto &prodStart = prodStartTimes[j];
        // per machine that is capable
        for (auto k = 0ul; k < machines.size(); ++ k) {
          const auto &machine = machines[k];
          const auto &timesOnMach = productionTime[i][j];

          // add constraint if the machine can produce this type of product
          // doesn't pose constraints if the machine cannot produce the product
          if (machine.capable(typeIndex)) {
            constraints.emplace_back(
              solver.MakeRowConstraint(-infinity,
                largeNumber - timesOnMach[k],
                purposeMessage + "_(" + std::to_string(i) + ", " +
                std::to_string(j) + ", " + std::to_string(k) + ")"));
            constraints.back()->SetCoefficient(prodStart, 1.0);
            constraints.back()->SetCoefficient(completionTimes[i], -1.0);
            constraints.back()->SetCoefficient(onMachine[i][j][k], largeNumber);
          }
        }
      }
    }
    LOG(INFO) << purposeMessage;
  }



  void Scheduler::factoryScheduler(const Factory &factory,
    MPSolver::OptimizationProblemType optimization_problem_type)
  {
    using namespace operations_research;
    LOG(INFO) << "Building planner";
    MPSolver solver("FactorySolver", optimization_problem_type);

    // prepare parameters
    computeTimeNeeded();

    // prepare variables
    std::vector<MPVariable *> tardyTime;
    std::vector<MPVariable *> earlyTime;
    Var3D onMachine; // bool var
    std::vector<Var3D> immediatePrec; // bool var
    std::vector<std::vector<MPVariable *>> startTime;
    MPVariable const *makeSpan = solver.MakeNumVar(0.0, infinity, "MakeSpan");
    std::vector<MPVariable *> completionTimes;

    const auto orderSize = factory.getOrders().size();
    solver.MakeNumVarArray(orderSize, 0.0, infinity,
                           "CompletionTime", &completionTimes);

    // adding constraints
    addConstraints_1(completionTimes, makeSpan, solver,
                     "CompletionTime shoule precede MakeSpan");
    addConstraints_2(startTime, completionTimes, onMachine, factory,
                     solver, "BOM product relation precedence relation");
  }
}
