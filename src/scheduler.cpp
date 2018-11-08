#include "factoryWorld.hpp"
#include <limits>
#include <cassert>

namespace FactoryWorld {
  Scheduler::OrderWithDep::
  OrderWithDep(Order noDepOrder,
               const RelationOfProducts &bom,
               const std::vector<Machine> &machines)
    : Order(noDepOrder)
  {
    auto &prodQuan = productQuan_;
    auto &prodType = productType_;
    // const auto &inAndDirectMask = bom.getInAndDirectMask();
    // // 1 unit of product i requires how many unit of j => predecessor(i, j)
    // const auto &predecessor = bom.getPredecessor();

    // // dependent type, excluding themselves
    // const auto dependentMask = inAndDirectMask -
    //   MatrixB::Identity(inAndDirectMask.rows(),
    //                     inAndDirectMask.cols());

    // const auto &finalProdQuan = productQuan_;
    // const auto &finalProdType = productType_;

    // for (auto i = 0ul; i < finalProdQuan.size(); ++ i) {
    //   const auto &typeIndex = finalProdType[i];
    //   const auto &prodNum = finalProdQuan[i];
    //   for (auto j = 0ul; j < dependentMask.cols(); ++ j) {
    //     if (dependentMask(typeIndex, j)) {
    //       productTypeDep__.emplace_back(j);
    //       productQuanDep__.emplace_back(prodNum * predecessor(typeIndex, j));
    //       DLOG(INFO) << prodNum << " of " << typeIndex << " relies on "
    //                  << prodNum * predecessor(typeIndex, j) << " of " << j;
    //     }
    //   }
    // }
  }

  /**
   * transfrom product number to time using each machine's capability
   *
   * Need to mark if this product is the final product to reduce number of
   * constraints for solver
   * if the machine isn't capable of manufacturing this type of product
   * takes infinity time to produce
   *
   * productionTime(j, k) records the time needed on machine k
   * for product j
   *
   * finalProduct is the boolean mask to tell whether some product
   * is the finalProduct corresponding to productionTime
   * the dep suffix means false
   */
  template <bool isFinal>
  inline void Scheduler::
  productNum2Time(
    std::vector<std::vector<TimeUnit>> &productionTime,
    std::vector<bool> &finalProduct,
    const std::vector<Machine> &machines,
    const std::vector<Integral> &productQuan,
    const std::vector<Integral> &productType)
  {
    assert(productQuan.size() == productType.size());
    const auto typeSize = productQuan.size();
    for (auto j = 0ul; j < typeSize; ++ j) {
      const auto &numProduct = productQuan[j];
      const auto &typeIndex = productType[j];

      productionTime.emplace_back(
        std::vector<TimeUnit>(machines.size()));
      finalProduct.emplace_back(isFinal);
      // newly added
      auto &currentProduct = productionTime.back();
      // per machine
      for (auto k = 0ul; k < machines.size(); ++ k) {
        const auto &machine = machines[k];
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

  void Scheduler::computeTimeNeeded()
  {
    const auto &bom = factory__->getBOM();
    std::vector<OrderWithDep> orders(factory__->getOrders().size());
    std::transform(factory__->getOrders().cbegin(),
                   factory__->getOrders().cend(),
                   orders.begin(),
                   [this](const Order &orderNoDep) {
                     return OrderWithDep(orderNoDep,
                                         factory__->getBOM());
                   });
    const auto &machines = factory__->getMachines();

    // predecessor matrix, each row corresponds to dependent product number

    // need to expand product by including
    // the dependent products on each order

    // per order
    for (auto i = 0ul; i < productionTime__.size(); ++ i) {
      auto &currentOrder = productionTime__[i];
      auto &finalProd = finalProduct__[i];
      const auto &order = orders[i];
      const auto &productQuan = order.getProductQuan();
      const auto &productType = order.getProductType();
      const auto &productQuanDep = order.getProductQuanDep();
      const auto &productTypeDep = order.getProductTypeDep();

      assert(productQuan.size() == productType.size() &&
             productQuanDep.size() == productTypeDep.size());
      // how many products type
      const auto typeSize = productQuan.size();
      const auto typeSizeDep = productQuanDep.size();
      currentOrder.reserve(typeSize + typeSizeDep);
      // per product in order
      // true => final product of the order
      productNum2Time<true>
        (currentOrder, finalProd, machines,
         productQuan, productType);
      productNum2Time<false>
        (currentOrder, finalProd, machines,
         productQuanDep, productTypeDep);
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
    const auto &finalProduct = finalProduct__;
    // here startTime one to one mapps to orders
    // with index, referring to underly products

    assert(orders.size() == startTime.size());
    // per order, per startTime. same thing
    for (auto i = 0ul; i < startTime.size(); ++ i) {
      const auto &prodStartTimes = startTime[i];
      const auto &finalProd = finalProduct[i];
      const auto &currentOrder = orders[i];
      const auto &productType = currentOrder.getProductType();

      assert(productType.size() == prodStartTime.size());
      // assume currentOrder.size() == prodStartTime.size()
      // per product of order
      for (auto j = 0ul; j < prodStartTimes.size(); ++ j) {
        // if this isn't the final product of the order
        // just skip the constraints
        if (!finalProd[j]) continue;

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

  // inline void Scheduler::
  // addConstraints_3(const std::vector<std::vector<MPVariable *>> &startTime,
  //                  const std::vector<MPVariable *> &completionTimes,
  //                  const Var3D &onMachine, const Factory &factory,
  //                  MPSolver &solver, const std::string &purposeMessage)
  // {

  // }


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
