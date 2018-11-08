#include "factoryWorld.hpp"
#include <limits>
#include <cassert>
#include <queue>

namespace FactoryWorld {
  using MPConstraint = operations_research::MPConstraint;

  /**
   * use a data structure to hold all the tuple
   * <i, j, productType> then iterate all permutation of pair
   * to find all possible gap between productTypes O(N^2).
   * given the symmetric property. only half of permutation is required
   *
   * after finding gapped productType, iterate all possible permutation
   * of two lists and add to the gap__ field O(NM)
   * N = num product of gapped type p, M = num product of gapped type q
   */
  void Scheduler::DataProvider::buildGap() {
    const auto typeSize = factory__->getBOM().getTypeSize();
    const auto &gap = factory__->getBOM().getGap();
    const auto &gapMask = factory__->getBOM().getGapMask();
    std::vector<std::vector<
      std::pair<Integral, Integral>>> typeInfo(typeSize);
    const auto &orders = orders__;

    // fill in all orders
    for (auto i = 0ul; i < orders.size(); ++ i) {
      const auto &prodType = orders[i].getProductType();
      for (auto j = 0ul; j < prodType.size(); ++ j) {
        const auto &typeIndex = prodType[j];
        typeInfo[typeIndex].emplace_back(i, j);
      }
    }

    // find gap relation and add to gap__, since symmetric,
    // only half of them permutated
    for (auto i = 0ul; i < typeSize; ++ i) {
      for (auto j = i+1; j < typeSize; ++ j) {
        if (gapMask(i, j)) {
          const auto &gapSize = gap(i, j);
          // gapped relation found, add all product of the type
          // into gap__
          for (const auto &info1 : typeInfo[i]) {
            for (const auto &info2 : typeInfo[j]) {
              gap__.emplace_back(
                std::array<Integral, 4>({
                    info1.first, info1.second,
                    info2.first, info2.second}), gapSize);
            }
          }
        }
      }
    }
  }

  Scheduler::OrderWithDep::
  OrderWithDep(Order noDepOrder,
               const RelationOfProducts &relation,
               const std::vector<Machine> &machines)
    : Order(noDepOrder)
  {
    auto &prodQuan = productQuan_;
    auto &prodType = productType_;
    const auto &bom = relation.getBOM();
    const auto &directMask = relation.getDirectMask();
    const auto &gap = relation.getGap();
    const auto &gapMask = relation.getGapMask();
    assert(prodQuan.size() == prodType.size());

    // mark end product
    endProduct__ = std::vector<bool>(prodQuan.size(), true);

    // pair of <typeIndex, num>
    std::queue<Integral> prodQueue;
    for (auto i = 0ul; i < prodQuan.size(); ++ i)
      prodQueue.emplace(i);

    // the algorithm requires that there is no cycle in the bom graph
    // otherwise, this would a dead loop
    while (!prodQueue.empty()) {
      const auto &index = prodQueue.front();

      assert(bom.cols() == directMask.cols());
      // find from bom list the dependent product
      for (auto j = 0ul; j < bom.cols(); ++ j) {
        if (directMask(prodType[index], j)) {
          const auto num = prodQuan[index] * bom(index, j);
          // the newly added element's index
          const auto indexNew = prodType.size();
          prodType.emplace_back(j);
          prodQuan.emplace_back(num);
          // add dependency for the two index
          dependency__.emplace_back(index, indexNew);
          prodQueue.emplace(indexNew);
          // not end product
          endProduct__.emplace_back(false);
        }
      }

      // pop too early may invalidate reference(prod)
      prodQueue.pop();
    }

    // load gap
    // for (auto i = 0ul; i < prodType.size(); ++ i) {
    //   for (auto j = i+1; j < prodType.size(); ++ j) {
    //     // assume gapMask and gap is symmetric
    //     const auto &a = prodType[i];
    //     const auto &b = prodType[j];
    //     if (gapMask(a, b))
    //       gap__.emplace_back(i, j, gap(a, b));
    //   }
    // }

    productionTime__.reserve(prodQuan.size());
    // transfrom to production time
    productNum2Time(productionTime__, machines, prodQuan, prodType);
  }

  /**
   * transfrom product number to time using each machine's capability
   *
   * if the machine isn't capable of manufacturing this type of product
   * takes infinity time to produce
   *
   * productionTime(j, k) records the time needed on machine k
   * for product j
   *
   */
  inline void Scheduler::OrderWithDep::
  productNum2Time(
    std::vector<std::vector<TimeUnit>> &productionTime,
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

  // void Scheduler::computeTimeNeeded()
  // {
  //   const auto &bom = factory__->getBOM();
  //   std::vector<OrderWithDep> orders(factory__->getOrders().size());
  //   std::transform(factory__->getOrders().cbegin(),
  //                  factory__->getOrders().cend(),
  //                  orders.begin(),
  //                  [this](const Order &orderNoDep) {
  //                    return OrderWithDep(orderNoDep,
  //                                        factory__->getBOM());
  //                  });
  //   const auto &machines = factory__->getMachines();

  //   // predecessor matrix, each row corresponds to dependent product number

  //   // need to expand product by including
  //   // the dependent products on each order

  //   // per order
  //   for (auto i = 0ul; i < productionTime__.size(); ++ i) {
  //     auto &currentOrder = productionTime__[i];
  //     auto &finalProd = finalProduct__[i];
  //     const auto &order = orders[i];
  //     const auto &productQuan = order.getProductQuan();
  //     const auto &productType = order.getProductType();
  //     const auto &productQuanDep = order.getProductQuanDep();
  //     const auto &productTypeDep = order.getProductTypeDep();

  //     assert(productQuan.size() == productType.size() &&
  //            productQuanDep.size() == productTypeDep.size());
  //     // how many products type
  //     const auto typeSize = productQuan.size();
  //     const auto typeSizeDep = productQuanDep.size();
  //     currentOrder.reserve(typeSize + typeSizeDep);
  //     // per product in order
  //     // true => final product of the order
  //     productNum2Time<true>
  //       (currentOrder, finalProd, machines,
  //        productQuan, productType);
  //     productNum2Time<false>
  //       (currentOrder, finalProd, machines,
  //        productQuanDep, productTypeDep);
  //   }
  // }

  inline std::vector<MPConstraint *> Scheduler::
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
    return constraints;
  }

  inline std::vector<MPConstraint *> Scheduler::
  addConstraints_2(const std::vector<std::vector<MPVariable *>> &startTime,
                   const std::vector<MPVariable *> &completionTimes,
                   const Var3D &onMachine, MPSolver &solver,
                   const std::string &purposeMessage)
  {
    std::vector<MPConstraint *> constraints;
    const auto &orders = dataProvier__.getOrders();
    const auto &machines = dataProvier__.getMachines();
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
        // if this isn't the final product of the order
        // just skip the constraints
        if (!currentOrder.finalProd(j)) continue;

        const auto &typeIndex = productType[j];
        const auto &prodStart = prodStartTimes[j];
        // per machine that is capable
        for (auto k = 0ul; k < machines.size(); ++ k) {
          const auto &machine = machines[k];
          const auto timesOnMach = currentOrder.requiredTime(j, k);

          // add constraint if the machine can produce this type of product
          // doesn't pose constraints if the machine cannot produce the product
          if (machine.capable(typeIndex)) {
            constraints.emplace_back(
              solver.MakeRowConstraint(-infinity,
                largeNumber - timesOnMach,
                // TODO: How to make this mess into a templated or macro
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
    return constraints;
  }

  inline std::vector<MPConstraint *> Scheduler::
  addConstraints_3(const std::vector<std::vector<MPVariable *>> &startTime,
                   const Var3D &onMachine, MPSolver &solver,
                   const std::string &purposeMessage)
  {
    std::vector<MPConstraint *> constraints;
    const auto &orders = dataProvier__.getOrders();
    const auto &machines = dataProvier__.getMachines();
    for (auto i = 0ul; i < startTime.size(); ++ i) {
      const auto &currentOrder = orders[i];
      const auto &dep = currentOrder.getDependency();
      const auto &currentStart = startTime[i];

      // for every pair of dependency, add to constraint
      for (const auto &pairIndex : dep) {
        // <p, q> pair, p can start only if q has finished
        for (auto k = 0ul; k < machines.size(); ++ k) {
          const auto &machine = machines[k];
          const auto p = pairIndex.first;
          const auto q = pairIndex.second;
          // product q required time on machine k
          const auto timeOnMach = currentOrder.requiredTime(q, k);
          constraints.emplace_back(
            solver.MakeRowConstraint(-infinity,
              largeNumber - timeOnMach,
              (purposeMessage +
                "_(" + std::to_string(i) + ", " +
                std::to_string(p) + ", " + std::to_string(q) + ", " +
                std::to_string(k) + ")")));
          constraints.back()->SetCoefficient(currentStart[q], 1.0);
          constraints.back()->SetCoefficient(currentStart[p], -1.0);
          constraints.back()->SetCoefficient(onMachine[i][q][k], largeNumber);
        }
      }
    }
    LOG(INFO) << purposeMessage;
    return constraints;
  }

  // inline std::vector<MPConstraint *> Scheduler::
  // addConstraints_4(
  //   const std::vector<std::vector<MPVariable *>> &startTime,
  //   const Var3D &onMachine,
  //   MPSolver &solver,
  //   const std::string &purposeMessage)
  // {
  //   std::vector<MPConstraint *> constraints;
  //   const auto &orders = dataProvier__.getOrders();
  //   const auto &machines = dataProvier__.getMachines();
  //   for (auto i = 0ul; i < startTime.size(); ++ i) {
  //     const auto &currentOrder = orders[i];
  //     const auto &gap = currentOrder.getGap();
  //     const auto &currentStart = startTime[i];

  //     // for every gap tuple <p, q, time> add to constraint
  //     for (const auto &tupleGap : gap) {
  //       // <p, q, time> is symmetric => <q, p, time>
  //       // so each of these results in two constraints
  //       for (auto k = 0ul; k < machines.size(); ++ k) {
  //         const auto &machine = machines[k];
  //         const auto p = std::get<0>(tupleGap);
  //         const auto q = std::get<1>(tupleGap);
  //         const auto gapTime = std::get<2>(tupleGap);
  //         // since symmetric, two constraints considered

  //         const auto timeOnMach1 = currentOrder.requiredTime(q, k);
  //         const auto timeOnMach2 = currentOrder.requiredTime(p, k);
  //         constraints.emplace_back(
  //           solver.MakeRowConstraint(-infinity,
  //             largeNumber - timeOnMach1 - gapTime,
  //             (purposeMessage +
  //               "_(" + std::to_string(i) + ", " +
  //               std::to_string(p) + ", " + std::to_string(q) + ", " +
  //               std::to_string(k) + ")")));
  //         constraints.back()->SetCoefficient(currentStart[q], 1.0);
  //         constraints.back()->SetCoefficient(currentStart[p], -1.0);
  //         constraints.back()->SetCoefficient(onMachine[i][q][k], largeNumber);

  //         constraints.emplace_back(
  //           solver.MakeRowConstraint(-infinity,
  //             largeNumber - timeOnMach2 - gapTime,
  //             (purposeMessage +
  //               "_(" + std::to_string(i) + ", " +
  //               std::to_string(q) + ", " + std::to_string(p) + ", " +
  //               std::to_string(k) + ")")));
  //         constraints.back()->SetCoefficient(currentStart[p], 1.0);
  //         constraints.back()->SetCoefficient(currentStart[q], -1.0);
  //         constraints.back()->SetCoefficient(onMachine[i][p][k], largeNumber);
  //       }
  //     }
  //   }
  //   return constraints;
  // }
  inline std::vector<MPConstraint *> Scheduler::
  addConstraints_4(
    const std::vector<std::vector<MPVariable *>> &startTime,
    const Var3D &onMachine,
    MPSolver &solver,
    const std::string &purposeMessage)
  {
      std::vector<MPConstraint *> constraints;
      const auto &orders = dataProvier__.getOrders();
      const auto &machines = dataProvier__.getMachines();

      for (const auto &gap : gap__) {

      }
  }

  inline std::vector<MPConstraint *> Scheduler::
  addConstraints_5(
    const std::vector<std::vector<MPVariable *>> &startTime,
    MPSolver &solver,
    const std::string &purposeMessage)
  {
    LOG(WARNING) << "material Date to TimeUnit unimplemented";
    std::vector<MPConstraint *> constraints;
    const auto &orders = dataProvier__.getOrders();
    for (auto i = 0ul; i < startTime.size(); ++ i) {
      const auto &currentStart = startTime[i];
      const auto &currentOrder = orders[i];
      // TODO: material to TimeUnit
      const auto &rawMaterial = currentOrder.getMaterialDate();
      // for every product, it should wait until raw material to arrive
      for (auto j = 0ul; j < currentStart.size(); ++ j) {
        constraints.emplace_back(
          solver.MakeRowConstraint(rawMaterial, infinity,
            (purposeMessage + "_(" + std::to_string(i) + ", " +
              std::to_string(j) + ")")));
        constraints.back()->SetCoefficient(currentStart[j], 1.0);
      }
    }
    return constraints;
  }

  inline std::vector<MPConstraint *> Scheduler::
  addConstraints_6(
    const std::vector<std::vector<MPVariable *>> &startTime,
    const Var3D &onMachine,
    MPSolver &solver,
    const std::string &purposeMessage)
  {
    std::vector<MPConstraint *> constraints;
    const auto &machines = dataProvier__.getMachines();
    const auto &orders = dataProvier__.getOrders();
    for (auto i = 0ul; i < startTime.size(); ++ i) {
      const auto &currentOrder = orders[i];
      const auto &currentStart = startTime[i];
      for (auto j = 0ul; j < currentStart.size(); ++ j) {
        for (auto k = 0ul; k < machines.size(); ++ k) {
          const auto &machine = machines[k];
          const auto &ready = machine.getReadyTime();
          constraints.emplace_back(
            solver.MakeRowConstraint(
              ready - largeNumber,
              infinity, (purposeMessage + "_(" + std::to_string(i) +
                ", " + std::to_string(j) + ", " + std::to_string(k) + ")")));
          constraints.back()->SetCoefficient(currentStart[j], 1.0);
          constraints.back()->SetCoefficient(onMachine[i][j][k], -largeNumber);
        }
      }
    }
    return constraints;
  }

  inline std::vector<MPConstraint *> Scheduler::
  addConstraints_7(
    const std::vector<std::vector<MPVariable *>> &startTime,
    const std::vector<Var3D> &immediatePrec,
    MPSolver &solver, const std::string &purposeMessage)
  {

    const auto &transCost = dataProvier__.getTransCost();
  }

  void Scheduler::factoryScheduler(const Factory &factory,
    MPSolver::OptimizationProblemType optimization_problem_type)
  {
    using namespace operations_research;
    LOG(INFO) << "Building planner";
    MPSolver solver("FactorySolver", optimization_problem_type);

    // prepare parameters
    //computeTimeNeeded();

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
    addConstraints_2(startTime, completionTimes, onMachine,
                     solver, "BOM product relation precedence relation");
    addConstraints_3(startTime, onMachine,
                     solver, "dependent relation between product of same order");
    // TODO: not corrent, should reimplement
    // addConstraints_4(startTime, onMachine,
    //   solver, "gap between products");
    addConstraints_5(startTime, solver, "manufacturing wait raw material");
    addConstraints_6(startTime, onMachine, solver, "manufacturing when machine is ready");
  }
}
