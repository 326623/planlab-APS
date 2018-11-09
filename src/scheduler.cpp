#include "factoryWorld.hpp"
#include <limits>
#include <cassert>
#include <queue>

namespace FactoryWorld {
  using MPConstraint = operations_research::MPConstraint;

  Scheduler::DataProvider::DataProvider(std::shared_ptr<const Factory> factory)
    : factory__(factory)
  {
    const auto &bom = factory__->getBOM();
    const auto &oldOrders = factory__->getOrders();
    const auto &machines = factory__->getMachines();
    orders__.reserve(oldOrders.size());
    for (auto i = 0ul; i < oldOrders.size(); ++ i)
      orders__.emplace_back(oldOrders[i], bom, machines);

    // build the gap that will be required by the scheduler
    buildGap();
  }

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
  inline void Scheduler::DataProvider::buildGap() {
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
    const auto &orders = dataProvider__.getOrders();
    const auto &machines = dataProvider__.getMachines();
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
    const auto &orders = dataProvider__.getOrders();
    const auto &machines = dataProvider__.getMachines();
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

  inline std::vector<MPConstraint *> Scheduler::
  addConstraints_4(
    const std::vector<std::vector<MPVariable *>> &startTime,
    const Var3D &onMachine,
    MPSolver &solver,
    const std::string &purposeMessage)
  {
      std::vector<MPConstraint *> constraints;
      const auto &orders = dataProvider__.getOrders();
      const auto &machines = dataProvider__.getMachines();
      const auto &dataGap = dataProvider__.getGap();

      for (const auto &gap : dataGap) {
        for (auto k = 0ul; k < machines.size(); ++ k) {
          const auto &gapArray = gap.first;
          const auto &gapTime = gap.second;
          //Integral i, p, j, q = , gapArray[1], gapArray[2], gapArray[3];
          auto i = gapArray[0], p = gapArray[1], j = gapArray[2], q = gapArray[3];
          // product p of order i
          const auto &timeOnMach1 = orders[i].requiredTime(p, k);
          // product q of order j
          const auto &timeOnMach2 = orders[j].requiredTime(q, k);

          // symmetric constraints
          constraints.emplace_back(
            solver.MakeRowConstraint(-infinity,
              largeNumber - timeOnMach1 - gapTime,
              (purposeMessage + "_(" + std::to_string(i) + ", " +
                std::to_string(p) + ", " + std::to_string(j) + ", " +
                std::to_string(q) + ")")));
          constraints.back()->SetCoefficient(startTime[i][p], 1.0);
          constraints.back()->SetCoefficient(startTime[j][q], -1.0);
          constraints.back()->SetCoefficient(onMachine[i][p][k], largeNumber);

          constraints.emplace_back(
            solver.MakeRowConstraint(-infinity,
              largeNumber - timeOnMach2 - gapTime,
              (purposeMessage + "_(" + std::to_string(j) + ", " +
                std::to_string(q) + ", " + std::to_string(i) + ", " +
                std::to_string(p) + ")")));
          constraints.back()->SetCoefficient(startTime[j][q], 1.0);
          constraints.back()->SetCoefficient(startTime[i][p], -1.0);
          constraints.back()->SetCoefficient(onMachine[j][q][k], largeNumber);
        }
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
    const auto &orders = dataProvider__.getOrders();
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
    LOG(WARNING) << "can reduce constraint on 6";
    const auto &machines = dataProvider__.getMachines();
    const auto &orders = dataProvider__.getOrders();
    for (auto i = 0ul; i < startTime.size(); ++ i) {
      const auto &currentOrder = orders[i];
      const auto &currentStart = startTime[i];
      for (auto j = 0ul; j < currentStart.size(); ++ j) {
        for (auto k = 0ul; k < machines.size(); ++ k) {
          const auto &machine = machines[k];
          const auto &ready = machine.getReadyTime();
          //if (machine.capable())
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
    const std::vector<std::vector<Var3D>> &immediatePrec,
    MPSolver &solver, const std::string &purposeMessage)
  {
    const auto &transCosts = dataProvider__.getTransCost();
    const auto &orders = dataProvider__.getOrders();
    const auto &machines = dataProvider__.getMachines();
    std::vector<MPConstraint *> constraints;

    for (auto i = 0ul; i < startTime.size(); ++ i) {
      for (auto j = 0ul; j < startTime.size(); ++ j) {
        const auto &order_i = orders[i];
        const auto &order_j = orders[j];
        const auto &typeIndex_i = order_i.getProductType();
        const auto &typeIndex_j = order_j.getProductType();

        for (auto p = 0ul; p < startTime[i].size(); ++ p) {
          for (auto q = 0ul; q < startTime[j].size(); ++ q) {
            // same product doesn't have restriction
            if (i == j && p == q) continue;

            const auto &type_p = typeIndex_i[p];
            const auto &type_q = typeIndex_j[q];
            // constraint on every machine
            for (auto k = 0ul; k < machines.size(); ++ k) {
              const auto &machine = machines[k];
              // product p that(if) precedes product q
              // product q can only be scheduled after p
              // has finished, and wait for transfer time from
              // p -> q(transCost)

              // if machine cannot produce both of it, just skip it
              if (!machine.capable(type_p) || !machine.capable(type_q))
                continue;

              const auto &p2qTransCost = transCosts(type_p, type_q);
              // immediate relationship between product p, and q on machine k
              const auto &immediate = immediatePrec[i][p][j][q][k];
              constraints.emplace_back(
                solver.MakeRowConstraint(-infinity,
                  largeNumber - order_i.requiredTime(p, k),
                  (purposeMessage + std::to_string(i) + std::to_string(p) +
                    std::to_string(j) + std::to_string(q) +
                    std::to_string(k))));
              constraints.back()->SetCoefficient(startTime[i][p], 1.0);
              constraints.back()->SetCoefficient(immediate, p2qTransCost);
              constraints.back()->SetCoefficient(immediate, largeNumber);
              constraints.back()->SetCoefficient(startTime[j][q], -1.0);
            }
          }
        }
      }
    }
    return constraints;
  }

  inline std::vector<MPConstraint *> Scheduler::
  addConstraints_8(
    const Var3D &onMachine,
    const std::vector<std::vector<Var3D>> &immediatePrec,
    MPSolver &solver, const std::string &purposeMessage) {

    LOG(WARNING) << "can reduce constraint num on 8";
    std::vector<MPConstraint *> constraints;
    const auto &orders = dataProvider__.getOrders();
    const auto &machines = dataProvider__.getMachines();
    for (auto i = 0ul; i < onMachine.size(); ++ i) {
      for (auto j = 0ul; j < onMachine.size(); ++ j) {
        const auto &iOnMachine = onMachine[i];
        const auto &jOnMachine = onMachine[j];
        const auto &order_i = orders[i];
        const auto &order_j = orders[j];
        const auto &typeIndex_i = order_i.getProductType();
        const auto &typeIndex_j = order_j.getProductType();

        for (auto p = 0ul; p < onMachine[i].size(); ++ p) {
          for (auto q = 0ul; q < onMachine[j].size(); ++ q) {
            const auto &pOnMachine = iOnMachine[p];
            const auto &qOnMachine = iOnMachine[q];
            // same product doesn't have restriction
            if (i == j && p == q) continue;

            const auto &type_p = typeIndex_i[p];
            const auto &type_q = typeIndex_j[q];
            // constraint on every machine
            for (auto k = 0ul; k < machines.size(); ++ k) {
              const auto &machine = machines[k];
              // product p that(if) precedes product q
              // product q can only be scheduled after p
              // has finished, and wait for transfer time from
              // p -> q(transCost)
              if (!machine.capable(type_p) || !machine.capable(type_q)) continue;

              // immediate relationship between product p, and q on machine k
              const auto &immediate_qp = immediatePrec[j][q][i][p][k];
              const auto &immediate_pq = immediatePrec[i][p][j][q][k];
              constraints.emplace_back(
                solver.MakeRowConstraint(-infinity, 0,
                  (purposeMessage + std::to_string(i) + std::to_string(p) +
                    std::to_string(j) + std::to_string(q) +
                    std::to_string(k))));
              constraints.back()->SetCoefficient(immediate_qp, 2.0);
              constraints.back()->SetCoefficient(immediate_pq, 2.0);
              constraints.back()->SetCoefficient(pOnMachine[k], -1.0);
              constraints.back()->SetCoefficient(qOnMachine[k], -1.0);
            }
          }
        }
      }
    }
    return constraints;
  }

  inline std::vector<MPConstraint *> Scheduler::
  addConstraints_9(
    const Var3D &onMachine,
    MPSolver &solver, const std::string &purposeMessage) {
    std::vector<MPConstraint *> constraints;
    const auto &machines = dataProvider__.getMachines();
    const auto &orders = dataProvider__.getOrders();

    for (auto i = 0ul; i < onMachine.size(); ++ i) {
      for (auto p = 0ul; p < onMachine[i].size(); ++ p) {
        const auto typeIndex = orders[i].getProductType()[p];

        constraints.emplace_back(solver.MakeRowConstraint(1, 1,
            (purposeMessage + std::to_string(i) + ", " + std::to_string(p))));

        // What would happen if there are no machine capable of manufacturing
        // this type of product

        for (auto k = 0ul; k < onMachine[i][p].size(); ++ k) {
          const auto &machine = machines[k];
          if (machine.capable(typeIndex))
            constraints.back()->SetCoefficient(onMachine[i][p][k], 1);
        }
      }
    }
    return constraints;
  }

  inline std::vector<MPConstraint *> Scheduler::
  addConstraints_10(
    const Var3D &onMachine,
    MPSolver &solver, const std::string &purposeMessage) {
    std::vector<MPConstraint *> constraints;
    const auto &machines = dataProvider__.getMachines();
    const auto &orders = dataProvider__.getOrders();

    for (auto i = 0ul; i < onMachine.size(); ++ i) {
      for (auto p = 0ul; p < onMachine[i].size(); ++ p) {
        for (auto k = 0ul; k < machines.size(); ++ k) {
          // on machine that is incapable of manufacturing this product
          // add constraint
          if (!machines[k].capable(p)) {
            constraints.emplace_back(solver.MakeRowConstraint(0, 0,
                (purposeMessage + std::to_string(i) + ", " + std::to_string(p))));
            constraints.back()->SetCoefficient(onMachine[i][p][k], 1.0);
          }
        }
      }
    }
    return constraints;
  }

  inline std::vector<MPConstraint *> Scheduler::
  addConstraints_11(
    const std::vector<std::vector<Var3D>> &immediatePrec,
    const Var3D &dummyPrec,
    const Var3D &dummySucc,
    MPSolver &solver, const std::string &purposeMessage) {

    const auto &orders = dataProvider__.getOrders();
    const auto &machines = dataProvider__.getMachines();

    // predecessor relation
    std::vector<std::vector<MPConstraint *>> constraintsPrec(orders.size());
    for (auto i = 0ul; i < orders.size(); ++ i) {
      for (auto p = 0ul; p < orders[i].size(); ++ p) {
        constraintsPrec[i].emplace_back(
          solver.MakeRowConstraint(1, 1,
            (purposeMessage + std::to_string(i) + ", " + std::to_string(p) + " Prec")));
      }
    }

    // immediatePrec i, p either is true for j, q that precedes on line k
    // or is true for dummpyPrec on line k, indicating it's the first on line k
    for (auto i = 0ul; i < orders.size(); ++ i) {
      for (auto p = 0ul; p < orders[i].size(); ++ p) {
        for (auto k = 0ul; k < machines.size(); ++ k) {
          constraintsPrec[i][p]->SetCoefficient(dummyPrec[i][p][k], 1);
        }
      }
    }

    for (auto j = 0ul; j < immediatePrec.size(); ++ j) {
      for (auto q = 0ul; q < immediatePrec[j].size(); ++ q) {
        for (auto i = 0ul; i < immediatePrec[j][q].size(); ++ i) {
          for (auto p = 0ul; p < immediatePrec[j][q][i].size(); ++ p) {
            for (auto k = 0ul; k < immediatePrec[j][q][i][p].size(); ++ k) {
              if (j == i && q == p) continue;
              // for constraint of product p, accumulate all product q
              // excluding q == p
              constraintsPrec[i][p]->SetCoefficient(immediatePrec[j][q][i][p][k], 1);
            }
          }
        }
      }
    }

    // successor relation
    std::vector<std::vector<MPConstraint *>> constraintsSucc(orders.size());
    for (auto i = 0ul; i < orders.size(); ++ i) {
      for (auto p = 0ul; p < orders[i].size(); ++ p) {
        constraintsSucc[i].emplace_back(
          solver.MakeRowConstraint(1, 1,
            (purposeMessage + std::to_string(i) + ", " + std::to_string(p) + " Succ")));
      }
    }

    // immediatePrec i, p either is true for j, q that is successor on line k
    // or is true for dummpySucc on line k, indicating it's the first on line k
    for (auto i = 0ul; i < orders.size(); ++ i) {
      for (auto p = 0ul; p < orders[i].size(); ++ p) {
        for (auto k = 0ul; k < machines.size(); ++ k) {
          constraintsSucc[i][p]->SetCoefficient(dummySucc[i][p][k], 1);
        }
      }
    }

    // TODO: should refactor this part
    for (auto j = 0ul; j < immediatePrec.size(); ++ j) {
      for (auto q = 0ul; q < immediatePrec[j].size(); ++ q) {
        for (auto i = 0ul; i < immediatePrec[j][q].size(); ++ i) {
          for (auto p = 0ul; p < immediatePrec[j][q][i].size(); ++ p) {
            for (auto k = 0ul; k < immediatePrec[j][q][i][p].size(); ++ k) {
              if (j == i && q == p) continue;
              // for constraint of product p, accumulate all product q
              // excluding q == p
              constraintsSucc[i][p]->SetCoefficient(immediatePrec[i][p][j][q][k], 1);
            }
          }
        }
      }
    }

    // std::vector<MPConstraint *> constraintDummyPrec;
    // // dummyPrec and dummySucc
    // for (auto k = 0ul; k < machines.size(); ++ k) {
    //   constraintDummyPrec.emplace_back(
    //     solver.MakeRowConstraint(1, 1, ))
    //     for (auto i = 0ul; i < dummyPrec.size(); ++ i) {
    //       for (auto p = 0ul; p < dummyPrec[i].size(); ++ p) {

    //       }
    //     }
    // }

    LOG(WARNING) << "return null constraint for now.";
    return std::vector<MPConstraint *>();
  }

  inline std::vector<MPConstraint *> Scheduler::
  addConstraints_12(
    const std::vector<MPVariable *> &completionTimes,
    const std::vector<MPVariable *> &tardyTime,
    MPSolver &solver, const std::string &purposeMessage) {
    std::vector<MPConstraint *> constraints;
    const auto &orders = dataProvider__.getOrders();
    assert(completionTimes.size() == tardyTime.size() &&
           tardyTime.size() == orders.size());

    const auto size = completionTimes.size();
    for (auto i = 0ul; i < size; ++ i) {
      const auto dueTime = orders[i].getDueTime();
      constraints.emplace_back(
        solver.MakeRowConstraint(-infinity,
          dueTime, (purposeMessage + "i")));

      constraints.back()->SetCoefficient(completionTimes[i], 1.0);
      constraints.back()->SetCoefficient(tardyTime[i], -1.0);
    }

    return constraints;
  }

  inline std::vector<MPConstraint *> Scheduler::
  addConstraints_13(
    const std::vector<MPVariable *> &completionTimes,
    const std::vector<MPVariable *> &earlyTime,
    MPSolver &solver, const std::string &purposeMessage) {
    std::vector<MPConstraint *> constraints;
    const auto &orders = dataProvider__.getOrders();
    assert(completionTimes.size() == tardyTime.size() &&
           tardyTime.size() == orders.size());

    const auto size = completionTimes.size();
    for (auto i = 0ul; i < size; ++ i) {
      const auto dueTime = orders[i].getDueTime();
      constraints.emplace_back(
        solver.MakeRowConstraint(
          dueTime, infinity, (purposeMessage + std::to_string(i))));
      constraints.back()->SetCoefficient(completionTimes[i], 1.0);
      constraints.back()->SetCoefficient(earlyTime[i], 1.0);
    }

    return constraints;
  }

  inline std::vector<MPConstraint *> Scheduler::
  addConstraints_14(
    const std::vector<MPVariable *> &tardyTime,
    MPSolver &solver, const std::string &purposeMessage) {
    std::vector<MPConstraint *> constraints;
    const auto size = tardyTime.size();
    for (auto i = 0ul; i < size; ++ i) {
      constraints.emplace_back(
        solver.MakeRowConstraint(
          0, infinity, (purposeMessage + std::to_string(i))));
      constraints.back()->SetCoefficient(tardyTime[i], 1.0);
    }
    return constraints;
  }

  inline std::vector<MPConstraint *> Scheduler::
  addConstraints_15(
    const std::vector<MPVariable *> &earlyTime,
    MPSolver &solver, const std::string &purposeMessage) {
    std::vector<MPConstraint *> constraints;
    const auto size = earlyTime.size();
    for (auto i = 0ul; i < size; ++ i) {
      constraints.emplace_back(
        solver.MakeRowConstraint(
          0, infinity, (purposeMessage + std::to_string(i))));
      constraints.back()->SetCoefficient(earlyTime[i], 1.0);
    }
    return constraints;
  }

  inline std::vector<MPConstraint *> Scheduler::
  addConstraints_16(
    const Var3D &dummyPrec,
    MPSolver &solver, const std::string &purposeMessage) {
    std::vector<MPConstraint *> constraints;
    const auto sizeOrder = dummyPrec.size();
    const auto machineSize = dataProvider__.getMachines().size();

    for (auto k = 0ul; k < machineSize; ++ k) {
      // on every production line, there is only one position for
      // the first product dummyPrec[i][p][k] means that
      // product p will be the first to produce on line k
      constraints.emplace_back(
        solver.MakeRowConstraint(1, 1,
          (purposeMessage + std::to_string(k))));
      for (auto i = 0ul; i < sizeOrder; ++ i) {
        const auto sizeProduct = dummyPrec[i].size();
        for (auto p = 0ul; p < sizeProduct; ++ p) {
          assert(dummyPrec[i][p].size() == machineSize);
          //dummyPrec[i][p][k]
          constraints.back()->SetCoefficient(dummyPrec[i][p][k], 1);
        }
      }
    }
    return constraints;
  }

  inline std::vector<MPConstraint *> Scheduler::
  addConstraints_17(
    const Var3D &dummySucc,
    MPSolver &solver, const std::string &purposeMessage) {
    std::vector<MPConstraint *> constraints;
    const auto sizeOrder = dummySucc.size();
    const auto machineSize = dataProvider__.getMachines().size();

    for (auto k = 0ul; k < machineSize; ++ k) {
      // on every production line, there is only one position for
      // the first product dummyPrec[i][p][k] means that
      // product p will be the first to produce on line k
      constraints.emplace_back(
        solver.MakeRowConstraint(1, 1,
          (purposeMessage + std::to_string(k))));
      for (auto i = 0ul; i < sizeOrder; ++ i) {
        const auto sizeProduct = dummySucc[i].size();
        for (auto p = 0ul; p < sizeProduct; ++ p) {
          assert(dummySucc[i][p].size() == machineSize);
          //dummyPrec[i][p][k]
          constraints.back()->SetCoefficient(dummySucc[i][p][k], 1);
        }
      }
    }
    return constraints;
  }

  void Scheduler::factoryScheduler(const Factory &factory,
    MPSolver::OptimizationProblemType optimization_problem_type)
  {
    using namespace operations_research;
    LOG(INFO) << "Building planner";
    MPSolver solver("FactorySolver", optimization_problem_type);

    // prepare parameters

    // prepare variables
    // need to be careful about the dummy variables
    std::vector<MPVariable *> tardyTime;
    std::vector<MPVariable *> earlyTime;
    Var3D onMachine; // bool var
    std::vector<std::vector<Var3D>> immediatePrec; // bool var
    // size == num of machine
    Var3D dummyPrec;
    Var3D dummySucc;
    std::vector<std::vector<MPVariable *>> startTime;
    MPVariable const *makeSpan = solver.MakeNumVar(0.0, infinity, "MakeSpan");
    std::vector<MPVariable *> completionTimes;

    const auto orderSize = factory.getOrders().size();
    solver.MakeNumVarArray(orderSize, 0.0, infinity,
                           "CompletionTime", &completionTimes);

    // adding constraints
    addConstraints_1(completionTimes, makeSpan, solver,
      "CompletionTime shoule precede MakeSpan");
    addConstraints_2(startTime, completionTimes, onMachine, solver,
      "BOM product relation precedence relation");
    addConstraints_3(startTime, onMachine, solver,
      "dependent relation between product of same order");
    addConstraints_4(startTime, onMachine, solver,
      "gap between products");
    addConstraints_5(startTime, solver,
      "manufacturing wait raw material");
    addConstraints_6(startTime, onMachine, solver,
      "manufacturing when machine is ready");
    addConstraints_7(startTime, immediatePrec, solver,
      "immediate pair should wait with transfer cost");
    addConstraints_8(onMachine, immediatePrec, solver,
      "immediate pair asymmetric on same line");
    addConstraints_9(onMachine, solver,
      "product only on 1 line");
    addConstraints_10(onMachine, solver,
      "product only on capable line");

    // unnecessary
    // addConstraints_11(makeSpan, solver,
    //   "makeSpan is positive");
    // startTime not negative

    // enforce immediate to be sequential
    addConstraints_11(immediatePrec, dummyPrec, dummySucc, solver,
      "sequentialize immediate relation");

    // rules of variables in objective function
    addConstraints_12(completionTimes, tardyTime, solver,
      "tardy time");
    addConstraints_13(completionTimes, earlyTime, solver,
      "early time");

    addConstraints_14(tardyTime, solver,
      "tardy time non-negative");
    addConstraints_15(earlyTime, solver,
      "early time non-negative");

    // rules on dummy variables
    addConstraints_16(dummyPrec, solver,
      "dummyPrec of each line only one successor");

    addConstraints_17(dummySucc, solver,
      "dummySucc of each line only one predecessor");
  }
}
