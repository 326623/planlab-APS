#include <cassert>
#include <limits>
#include <nlohmann/json.hpp>
#include <queue>
#include "factoryWorld.hpp"

namespace FactoryWorld {
/** \class Scheduler
 * Scheduler class handles the planning by expressing it under
 * linear constraints.
 *
 * \todo
 * - Scheduler should take some unmutable Factory as data input, here a
 * shared_ptr is clearly not a right choise.
 * - Need a faster solution for this task. The current version cannot
 * effectively solve product size of over 200.
 *
 * The current implementation has 19 linear constraints expressing the factory
 * scheduling model.
 * To discuss these constraints, we first need the following notations:
 *
 * \f$ i, j \f$: index of order i, j. i, j = 1, 2, \f$ \cdots \f$, N. N is the
 * number of orders.
 *
 * \f$ p, q \f$: index of product p, q. p, q = 1, 2, \f$ \cdots \f$, P. P is the
 * number of products.
 *
 * \f$ k \f$: index of production line k. k = 1, 2, \f$ \cdots \f$, M. M is the
 * number of production lines.
 *
 * \section model_var planning variables:
 *
 * \f$ T_i \f$: tardy time of order i
 *
 * \f$ E_i \f$: early time of order i
 *
 * \f$ Z_{pk} \f$: the product p is scheduled on production line k
 *
 * \f$ Y_{pqk} \f$: product p directly precedes product q on line k
 *
 * \f$ DH_{pk} \f$: the first production of line k is product p if \f$ DH_{pk}
 * \f$ is true, dicision variable, abbrev.: dummy head, or dummy precedence
 *
 * \f$ DT_{pk} \f$: the last production of line k is product p if \f$ DT_{pk}
 * \f$ is true, dicision variable, abbrev.: dummy tail, or dummy succedent
 *
 * \f$ S_p \f$: production start time of product p
 *
 * \f$ C_{max} \f$: make span of planner
 *
 * \f$ C_i \f$: product completion of order i
 *
 * \f$ L_i \f$: unit time of tardy production of order i
 *
 * \f$ E_i \f$: unit time of eardy production of order i
 *
 * \section model_parm Predetermined parameters:
 *
 * \f$ t_{pk} \f$: required time for product p on line k
 *
 * \f$ r_k \f$: ready time of line k
 *
 * \f$ d_i \f$: due time of order i
 *
 * \f$ F_p \f$: the set of production line that can manufacture product p
 *
 * \f$ R \f$: set of dependent relationnal pairs, with \f$ (p, q) \in R \f$,
 * means that q must be processed before p
 *
 * \f$ G \f$: Set of tuple gap time between product p, q, with \f$ (p, q, t) \in
 * G \f$. means that these two products' scheduled interval must be larger than
 * t. Note that this relation is symmetric.
 *
 * \f$ WP_i \f$: Order i preparation time, cannot schedule any products of i
 * before this time
 *
 * \f$ P_i \f$: product set of order i, all products belong to order i
 *
 * \f$ WT_{pq} \f$: transfer time(setup time) from product p to product q
 *
 * \f$ IC \f$: idle cost per unit time
 *
 * \f$ TC \f$: tardy cost per unit time
 *
 * \f$ EC \f$: early cost per unit time
 *
 * \f$ LN \f$: Large number for special linear constriants. Numerical
 * experiments show that if too large would affect solving speed and even unable
 * to solve because of the unstability.
 *
 * Next we formulate the objective functions of the problem:
 *
 * \f[ f_1 = (M C_{max} - \sum_{p=1}^{P}\sum_{k \in F_p}{Z_{pk} t_{pk} +
 * \sum_{k=1}^P r_k}), \f]
 * \f[ f_2 = (\sum_{i=1}^N {L_i}), \f]
 * \f[ f_3 = (\sum_{i=1}^N {E_i}), \f]
 * \f[ f = IC \cdot f_1 + TC \cdot f_2 + EC \cdot f_3 \f]
 *
 * We then have the following linear constraints:
 *
 *
 * \f$ C_i \le C_{max}, i=1,2,\cdots,N \f$  completion time should precede make
 * span
 *
 * \f$ \displaystyle{S_p + \sum_{k \in F_p} {t_{pk} Z_{pk}} \le C_i ,\ \ \forall
 * p \in P_i} \f$  completion time later than all finished time of products
 *
 * \f$ \displaystyle{S_q + \sum_{k \in F_q} {t_{qk} Z_{qk}} \le S_p ,\ \ \forall
 * (p, q) \in R} \f$ dependent relation between products of same order(R is
 * constructed by combining the dep information of BOM and same order product)
 *
 * \f$ \displaystyle{S_q + \sum_{k} {t_{qk} Z_{qk}} + t \le S_p,\ \ \ \forall
 * (p, q, t) \in G} \f$.
 *
 * \f$ WP_i \le S_p, \ \ \forall p \in P_i \f$ manufacturing wait raw material
 *
 * \f$ \displaystyle{\sum_{k \in F_p} r_k Z_{pk} \le S_p, \ \ \forall p} \f$
 * manufacturing when machine is ready
 *
 * \f$ S_p + t_{pk} + WT_{pq} Y_{pqk} - LN(1 - Y_{pqk}) \le S_q ,\ \ \forall p,
 * q, k \in K_p \cap K_q \f$ immediate pair should wait with transfer cost
 *
 * \f$ 2(Y_{pqk} + Y_{qpk}) \le Z_{pk} + Z_{qk}, \ \ \forall p, q, k \in K_p
 * \cap K_q \f$ immediate pair asymmetric on same line
 *
 * \f$ \displaystyle{\sum_{k=1}^M Z_{pk} = 1, \ \ \forall p} \f$ product only on
 * 1 line
 *
 * \f$ \displaystyle{\sum_{k \notin F_p} Z_{pk} \le 0, \ \ \forall p} \f$
 * product only on capable line
 *
 * Enforce sequential order of production line, i.e., Some product p can be the
 * head of line k.
 * In this case \f$ Y_{qpk} = 0 \ \ \forall q, k \f$, therefore we need \f$
 * DH_{pk} \f$ to
 * specifically amend this. Likewise, if product p is the tail of line k
 * blablabla.
 *
 * \f$ \displaystyle{\sum_{q \ne p} {\sum_{k \in F_p \cap F_q} Y_{qpk}} +
 * \sum_{k \in F_p \cap F_q}^M DH_{pk} = 1},\ \ \forall p \f$
 *
 * \f$ \displaystyle{\sum_{q \ne p} {\sum_{k \in F_p \cap F_q}^{M} Y_{pqk}} +
 * \sum_{k \in F_p \cap F_q}^M DT_{pk} = 1},\ \ \forall p \f$
 *
 * \f$ 0 \le L_i, \ \ C_i - d_i \le L_i, \ \ \forall i \f$ tardy unit time
 * constraints
 *
 * \f$ 0 \le E_i, \ \ d_i - C_i \le L_i, \ \ \forall i \f$ early unit time
 * constraints
 *
 * \f$ \displaystyle{\sum_{p=1}^{N} {DH_{pk}}, \ \ \forall k} \f$ There can only
 * be one head production for line k
 *
 * \f$ \displaystyle{\sum_{p=1}^{N} {DT_{pk}}, \ \ \forall k} \f$ There can only
 * be one tail production for line k
 *
 * \f$ DH_{pk} \le Z_{pk}, \ \ \forall p, k \f$ product not head if not on the
 * line
 *
 * \f$ DT_{pk} \le Z_{pk}, \ \ \forall p, k \f$ product not tail if not on the
 * line
 */

template <typename... IndexTypes>
std::string makeName(std::string prefix, IndexTypes... nums) {
  if (sizeof...(nums))
    return prefix + '_' + utils::numToBracket(nums...) + '_';
  else
    return prefix;
}

using MPConstraint = operations_research::MPConstraint;

Scheduler::DataProvider::DataProvider(std::shared_ptr<const Factory> factory)
    : factory__(factory) {
  const auto& bom = factory__->getBOM();
  const auto& oldOrders = factory__->getOrders();
  const auto& machines = factory__->getMachines();
  orders__.reserve(oldOrders.size());
  for (auto i = 0ul; i < oldOrders.size(); ++i)
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
  const auto& gap = factory__->getBOM().getGap();
  const auto& gapMask = factory__->getBOM().getGapMask();
  std::vector<std::vector<std::pair<Integral, Integral>>> typeInfo(typeSize);
  const auto& orders = orders__;

  // fill in all orders
  for (auto i = 0ul; i < orders.size(); ++i) {
    const auto& prodType = orders[i].getProductType();
    for (auto j = 0ul; j < prodType.size(); ++j) {
      const auto& typeIndex = prodType[j];
      typeInfo[typeIndex].emplace_back(i, j);
    }
  }
  // find gap relation and add to gap__, since symmetric,
  // only half of them permutated
  for (int i = 0; i < typeSize; ++i) {
    for (auto j = i + 1; j < typeSize; ++j) {
      if (gapMask(i, j)) {
        const auto& gapSize = gap(i, j);
        // gapped relation found, add all product of the type
        // into gap__
        for (const auto& info1 : typeInfo[i]) {
          for (const auto& info2 : typeInfo[j]) {
            gap__.emplace_back(
                std::array<Integral, 4>(
                    {info1.first, info1.second, info2.first, info2.second}),
                gapSize);
          }
        }
      }
    }
  }
}

Scheduler::OrderWithDep::OrderWithDep(Order noDepOrder,
                                      const RelationOfProducts& relation,
                                      const std::vector<Machine>& machines)
    : Order(noDepOrder) {
  auto& prodQuan = productQuan_;
  auto& prodType = productType_;
  const auto& bom = relation.getBOM();
  const auto& directMask = relation.getDirectMask();
  // const auto &gap = relation.getGap();
  // const auto &gapMask = relation.getGapMask();
  assert(prodQuan.size() == prodType.size());

  // mark end product
  endProduct__ = std::vector<bool>(prodQuan.size(), true);

  // pair of <typeIndex, num>
  std::queue<Integral> prodQueue;
  for (auto i = 0ul; i < prodQuan.size(); ++i) prodQueue.emplace(i);

  // the algorithm requires that there is no cycle in the bom graph
  // otherwise, this would a dead loop
  while (!prodQueue.empty()) {
    const auto& index = prodQueue.front();

    assert(bom.cols() == directMask.cols());
    // find from bom list the dependent product
    for (long j = 0; j < bom.cols(); ++j) {
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
inline void Scheduler::OrderWithDep::productNum2Time(
    std::vector<std::vector<TimeUnit>>& productionTime,
    const std::vector<Machine>& machines,
    const std::vector<Integral>& productQuan,
    const std::vector<Integral>& productType) {
  assert(productQuan.size() == productType.size());
  const auto typeSize = productQuan.size();
  for (auto j = 0ul; j < typeSize; ++j) {
    const auto& numProduct = productQuan[j];
    const auto& typeIndex = productType[j];

    productionTime.emplace_back(std::vector<TimeUnit>(machines.size()));
    // newly added
    auto& currentProduct = productionTime.back();
    // per machine
    for (auto k = 0ul; k < machines.size(); ++k) {
      const auto& machine = machines[k];
      if (!machine.capable(typeIndex)) {
        currentProduct[k] = infinity;
      } else {
        currentProduct[k] = machine.produceTime(typeIndex, numProduct);
      }
    }
  }
}

inline std::vector<MPConstraint*> Scheduler::addConstraints_1(
    std::vector<MPVariable*> completionTimes, MPVariable const* makeSpan,
    const std::unique_ptr<MPSolver>& solver,
    const std::string& purposeMessage) {
  std::vector<MPConstraint*> constraints;
  for (auto i = 0ul; i < completionTimes.size(); ++i) {
    constraints.emplace_back(
        solver->MakeRowConstraint(-infinity, 0.0, makeName(purposeMessage, i)));
  }

  for (auto i = 0ul; i < constraints.size(); ++i) {
    constraints[i]->SetCoefficient(completionTimes[i], 1.0);
    constraints[i]->SetCoefficient(makeSpan, -1.0);
  }

  LOG(INFO) << purposeMessage << " added number of constraints "
            << constraints.size();
  return constraints;
}

inline std::vector<MPConstraint*> Scheduler::addConstraints_2(
    const std::vector<std::vector<MPVariable*>>& startTime,
    const std::vector<MPVariable*>& completionTimes, const Var3D& onMachine,
    const std::unique_ptr<MPSolver>& solver,
    const std::string& purposeMessage) {
  std::vector<MPConstraint*> constraints;
  const auto& orders = dataProvider__.getOrders();
  const auto& machines = dataProvider__.getMachines();
  // here startTime one to one maps to orders
  // with index, referring to underly products

  assert(orders.size() == startTime.size());
  // per order, per startTime. same thing
  for (auto i = 0ul; i < startTime.size(); ++i) {
    const auto& prodStartTimes = startTime[i];
    const auto& currentOrder = orders[i];
    const auto& productType = currentOrder.getProductType();

    assert(productType.size() == prodStartTimes.size());
    // assume currentOrder.size() == prodStartTime.size()
    // per product of order
    for (auto p = 0ul; p < prodStartTimes.size(); ++p) {
      // if this isn't the final product of the order
      // just skip the constraints
      if (!currentOrder.finalProd(p)) continue;

      const auto& typeIndex = productType[p];
      const auto& prodStart = prodStartTimes[p];
      constraints.emplace_back(solver->MakeRowConstraint(
          -infinity, 0, makeName(purposeMessage, i, p)));

      // per machine that is capable
      for (auto k = 0ul; k < machines.size(); ++k) {
        const auto& machine = machines[k];
        const auto timeOnMach = currentOrder.requiredTime(p, k);

        // add constraint if the machine can produce this type of product
        // doesn't pose constraints if the machine cannot produce the product
        if (machine.capable(typeIndex)) {
          constraints.back()->SetCoefficient(prodStart, 1.0);
          constraints.back()->SetCoefficient(completionTimes[i], -1.0);
          constraints.back()->SetCoefficient(onMachine[i][p][k], timeOnMach);
        }
      }
    }
  }
  LOG(INFO) << purposeMessage << " added number of constraints "
            << constraints.size();
  return constraints;
}

inline std::vector<MPConstraint*> Scheduler::addConstraints_3(
    const std::vector<std::vector<MPVariable*>>& startTime,
    const Var3D& onMachine, const std::unique_ptr<MPSolver>& solver,
    const std::string& purposeMessage) {
  std::vector<MPConstraint*> constraints;
  const auto& orders = dataProvider__.getOrders();
  const auto& machines = dataProvider__.getMachines();
  for (auto i = 0ul; i < startTime.size(); ++i) {
    const auto& currentOrder = orders[i];
    const auto& dep = currentOrder.getDependency();
    const auto& currentStart = startTime[i];

    // for every pair of dependency, add to constraint
    for (const auto& pairIndex : dep) {
      // <p, q> pair, p can start only if q has finished
      const auto p = pairIndex.first;
      const auto q = pairIndex.second;
      constraints.emplace_back(solver->MakeRowConstraint(
          -infinity, 0, makeName(purposeMessage, i, p, q)));

      constraints.back()->SetCoefficient(currentStart[q], 1.0);
      constraints.back()->SetCoefficient(currentStart[p], -1.0);
      for (auto k = 0ul; k < machines.size(); ++k) {
        // product q required time on machine k
        const auto timeOnMach = currentOrder.requiredTime(q, k);
        if (timeOnMach != infinity)
          constraints.back()->SetCoefficient(onMachine[i][q][k], timeOnMach);
      }
    }
  }

  LOG(INFO) << purposeMessage << " added number of constraints "
            << constraints.size();
  return constraints;
}

inline std::vector<MPConstraint*> Scheduler::addConstraints_4(
    const std::vector<std::vector<MPVariable*>>& startTime,
    const Var3D& onMachine, const std::unique_ptr<MPSolver>& solver,
    const std::string& purposeMessage) {
  std::vector<MPConstraint*> constraints;
  const auto& orders = dataProvider__.getOrders();
  const auto& machines = dataProvider__.getMachines();
  const auto& dataGap = dataProvider__.getGap();

  for (const auto& gap : dataGap) {
    for (auto k = 0ul; k < machines.size(); ++k) {
      const auto& gapArray = gap.first;
      const auto& gapTime = gap.second;
      // Integral i, p, j, q = , gapArray[1], gapArray[2], gapArray[3];
      auto i = gapArray[0], p = gapArray[1], j = gapArray[2], q = gapArray[3];
      // product p of order i
      const auto& timeOnMach1 = orders[i].requiredTime(p, k);
      // product q of order j
      const auto& timeOnMach2 = orders[j].requiredTime(q, k);

      // symmetric constraints
      constraints.emplace_back(solver->MakeRowConstraint(
          -infinity, largeNumber - timeOnMach1 - gapTime,
          makeName(purposeMessage, i, p, j, q, k)));
      constraints.back()->SetCoefficient(startTime[i][p], 1.0);
      constraints.back()->SetCoefficient(startTime[j][q], -1.0);
      constraints.back()->SetCoefficient(onMachine[i][p][k], largeNumber);

      constraints.emplace_back(solver->MakeRowConstraint(
          -infinity, largeNumber - timeOnMach2 - gapTime,
          makeName(purposeMessage, j, q, i, p, k)));
      constraints.back()->SetCoefficient(startTime[j][q], 1.0);
      constraints.back()->SetCoefficient(startTime[i][p], -1.0);
      constraints.back()->SetCoefficient(onMachine[j][q][k], largeNumber);
    }
  }
  LOG(INFO) << purposeMessage << " added number of constraints "
            << constraints.size();
  return constraints;
}

inline std::vector<MPConstraint*> Scheduler::addConstraints_5(
    const std::vector<std::vector<MPVariable*>>& startTime,
    const std::unique_ptr<MPSolver>& solver,
    const std::string& purposeMessage) {
  LOG(WARNING) << "material Date to TimeUnit unimplemented";
  std::vector<MPConstraint*> constraints;
  const auto& orders = dataProvider__.getOrders();
  for (auto i = 0ul; i < startTime.size(); ++i) {
    const auto& currentStart = startTime[i];
    const auto& currentOrder = orders[i];
    // TODO: material to TimeUnit
    const auto rawMaterial = currentOrder.getMaterialDate();
    // for every product, it should wait until raw material to arrive
    for (auto j = 0ul; j < currentStart.size(); ++j) {
      constraints.emplace_back(solver->MakeRowConstraint(
          rawMaterial, infinity, makeName(purposeMessage, i, j)));
      constraints.back()->SetCoefficient(currentStart[j], 1.0);
    }
  }
  LOG(INFO) << purposeMessage << " added number of constraints "
            << constraints.size();
  return constraints;
}

inline std::vector<MPConstraint*> Scheduler::addConstraints_6(
    const std::vector<std::vector<MPVariable*>>& startTime,
    const Var3D& onMachine, const std::unique_ptr<MPSolver>& solver,
    const std::string& purposeMessage) {
  std::vector<MPConstraint*> constraints;
  LOG(WARNING) << "can reduce constraint on 6";
  const auto& machines = dataProvider__.getMachines();
  const auto& orders = dataProvider__.getOrders();
  for (auto i = 0ul; i < startTime.size(); ++i) {
    const auto& currentOrder = orders[i];
    const auto& currentStart = startTime[i];
    for (auto p = 0ul; p < currentStart.size(); ++p) {
      const auto typeIndex = currentOrder.getProductType()[p];
      constraints.emplace_back(solver->MakeRowConstraint(
          -infinity, 0, makeName(purposeMessage, i, p)));

      for (auto k = 0ul; k < machines.size(); ++k) {
        const auto& machine = machines[k];
        const auto& ready = machine.getReadyTime();
        if (machine.capable(typeIndex)) {
          constraints.back()->SetCoefficient(currentStart[p], -1.0);
          constraints.back()->SetCoefficient(onMachine[i][p][k], ready);
        }
      }
    }
  }
  LOG(INFO) << purposeMessage << " added number of constraints "
            << constraints.size();
  return constraints;
}

inline std::vector<MPConstraint*> Scheduler::addConstraints_7(
    const std::vector<std::vector<MPVariable*>>& startTime,
    const std::vector<std::vector<Var3D>>& immediatePrec,
    const std::unique_ptr<MPSolver>& solver,
    const std::string& purposeMessage) {
  const auto& transCosts = dataProvider__.getTransCost();
  const auto& orders = dataProvider__.getOrders();
  const auto& machines = dataProvider__.getMachines();
  std::vector<MPConstraint*> constraints;

  assert(orders.size() == startTime.size());
  for (auto i = 0ul; i < startTime.size(); ++i) {
    for (auto j = 0ul; j < startTime.size(); ++j) {
      const auto& order_i = orders[i];
      const auto& order_j = orders[j];
      const auto& typeIndex_i = order_i.getProductType();
      const auto& typeIndex_j = order_j.getProductType();

      for (auto p = 0ul; p < startTime[i].size(); ++p) {
        for (auto q = 0ul; q < startTime[j].size(); ++q) {
          assert(startTime[i].size() == orders[i].size());
          assert(startTime[j].size() == orders[j].size());
          // same product doesn't have restriction
          if (i == j && p == q) continue;

          const auto& type_p = typeIndex_i[p];
          const auto& type_q = typeIndex_j[q];
          // constraint on every machine
          for (auto k = 0ul; k < machines.size(); ++k) {
            const auto& machine = machines[k];
            // product p that(if) precedes product q
            // product q can only be scheduled after p
            // has finished, and wait for transfer time from
            // p -> q(transCost)

            // if machine cannot produce both of it, just skip it
            if (!machine.capable(type_p) || !machine.capable(type_q)) continue;

            const auto& p2qTransCost = transCosts(type_p, type_q);
            // immediate relationship between product p, and q on machine k
            const auto& immediate = immediatePrec[i][p][j][q][k];
            constraints.emplace_back(solver->MakeRowConstraint(
                -infinity, largeNumber - order_i.requiredTime(p, k),
                makeName(purposeMessage, i, p, j, q, k)));
            constraints.back()->SetCoefficient(startTime[i][p], 1.0);
            constraints.back()->SetCoefficient(immediate, p2qTransCost);
            constraints.back()->SetCoefficient(immediate, largeNumber);
            constraints.back()->SetCoefficient(startTime[j][q], -1.0);
          }
        }
      }
    }
  }
  LOG(INFO) << purposeMessage << " added number of constraints "
            << constraints.size();

  return constraints;
}

inline std::vector<MPConstraint*> Scheduler::addConstraints_8(
    const Var3D& onMachine,
    const std::vector<std::vector<Var3D>>& immediatePrec,
    const std::unique_ptr<MPSolver>& solver,
    const std::string& purposeMessage) {
  LOG(WARNING) << "can reduce constraint num on 8";
  std::vector<MPConstraint*> constraints;
  const auto& orders = dataProvider__.getOrders();
  const auto& machines = dataProvider__.getMachines();
  for (auto i = 0ul; i < onMachine.size(); ++i) {
    for (auto j = 0ul; j < onMachine.size(); ++j) {
      const auto& iOnMachine = onMachine[i];
      const auto& jOnMachine = onMachine[j];
      const auto& order_i = orders[i];
      const auto& order_j = orders[j];
      const auto& typeIndex_i = order_i.getProductType();
      const auto& typeIndex_j = order_j.getProductType();

      for (auto p = 0ul; p < onMachine[i].size(); ++p) {
        for (auto q = 0ul; q < onMachine[j].size(); ++q) {
          const auto& pOnMachine = iOnMachine[p];
          // i -> j, bug
          const auto& qOnMachine = jOnMachine[q];
          // same product doesn't have restriction
          if (i == j && p == q) continue;

          const auto& type_p = typeIndex_i[p];
          const auto& type_q = typeIndex_j[q];
          // constraint on every machine
          for (auto k = 0ul; k < machines.size(); ++k) {
            const auto& machine = machines[k];
            // product p that(if) precedes product q
            // product q can only be scheduled after p
            // has finished, and wait for transfer time from
            // p -> q(transCost)

            // on every machine, since there is no onMachine for
            // dummyPrec, dummySucc
            // DLOG(INFO) << machine.capable(type_p) << ' ' <<
            // machine.capable(type_q);
            if (!machine.capable(type_p) || !machine.capable(type_q)) continue;
            // DLOG(INFO) << makeName(purposeMessage, i, p, j, q, k);

            // immediate relationship between product p, and q on machine k
            const auto& immediate_qp = immediatePrec[j][q][i][p][k];
            const auto& immediate_pq = immediatePrec[i][p][j][q][k];
            constraints.emplace_back(solver->MakeRowConstraint(
                -infinity, 0, makeName(purposeMessage, i, p, j, q, k)));

            constraints.back()->SetCoefficient(immediate_qp, 2.0);
            constraints.back()->SetCoefficient(immediate_pq, 2.0);
            constraints.back()->SetCoefficient(pOnMachine[k], -1.0);
            constraints.back()->SetCoefficient(qOnMachine[k], -1.0);
          }
        }
      }
    }
  }
  LOG(INFO) << purposeMessage << " added number of constraints "
            << constraints.size();
  return constraints;
}

inline std::vector<MPConstraint*> Scheduler::addConstraints_9(
    const Var3D& onMachine, const std::unique_ptr<MPSolver>& solver,
    const std::string& purposeMessage) {
  std::vector<MPConstraint*> constraints;
  const auto& machines = dataProvider__.getMachines();
  const auto& orders = dataProvider__.getOrders();

  assert(onMachine.size() == orders.size());
  for (auto i = 0ul; i < onMachine.size(); ++i) {
    for (auto p = 0ul; p < onMachine[i].size(); ++p) {
      assert(onMachine[i].size() == orders[i].size());

      const auto typeIndex = orders[i].getProductType()[p];
      constraints.emplace_back(
          solver->MakeRowConstraint(1, 1, makeName(purposeMessage, i, p)));

      // What would happen if there are no machine capable of manufacturing
      // this type of product

      assert(onMachine[i][p].size() == machines.size());
      for (auto k = 0ul; k < onMachine[i][p].size(); ++k) {
        const auto& machine = machines[k];
        if (machine.capable(typeIndex))
          constraints.back()->SetCoefficient(onMachine[i][p][k], 1);
      }
    }
  }
  LOG(INFO) << purposeMessage << " added number of constraints "
            << constraints.size();

  return constraints;
}

inline std::vector<MPConstraint*> Scheduler::addConstraints_10(
    const Var3D& onMachine, const std::unique_ptr<MPSolver>& solver,
    const std::string& purposeMessage) {
  std::vector<MPConstraint*> constraints;
  const auto& machines = dataProvider__.getMachines();
  const auto& orders = dataProvider__.getOrders();

  assert(orders.size() == onMachine.size());
  for (auto i = 0ul; i < onMachine.size(); ++i) {
    assert(orders[i].size() == onMachine[i].size());
    for (auto p = 0ul; p < onMachine[i].size(); ++p) {
      const auto typeIndex = orders[i].getProductType()[p];
      for (auto k = 0ul; k < machines.size(); ++k) {
        // on machine that is incapable of manufacturing this product
        // add constraint
        // bug fix here, using p instead of typeIndex
        if (!machines[k].capable(typeIndex)) {
          constraints.emplace_back(solver->MakeRowConstraint(
              -infinity, 0, makeName(purposeMessage, i, p, k)));
          constraints.back()->SetCoefficient(onMachine[i][p][k], 1.0);
        }
      }
    }
  }
  LOG(INFO) << purposeMessage << " added number of constraints "
            << constraints.size();
  return constraints;
}

inline std::vector<MPConstraint*> Scheduler::addConstraints_11(
    const std::vector<std::vector<Var3D>>& immediatePrec,
    const Var3D& dummyPrec, const Var3D& dummySucc,
    const std::unique_ptr<MPSolver>& solver,
    const std::string& purposeMessage) {
  const auto& orders = dataProvider__.getOrders();
  const auto& machines = dataProvider__.getMachines();

  // predecessor relation
  std::vector<std::vector<MPConstraint*>> constraintsPrec(orders.size());
  auto countConstraints = 0;
  for (auto i = 0ul; i < orders.size(); ++i) {
    for (auto p = 0ul; p < orders[i].size(); ++p) {
      ++countConstraints;
      constraintsPrec[i].emplace_back(solver->MakeRowConstraint(
          1, 1, makeName(purposeMessage, i, p) + "_Prec"));
    }
  }

  // immediatePrec i, p either is true for j, q that precedes on line k
  // or is true for dummpyPrec on line k, indicating it's the first on line k
  for (auto i = 0ul; i < orders.size(); ++i) {
    for (auto p = 0ul; p < orders[i].size(); ++p) {
      const auto& typeIndex_p = orders[i].getProductType()[p];
      for (auto k = 0ul; k < machines.size(); ++k) {
        if (!machines[k].capable(typeIndex_p)) continue;
        constraintsPrec[i][p]->SetCoefficient(dummyPrec[i][p][k], 1);
      }
    }
  }

  for (auto j = 0ul; j < immediatePrec.size(); ++j) {
    for (auto q = 0ul; q < immediatePrec[j].size(); ++q) {
      for (auto i = 0ul; i < immediatePrec[j][q].size(); ++i) {
        for (auto p = 0ul; p < immediatePrec[j][q][i].size(); ++p) {
          const auto& typeIndex_p = orders[i].getProductType()[p];
          const auto& typeIndex_q = orders[j].getProductType()[q];
          for (auto k = 0ul; k < immediatePrec[j][q][i][p].size(); ++k) {
            if (j == i && q == p) continue;
            // for constraint of product p, accumulate all product q
            // excluding q == p
            // if ((i == 4 || j == 4) && k == 13)
            //   std::cout << i << ' ' << j << ' ' <<  p << ' ' << q << '\n';
            if (!machines[k].capable(typeIndex_p) ||
                !machines[k].capable(typeIndex_q))
              continue;
            // if ((i == 4 || j == 4) && k == 13)
            //   std::cout << i << ' ' << j << ' ' <<  p << ' ' << q << '\n';
            constraintsPrec[i][p]->SetCoefficient(immediatePrec[j][q][i][p][k],
                                                  1);
          }
        }
      }
    }
  }

  // successor relation
  std::vector<std::vector<MPConstraint*>> constraintsSucc(orders.size());
  for (auto i = 0ul; i < orders.size(); ++i) {
    for (auto p = 0ul; p < orders[i].size(); ++p) {
      ++countConstraints;
      constraintsSucc[i].emplace_back(solver->MakeRowConstraint(
          1, 1, makeName(purposeMessage, i, p) + "_Succ"));
    }
  }

  // immediatePrec i, p either is true for j, q that is successor on line k
  // or is true for dummpySucc on line k, indicating it's the first on line k
  for (auto i = 0ul; i < orders.size(); ++i) {
    for (auto p = 0ul; p < orders[i].size(); ++p) {
      const auto& typeIndex_p = orders[i].getProductType()[p];
      for (auto k = 0ul; k < machines.size(); ++k) {
        if (!machines[k].capable(typeIndex_p)) continue;
        constraintsSucc[i][p]->SetCoefficient(dummySucc[i][p][k], 1);
      }
    }
  }

  // TODO: should refactor this part
  for (auto j = 0ul; j < immediatePrec.size(); ++j) {
    for (auto q = 0ul; q < immediatePrec[j].size(); ++q) {
      for (auto i = 0ul; i < immediatePrec[j][q].size(); ++i) {
        for (auto p = 0ul; p < immediatePrec[j][q][i].size(); ++p) {
          for (auto k = 0ul; k < immediatePrec[j][q][i][p].size(); ++k) {
            const auto& typeIndex_p = orders[i].getProductType()[p];
            const auto& typeIndex_q = orders[j].getProductType()[q];
            if (j == i && q == p) continue;
            // for constraint of product p, accumulate all product q
            // excluding q == p
            if (!machines[k].capable(typeIndex_p) ||
                !machines[k].capable(typeIndex_q))
              continue;
            constraintsSucc[i][p]->SetCoefficient(immediatePrec[i][p][j][q][k],
                                                  1);
          }
        }
      }
    }
  }

  // std::vector<MPConstraint *> constraintDummyPrec;
  // // dummyPrec and dummySucc
  // for (auto k = 0ul; k < machines.size(); ++ k) {
  //   constraintDummyPrec.emplace_back(
  //     solver->MakeRowConstraint(1, 1, ))
  //     for (auto i = 0ul; i < dummyPrec.size(); ++ i) {
  //       for (auto p = 0ul; p < dummyPrec[i].size(); ++ p) {

  //       }
  //     }
  // }

  LOG(INFO) << purposeMessage << " added number of constraints "
            << countConstraints;
  LOG(WARNING) << "return null constraint for now.";
  return std::vector<MPConstraint*>();
}

inline std::vector<MPConstraint*> Scheduler::addConstraints_12(
    const std::vector<MPVariable*>& completionTimes,
    const std::vector<MPVariable*>& tardyTime,
    const std::unique_ptr<MPSolver>& solver,
    const std::string& purposeMessage) {
  std::vector<MPConstraint*> constraints;
  const auto& orders = dataProvider__.getOrders();
  assert(completionTimes.size() == tardyTime.size() &&
         tardyTime.size() == orders.size());

  const auto size = completionTimes.size();
  for (auto i = 0ul; i < size; ++i) {
    const auto dueTime = orders[i].getDueTime();
    constraints.emplace_back(solver->MakeRowConstraint(
        -infinity, dueTime, makeName(purposeMessage, i)));

    constraints.back()->SetCoefficient(completionTimes[i], 1.0);
    constraints.back()->SetCoefficient(tardyTime[i], -1.0);
  }

  LOG(INFO) << purposeMessage << " added number of constraints "
            << constraints.size();
  return constraints;
}

inline std::vector<MPConstraint*> Scheduler::addConstraints_13(
    const std::vector<MPVariable*>& completionTimes,
    const std::vector<MPVariable*>& earlyTime,
    const std::unique_ptr<MPSolver>& solver,
    const std::string& purposeMessage) {
  std::vector<MPConstraint*> constraints;
  const auto& orders = dataProvider__.getOrders();
  assert(completionTimes.size() == earlyTime.size() &&
         earlyTime.size() == orders.size());

  const auto size = completionTimes.size();
  for (auto i = 0ul; i < size; ++i) {
    const auto dueTime = orders[i].getDueTime();
    constraints.emplace_back(solver->MakeRowConstraint(
        dueTime, infinity, makeName(purposeMessage, i)));
    constraints.back()->SetCoefficient(completionTimes[i], 1.0);
    constraints.back()->SetCoefficient(earlyTime[i], 1.0);
  }

  LOG(INFO) << purposeMessage << " added number of constraints "
            << constraints.size();
  return constraints;
}

inline std::vector<MPConstraint*> Scheduler::addConstraints_14(
    const std::vector<MPVariable*>& tardyTime,
    const std::unique_ptr<MPSolver>& solver,
    const std::string& purposeMessage) {
  std::vector<MPConstraint*> constraints;
  const auto size = tardyTime.size();
  for (auto i = 0ul; i < size; ++i) {
    constraints.emplace_back(
        solver->MakeRowConstraint(0, infinity, makeName(purposeMessage, i)));
    constraints.back()->SetCoefficient(tardyTime[i], 1.0);
  }

  LOG(INFO) << purposeMessage << " added number of constraints "
            << constraints.size();
  return constraints;
}

inline std::vector<MPConstraint*> Scheduler::addConstraints_15(
    const std::vector<MPVariable*>& earlyTime,
    const std::unique_ptr<MPSolver>& solver,
    const std::string& purposeMessage) {
  std::vector<MPConstraint*> constraints;
  const auto size = earlyTime.size();
  for (auto i = 0ul; i < size; ++i) {
    constraints.emplace_back(
        solver->MakeRowConstraint(0, infinity, makeName(purposeMessage, i)));
    constraints.back()->SetCoefficient(earlyTime[i], 1.0);
  }
  LOG(INFO) << purposeMessage << " added number of constraints "
            << constraints.size();
  return constraints;
}

inline std::vector<MPConstraint*> Scheduler::addConstraints_16(
    const Var3D& dummyPrec, const std::unique_ptr<MPSolver>& solver,
    const std::string& purposeMessage) {
  std::vector<MPConstraint*> constraints;
  const auto sizeOrder = dummyPrec.size();
  const auto machineSize = dataProvider__.getMachines().size();

  for (auto k = 0ul; k < machineSize; ++k) {
    // on every production line, there is only one position for
    // the first product dummyPrec[i][p][k] means that
    // product p will be the first to produce on line k
    constraints.emplace_back(
        solver->MakeRowConstraint(-infinity, 1, makeName(purposeMessage, k)));
    for (auto i = 0ul; i < sizeOrder; ++i) {
      const auto sizeProduct = dummyPrec[i].size();
      for (auto p = 0ul; p < sizeProduct; ++p) {
        assert(dummyPrec[i][p].size() == machineSize);
        // dummyPrec[i][p][k]
        constraints.back()->SetCoefficient(dummyPrec[i][p][k], 1);
      }
    }
  }
  LOG(INFO) << purposeMessage << " added number of constraints "
            << constraints.size();
  return constraints;
}

inline std::vector<MPConstraint*> Scheduler::addConstraints_17(
    const Var3D& dummySucc, const std::unique_ptr<MPSolver>& solver,
    const std::string& purposeMessage) {
  std::vector<MPConstraint*> constraints;
  const auto sizeOrder = dummySucc.size();
  const auto machineSize = dataProvider__.getMachines().size();

  for (auto k = 0ul; k < machineSize; ++k) {
    // on every production line, there is only one position for
    // the first product dummyPrec[i][p][k] means that
    // product p will be the first to produce on line k
    constraints.emplace_back(
        solver->MakeRowConstraint(0, 1, makeName(purposeMessage, k)));
    for (auto i = 0ul; i < sizeOrder; ++i) {
      const auto sizeProduct = dummySucc[i].size();
      for (auto p = 0ul; p < sizeProduct; ++p) {
        assert(dummySucc[i][p].size() == machineSize);
        // dummyPrec[i][p][k]
        constraints.back()->SetCoefficient(dummySucc[i][p][k], 1);
      }
    }
  }
  LOG(INFO) << purposeMessage << " added number of constraints "
            << constraints.size();
  return constraints;
}

inline std::vector<MPConstraint*> Scheduler::addConstraints_18(
    const Var3D& onMachine, const Var3D& dummyPrec,
    const std::unique_ptr<MPSolver>& solver,
    const std::string& purposeMessage) {
  std::vector<MPConstraint*> constraints;
  const auto& orders = dataProvider__.getOrders();
  const auto sizeOrder = dummyPrec.size();
  const auto& machines = dataProvider__.getMachines();
  const auto machineSize = machines.size();

  for (auto k = 0ul; k < machineSize; ++k) {
    // product can only be head if it's on the line
    for (auto i = 0ul; i < sizeOrder; ++i) {
      const auto sizeProduct = dummyPrec[i].size();
      for (auto p = 0ul; p < sizeProduct; ++p) {
        const auto& typeIndex_p = orders[i].getProductType()[p];
        if (!machines[k].capable(typeIndex_p)) continue;
        constraints.emplace_back(solver->MakeRowConstraint(
            -infinity, 0, makeName(purposeMessage, i, p, k)));
        assert(dummyPrec[i][p].size() == machineSize);
        constraints.back()->SetCoefficient(dummyPrec[i][p][k], 1);
        constraints.back()->SetCoefficient(onMachine[i][p][k], -1);
      }
    }
  }

  LOG(INFO) << purposeMessage << " added number of constraints "
            << constraints.size();
  return constraints;
}

inline std::vector<MPConstraint*> Scheduler::addConstraints_19(
    const Var3D& onMachine, const Var3D& dummySucc,
    const std::unique_ptr<MPSolver>& solver,
    const std::string& purposeMessage) {
  std::vector<MPConstraint*> constraints;
  const auto& orders = dataProvider__.getOrders();
  const auto sizeOrder = dummySucc.size();
  const auto& machines = dataProvider__.getMachines();
  const auto machineSize = dataProvider__.getMachines().size();

  for (auto k = 0ul; k < machineSize; ++k) {
    // product can only be tail if it's on the line
    for (auto i = 0ul; i < sizeOrder; ++i) {
      const auto sizeProduct = dummySucc[i].size();
      for (auto p = 0ul; p < sizeProduct; ++p) {
        const auto& typeIndex_p = orders[i].getProductType()[p];
        if (!machines[k].capable(typeIndex_p)) continue;
        constraints.emplace_back(solver->MakeRowConstraint(
            -infinity, 0, makeName(purposeMessage, i, p, k)));
        assert(dummySucc[i][p].size() == machineSize);
        // dummyPrec[i][p][k]
        constraints.back()->SetCoefficient(dummySucc[i][p][k], 1);
        constraints.back()->SetCoefficient(onMachine[i][p][k], -1);
      }
    }
  }
  LOG(INFO) << purposeMessage << " added number of constraints "
            << constraints.size();
  return constraints;
}

void Scheduler::factoryScheduler(
    std::shared_ptr<const Factory> factory,
    MPSolver::OptimizationProblemType optimization_problem_type, double lambda,
    double timeLimit, std::ofstream& outputStream) {
  dataProvider__ = DataProvider(factory);
  factory__ = factory;
  using namespace operations_research;
  LOG(INFO) << "Building planner";
  // MPSolver solver("FactorySolver", optimization_problem_type);
  // MPSolver solver("FactorySolver", optimization_problem_type);
  solver =
      utils::make_unique<MPSolver>("FactorySolver", optimization_problem_type);

  // prepare parameters
  // need to be careful about the dummy variables
  std::vector<MPVariable*> tardyTime;
  makeSpan = solver->MakeNumVar(0.0, infinity, "MakeSpan");
  std::vector<MPVariable*> completionTimes;

  const auto& orders = dataProvider__.getOrders();
  const auto orderSize = orders.size();
  const auto& machines = dataProvider__.getMachines();
  const auto machineSize = machines.size();

  solver->MakeNumVarArray(orderSize, 0.0, infinity, "tardyTime", &tardyTime);
  solver->MakeNumVarArray(orderSize, 0.0, infinity, "earlyTime", &earlyTime);
  for (auto i = 0ul; i < orderSize; ++i)
    onMachine.emplace_back(orders[i].size(), std::vector<MPVariable*>(0));

  /**
   * OR TOOLS implementation detail reveals that it's pushing back into vector
   * without before hand clearing the vector, probably because
   * a single variable can contain multiple MakeNumVarArray's initialization
   */
  for (auto i = 0ul; i < orderSize; ++i)
    for (auto p = 0ul; p < orders[i].size(); ++p)
      solver->MakeBoolVarArray(machineSize, makeName("onMachine", i, p),
                               &onMachine[i][p]);

  // need refactor
  immediatePrec.resize(orderSize);
  for (auto i = 0ul; i < orderSize; ++i) {
    immediatePrec[i].resize(orders[i].size());
    for (auto p = 0ul; p < orders[i].size(); ++p) {
      for (auto j = 0ul; j < orderSize; ++j) {
        immediatePrec[i][p].emplace_back(orders[j].size(),
                                         std::vector<MPVariable*>(0));
      }
    }
  }

  for (auto i = 0ul; i < orderSize; ++i) {
    for (auto p = 0ul; p < orders[i].size(); ++p) {
      for (auto j = 0ul; j < orderSize; ++j) {
        for (auto q = 0ul; q < orders[j].size(); ++q) {
          solver->MakeBoolVarArray(machineSize,
                                   makeName("immediatePrec", i, p, j, q),
                                   &immediatePrec[i][p][j][q]);
        }
      }
    }
  }

  for (auto i = 0ul; i < orderSize; ++i)
    dummyPrec.emplace_back(orders[i].size(), std::vector<MPVariable*>(0));

  for (auto i = 0ul; i < orderSize; ++i)
    for (auto p = 0ul; p < orders[i].size(); ++p)
      solver->MakeBoolVarArray(machineSize, makeName("dummyPrec", i, p),
                               &dummyPrec[i][p]);

  for (auto i = 0ul; i < orderSize; ++i)
    dummySucc.emplace_back(orders[i].size(), std::vector<MPVariable*>(0));

  for (auto i = 0ul; i < orderSize; ++i)
    for (auto p = 0ul; p < orders[i].size(); ++p)
      solver->MakeBoolVarArray(machineSize, makeName("dummySucc", i, p),
                               &dummySucc[i][p]);

  startTime.resize(orderSize);
  for (auto i = 0ul; i < orderSize; ++i)
    solver->MakeNumVarArray(orders[i].size(), 0.0, infinity,
                            makeName("startTime", i), &startTime[i]);

  solver->MakeNumVarArray(orderSize, 0.0, infinity, makeName("completionTime"),
                          &completionTimes);

  // adding constraints
  addConstraints_1(completionTimes, makeSpan, solver,
                   "CompletionTime shoule precede MakeSpan");
  addConstraints_2(startTime, completionTimes, onMachine, solver,
                   "CompletionTime later than all finished time of product");
  addConstraints_3(startTime, onMachine, solver,
                   "dependent relation between products of same order");
  addConstraints_4(startTime, onMachine, solver, "gap between products");
  addConstraints_5(startTime, solver, "manufacturing wait raw material");
  addConstraints_6(startTime, onMachine, solver,
                   "manufacturing when machine is ready");
  addConstraints_7(startTime, immediatePrec, solver,
                   "immediate pair should wait with transfer cost");
  addConstraints_8(onMachine, immediatePrec, solver,
                   "immediate pair asymmetric on same line");
  addConstraints_9(onMachine, solver, "product only on 1 line");
  addConstraints_10(onMachine, solver, "product only on capable line");

  // unnecessary
  // addConstraints_11(makeSpan, solver,
  //   "makeSpan is positive");
  // startTime not negative

  // enforce immediate to be sequential
  addConstraints_11(immediatePrec, dummyPrec, dummySucc, solver,
                    "sequentialize immediate relation");

  // rules of variables in objective function
  addConstraints_12(completionTimes, tardyTime, solver, "tardy time");
  addConstraints_13(completionTimes, earlyTime, solver, "early time");

  // unnecessary given, the variable is bounded at initialization
  // see before
  // addConstraints_14(tardyTime, solver,
  //   "tardy time non-negative");
  // addConstraints_15(earlyTime, solver,
  //   "early time non-negative");

  // rules on dummy variables
  addConstraints_16(dummyPrec, solver,
                    "dummyPrec of each line only one successor");

  addConstraints_17(dummySucc, solver,
                    "dummySucc of each line only one predecessor");

  addConstraints_18(onMachine, dummyPrec, solver,
                    "product not head if not on the line");

  addConstraints_19(onMachine, dummySucc, solver,
                    "product not tail if not on the line");

  // Objective
  MPObjective* const objective = solver->MutableObjective();
  // not implementable given or tool's inability
  //  objective->SetCoefficient(MakeSpan, machineSize);
  //   for (auto i = 0ul; i < orderSize; ++ i) {
  //     for (auto p = 0ul; i < orders[i].size(); ++ i) {
  //       const auto typeIndex = orders.getProductType()[p];
  //       for (auto k = 0ul; k < machines.size(); ++ k) {
  //         const auto &machine = machines[k];
  //         if (machine.capable(typeIndex)) {
  //           objective->SetCoefficient()
  //         }
  //       }
  //     }
  //   }
  // }

  // for (const auto & var: solver->variables()) {
  //   DLOG(INFO) << var->name() << '\n';
  // }

  // for (const auto &constraint : solver->constraints()) {
  //   DLOG(INFO) << constraint->name() << '\n';
  // }

  const auto tardyCost = factory__->getTardyCost();
  const auto earlyCost = factory__->getEarlyCost();
  const auto idleCost = factory__->getIdleCost();
  for (std::size_t i = 0; i < orderSize; ++i)
    objective->SetCoefficient(tardyTime[i], tardyCost);

  for (std::size_t i = 0; i < orderSize; ++i)
    objective->SetCoefficient(earlyTime[i], earlyCost);

  // idle time
  objective->SetCoefficient(makeSpan, idleCost * machines.size());
  // minus ready time
  Float readyTotal = 0.0;
  for (auto k = 0ul; k < machines.size(); ++k) {
    readyTotal += machines[k].getReadyTime();
  }
  objective->SetOffset(readyTotal);

  for (std::size_t i = 0; i < orderSize; ++i) {
    for (auto p = 0ul; p < orders[i].size(); ++p) {
      // const auto typeIndex = orders[i].getProductType()[p];
      for (auto k = 0ul; k < machines.size(); ++k) {
        const auto time = orders[i].requiredTime(p, k);
        // product manufacturing time
        if (time != infinity) {
          objective->SetCoefficient(onMachine[i][p][k], -time * idleCost);
        }
        // l1 normalization
        objective->SetCoefficient(onMachine[i][p][k], lambda);
      }
    }
  }

  for (std::size_t i = 0; i < orderSize; ++i) {
    for (auto p = 0ul; p < orders[i].size(); ++p) {
      for (auto j = 0ul; j < orderSize; ++j) {
        for (auto q = 0ul; q < orders[j].size(); ++q) {
          for (auto k = 0ul; k < machines.size(); ++k) {
            // l1 normalization
            objective->SetCoefficient(immediatePrec[i][p][j][q][k], lambda);
          }
        }
      }
    }
  }

  // auto model =
  // static_cast<OsiClpSolverInterface>(solver->underlying_solver());
  // if (!model.haveMultiThreadSupport()) LOG(FATAL) << "end";
  // model.setThreadMode(2);
  // model.setNumberThreads(12);

  objective->SetMinimization();
  solver->EnableOutput();
  LOG(INFO) << "took " << static_cast<double>(solver->wall_time()) / 1000
            << " seconds to build solver";

  LOG(INFO) << "Number of constraints " << solver->NumConstraints();
  LOG(INFO) << "Number of variables " << solver->NumVariables();
  LOG(INFO) << "Using solver: " << solver->SolverVersion();
  solver->set_time_limit(timeLimit);
  // std::cout << solver->time_limit() << '\n';

  //    std::cout << solver->ComputeExactConditionNumber() << '\n';
  //    solver->ClampSolutionWithinBounds();
  std::string LPOutput;
  // solver->ExportModelAsMpsFormat(true, false, &MPSOutput);
  solver->ExportModelAsLpFormat(true, &LPOutput);
  std::ofstream MPSOutStream("problem.lp");
  MPSOutStream << LPOutput << std::endl;
  // solver->Solve();
  // if (solver->VerifySolution(1e-8, true)) {
  //   LOG(INFO) << "No solution found";
  //   return;
  // }

  // collectInfoAndOutput(this, outputStream, lambda);

  // std::cout << objective->Value() << '\n';
}

/**
 * A friend function of scheduler
 * Scheduler* is a scheduler after calling solve
 * OutputStream is to be streamed output info
 */
template <typename OutputStream>
void collectInfoAndOutput(FactoryWorld::Scheduler* scheduler,
                          OutputStream& outputStream, double lambda) {
  double l1_sum = 0.0;
  const int tickSize = 10;
  const auto& orders = scheduler->dataProvider__.getOrders();
  const auto orderSize = orders.size();
  const auto& machines = scheduler->dataProvider__.getMachines();
  const auto machineSize = machines.size();
  const auto& onMachine = scheduler->onMachine;
  const auto& immediatePrec = scheduler->immediatePrec;

  // collecting l1 factor
  for (std::size_t i = 0; i < orderSize; ++i) {
    for (auto p = 0ul; p < orders[i].size(); ++p) {
      // const auto typeIndex = orders[i].getProductType()[p];
      for (auto k = 0ul; k < machines.size(); ++k) {
        // l1 normalization
        l1_sum += onMachine[i][p][k]->solution_value() * lambda;
      }
    }
  }

  for (std::size_t i = 0; i < orderSize; ++i) {
    for (auto p = 0ul; p < orders[i].size(); ++p) {
      for (auto j = 0ul; j < orderSize; ++j) {
        for (auto q = 0ul; q < orders[j].size(); ++q) {
          for (auto k = 0ul; k < machines.size(); ++k) {
            // l1 normalization
            l1_sum += immediatePrec[i][p][j][q][k]->solution_value() * lambda;
          }
        }
      }
    }
  }

  std::cout << l1_sum << '\n';
  std::cout << scheduler->makeSpan->name() << ' '
            << scheduler->makeSpan->solution_value() << '\n';

  nlohmann::json outputJson;
  const auto& startTime = scheduler->startTime;
  // outputString["packages"] = {"line"}
  for (auto i = 0ul; i < startTime.size(); ++i) {
    for (auto p = 0ul; p < startTime[i].size(); ++p) {
      for (auto k = 0ul; k < onMachine[i][p].size(); ++k) {
        // item i, product p, is scheduled on line k
        if (onMachine[i][p][k]->solution_value()) {
          outputJson["packages"].emplace_back(
              nlohmann::json({{"line", std::to_string(k)},
                              {"item", std::to_string(i)},
                              {"package", std::to_string(p)},
                              {"start", startTime[i][p]->solution_value()},
                              {"end", startTime[i][p]->solution_value() +
                                          orders[i].requiredTime(p, k)},
                              {"ddl", orders[i].getDueTime()},
                              {"color", "black"}}));
        }
      }
    }
  }
  // TODO: maybe adding a version here
  outputJson["title"] = "Scheduler Output";
  for (std::size_t i = 0; i < machineSize; ++i) {
    outputJson["line_names"].emplace_back(std::to_string(i));
  }
  outputJson["xlabel"] = "time (hours)";
  // makeSpan divided by tickSize, default 10% each
  for (int i = 0; i < tickSize + 1; ++i)
    outputJson["xticks"].emplace_back(scheduler->makeSpan->solution_value() /
                                      static_cast<double>(tickSize) *
                                      static_cast<double>(i));

  outputStream << outputJson << '\n';
}

}  // namespace FactoryWorld

// debug segment
// for (auto i = 0ul; i < startTime.size(); ++i) {
//   for (auto p = 0ul; p < startTime[i].size(); ++p) {
//     for (auto k = 0ul; k < onMachine[i][p].size(); ++k) {
//       if (onMachine[i][p][k]->solution_value()) {
//         std::cout << i << ' ' << p << ' ' << k << ' '
//                   << startTime[i][p]->solution_value() << ' '
//                   << startTime[i][p]->solution_value() +
//                          orders[i].requiredTime(p, k);
//       }
//     }
//     std::cout << '\n';
//   }
// }

// for (auto i = 0ul; i < startTime.size(); ++ i) {
//   for (auto p = 0ul; p < startTime[i].size(); ++ p) {
//     for (auto k = 0ul; k < onMachine[i][p].size(); ++ k) {
//       if (onMachine[i][p][k]->solution_value()) {
//         std::cout << makeName("product", i, p) << ' '
//                   << startTime[i][p]->solution_value() << ' '
//                   << " on line " << k << " requires "
//                   << orders[i].requiredTime(p, k) << k;
//       }
//     }
//     std::cout << '\n';
//   }
// }

// for (auto j = 0ul; j < immediatePrec.size(); ++j) {
//   for (auto q = 0ul; q < immediatePrec[j].size(); ++q) {
//     for (auto i = 0ul; i < immediatePrec[j][q].size(); ++i) {
//       for (auto p = 0ul; p < immediatePrec[j][q][i].size(); ++p) {
//         const auto& typeIndex_p = orders[i].getProductType()[p];
//         const auto& typeIndex_q = orders[j].getProductType()[q];
//         for (auto k = 0ul; k < immediatePrec[j][q][i][p].size(); ++k) {
//           if (j == i && q == p) continue;
//           if (!machines[k].capable(typeIndex_p) ||
//               !machines[k].capable(typeIndex_q))
//             continue;
//           if (i == 4 && p == 0)
//             std::cout << i << ' ' << j << ' ' <<  q << ' ' << p << ' ' << k
//             << ' '
//                       << immediatePrec[j][q][i][p][k]->solution_value() <<
//                       '\n';
//         }
//       }
//     }
//   }
// }

// for (auto i = 0ul; i < orders.size(); ++i) {
//   for (auto p = 0ul; p < orders[i].size(); ++p) {
//     const auto& typeIndex_p = orders[i].getProductType()[p];
//     for (auto k = 0ul; k < machines.size(); ++k) {
//       if (!machines[k].capable(typeIndex_p)) continue;
//       if (i == 4 && p == 0)
//         std::cout << i << ' ' << p << ' ' << k << ' '
//                   << dummyPrec[i][p][k]->solution_value() << ' '
//                   << onMachine[i][p][k]->solution_value() << '\n';
//     }
//   }
// }

// for (auto i = 0ul; i < immediatePrec.size(); ++ i) {
//   for (auto p = 0ul; p < immediatePrec[i].size(); ++ p) {
//     for (auto j = 0ul; j < immediatePrec[i][p].size(); ++ j) {
//       for (auto q = 0ul; q < immediatePrec[i][p][j].size(); ++ q) {
//         for (auto k = 0ul; k < immediatePrec[i][p][j][q].size(); ++ k) {
//           if (i == j && p == q) continue;
//           if (immediatePrec[i][p][j][q][k]->solution_value() &&
//               (onMachine[i][p][k]->solution_value() &&
//               onMachine[j][q][k]->solution_value())) {
//             std::cout << makeName("product", i, p)
//                       << " precedes "
//                       << makeName("product", j, q)
//                       << " on "
//                       << makeName("machine", k)
//                       << '\n';
//           }
//         }
//       }
//     }
//   }
// }

// for (auto i = 0ul; i < dummyPrec.size(); ++ i) {
//   for (auto p = 0ul; p < dummyPrec[i].size(); ++ p) {
//     for (auto k = 0ul; k < dummyPrec[i][p].size(); ++ k) {
//       if (dummyPrec[i][p][k]->solution_value() &&
//           onMachine[i][p][k]->solution_value()) {
//         std::cout << makeName("product", i, p)
//                   << " is the first on line "
//                   << k << '\n';
//       }
//     }
//   }
// }

// for (auto i = 0ul; i < dummySucc.size(); ++ i) {
//   for (auto p = 0ul; p < dummySucc[i].size(); ++ p) {
//     for (auto k = 0ul; k < dummySucc[i][p].size(); ++ k) {
//       if (dummySucc[i][p][k]->solution_value() &&
//           onMachine[i][p][k]->solution_value()) {
//         std::cout << makeName("product", i, p)
//                   << " is the last on line "
//                   << k << '\n';
//       }
//     }
//   }
// }

// for (auto i = 0ul; i < machines.size(); ++ i) {
//   if (machines[i].capable(46)) {
//     std::cout << i << ' ';
//   }
// }
