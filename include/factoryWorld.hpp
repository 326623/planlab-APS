/* Copyright (C) 2018 New Joy - All Rights Reserved
 * You may use, distribute and modify this code under the
 * terms of the GPLv3
 *
 *
 * You should have received a copy of the GPLv3 license with
 * this file. If not, please visit https://www.gnu.org/licenses/gpl-3.0.en.html
 *
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: yangqp5@outlook.com (New Joy)
 *
 */
#ifndef _NEWJOY_FACTORYWORLD_HPP_
#define _NEWJOY_FACTORYWORLD_HPP_

// ortools dependencies
#include <ortools/base/filelineiter.h>
#include <ortools/base/logging.h>
#include <ortools/base/split.h>
#include <ortools/constraint_solver/constraint_solver.h>
#include <ortools/linear_solver/linear_solver.h>

// small help from eigen to get inverse of BOM
#include <Eigen/Dense>

// std library io stream
#include <fstream>
#include <sstream>

#include "utils.hpp"
/**
 * \brief This namespace provides a collection of classes and methods to read
 * raw data, process, and solve.
 *
 * TODO
 */
namespace FactoryWorld {
// should guarantee that these are streamable
using Integral = int;
using Float = double;
using IndexType = unsigned int;
using TimeUnit = double;
using MatrixTime = Eigen::MatrixXd;

class Machine {
 private:
  // which product the machine can process
  std::vector<bool> capableProduct_;
  // how many product per unit time(assume hours)
  std::vector<Float> capability_;
  // machine not functional before readyTime_
  // (work already scheduled on the machine)
  Float readyTime_;

 public:
  explicit Machine() {}
  explicit Machine(std::vector<Float> capability, Float readyTime)
      : capableProduct_(capability.size()),
        capability_(capability),
        readyTime_(readyTime) {
    CHECK_GE(readyTime_, 0.0) << "negative readyTime";
    // all capability should be above or equal to zero
    for (const auto& cap : capability_) CHECK_GE(cap, 0.0);
    // NOTE: don't know if this floating point comparison
    // would be an issue
    std::transform(capability_.cbegin(), capability_.cend(),
                   capableProduct_.begin(), [](Float x) { return x > 0.0; });
  }

  bool capable(Integral typeIndex) const { return capableProduct_[typeIndex]; }

  Float produceTime(Integral typeIndex, Integral numProduct) const {
    return static_cast<double>(numProduct) / capability_[typeIndex];
  }

  const std::vector<bool>& getCapableProduct() const { return capableProduct_; }

  const std::vector<Float>& getCapability() const { return capability_; }

  const Float& getReadyTime() const { return readyTime_; }
};

class RelationOfProducts {
  using MatrixXd = Eigen::MatrixXd;
  using MatrixB =
      Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

 private:
  // bill of material matrix
  MatrixXd bom_;
  // The product required directly or indirectly,
  // computed directly from bom_
  MatrixXd predecessor_;
  // a bool matrix to check if a product is directly dependent on another
  // product
  MatrixB directMask_;
  MatrixB inAndDirectMask_;

  // a constraint relation between 2 products
  // of such product would require some time intervel
  // in between i.e. gap
  MatrixTime gapProduct_;
  MatrixB gapProductMask_;

  // the time cost on product transcation on the pipeline
  MatrixTime productTransCost_;
  Integral typeSize__;

 public:
  explicit RelationOfProducts() {}
  explicit RelationOfProducts(MatrixXd bom, MatrixTime gap)
      : bom_(bom),
        predecessor_(
            (MatrixXd::Identity(bom.rows(), bom.cols()) - bom_).inverse()),
        gapProduct_(gap),
        productTransCost_(bom.rows(), bom.cols()),
        typeSize__(bom.rows()) {
    CHECK_EQ(bom.rows(), bom.cols()) << "bom matrix not square.";
    // NOTE: might need to consider predecessor.
    // Its floating point is inaccurate
    CHECK((bom_.array() >= 0.0).all()) << "Bom list error! No negative pls.";
    CHECK((gapProduct_.array() >= 0.0).all())
        << "Gap list error! No negative pls.";
    // for (auto i = 0ul; i < predecessor_.cols(); ++ i)
    //   for (auto j = 0ul; j < predecessor_.rows(); ++ j)
    //     if (utils::almost_equal(predecessor_(j, i), 0.0))
    //       predecessor_(i, j) = 0.0;
    assert((gapProduct_.array() >= 0.0).all() && "possibly floating error.");
    assert((predecessor_.array() >= 0.0).all() && "possibly floating error.");
    directMask_ = bom_.array() > 0.0;
    inAndDirectMask_ = predecessor_.array() > 0.0;
    gapProductMask_ = gapProduct_.array() > 0.0;

    // TODO: not implemented features
    productTransCost_.setConstant(0.0);
    LOG(WARNING) << "transcation costs are not loaded into the system yet, "
                 << "should be implemented in the next iteration, "
                 << "Now all costs are zero";
  }

  const MatrixXd& getBOM() const { return bom_; }

  const MatrixXd& getPredecessor() const { return predecessor_; }

  const MatrixB& getDirectMask() const { return directMask_; }

  const MatrixB& getInAndDirectMask() const { return inAndDirectMask_; }

  const MatrixTime& getGap() const { return gapProduct_; }

  const MatrixB& getGapMask() const { return gapProductMask_; }

  const MatrixTime& getProductTransCost() const { return productTransCost_; }

  Integral getTypeSize() const { return typeSize__; }
};

class Order {
 protected:
  std::vector<Integral> productQuan_;
  std::vector<Integral> productType_;
  TimeUnit dueTime_;
  Integral clientID_;
  Integral materialDate_;  // raw material time
 public:
  explicit Order() {}
  explicit Order(std::vector<Integral> productQuan,
                 std::vector<Integral> productType, Float dueTime,
                 Integral clientID, Integral materialDate)
      : productQuan_(productQuan),
        productType_(productType),
        dueTime_(dueTime),
        clientID_(clientID),
        materialDate_(materialDate) {
    CHECK_EQ(productQuan.size(), productType.size());
  }

  // explicit Order(std::ifstream &);

  const std::vector<Integral>& getProductQuan() const { return productQuan_; }

  const std::vector<Integral>& getProductType() const { return productType_; }

  TimeUnit getDueTime() const { return dueTime_; }

  Integral getClientID() const { return clientID_; }

  Integral getMaterialDate() const { return materialDate_; }

  std::size_t size() const { return productType_.size(); }
};

class Factory {
 private:
  constexpr static double infinity = std::numeric_limits<double>::infinity();
  std::vector<Machine> machines__;
  RelationOfProducts bom__;
  std::vector<Order> orders__;

  Float tardyCost_;
  Float earlyCost_;
  Float idleCost_;

 public:
  explicit Factory() {}

  // Load data from file, given the path to file
  void load(const std::string& filename);

  const RelationOfProducts& getBOM() const { return bom__; }

  const std::vector<Order>& getOrders() const { return orders__; }

  const std::vector<Machine>& getMachines() const { return machines__; }

  Float getTardyCost() const { return tardyCost_; }

  Float getEarlyCost() const { return earlyCost_; }

  Float getIdleCost() const { return idleCost_; }
};

// for debugging
std::ostream& operator<<(std::ostream& out, const Order& order);

/**
 * \brief Scheduler takes a Factory <std::shared_ptr> and generate plans based
 * on a
 * linear constraint model.
 */
class Scheduler {
 private:
  // using namespace operations_research;
  using MatrixB = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>;
  using MPSolver = operations_research::MPSolver;
  using MPConstraint = operations_research::MPConstraint;
  using MPVariable = operations_research::MPVariable;
  using Var3D = std::vector<std::vector<std::vector<MPVariable*>>>;
  constexpr static double infinity = std::numeric_limits<double>::infinity();
  // due to limitation of or tool, have to limit largeNumber range
  // critical parameter to tweak, 1.0e20 can't yield good solution
  constexpr static double largeNumber = 1.0e2;
  // std::numeric_limits<double>::max();

  /**
   * OrderWithDep will extend the the original product list
   * with dependent product list
   *
   * And will add additional data structure to add relation between
   * the order's products (precedence and gap)
   */
  class OrderWithDep : public Order {
   private:
    // end product needed by the client
    std::vector<bool> endProduct__;
    // dependency between two products <p, q>, q must be manufactured
    // before p
    std::vector<std::pair<Integral, Integral>> dependency__;
    // std::vector<std::tuple<
    //               Integral, Integral, TimeUnit>> gap__;
    std::vector<std::vector<TimeUnit>> productionTime__;

    inline void productNum2Time(
        std::vector<std::vector<TimeUnit>>& productionTime,
        const std::vector<Machine>& machines,
        const std::vector<Integral>& productQuan,
        const std::vector<Integral>& productType);

   public:
    // explicit OrderWithDep() = delete;
    explicit OrderWithDep() {}
    explicit OrderWithDep(Order noDepOrder, const RelationOfProducts& relation,
                          const std::vector<Machine>& machines);
    bool finalProd(Integral index) const { return endProduct__[index]; }

    TimeUnit requiredTime(Integral index, Integral machineIndex) const {
      return productionTime__[index][machineIndex];
    }

    const std::vector<std::pair<Integral, Integral>>& getDependency() const {
      return dependency__;
    }

    // const std::vector<std::tuple<Integral, Integral, TimeUnit>> &
    // getGapOfProd() const { return gap__; }
  };

  /**
   * This class handles the raw data and transform into data type
   * that can be passed into the scheduler easily
   */
  class DataProvider {
   private:
    std::shared_ptr<const Factory> factory__;
    std::vector<OrderWithDep> orders__;

    // a array of size 4, first pair(2 numbers) is a pair (i, j) of
    // order i product j.
    // second pair(2 numbers) is another pair (i, j)
    // (same name, different value)
    //
    // the last number is how long they should stay apart in production
    // to fullfill the constraint that two types of products cannot be
    // manufactured in the same time period

    // NOTE: this field would not be checked if the there were any duplicate
    // pair (symmetric pair) for instance <p, q> <q, p> should appear together
    std::vector<std::pair<std::array<Integral, 4>, TimeUnit>> gap__;

    /**
     * Assume that all the orders are already set up
     */
    inline void buildGap();

   public:
    // DataProvider() = delete;
    explicit DataProvider() = default;  //: factory__(nullptr) {}
    explicit DataProvider(std::shared_ptr<const Factory> factory);

    DataProvider(const DataProvider&) = default;
    DataProvider& operator=(const DataProvider&) = default;

    DataProvider(DataProvider&&) = default;
    DataProvider& operator=(DataProvider&&) = default;

    //   : factory__(factory)
    // {
    //   const auto &bom = factory__->getBOM();
    //   const auto &oldOrders = factory__->getOrders();
    //   const auto &machines = factory__->getMachines();
    //   orders__.reserve(oldOrders.size());
    //   for (auto i = 0ul; i < oldOrders.size(); ++ i)
    //     orders__.emplace_back(oldOrders[i], bom, machines);

    //   // build the gap that will be required by the scheduler
    //   buildGap();
    // }

    const std::vector<OrderWithDep>& getOrders() const { return orders__; }

    const std::vector<Machine>& getMachines() const {
      return factory__->getMachines();
    }

    const MatrixTime& getTransCost() const {
      return factory__->getBOM().getProductTransCost();
    }

    const std::vector<std::pair<std::array<Integral, 4>, TimeUnit>>& getGap() {
      return gap__;
    }
  };

  /*
   * data field used to deal with relationship between products
   */
  // this variable is computed using orders and machines, and would be extended
  // to include the dependent products
  DataProvider dataProvider__;
  std::shared_ptr<const Factory> factory__;

  // internal state variables of the scheduler
  std::unique_ptr<MPSolver> solver;

  // planning variables
  // need to be careful about the dummy variables
  std::vector<MPVariable*> tardyTime;
  std::vector<MPVariable*> earlyTime;
  Var3D onMachine;                                // bool var
  std::vector<std::vector<Var3D>> immediatePrec;  // bool var
  // size == num of machine
  Var3D dummyPrec;  // bool var
  Var3D dummySucc;  // bool var
  std::vector<std::vector<MPVariable*>> startTime;
  MPVariable* makeSpan;
  std::vector<MPVariable*> completionTimes;

  /**
   * Collect info and output to OutputStream
   */
  template <typename OutputStream>
  friend void collectInfoAndOutput(Scheduler*, OutputStream&, double lambda);

  // used to compute time needed for each order or products on each machine
  // inline void computeTimeNeeded();

  /// \cond Constraints
  inline std::vector<MPConstraint*> addConstraints_1(
      std::vector<MPVariable*> completionTimes, MPVariable const* makeSpan,
      const std::unique_ptr<MPSolver>& solver,
      const std::string& purposeMessage);

  inline std::vector<MPConstraint*> addConstraints_2(
      const std::vector<std::vector<MPVariable*>>& startTime,
      const std::vector<MPVariable*>& completionTimes, const Var3D& onMachine,
      const std::unique_ptr<MPSolver>& solver,
      const std::string& purposeMessage);

  inline std::vector<MPConstraint*> addConstraints_3(
      const std::vector<std::vector<MPVariable*>>& startTime,
      const Var3D& onMachine, const std::unique_ptr<MPSolver>& solver,
      const std::string& purposeMessage);

  // TODO: reimplementation of constraint 4
  // gap time should be between all order, among all products
  inline std::vector<MPConstraint*> addConstraints_4(
      const std::vector<std::vector<MPVariable*>>& startTime,
      const Var3D& onMachine, const std::unique_ptr<MPSolver>& solver,
      const std::string& purposeMessage);

  inline std::vector<MPConstraint*> addConstraints_5(
      const std::vector<std::vector<MPVariable*>>& startTime,
      const std::unique_ptr<MPSolver>& solver,
      const std::string& purposeMessage);

  inline std::vector<MPConstraint*> addConstraints_6(
      const std::vector<std::vector<MPVariable*>>& startTime,
      const Var3D& onMachine, const std::unique_ptr<MPSolver>& solver,
      const std::string& purposeMessage);

  inline std::vector<MPConstraint*> addConstraints_7(
      const std::vector<std::vector<MPVariable*>>& startTime,
      const std::vector<std::vector<Var3D>>& immediatePrec,
      const std::unique_ptr<MPSolver>& solver,
      const std::string& purposeMessage);

  inline std::vector<MPConstraint*> addConstraints_8(
      const Var3D& onMachine,
      const std::vector<std::vector<Var3D>>& immediatePrec,
      const std::unique_ptr<MPSolver>& solver,
      const std::string& purposeMessage);

  inline std::vector<MPConstraint*> addConstraints_9(
      const Var3D& onMachine, const std::unique_ptr<MPSolver>& solver,
      const std::string& purposeMessage);

  inline std::vector<MPConstraint*> addConstraints_10(
      const Var3D& onMachine, const std::unique_ptr<MPSolver>& solver,
      const std::string& purposeMessage);

  inline std::vector<MPConstraint*> addConstraints_11(
      const std::vector<std::vector<Var3D>>& immediatePrec,
      const Var3D& dummyPrec, const Var3D& dummySucc,
      const std::unique_ptr<MPSolver>& solver,
      const std::string& purposeMessage);

  inline std::vector<MPConstraint*> addConstraints_12(
      const std::vector<MPVariable*>& completionTimes,
      const std::vector<MPVariable*>& tardyTime,
      const std::unique_ptr<MPSolver>& solver,
      const std::string& purposeMessage);

  inline std::vector<MPConstraint*> addConstraints_13(
      const std::vector<MPVariable*>& completionTimes,
      const std::vector<MPVariable*>& earlyTime,
      const std::unique_ptr<MPSolver>& solver,
      const std::string& purposeMessage);

  inline std::vector<MPConstraint*> addConstraints_14(
      const std::vector<MPVariable*>& tardyTime,
      const std::unique_ptr<MPSolver>& solver,
      const std::string& purposeMessage);

  inline std::vector<MPConstraint*> addConstraints_15(
      const std::vector<MPVariable*>& earlyTime,
      const std::unique_ptr<MPSolver>& solver,
      const std::string& purposeMessage);

  inline std::vector<MPConstraint*> addConstraints_16(
      const Var3D& dummyPrec, const std::unique_ptr<MPSolver>& solver,
      const std::string& purposeMessage);

  inline std::vector<MPConstraint*> addConstraints_17(
      const Var3D& dummySucc, const std::unique_ptr<MPSolver>& solver,
      const std::string& purposeMessage);

  inline std::vector<MPConstraint*> addConstraints_18(
      const Var3D& onMachine, const Var3D& dummyPrec,
      const std::unique_ptr<MPSolver>& solver,
      const std::string& purposeMessage);

  inline std::vector<MPConstraint*> addConstraints_19(
      const Var3D& onMachine, const Var3D& dummySucc,
      const std::unique_ptr<MPSolver>& solver,
      const std::string& purposeMessage);

  /// \endcond

 public:
  explicit Scheduler() {}

  void factoryScheduler(
      std::shared_ptr<const Factory> factory,
      MPSolver::OptimizationProblemType optimization_problem_type,
      double lambda, double timeLimit, std::ofstream& outputStream);
};
}

namespace FactoryWorld {
/** \class Machine
*\brief   Contains information of production line.
*
*\fn Machine::capable
*\brief   If the production line can produce this type represented by index.
*
*\fn explicit Machine::Machine(std::vector<Float> capability, Float readyTime)
*\brief   Construct product line given information.
*\param[in]   capability   products per unit time, i.e. speed of production,
* each product type.
*\param[in]   readyTime    No product can be scheduled before this time.
*
*\fn bool Machine::capable(Integral typeIndex) const
*\brief   if the line can produce this product by index.
*\param[in]   typeIndex   product index.
*
*\fn Float Machine::produceTime(Integral typeIndex, Integral numProduct) const
*\brief   the time needed to produce 1 unit of some product.
*\param[in]   typeIndex   product index.
*\param[in]   numProduct  the number of products.
*
*\fn const std::vector<bool> & Machine::getCapableProduct() const
*\brief   Returns the boolean vector indicating product capabilities.
*
*\fn const std::vector<Float> & Machine::getCapability() const
*\brief   Returns the product per unit time of all products.
*
*/

/** \class RelationOfProducts
 * \brief   This class contains the relational information between products.
 *
 * This is the Generalization of Bill of Material(BOM) model, with the following
 * types of relation:
 * - Dependency between two products, i.e., one product precedes another.
 * - Mutex between two products, i.e., enforce time interval between two
 * products.
 * - Transcation Costs between products.
 *
 * \fn   explicit RelationOfProducts::RelationOfProducts(MatrixXd bom,
 * MatrixTime gap)
 * Checking of constraints will be done inside:
 * - Both bom and gap must be square.
 * - No negative value in bom and gap.
 * -
 * \param[in]   MatrixXd   bom dependency matrix, number of products to produce
 * another.
 * \param[in]   MatrixTime time gap between two products.
 *
 * \fn const RelationOfProducts::MatrixXd &getBOM() const
 * \brief Getter for Bom matrix.
 *
 * \fn const MatrixXd &RelationOfProducts::getPredecessor() const
 * \brief Getter for predecessor, the product num required directly or
 * indirectly.
 *
 * Computed directly from this formula, with B as BOM matrix, M as directly or
 * indirectly dep matrix:
 * \f[ M = BM + I \Rightarrow M = (I - B)^{-1} \f]
 * Given that:
 * \f[ M_{ij} = \sum_{k=1}^n B_{ik}M_{kj}, \forall i \ne j \f]
 * \f[ m_{ii} = 1 \f]
 *
 * \fn const MatrixB &RelationOfProducts::getDirectMask() conts
 * \brief Boolean mask matrix from BOM matrix.
 *
 * \fn const MatrixB &RelationOfProducts::getInAndDirectMask() const
 * \brief Boolean mask matrix from predecessor matirx.
 *
 * \fn const MatrixTime &RelationOfProducts::getGap() const
 * \brief Gap time matrix between two products.
 *
 * \fn const MatrixB &RelationOfProducts::getGapMask() const
 * \brief Boolean mask matrix from gap matrix.
 *
 * \fn const MatrixTime &RelationOfProducts::getProductTransCost() const
 * \brief Product transaction cost matrix.
 *
 * \fn Integral RelationOfProducts::getTypeSize() const
 * \brief Number of total products.
 */

/** \class Order
 * \brief Information of order.
 *
 * - Product type.
 * - Product quantity.
 * - Due time
 * - Client id
 * - Material date
 *
 * \fn explicit Order::Order(std::vector<Integral> productQuan,
 *                           std::vector<Integral> productType,
 *                           Float dueTime, Integral clientID, Integral
 * materialDate)
 *
 * The length of product quantity and product type will be checked.
 *
 * \fn const std::vector<Integral> & Order::getProductQuan() const
 * \brief Vector of product quantities of the order.
 *
 * \fn const std::vector<Integral> & Order::getProductType() const
 * \brief Vector of product type of the order.
 *
 * \fn TimeUnit Order::getDueTime() const
 * \brief Due time of the order.
 *
 * \fn Integral Order::getClientID() const
 * \brief Client id of the order.
 *
 * \fn Integral Order::getMaterialDate() const
 * \brief Matrerial date of the order.
 *
 * \fn std::size_t Order::size() const
 * \brief Number of products of the order.
 */

/** \class Factory
 * \brief This class encapsulates all the information needed by the planner.
 *
 * \fn void Factory::load(const std::string &filename)
 * \brief Load from the file with the specified format.
 *
 * Format: TODO
 *
 * \fn const Factory::RelationOfProducts & Factory::getBOM() const
 * \brief The relational info of all products.
 *
 * \fn const std::vector<Order> & Factory::getOrders() const
 * \brief Vector of all orders.
 *
 * \fn const std::vector<Machine> & Factory::getMachines() const
 * \brief Vector of all production lines
 *
 * \fn Float Factory::getTardyCost() const
 * \brief Cost per unit time for not delivering products on time.
 *
 * \fn Float Factory::getEarlyCost() const
 * \brief Cost per unit time for products on stocks.
 *
 * \fn Float Factory::getIdleCost() const
 * \brief Cost per unit time for idle production lines.
 *
 */
}
#endif /* _NEWJOY_FACTORYWORLD_HPP_ */
