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

#include <ortools/base/logging.h>
#include <ortools/base/filelineiter.h>
#include <ortools/base/split.h>
#include <ortools/constraint_solver/constraint_solver.h>
#include <ortools/linear_solver/linear_solver.h>

// small help from eigen to get inverse of BOM
#include <Eigen/Dense>

#include <fstream>
#include <sstream>

#include "utils.hpp"
/**
 * Implementation of processing raw data(input) into internel
 * factoryWorld representation
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
    explicit Machine(std::vector<Float> capability, Float readyTime) :
      capableProduct_(capability.size()), capability_(capability),
      readyTime_(readyTime)
    {
      CHECK_GE(readyTime_, 0.0) << "negative readyTime";
      // all capability should be above or equal to zero
      for (const auto & cap : capability_)
        CHECK_GE(cap, 0.0);
      // NOTE: don't know if this floating point comparison
      // would be an issue
      std::transform(capability_.cbegin(), capability_.cend(),
                     capableProduct_.begin(),
                     [](Float x) { return x > 0.0; });
    }

    // return true if the machine is capable of manufacturing
    // this type of product(by index)
    bool capable(Integral typeIndex) const
    { return capableProduct_[typeIndex]; }

    // compute production time when given product type and its quantity
    Float produceTime(Integral typeIndex, Integral numProduct) const
    { return static_cast<double>(numProduct) / capability_[typeIndex]; }

    const std::vector<bool> & getCapableProduct() const
    { return capableProduct_; }

    const std::vector<Float> & getCapability() const
    { return capability_; }

    const Float & getReadyTime() const
    { return readyTime_; }
  };

  /**
   * Generalize from Bill of Material model
   * This class contains the information between products
   * i.e, which two product has dependency
   * which two product has to be kept apart for some time
   * which two product
   */
  class RelationOfProducts {
    using MatrixXd = Eigen::MatrixXd;
    using MatrixB = Eigen::Matrix<bool, Eigen::Dynamic,
                                  Eigen::Dynamic, Eigen::RowMajor>;
  private:
    // bill of material matrix
    MatrixXd bom_;
    // The product required directly or indirectly,
    // computed directly from bom_
    MatrixXd predecessor_;
    // a bool matrix to check if a product is directly dependent on another product
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
    explicit RelationOfProducts(MatrixXd bom, MatrixTime gap) :
      bom_(bom), predecessor_(
        (MatrixXd::Identity(bom.rows(), bom.cols()) - bom_).inverse()),
      gapProduct_(gap), typeSize__(bom.rows()),
      productTransCost_(bom.rows(), bom.cols())
    {
      CHECK_EQ(bom.rows(), bom.cols()) << "bom matrix not square.";
      // NOTE: might need to consider predecessor.
      // Its floating point is inaccurate
      CHECK((bom_.array() >= 0.0).all()) << "Bom list error! No negative pls.";
      CHECK((gapProduct_.array() >= 0.0).all()) << "Gap list error! No negative pls.";
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

    const MatrixXd &getBOM() const { return bom_; }

    const MatrixXd &getPredecessor() const { return predecessor_; }

    const MatrixB &getDirectMask() const
    { return directMask_; }

    const MatrixB &getInAndDirectMask() const
    { return inAndDirectMask_; }

    const MatrixTime &getGap() const
    { return gapProduct_; }

    const MatrixB &getGapMask() const
    { return gapProductMask_; }

    const MatrixTime &getProductTransCost() const
    { return productTransCost_; }

    Integral getTypeSize() const
    { return typeSize__; }
  };

  class Order {
  protected:
    std::vector<Integral> productQuan_;
    std::vector<Integral> productType_;
    TimeUnit dueTime_;
    Integral clientID_;
    Integral materialDate_; // raw material time
  public:
    explicit Order() { }
    explicit Order(std::vector<Integral> productQuan,
                   std::vector<Integral> productType,
                   Float dueTime, Integral clientID, Integral materialDate)
      : productQuan_(productQuan), productType_(productType),
        dueTime_(dueTime), clientID_(clientID),
        materialDate_(materialDate)
    {
      CHECK_EQ(productQuan.size(), productType.size());
    }

    explicit Order(std::ifstream &);

    const std::vector<Integral> &
    getProductQuan() const { return productQuan_; }

    const std::vector<Integral> &
    getProductType() const { return productType_; }

    TimeUnit getDueTime() const { return dueTime_; }

    Integral getClientID() const { return clientID_; }

    Integral getMaterialDate() const { return materialDate_; }

    std::size_t size() const
    { return productType_.size(); }
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
    void load(const std::string &filename);

    const RelationOfProducts & getBOM() const
    { return bom__; }

    const std::vector<Order> & getOrders() const
    { return orders__; }

    const std::vector<Machine> &getMachines() const
    { return machines__; }

    Float getTardyCost() const
    { return tardyCost_; }

    Float getearlyCost() const
    { return earlyCost_; }
  };

  // for debugging
  std::ostream &operator<< (std::ostream &out, const Order &order);

  /**
   * Scheduler class handles the planning by expressing it under
   * linear constraint
   *
   * Itself would takes some unmutable Factory as data input
   * compute and store some temporary variables
   */
  class Scheduler {
  private:
    //using namespace operations_research;
    using MatrixB = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>;
    using MPSolver = operations_research::MPSolver;
    using MPConstraint = operations_research::MPConstraint;
    using MPVariable = operations_research::MPVariable;
    using Var3D = std::vector<std::vector<std::vector<MPVariable *>>>;
    constexpr static double infinity = std::numeric_limits<double>::infinity();
    // due to limitation of or tool, have to limit largeNumber range
    constexpr static double largeNumber = 1.0e5;//std::numeric_limits<double>::max();

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
        std::vector<std::vector<TimeUnit>> &productionTime,
        const std::vector<Machine> &machines,
        const std::vector<Integral> &productQuan,
        const std::vector<Integral> &productType);

    public:
      //explicit OrderWithDep() = delete;
      explicit OrderWithDep() {}
      explicit OrderWithDep(Order noDepOrder,
                            const RelationOfProducts &relation,
                            const std::vector<Machine> &machines);
      bool finalProd(Integral index) const
      { return endProduct__[index]; }

      TimeUnit requiredTime(Integral index, Integral machineIndex) const
      { return productionTime__[index][machineIndex]; }

      const std::vector<std::pair<Integral, Integral>> &
      getDependency() const { return dependency__; }

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
      std::vector<std::pair<
                    std::array<Integral, 4>,
                    TimeUnit>> gap__;

      /**
       * Assume that all the orders are already set up
       */
      inline void buildGap();

    public:
      // DataProvider() = delete;
      explicit DataProvider() = default; //: factory__(nullptr) {}
      explicit DataProvider(std::shared_ptr<const Factory> factory);

      DataProvider(const DataProvider &) = default;
      DataProvider &operator=(const DataProvider&) = default;

      DataProvider(DataProvider &&) = default;
      DataProvider &operator=(DataProvider &&) = default;

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

      const std::vector<OrderWithDep> &getOrders() const
      { return orders__; }

      const std::vector<Machine> &getMachines() const
      { return factory__->getMachines(); }

      const MatrixTime &getTransCost() const
      { return factory__->getBOM().getProductTransCost(); }

      const std::vector<std::pair<std::array<Integral, 4>, TimeUnit>> &getGap()
      { return gap__; }
    };

    /*
     * data field used to deal with relationship between products
     */
    // this variable is computed using orders and machines, and would be extended
    // to include the dependent products
    DataProvider dataProvider__;
    std::shared_ptr<const Factory> factory__;

    // used to compute time needed for each order or products on each machine
    //inline void computeTimeNeeded();

    inline std::vector<MPConstraint *> addConstraints_1(
      std::vector<MPVariable *> completionTimes,
      MPVariable const *makeSpan,
      MPSolver &solver, const std::string &purposeMessage);

    inline std::vector<MPConstraint *> addConstraints_2(
      const std::vector<std::vector<MPVariable *>> &startTime,
      const std::vector<MPVariable *> &completionTimes,
      const Var3D &onMachine,
      MPSolver &solver, const std::string &purposeMessage);

    inline std::vector<MPConstraint *> addConstraints_3(
      const std::vector<std::vector<MPVariable *>> &startTime,
      const Var3D &onMachine,
      MPSolver &solver, const std::string &purposeMessage);

    // TODO: reimplementation of constraint 4
    // gap time should be between all order, among all products
    inline std::vector<MPConstraint *> addConstraints_4(
      const std::vector<std::vector<MPVariable *>> &startTime,
      const Var3D &onMachine,
      MPSolver &solver, const std::string &purposeMessage);

    inline std::vector<MPConstraint *> addConstraints_5(
      const std::vector<std::vector<MPVariable *>> &startTime,
      MPSolver &solver, const std::string &purposeMessage);

    inline std::vector<MPConstraint *> addConstraints_6(
      const std::vector<std::vector<MPVariable *>> &startTime,
      const Var3D &onMachine,
      MPSolver &solver, const std::string &purposeMessage);

    inline std::vector<MPConstraint *> addConstraints_7(
      const std::vector<std::vector<MPVariable *>> &startTime,
      const std::vector<std::vector<Var3D>> &immediatePrec,
      MPSolver &solver, const std::string &purposeMessage);

    inline std::vector<MPConstraint *> addConstraints_8(
      const Var3D &onMachine,
      const std::vector<std::vector<Var3D>> &immediatePrec,
      MPSolver &solver, const std::string &purposeMessage);

    inline std::vector<MPConstraint *> addConstraints_9(
      const Var3D &onMachine,
      MPSolver &solver, const std::string &purposeMessage);

    inline std::vector<MPConstraint *> addConstraints_10(
      const Var3D &onMachine,
      MPSolver &solver, const std::string &purposeMessage);

    inline std::vector<MPConstraint *> addConstraints_11(
      const std::vector<std::vector<Var3D>> &immediatePrec,
      const Var3D &dummyPrec,
      const Var3D &dummySucc,
      MPSolver &solver, const std::string &purposeMessage);

    inline std::vector<MPConstraint *> addConstraints_12(
      const std::vector<MPVariable *> &completionTimes,
      const std::vector<MPVariable *> &tardyTime,
      MPSolver &solver, const std::string &purposeMessage);

    inline std::vector<MPConstraint *> addConstraints_13(
      const std::vector<MPVariable *> &completionTimes,
      const std::vector<MPVariable *> &earlyTime,
      MPSolver &solver, const std::string &purposeMessage);

    inline std::vector<MPConstraint *>
    addConstraints_14(
      const std::vector<MPVariable *> &tardyTime,
      MPSolver &solver, const std::string &purposeMessage);

    inline std::vector<MPConstraint *>
    addConstraints_15(
      const std::vector<MPVariable *> &earlyTime,
      MPSolver &solver, const std::string &purposeMessage);

    inline std::vector<MPConstraint *>
    addConstraints_16(
      const Var3D &dummyPrec,
      MPSolver &solver, const std::string &purposeMessage);

    inline std::vector<MPConstraint *>
    addConstraints_17(
      const Var3D &dummySucc,
      MPSolver &solver, const std::string &purposeMessage);

    inline std::vector<MPConstraint *>
    addConstraints_18(
      const Var3D &onMachine,
      const Var3D &dummyPrec,
      MPSolver &solver, const std::string &purposeMessage);

    inline std::vector<MPConstraint *>
    addConstraints_19(
      const Var3D &onMachine,
      const Var3D &dummySucc,
      MPSolver &solver, const std::string &purposeMessage);

  public:
    explicit Scheduler() {}

    void factoryScheduler(std::shared_ptr<const Factory> factory,
      MPSolver::OptimizationProblemType optimization_problem_type);
  };
}

#endif /* _NEWJOY_FACTORYWORLD_HPP_ */
