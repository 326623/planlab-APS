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

/**
 * Implementation of processing raw data(input) into internel factoryWorld representation
 */

namespace FactoryWorld {
  // should guarantee that these are streamable
  using Integral = int;
  using Float = double;
  using IndexType = unsigned int;
  using TimeUnit = double;

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
      // all capability should be above or equal to zero
      for (const auto & cap : capability_)
        CHECK_GE(cap, 0.0);
      // NOTE: don't know if this floating point comparison
      // would be an issue
      std::transform(capability_.cbegin(), capability_.cend(),
                     capableProduct_.begin(),
                     [](Float x) { return x > 0.0; });
    }

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
    using MatrixB = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>;
  private:
    // bill of material matrix
    MatrixXd bom_;
    // The product required directly or indirectly,
    // computed directly from bom_
    MatrixXd predecessor_;
    // a bool matrix to check if a product is dependent on another product
    MatrixB requiredMask_;
  public:
    explicit RelationOfProducts() {}
    explicit RelationOfProducts(MatrixXd bom) :
      bom_(bom), predecessor_(
        (MatrixXd::Identity(bom.rows(), bom.cols()) - bom_).inverse())
    {
      // NOTE: might need to consider predecessor.
      // Its floating point is inaccurate
      CHECK((bom_.array() >= 0.0).all());
      requiredMask_ = bom_.array() > 0.0;
    }

    const MatrixXd & getBOM() const { return bom_; }
    const MatrixXd & getPredecessor() const { return predecessor_; }
    const MatrixB & getRequiredMask() const
    { return requiredMask_; }
  };

  class Order {
  private:
    std::vector<Integral> productQuan_;
    std::vector<Integral> productType_;
    Float dueTime_;
    Integral clientID_;
    Integral materialDate_; // raw material time
  public:
    explicit Order() { }
    explicit Order(std::vector<Integral> productQuan,
                   std::vector<Integral> productType,
                   Float dueTime, Integral clientID, Integral materialDate)
      : productQuan_(productQuan), productType_(productType),
        dueTime_(dueTime), clientID_(clientID), materialDate_(materialDate)
    { CHECK_EQ(productQuan.size(), productType.size()); }

    explicit Order(std::ifstream &);

    const std::vector<Integral> &
    getProductQuan() const { return productQuan_; }

    const std::vector<Integral> &
    getProductType() const { return productType_; }

    Float getDueTime() const { return dueTime_; }

    Integral getClientID() const { return clientID_; }

    Integral getMaterialDate() const { return materialDate_; }
  };


  class Factory {
  private:
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

  };

  // for debugging
  std::ostream &operator<< (std::ostream &out, const Order &order) {
    const auto & productQuan = order.getProductQuan();
    const auto & productType = order.getProductType();
    const auto dueTime = order.getDueTime();
    const auto clientID = order.getClientID();
    const auto materialDate = order.getMaterialDate();

    out << "client " << clientID << " order ";
    IndexType size = productType.size();
    for (IndexType i = 0; i < size; ++ i) {
      out << productQuan[i] << " of product "
          << productType[i] << '\n';
    }
    out << "Due at " << dueTime << '\n';
    out << "Material arrived at " << materialDate << '\n';
    return out;
  }

  /**
   * Scheduler class handles the planning by expressing it under
   * linear constraint
   *
   * Itself would takes some unmutable Factory as data input
   * compute and store some temporary variables
   */
  class Scheduler {
    //using namespace operations_research;
    using MPSolver = operations_research::MPSolver;
  private:
    // this variable is computed using orders and machines
    std::vector<std::vector<TimeUnit>> timeNeeded__;
    const Factory *factory__;

    // used to compute time needed for each order or products on each machine
    void computeTimeNeeded();

  public:
    explicit Scheduler() {}
    void factoryScheduler(const Factory *factory,
      MPSolver::OptimizationProblemType optimization_problem_type);
  };
}

#endif /* _NEWJOY_FACTORYWORLD_HPP_ */
