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

// small help from eigen to get inverse of BOM
#include <Eigen/Dense>

/**
 * Implementation of processing raw data(input) into internel factoryWorld representation
 */

namespace FactoryWorld {
  using Integral = int;
  using Float = double;
  using IndexType = unsigned int;

  class Factory {
    class Machine {
    private:
      std::vector<bool> capableProduct_;
      std::vector<Float> capability_;
    public:
      explicit Machine() {}
      explicit Machine(std::vector<Float> capability) :
        capableProduct_(capability.size()), capability_(capability)
      {
        // NOTE: don't know if this floating point comparison would be an issue
        std::transform(capability_.cbegin(), capability_.cend(),
                       capableProduct_.begin(), [](Float x) { return x > 0.0; });
      }

    };

    class BillOfMaterial {
    private:
      // bill of material matrix
      MatrixXd bom_;
      // The product required directly or indirectly,
      // computed directly from bom_
      MatrixXd predecessor_;
      // a bool matrix to check if a product is dependent on another product
      Matrix<bool, Dynamic, Dynamic> requiredMask_;
    public:
      explicit BillOfMaterial() {}
      explicit BillOfMaterial(MatrixXd bom) :
        bom_(bom), predecessor_((MatrixXd::Identity(bom.rows(), bom.cols()) - bom_).inverse())
      { requiredMask_ = bom_.array() > 0.0; }
    };

    class Order {
    private:
      std::vector<IndexType> productQuan_;
      std::vector<Integral> productType_;

    public:

    };

  private:

  public:
  };

}

#endif /* _NEWJOY_FACTORYWORLD_HPP_ */
