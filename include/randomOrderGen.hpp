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
#ifndef _NEWJOY_RANDOMORDERGEN_HPP_
#define _NEWJOY_RANDOMORDERGEN_HPP_
#include <random>
#include <memory>
#include <iostream>
#include <ortools/base/logging.h>
namespace Simulator
{
  /**
   * Order class is extended from the original
   * Member field is compatible with the original
   */
  template <typename Integral=int>
  class Order {
  public:
    Integral client;
    Integral materialDate;
    Integral deadline;
    std::vector<Integral> nums;
    std::vector<Integral> ids;

    explicit Order(Integral client, Integral material_date,
                   Integral deadline, std::vector<Integral> num,
                   std::vector<Integral> id) :
      client(client), materialDate(material_date), deadline(deadline),
      nums(num), ids(id)
    { }
  };

  /* output format:
   * productTypeSize -> [order.id[i], order.num[i] for i in range]
   * -> raw material ready time -> due time -> clientID -> newline
   * order per line
   */
  template <typename Integral>
  std::ostream & operator<< (std::ostream &out, const Order<Integral> &order) {
    CHECK_EQ(order.ids.size(), order.nums.size());
    const auto size = order.ids.size();
    const auto &ids = order.ids;
    const auto &nums = order.nums;
    const auto &client = order.client;
    const auto &materialDate = order.materialDate;
    const auto &deadline = order.deadline;
    out << size << ' ';
    for (auto i = 0ul; i < size; ++ i) {
      out << ids[i] << ' ' << nums[i] << ' ';
    }
    out << ' ' << materialDate << ' ' << deadline << ' ' << client;
    return out;
  }

  /**
   * OrderGenerator
   */
  template <typename Integral=int>
  class OrderGenerator {
  private:
    std::random_device _randomDev;
    Integral _partNum;
    std::vector<double> _leadScale;
    mutable std::mt19937 _gen;
    mutable std::uniform_int_distribution<Integral> _productNumRand;
    mutable std::uniform_int_distribution<Integral> _clientIdRand;
    mutable std::uniform_int_distribution<Integral> _typeRand;
    mutable std::uniform_int_distribution<Integral> _scaleRand;
    Integral _startTime;

  public:
    /**
     * partNums how many types of product
     * leadScale => time per product, end - start == leadScale x productSize
     * productSize is randomly generated, leadScale is randomly picked
     * minMax specifies the range of randomly generated product number
     *
     * NOTE: This implementation have to constrain two types are the same,
     * therefore part num generated for different types are the same
     */
    explicit OrderGenerator(Integral partNums,
                            std::vector<double> leadScale,
                            Integral clientSize,
                            // minMax range of random generated product size
                            std::pair<Integral, Integral> minMax) :
      _partNum(partNums), _leadScale(leadScale), _gen(_randomDev()),
      _productNumRand(minMax.first, minMax.second), _clientIdRand(0, clientSize),
      _typeRand(0, partNums/2-1), _scaleRand(0, leadScale.size()-1)
    { }

    Order<Integral> operator() (Integral startTime) const {
      const auto productNum = _typeRand(_gen);
      const auto clientId = _clientIdRand(_gen);
      // for simplicity, just use one type product
      const auto firstTypeNum = _productNumRand(_gen);
      // used to scale time for product size
      // (product size linear to production time)
      const auto scaleRate = _leadScale[_scaleRand(_gen)];
      const auto materialDate = startTime;
      const auto deadline = startTime + scaleRate * productNum;
      return Order<Integral> (clientId, materialDate, deadline,
                              {productNum, productNum + _partNum / 2},
                              {firstTypeNum, firstTypeNum});
    }

    void setStartTime(Integral startTime)
    { _startTime = startTime; }

    Order<Integral> operator() () const {
      return this->operator() (_startTime);
    }
  };

  /**
   * Taken a lambda simulate poisson process
   * Template Object is the object needed to be generated
   * Template ObjGenerator is a factor used to generate Object
   * Integral as the type for poisson distribution
   */
  template <typename Object,
            typename ObjGenerator,
            typename Integral=int>
  class Simulator {
  private:
    mutable std::poisson_distribution<Integral> _poissonGen;
    std::random_device _randomDev;
    mutable std::mt19937 _gen;

  public:
    /**
     * lambda is the average occurance
     */
    explicit Simulator(double lambda) :
      _poissonGen(lambda), _gen(_randomDev())
    { }

    /**
     * factory function for creating Object
     * objGen is the factor used to generate object
     */
    std::vector<Object> simulate(ObjGenerator &objGen) const {
      std::vector<Object> simulations;
      // number of occurance for the time
      auto num = _poissonGen(_gen);
      for (auto i = 0; i < num; ++ i) {
        // Generate Object using ObjGenerator
        // The ObjGenerator should have operator() as a factory function
        simulations.emplace_back(objGen());
      }
      return simulations;
    }
  };
}

#endif /* _NEWJOY_RANDOMORDERGEN_HPP_ */
