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
#ifndef _NEWJOY_UTILS_HPP_
#define _NEWJOY_UTILS_HPP_

#include <algorithm>
#include <functional>
#include <memory>

namespace utils {
/*
 * Implement the almost_equal mostly for floating point comparason
 * snippets from std library
 */
template <class T>
typename std::enable_if<!std::numeric_limits<T>::is_integer, bool>::type
almost_equal(T x, T y, int ulp = 1)
// as in how many unit of precision, let's default to 1
{
  // the machine epsilon has to be scaled to the magnitude of the values used
  // and multiplied by the desired precision in ULPs (units in the last place)
  return std::abs(x - y) <=
             std::numeric_limits<T>::epsilon() * std::abs(x + y) * ulp
         // unless the result is subnormal
         || std::abs(x - y) < std::numeric_limits<T>::min();
}

template <typename ForwardIterator, typename BinaryOperation>
void permPair(ForwardIterator first, ForwardIterator last, BinaryOperation f) {
  for (auto iter1 = first; iter1 != last; ++iter1)
    for (auto iter2 = iter1 + 1; iter2 != last; ++iter2) f(iter1, iter2);
}

template <typename... Args>
std::string numPacking() {
  return "";
}

template <typename IndexType, typename... Args>
std::string numPacking(IndexType head, Args... tail) {
  if (sizeof...(tail))
    return std::to_string(head) + ',' + numPacking(tail...);
  else
    return std::to_string(head);
}

template <typename... IndexTypes>
std::string numToBracket(IndexTypes... indices) {
  return '[' + numPacking(indices...) + ']';
}

template<typename T, typename... Args>
std::unique_ptr<T> make_unique(Args&&... args) {
  return std::unique_ptr<T>(new T(std::forward<Args>(args)...));
}
} // namespace utils

#endif /* _NEWJOY_UTILS_HPP_ */
