/*
Copyright (c) 2018 TOYOTA MOTOR CORPORATION
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#ifndef HSRB_BASE_CONTROLLERS_FILTER_HPP_
#define HSRB_BASE_CONTROLLERS_FILTER_HPP_

#include <cstddef>
#include <algorithm>
#include <vector>

namespace hsrb_base_controllers {

// matlabの下記関数と仕様を同じにしてある
// https://jp.mathworks.com/help/matlab/ref/filter.html
// Y(z)={b[1]+b[2]z^-1 +...+b[nb+1]z^−nb}/{1+a[2]z^−1+...+a[na-1]z^-na} * X(z)
// y[n]=b[1]x[n]+b[2]x[n−1]+...+b[nb+1]x[n-nb]−a[2]y[n−1]−...−a[na-1]y[n-na]
//  ただしb[1](matlab)->b[0](c++), a[1](matlab)->a[0](c++), a[0] = 1.0
// Tは入出力の型, Vは内部の型(基本double)
template<typename T = double, typename V = double>
class Filter {
 public:
  Filter() {
    a_.resize(1);
    b_.resize(1);
    a_[0] = 1.0;
    b_[0] = 1.0;
    reset(0.0);
  }
  Filter(const std::vector<V>& a, const std::vector<V>& b)
      : a_(a), b_(b) {
    // パラメータチェックは基本上でやる
    assert(a_.size() > 0);
    assert(b_.size() > 0);
    reset(0.0);
  }
  virtual ~Filter() {}

  // 内部状態を一定値リセットする
  void reset(const T& value) {
    x_.resize(b_.size());
    y_.resize(a_.size());
    std::fill(x_.begin(), x_.end(), static_cast<V>(value));
    std::fill(y_.begin(), y_.end(), static_cast<V>(value));
  }

  // フィルタをかける
  T update(const T& x) {
    V y = 0.0;
    // a[0]は使わない
    for (size_t i = a_.size() - 1; i > 0; --i) {
      y_[i] = y_[i - 1];
      y -= a_[i] * y_[i];
    }
    for (size_t i = b_.size() - 1; i > 0; --i) {
      x_[i] = x_[i - 1];
      y += b_[i] * x_[i];
    }
    y += b_[0] * x;
    x_[0] = static_cast<V>(x);
    y_[0] = static_cast<V>(y);
    return static_cast<T>(y);
  }

 private:
  std::vector<V> a_;
  std::vector<V> b_;
  std::vector<V> y_;
  std::vector<V> x_;
};

}  // namespace hsrb_base_controllers

#endif
