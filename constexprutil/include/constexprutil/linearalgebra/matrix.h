#pragma once
#ifndef TRANSFORM_LITE_MATRIX_H
#define TRANSFORM_LITE_MATRIX_H

#include <algorithm>
#include <array>
#include <iostream>

#include "constexprutil/math/trigonometry.h"

namespace constexprutil {
namespace linearalgebra {

constexpr std::size_t row_index(std::size_t num_rows, std::size_t num_cols,
                                std::size_t index) {
  return index % num_rows;
}

constexpr std::size_t col_index(std::size_t num_rows, std::size_t num_cols,
                                std::size_t index) {
  return index / num_rows;
}

constexpr std::size_t sub2ind(std::size_t num_rows, std::size_t num_cols,
                              std::size_t row_index, std::size_t col_index) {
  return col_index * num_rows + row_index;
}

template <typename T, std::size_t Rows, std::size_t Cols>
struct ConstexprMatrix {
 public:
  static constexpr std::size_t rows = Rows;
  static constexpr std::size_t cols = Cols;
  static constexpr std::size_t size = Rows * Cols;
  std::array<T, size> data;
};

template<typename T, std::size_t Dims>
constexpr std::array<T, Dims*Dims> make_eye_array() {
  std::array<T, Dims*Dims> result;
  for (std::size_t i = 0; i < Dims; ++i) {
    result[sub2ind(Dims,Dims,i,i)] = T(1);
  }
  return result;
}

template <typename T, std::size_t Dims>
struct Identity : public ConstexprMatrix<T, Dims, Dims> {
  using ConstexprMatrix<T, Dims, Dims>::rows;
  using ConstexprMatrix<T, Dims, Dims>::cols;
  using ConstexprMatrix<T, Dims, Dims>::size;

  using ConstexprMatrix<T, Dims, Dims>::data;

  Identity() {
    this->data.fill(0);
    for (std::size_t i = 0; i < Dims; ++i) {
      this->data.at(sub2ind(Dims,Dims,i,i)) = T(1);
    }
  }
};

template <typename T, std::size_t N>
constexpr T sum_array(std::array<T, N> const& a) {
  T result = T{0};
  for (std::size_t i = 0; i < N; ++i) {
    result += a.at(i);
  }
  return result;
}

template <typename T1, std::size_t N1, std::size_t... Is1, typename T2,
          std::size_t N2, std::size_t... Is2>
constexpr auto matrix_multiply_impl(std::array<T1, N1> const& lhs,
                                    std::index_sequence<Is1...>,
                                    std::array<T2, N2> const& rhs,
                                    std::index_sequence<Is2...>) {
  return sum_array(
      std::array<typename std::common_type<T1, T2>::type, sizeof...(Is1)>{
          (lhs[Is1] * rhs[Is2])...});
}

template <typename T1, std::size_t N1, typename T2, std::size_t N2>
constexpr auto vector_multiply(std::array<T1, N1> const& lhs,
                               std::array<T2, N2> const& rhs) {
  return matrix_multiply_impl(lhs, std::make_index_sequence<N1>(), rhs,
                              std::make_index_sequence<N2>());
}



template <typename T, T Stride, typename IS>
struct strided_index_sequence_impl;

template <typename T, T Stride, T... N>
struct strided_index_sequence_impl<T, Stride, std::integer_sequence<T, N...>> {
  using type = std::integer_sequence<T, Stride * N...>;
};

template <std::size_t N, std::size_t Stride>
struct strided_index_sequence
    : strided_index_sequence_impl<std::size_t, Stride,
                                  std::make_index_sequence<N>> {};

template <std::size_t N, std::size_t Stride>
using strided_index_sequence_t =
    typename strided_index_sequence<N, Stride>::type;

template <typename T, typename, T C>
struct add_constant;

template <typename T, T... Is, T C>
struct add_constant<T, std::integer_sequence<T, Is...>, C> {
  using type = std::integer_sequence<T, C + Is...>;
};

template <std::size_t NumRows, std::size_t NumCols, std::size_t RowIndex>
constexpr auto indicies_for_row() {
  return typename add_constant<std::size_t,
                               strided_index_sequence_t<NumCols, NumRows>,
                               RowIndex>::type{};
}

template <std::size_t NumRows, std::size_t NumCols, std::size_t ColIndex>
constexpr auto indicies_for_column() {
  return typename add_constant<std::size_t, std::make_index_sequence<NumRows>,
                               ColIndex * NumRows>::type{};
}

template <typename LhsT, std::size_t LhsRows, std::size_t LhsCols,
          typename RhsT, std::size_t RhsRows, std::size_t RhsCols,
          std::size_t index>
constexpr auto mul_helper(ConstexprMatrix<LhsT, LhsRows, LhsCols> const& lhs,
                          ConstexprMatrix<RhsT, RhsRows, RhsCols> const& rhs) {
  constexpr auto row_i = row_index(LhsRows, RhsCols, index);
  constexpr auto col_i = col_index(LhsRows, RhsCols, index);
  return matrix_multiply_impl(
      lhs.data, indicies_for_row<LhsRows, LhsCols, row_i>(), rhs.data,
      indicies_for_column<RhsRows, RhsCols, col_i>());
}

template <typename LhsT, std::size_t LhsRows, std::size_t LhsCols,
          typename RhsT, std::size_t RhsRows, std::size_t RhsCols,
          std::size_t... Is>
constexpr auto mul_helper(ConstexprMatrix<LhsT, LhsRows, LhsCols> const& lhs,
                          ConstexprMatrix<RhsT, RhsRows, RhsCols> const& rhs,
                          std::index_sequence<Is...>) {
  return std::array<typename std::common_type<LhsT, RhsT>::type,
                    LhsRows * RhsCols>{
      mul_helper<LhsT, LhsRows, LhsCols, RhsT, RhsRows, RhsCols, Is>(lhs,
                                                                     rhs)...};
}

template <typename LhsT, std::size_t LhsRows, std::size_t LhsCols,
          typename RhsT, std::size_t RhsRows, std::size_t RhsCols>
constexpr auto operator*(ConstexprMatrix<LhsT, LhsRows, LhsCols> const& lhs,
                         ConstexprMatrix<RhsT, RhsRows, RhsCols> const& rhs)
    -> ConstexprMatrix<typename std::common_type<LhsT, RhsT>::type, LhsRows,
                       RhsCols> {
  return ConstexprMatrix<typename std::common_type<LhsT, RhsT>::type, LhsRows,
                         RhsCols>{
      mul_helper(lhs, rhs, std::make_index_sequence<LhsRows * RhsCols>())};
}

template <typename LhsT, std::size_t LhsRows, std::size_t LhsCols,
          typename RhsT, std::size_t RhsRows, std::size_t RhsCols>
constexpr bool operator==(ConstexprMatrix<LhsT, LhsRows, LhsCols> const& lhs,
                          ConstexprMatrix<RhsT, RhsRows, RhsCols> const& rhs) {
  static_assert(LhsRows == RhsRows && LhsCols == RhsCols, "Matrix dimension mismatch");
  return lhs.data == rhs.data;
}

template <typename T, std::size_t Rows, std::size_t Cols>
std::ostream& operator<<(std::ostream& os,
                         ConstexprMatrix<T, Rows, Cols> const& x) {
  std::copy(std::begin(x.data), std::end(x.data),
            std::ostream_iterator<T>(os, " "));
  return os;
}

template <std::size_t SOGroup>
struct Transformation;

template <>
struct Transformation<2> {
  static constexpr std::size_t SOGroup = 2;
  ConstexprMatrix<double, 3, 3> homogenous_form;

  constexpr Transformation(double x, double y, double ang_yaw)
      : homogenous_form{{math::my_cos(ang_yaw), math::my_sin(ang_yaw), 0.0,
                         -math::my_sin(ang_yaw), math::my_cos(ang_yaw), 0.0, x,
                         y, 1.0}} {}
};

}  // namespace linearalgebra
}  // namespace constexprutil

#endif  // TRANSFORM_LITE_MATRIX_H
