#include <array>
#include <iostream>
#include <numeric>
#include <vector>

#include "constexprutil/linearalgebra/matrix.h"

using namespace constexprutil::linearalgebra;

template<typename Ch, typename Tr, typename T, T... Ns, std::size_t... Is>
void print_integer_sequence(std::basic_ostream<Ch,Tr>& os, std::integer_sequence<T,Ns...>, std::index_sequence<Is...>) {
  ((os << (Is == 0 ? "" : ", ") << Ns), ...);
};

template<typename Ch, typename Tr, typename T, T... Ns>
decltype(auto) operator<<(std::basic_ostream<Ch, Tr>& os,
                          std::integer_sequence<T,Ns...> is) {
  os << "(";
  print_integer_sequence(os, is, std::make_index_sequence<sizeof...(Ns)>{});
  return os << ")";
}

int main() {

  std::array<int, 3> a = {1,2,3};
  std::array<int, 3> b = {4,5,6};

  std::cout << vector_multiply(a,b) << '\n';

  constexpr std::size_t num_rows = 2;
  constexpr std::size_t num_cols = 2;
  std::array<std::size_t, num_rows * num_cols> is;
  std::iota(std::begin(is), std::end(is), 0);

  for(auto i : is) {
    std::cout << i << ' ';
  }
  std::cout << '\n';
  for(auto i : is) {
    std::cout << row_index(num_rows, num_cols, i) << ' ';
  }
  std::cout << '\n';
  for(auto i : is) {
    std::cout << col_index(num_rows, num_cols, i) << ' ';
  }
  std::cout << '\n';


  constexpr ConstexprMatrix<double,3,3> c{1,0,0,0,1,0,0,0,1};
  constexpr ConstexprMatrix<double,3,1> d{1,2,3};

  constexpr auto e = c*d;
  std::cout << e << '\n';



//  constexprutil::linearalgebra::Transformation<2> g(1.0,2.0,0.0);
//  std::cout << g.homogenous_form << '\n';

  return 0;
}
