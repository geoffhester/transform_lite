#include <iostream>
#include <type_traits>

#include <boost/type_index.hpp>

#include "poena/poena.h"

using namespace poena;

struct MyType {};

template<int i>
struct NumType {
  static constexpr int value = i;
};

using One = NumType<1>;
using Two = NumType<2>;
using Three = NumType<3>;


template<typename A, typename B>
struct sum_num_type;

template<int A, int B>
struct sum_num_type<NumType<A>, NumType<B>> {
  typedef NumType<A+B> type;
};

template<typename A, typename B>
using sum_num_type_t = typename sum_num_type<A,B>::type;

int main() {
  using A = TypeList<One, Two, Three, NumType<7>>;

  std::cout
      << boost::typeindex::type_id<typename fold_left<A, sum_num_type>::type>()
             .pretty_name()
      << '\n';

  std::cout << sum_num_type_t<One, Three>::value << std::endl;

  //  using Ts = TypeList<int, double>;
  //  using Us = TypeList<char>;
  //
  //  std::cout << boost::typeindex::type_id<concat<Ts,
  //  Us>::type>().pretty_name()
  //            << '\n';
  //
  //  std::cout
  //      << boost::typeindex::type_id<concat<TypeList<>,
  //      Us>::type>().pretty_name()
  //      << '\n';
  //  std::cout
  //      << boost::typeindex::type_id<concat<TypeList<int>,
  //      TypeList<>>::type>()
  //             .pretty_name()
  //      << '\n';
  //
  //  std::cout
  //      << boost::typeindex::type_id<Front<TypeList<int, double,
  //      char>>::type>()
  //             .pretty_name()
  //      << '\n';

//  std::cout
//      << boost::typeindex::type_id<filter<TypeList<double, int, char, MyType>,
//                                          std::is_integral>::type>()
//             .pretty_name()
//      << '\n';
//
//  std::cout << boost::typeindex::type_id<transform<
//                   TypeList<double, int, char, MyType>, std::add_const>::type>()
//                   .pretty_name()
//            << '\n';
//
//  std::cout << contains<int, TypeList<>, std::is_same>::value << '\n'
//            << contains<int, TypeList<int>, std::is_same>::value << '\n'
//            << contains<int, TypeList<double>, std::is_same>::value << '\n'
//            << contains<int, TypeList<int, double>, std::is_same>::value << '\n'
//            << contains<int, TypeList<double, int>, std::is_same>::value << '\n'
//            << contains<int, TypeList<char, char>, std::is_same>::value << '\n';
//
//  std::cout << boost::typeindex::type_id<append_unique<
//                   TypeList<std::pair<int, double>, std::pair<double, int>>,
//                   TypeList<std::pair<double, int>>, std::is_same>::type>()
//                   .pretty_name()
//            << '\n';
//
//  std::cout << boost::typeindex::type_id<
//                   reverse_t<TypeList<int, double, char, float>>>()
//                   .pretty_name()
//            << '\n';
}