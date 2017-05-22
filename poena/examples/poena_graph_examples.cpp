#include <iostream>
#include <string>
#include <type_traits>

#include <boost/type_index.hpp>
#include <poena/graph/graph.h>

#include "poena/poena.h"
#include "poena/graph/graph.h"

using namespace poena;
using namespace poena::graph;

template <typename Dest, typename Src>
struct Thing {
  using source_frame = Src;
  using dest_frame = Dest;
};

template <typename Dest, typename Src>
std::string to_string(Thing<Dest, Src>) {
  return (std::string("[") + std::to_string(Dest::value) + " " +
      std::to_string(Src::value) + "]");
};

template <typename T>
struct thing_inverse {};

template <typename Dest, typename Src>
struct thing_inverse<Thing<Dest, Src>> {
  typedef Thing<Src, Dest> type;
};

template <typename T>
using thing_inverse_t = typename thing_inverse<T>::type;

template <typename LHS, typename RHS>
struct thing_compose {};

template <typename LhsDest, typename Common, typename RhsSrc>
struct thing_compose<Thing<LhsDest, Common>, Thing<Common, RhsSrc>> {
  typedef Thing<LhsDest, RhsSrc> type;
};

template <typename LHS, typename RHS>
using thing_compose_t = typename thing_compose<LHS, RHS>::type;

template <typename LHS, typename RHS>
struct thing_equal_impl : std::false_type {};

template <typename Dest, typename Src>
struct thing_equal_impl<Thing<Dest, Src>, Thing<Dest, Src>> : std::true_type {};

template <typename LHS, typename RHS>
struct thing_equal
    : thing_equal_impl<std::remove_cv_t<LHS>, std::remove_cv_t<RHS>> {};

template <typename LHS, typename RHS>
constexpr bool thing_equal_v = thing_equal<LHS, RHS>::value;

template <typename LHSD, typename LHSS, typename RHSD, typename RHSS>
constexpr bool operator==(Thing<LHSD, LHSS>, Thing<RHSD, RHSS>) {
  return thing_equal_v<Thing<LHSD, LHSS>, Thing<RHSD, RHSS>>;
}

template <typename T>
struct source_frame {};

template <typename DestFrame, typename SrcFrame>
struct source_frame<Thing<DestFrame, SrcFrame>> {
  typedef SrcFrame type;
};

template <typename T>
using source_frame_t = typename source_frame<T>::type;

template <typename T>
struct destination_frame {};

template <typename DestFrame, typename SrcFrame>
struct destination_frame<Thing<DestFrame, SrcFrame>> {
  typedef DestFrame type;
};

template <typename T>
using destination_frame_t = typename destination_frame<T>::type;

template <int N>
struct NumTag {
  static constexpr auto value = N;
};

using One = NumTag<1>;
using Two = NumTag<2>;
using Three = NumTag<3>;
using Four = NumTag<4>;
using Five = NumTag<5>;
using Six = NumTag<6>;
using Seven = NumTag<7>;
using Eight = NumTag<8>;
using Nine = NumTag<9>;

template<typename T>
struct edge_from_thing;

template<typename D, typename S>
struct edge_from_thing<Thing<D,S>> {
  typedef TypeList<D,S> type;
};

template<typename Transforms>
struct TransformGraphFromTransforms;

template<typename Transforms>
struct EdgeListFromTransformList;

template <typename... Ts>
struct EdgeListFromTransformList<TypeList<Ts...>>
    : public transform<unique_t<concat_t<TypeList<Ts...>, transform_t<TypeList<Ts...>, thing_inverse>>>,
                         edge_from_thing> {};



template <typename... Ts>
struct TransformGraphFromTransforms<TypeList<Ts...>> {
  struct type {
    static constexpr GraphTypeTag graph_type = GraphTypeTag::EdgeList;
    static constexpr GraphDirectedProperty directed = GraphDirectedProperty::DirectedEdges;
    using all_edges = transform_t<unique_t<concat_t<TypeList<Ts...>, transform_t<TypeList<Ts...>, thing_inverse>>>,
                                edge_from_thing>;
  };
};

//template<>
//struct TransformGraphImpl<TypeList<>> {
//  typedef TypeList<> type;
//};
//
//template<typename T, typename... Transforms>
//struct TransformGraphImpl<TypeList<T, Transforms...>> {
//
//};
//


using One = NumTag<1>;
using Two = NumTag<2>;
using Three = NumTag<3>;
using Four = NumTag<4>;
using Five = NumTag<5>;
using Six = NumTag<6>;
using Seven = NumTag<7>;
using Eight = NumTag<8>;
using Nine = NumTag<9>;

template <typename T>
void print_type() {
  std::cout << boost::typeindex::type_id<T>().pretty_name() << '\n';
}

struct MyGraph {
  static constexpr GraphTypeTag graph_type = GraphTypeTag::EdgeList;
  static constexpr GraphDirectedProperty directed = GraphDirectedProperty::DirectedEdges;
  using all_edges =
      TypeList<TypeList<One, Two>, TypeList<Two, Three>, TypeList<Two, Five>,
               TypeList<Four, One>, TypeList<Four, Seven>, TypeList<Five, Four>,
               TypeList<Six, Five>, TypeList<Seven, Eight>,
               TypeList<Eight, Five>, TypeList<Eight, Six>,
               TypeList<Eight, Nine>, TypeList<Nine, Six>>;
};

int main() {
  print_type<typename bfs<MyGraph, Six, One>::type>();
}


//int main() {
//  using CalibrationTransforms = TypeList<Thing<One, Two>, Thing<One, Five>, Thing<Two, Three>,
//                     Thing<Two, Six>, Thing<Three, Four>, Thing<Three, Seven>,
//                     Thing<Four, Eight>, Thing<Five, Six>, Thing<Six, Seven>,
//                     Thing<Seven, Eight>>;
//
//  using CalibrationTransformGraph = typename TransformGraphFromTransforms<CalibrationTransforms>::type;
//
//  print_type<typename bfs<CalibrationTransformGraph, Eight, Two>::type>();
//
//}

// namespace hana = boost::hana;
// using namespace hana::literals;
// using namespace hana::traits;
//
// auto hana_source_frame = [](
//    auto x) -> hana::type<source_frame_t<typename decltype(x)::type>> {
//  return {};
//};
//
// auto hana_destination_frame = [](
//    auto x) -> hana::type<destination_frame_t<typename decltype(x)::type>> {
//  return {};
//};
//
// auto hana_inverse = [](
//    auto x) -> hana::type<thing_inverse_t<typename decltype(x)::type>> {
//  return {};
//};
//
// auto hana_compose = [](auto x, auto y) -> hana::type<
//    thing_compose_t<typename decltype(x)::type, typename decltype(y)::type>> {
//  return {};
//};
//
// template<typename LHS, typename RHS>
// constexpr auto hana_thing_compare(LHS lhs, RHS rhs) {
//  return hana::bool_c<lhs == rhs>;
//}
//
// template<typename InitialFrame, typename TargetFrame, typename Transforms>
// auto DoStuff(InitialFrame initial_frame, TargetFrame target_frame, Transforms
// transforms) {
//
//}
//
// constexpr auto type_map =
//    hana::make_map(hana::make_pair(hana::type_c<OneTwo>, OneTwo{}),
//                   hana::make_pair(hana::type_c<TwoThree>, TwoThree{}));
//
// int main() {
//
// auto x = hana::find(type_map, hana::type_c<OneTwo>);
//
//  std::cout << boost::typeindex::type_id_runtime(x) << '\n';
//  //std::cout << boost::typeindex::type_id_runtime(decltype(x())) << '\n';
//
//
//  //BOOST_HANA_CONSTANT_CHECK(x == hana::just(OneTwo{}));
//
//
////  auto bar = is_S(hana::type<int>{})();
////  std::cout << boost::typeindex::type_id<decltype(bar)>().pretty_name() <<
///'\n';
////
////  std::cout << "is_S (    S { }) = " << is_S(hana::type<S>{})() <<
/// std::endl;
////  std::cout << "is_S (    T { }) = " << is_S(hana::type<T>{})() <<
/// std::endl;
////  std::cout << "is_S (float { }) = " << is_S(hana::type<float>{})()
////            << std::endl;
//
////  auto AllThings = boost::hana::tuple_t<OneTwo, TwoThree>;
////
////  auto SomeThings = boost::hana::filter(AllThings, [](auto x) {
////    return is_same(hana::just(hana_destination_frame(x)),
////                   hana::just(hana::type<Two>{}));
////  });
////  BOOST_HANA_CONSTANT_CHECK(SomeThings == hana::tuple_t<TwoThree>);
////  auto OtherThingsTmp = boost::hana::filter(AllThings, [](auto x) {
////    return is_same(hana::just(hana_source_frame(x)),
////                   hana::just(hana::type<Two>{}));
////  });
////  BOOST_HANA_CONSTANT_CHECK(OtherThingsTmp == hana::tuple_t<OneTwo>);
////  auto OtherThings = hana::transform(OtherThingsTmp, hana_inverse);
////  BOOST_HANA_CONSTANT_CHECK(OtherThings ==
////                            hana::tuple_t<thing_inverse_t<OneTwo>>);
////
////  auto Foo = hana::fold_left(AllThings, hana_compose);
////  std::cout << boost::typeindex::type_id_runtime(Foo).pretty_name() << '\n';
////
////  auto AllThings2 = hana::make_tuple(OneTwo{}, TwoThree{});
////
////
////
////  auto SomeThings2 = hana::filter(AllThings2, [](auto x) {
////    return
/// is_same(hana::just(hana::type<destination_frame_t<decltype(x)>>{}),
////                   hana::just(hana::type<Two>{}));
////  });
////  std::cout << boost::typeindex::type_id_runtime(SomeThings).pretty_name()
////            << '\n';
////  std::cout << boost::typeindex::type_id_runtime(SomeThings2).pretty_name()
////            << '\n';
////
////  std::cout << boost::typeindex::type_id_runtime(AllThings2).pretty_name()
////            << '\n';
////
////  static_assert(std::is_same<decltype(SomeThings2),
////                             decltype(hana::make_tuple(TwoThree{}))>::value,
////                "");
////
////  auto OtherThings2 = hana::transform(
////      boost::hana::filter(
////          AllThings2,
////          [](auto x) {
////            return is_same(
////                hana::just(hana::type<source_frame_t<decltype(x)>>{}),
////                hana::just(hana::type<Two>{}));
////          }),
////      [](auto x) { return thing_inverse_t<decltype(x)>{}; });
////  std::cout << boost::typeindex::type_id_runtime(OtherThings2).pretty_name()
////            << '\n';
////
////  auto result_things = hana::concat(SomeThings2, OtherThings2);
////  std::cout <<
/// boost::typeindex::type_id_runtime(result_things).pretty_name()
////            << '\n';
////  static_assert(
////      std::is_same<decltype(result_things),
////                   hana::tuple<TwoThree, thing_inverse_t<OneTwo>>>::value,
////      "");
////
////  std::cout <<
/// boost::typeindex::type_id<decltype(hana::type_c<TwoThree>)>().pretty_name()
///<< '\n';
////
////  auto y = hana::find(result_things, hana::type_c<TwoThree>);
////
////  //BOOST_HANA_CONSTANT_CHECK(hana::find(result_things,
/// hana::type_c<TwoThree>) == hana::just(hana::type_c<TwoThree>));
////
////  std::cout << boost::typeindex::type_id_runtime(y).pretty_name() << '\n';
////
////  std::cout << (OneTwo{} == Thing<Two,One>{}) << '\n';
