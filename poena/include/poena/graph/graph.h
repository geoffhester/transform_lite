//! \file
#pragma once
#ifndef SANDBOX_MPL_GRAPH_H
#define SANDBOX_MPL_GRAPH_H

#include <type_traits>

#include "poena/poena.h"

namespace poena {
namespace graph {

enum class GraphTypeTag { EdgeList, AdjacencyMatrix };

enum class GraphDirectedProperty { DirectedEdges, BidirectionalEdges };

template<typename Edge>
struct source;

template<typename Edge>
using source_t = typename source<Edge>::type;

template<typename S, typename T>
struct source<TypeList < S, T>> {
typedef S type;
};

template<typename Edge>
struct target;

template<typename Edge>
using target_t = typename target<Edge>::type;

template<typename S, typename T>
struct target<TypeList < S, T>> {
typedef T type;
};

template<typename Source, typename E>
struct is_out_edge : std::is_same<Source, source_t < E>> {
};

template<typename Source, typename E>
struct is_in_edge : std::is_same<Source, target_t < E>> {
};

template<typename T, typename G, GraphTypeTag GraphType,
    GraphDirectedProperty Directed>
struct out_edges_impl;

template<typename T, typename G>
struct out_edges_impl<T, G, GraphTypeTag::EdgeList,
                      GraphDirectedProperty::DirectedEdges> {
 private:
  using edge_list = typename G::all_edges;

  template<typename E>
  using is_out_edge_of_T = is_out_edge<T, E>;

  using vertex_out_edges = filter_t<edge_list, is_out_edge_of_T>;

 public:
  typedef vertex_out_edges type;
};

template<typename T, typename G>
struct out_edges : out_edges_impl<T, G, G::graph_type, G::directed> {};

template<typename T, typename G>
using out_edges_t = typename out_edges<T, G>::type;

template<typename T, typename G, GraphTypeTag GraphType>
struct neighbours_impl;

template<typename T, typename G>
struct neighbours_impl<T, G, GraphTypeTag::EdgeList> {
 private:
  using out_edges_of_T = out_edges_t<T, G>;

 public:
  typedef transform_t <out_edges_of_T, target> type;
};

template<typename T, typename G>
struct neighbours : neighbours_impl<T, G, G::graph_type> {};

template<typename T, typename G>
using neighbours_t = typename neighbours<T, G>::type;

template<typename Source, typename Neighbours>
struct make_parent_pairs;

template<typename Source, typename Neighbours>
struct make_parent_pairs_impl;

template<typename Source, typename Neighbours>
struct make_parent_pairs : make_parent_pairs_impl<Source, Neighbours> {};

template<typename Source>
struct make_parent_pairs<Source, TypeList<>> {
  typedef TypeList<> type;
};

template<typename Source, typename N, typename... Ns>
struct make_parent_pairs<Source, TypeList < N, Ns...>> {
typedef concat_t <TypeList<TypeList < N, Source>>,
typename make_parent_pairs<Source, TypeList < Ns...>>
::type>
type;
};

template<typename T, typename Parents>
struct parent_of;

template<typename T, typename Parents>
struct parent_of_impl;

template<typename T, typename... Parents>
struct parent_of<T, TypeList < Parents...>>
: public parent_of_impl<T, TypeList < Parents...>> {
};

template<typename T>
struct parent_of_impl<T, TypeList<>> {
  static_assert(sizeof(T) == 0, "");
};

template<typename T, typename P, typename... Parents>
struct parent_of_impl<T, TypeList < TypeList < T, P>, Parents...>> {
typedef P type;
};

template<typename T, typename S, typename P, typename... Parents>
struct parent_of_impl<T, TypeList < TypeList < S, P>, Parents...>>
: parent_of<T, TypeList < Parents...>> {
};

template<typename T, typename Parents>
struct path_to;

template<bool done, typename T, typename Parents>
struct path_to_impl;

template<typename T, typename Parents>
struct path_to_impl<true, T, Parents> {
  typedef TypeList <T> type;
};

template<typename T, typename... Parents>
struct path_to_impl<false, T, TypeList < Parents...>>
: public concat<
    typename path_to<typename parent_of<T, TypeList < Parents...>>::type,
    TypeList < Parents...>>
::type,
TypeList <T>> {
};

template<typename T, typename PT, typename PP, typename... Parents>
struct path_to<T, TypeList < TypeList < PT, PP>, Parents...>>
: public path_to_impl<std::is_same<T, PT>::value, T,
                      TypeList < TypeList < PT, PP>, Parents...>> {
};

template<typename Graph, typename Goal, typename Root>
struct bfs;

template<typename Graph, typename Goal, typename Open, typename Closed,
    typename Parent>
struct bfs_iteration;

enum struct BfsSearchState {
  FoundGoalTag,
  CycleDetectedTag,
  NeitherGoalNorCycleTag
};

template<bool found_goal, bool cycle_detected>
struct bfs_vertex_search_state_impl;

template<>
struct bfs_vertex_search_state_impl<false, false> {
  static constexpr BfsSearchState value =
      BfsSearchState::NeitherGoalNorCycleTag;
};

template<>
struct bfs_vertex_search_state_impl<true, false> {
  static constexpr BfsSearchState value = BfsSearchState::FoundGoalTag;
};

template<>
struct bfs_vertex_search_state_impl<false, true> {
  static constexpr BfsSearchState value = BfsSearchState::CycleDetectedTag;
};

template<typename Current, typename Goal, typename... Closed>
constexpr BfsSearchState bfs_vertex_search_state_v =
    bfs_vertex_search_state_impl<
        std::is_same<Current, Goal>::value,
        contains_v < Current, TypeList < Closed...>, std::is_same
>>::value;

template<BfsSearchState IterationTag, typename Graph, typename Goal,
    typename Open, typename Closed, typename Parent>
struct bfs_iteration_impl;

template<typename Graph, typename Goal, typename... Closed, typename... Parent>
struct bfs_iteration<Graph, Goal, TypeList<>, TypeList < Closed...>,
TypeList<Parent...>> {
typedef TypeList<> type;
};

template<typename Graph, typename Goal, typename Current, typename... Open,
    typename... Closed, typename... Parent>
struct bfs_iteration<Graph, Goal, TypeList < Current, Open...>,
TypeList<Closed...>, TypeList<Parent...>>
: public bfs_iteration_impl<
    bfs_vertex_search_state_v<Current, Goal, TypeList < Closed...>>, Graph,
Goal, TypeList<Current, Open...>, TypeList<Closed...>,
TypeList<Parent...>> {
};

template<typename Graph, typename Goal, typename Current, typename... Open,
    typename... Closed, typename... Parent>
struct bfs_iteration_impl<BfsSearchState::FoundGoalTag, Graph, Goal,
                          TypeList < Current, Open...>, TypeList<Closed...>,
TypeList<Parent...>> {
typedef typename path_to<Current, TypeList < Parent...>>
::type type;
};

template<typename Graph, typename Goal, typename Current, typename... Open,
    typename Closed, typename Parent>
struct bfs_iteration_impl<BfsSearchState::CycleDetectedTag, Graph, Goal,
                          TypeList < Current, Open...>, Closed, Parent> {
private:
using next_closed = concat_t<Closed, Current>;

public:
typedef typename bfs_iteration<Graph, Goal, TypeList < Open...>, next_closed,
Parent>
::type type;
};

template<typename LHS, typename RHS>
struct match_key : std::false_type {};

template<typename Key, typename LhsVal, typename RhsVal>
struct match_key<TypeList < Key, LhsVal>, TypeList <Key, RhsVal>>
: std::true_type {
};

template<typename Graph, typename Goal, typename Current, typename... Open,
    typename Closed, typename Parent>
struct bfs_iteration_impl<BfsSearchState::NeitherGoalNorCycleTag, Graph, Goal,
                          TypeList < Current, Open...>, Closed, Parent> {
private:
using current_neighbours = typename neighbours<Current, Graph>::type;
using next_open =
typename append_unique<current_neighbours, TypeList < Open...>,
std::is_same>::type;
using next_closed = concat_t <Closed, TypeList<Current>>;
using next_parent = typename append_unique<
    typename make_parent_pairs<Current, current_neighbours>::type, Parent,
    match_key>::type;

public:
typedef typename bfs_iteration<Graph, Goal, next_open, next_closed,
                               next_parent>::type type;
};

template<typename Graph, typename Goal, typename Root>
struct bfs : bfs_iteration<Graph, Goal, TypeList < Root>, TypeList<>,
             TypeList<TypeList < Root, Root>>> {
};

} // namespace graph
} // namespace poena

#endif  // SANDBOX_MPL_GRAPH_H
