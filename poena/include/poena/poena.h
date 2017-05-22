//! \file A meta programming library. Because I wanted to learn.
//! Named after the goddess of punishment as it seemed apt.
#pragma once
#ifndef POENA_H_
#define POENA_H_

#include <type_traits>

namespace poena {

template<typename T>
struct TypeDisplayer;

template<typename...>
struct TypeList {};

//------------------------------------------------------------------------------
// Type list traits
template<typename T>
struct is_empty;

template<>
struct is_empty<TypeList<>> : std::true_type {};

template<typename T, typename... Ts>
struct is_empty<TypeList<T, Ts...>> : std::false_type {};

//------------------------------------------------------------------------------
// Concatenate two TypeLists
// TypeList<A,B,C>, TypeList<B,C,D> -> TypeList<A,B,C,B,C,D>
template<typename T1, typename T2>
struct concat {};

template<typename... Ts, typename... Us>
struct concat<TypeList<Ts...>, TypeList<Us...>> {
  typedef TypeList<Ts..., Us...> type;
};

template<typename T1, typename T2>
using concat_t = typename concat<T1, T2>::type;

//------------------------------------------------------------------------------
// Split a TypeList into head and tail
template<typename Ts>
struct split_list;

template<>
struct split_list<TypeList<>> {
  typedef TypeList<> head;
  typedef TypeList<> tail;
};

template<typename Front, typename... Ts>
struct split_list<TypeList<Front, Ts...>> {
  typedef TypeList<Front> head;
  typedef TypeList<Ts...> tail;
};

template<typename Ts>
struct front;

template<typename... Ts>
struct front<TypeList<Ts...>> {
  typedef typename split_list<TypeList<Ts...>>::head type;
};

template<typename Ts>
using front_t = typename front<Ts>::type;

template<typename Ts>
struct tail;

template<typename... Ts>
struct tail<TypeList<Ts...>> {
  typedef typename split_list<TypeList<Ts...>>::tail type;
};

template<typename Ts>
using tail_t = typename tail<Ts>::type;

//------------------------------------------------------------------------------
// Filter a TypeList to one where are predicate holds
template<typename Ts, template<typename> class UnaryPredicate>
struct filter;

template<template<typename> class UnaryPredicate>
struct filter<TypeList<>, UnaryPredicate> {
  typedef TypeList<> type;
};

template<typename Ts, template<typename> class UnaryPredicate>
using filter_t = typename filter<Ts, UnaryPredicate>::type;

template<bool Accept, typename Us, template<typename> class UnaryPredicate>
struct filter_helper;

template<typename T, typename... Ts, template<typename> class UnaryPredicate>
struct filter<TypeList<T, Ts...>, UnaryPredicate>
    : public filter_helper<UnaryPredicate<T>::value, TypeList<T, Ts...>,
                           UnaryPredicate> {
};

template<typename T, typename... Ts, template<typename> class UnaryPredicate>
struct filter_helper<true, TypeList<T, Ts...>, UnaryPredicate>
    : public concat<TypeList<T>, filter_t<TypeList<Ts...>, UnaryPredicate>> {
};

template<typename T, typename... Ts, template<typename> class UnaryPredicate>
struct filter_helper<false, TypeList<T, Ts...>, UnaryPredicate>
    : public concat<TypeList<>, filter_t<TypeList<Ts...>, UnaryPredicate>> {
};

//------------------------------------------------------------------------------
// Transform a TypeList applying a transform to every element
template<typename Ts, template<typename> class UnaryMetaFunction>
struct transform;

template<typename... Ts, template<typename> class UnaryMetaFunction>
struct transform<TypeList<Ts...>, UnaryMetaFunction> {
  typedef TypeList<typename UnaryMetaFunction<Ts>::type...> type;
};

template<typename Ts, template<typename> class UnaryMetaFunction>
using transform_t = typename transform<Ts, UnaryMetaFunction>::type;

//------------------------------------------------------------------------------
// Test if a TypeList contains a specified type
template<typename T, typename Collection,
    template<typename, typename> class BinaryComparison>
struct contains;

template<typename T, template<typename, typename> class BinaryComparison>
struct contains<T, TypeList<>, BinaryComparison> : std::false_type {};

template<bool found, typename T, typename Collection,
    template<typename, typename> class BinaryComparison>
struct contains_impl;

template<typename T, typename A, typename... Ts,
    template<typename, typename> class BinaryComparison>
struct contains<T, TypeList<A, Ts...>, BinaryComparison>
    : contains_impl<BinaryComparison<T, A>::value, T, TypeList<Ts...>,
                    BinaryComparison> {
};

template<typename T, typename Collection,
    template<typename, typename> class BinaryComparison>
struct contains_impl<true, T, Collection, BinaryComparison> : std::true_type {};

template<typename T, typename... Ts,
    template<typename, typename> class BinaryComparison>
struct contains_impl<false, T, TypeList<Ts...>, BinaryComparison>
    : contains<T, TypeList<Ts...>, BinaryComparison> {
};

template<typename T, typename Collection,
    template<typename, typename> class BinaryComparison>
constexpr bool contains_v = contains<T, Collection, BinaryComparison>::value;

//------------------------------------------------------------------------------
// Append elements from the first TypeList that are not present in the second
// TypeList to the second TypeList
template<typename Append, typename To,
    template<typename, typename> class EqualityMetaFunction>
struct append_unique;

template<typename... To,
    template<typename, typename> class EqualityMetaFunction>
struct append_unique<TypeList<>, TypeList<To...>, EqualityMetaFunction> {
  typedef TypeList<To...> type;
};

template<bool append_first, typename From, typename To,
    template<typename, typename> class EqualityMetaFunction>
struct append_unique_impl;

template<typename T, typename... Ts, typename... To,
    template<typename, typename> class EqualityMetaFunction>
struct append_unique_impl<false, TypeList<T, Ts...>, TypeList<To...>,
                          EqualityMetaFunction>
    : append_unique<TypeList<Ts...>, TypeList<To...>, EqualityMetaFunction> {
};

template<typename T, typename... Ts, typename... To,
    template<typename, typename> class EqualityMetaFunction>
struct append_unique_impl<true, TypeList<T, Ts...>, TypeList<To...>,
                          EqualityMetaFunction>
    : append_unique<TypeList<Ts...>, TypeList<To..., T>, EqualityMetaFunction> {
};

template<typename T, typename... Ts, typename... To,
    template<typename, typename> class EqualityMetaFunction>
struct append_unique<TypeList<T, Ts...>, TypeList<To...>, EqualityMetaFunction>
    : append_unique_impl<!contains_v<T, TypeList<To...>, EqualityMetaFunction>,
                         TypeList<T, Ts...>, TypeList<To...>,
                         EqualityMetaFunction> {
};

template<typename Append, typename To,
    template<typename, typename> class EqualityMetaFunction>
using append_unique_t =
typename append_unique<Append, To, EqualityMetaFunction>::type;

//------------------------------------------------------------------------------
// Define a TypeList containing the unique types from the input TypeList
// TypeList<A,B,B,A,D,A,C> -> TypeList<A,B,D,C>
template<typename T, template<typename, typename> class EqualityMetaOperator =
std::is_same>
struct unique;

template<typename... Ts,
    template<typename, typename> class EqualityMetaOperator>
struct unique<TypeList<Ts...>, EqualityMetaOperator>
    : append_unique<TypeList<Ts...>, TypeList<>, EqualityMetaOperator> {
};

template<typename T>
using unique_t = typename unique<T>::type;

//------------------------------------------------------------------------------
// Reverse a TypeList TypeList<A,B,C> -> TypeList<C,B,A>
template<typename T>
struct reverse;

template<typename T>
struct reverse_impl;

template<>
struct reverse_impl<TypeList<>> {
  typedef TypeList<> type;
};

template<typename T, typename... Ts>
struct reverse_impl<TypeList<T, Ts...>>
    : public concat<typename reverse<TypeList<Ts...>>::type, TypeList<T>> {
};

template<typename... Ts>
struct reverse<TypeList<Ts...>> : public reverse_impl<TypeList<Ts...>> {};

template<typename T>
using reverse_t = typename reverse<T>::type;

//------------------------------------------------------------------------------
// Fold left. Take a TypeList and a binary meta function f
// TypeList<A,B,C,D>, F -> F(F(F(A,B),C),D)
template<typename T, template<typename, typename> class F>
struct fold_left;

template<template<typename, typename> class F>
struct fold_left<TypeList<>, F> {
  typedef TypeList<> type;
};

template<typename T, template<typename, typename> class F>
struct fold_left<TypeList<T>, F> {
  typedef TypeList<T> type;
};

template<typename T1, typename T2, typename... Ts, template<typename, typename> class F>
struct fold_left<TypeList<T1, T2, Ts...>, F> : fold_left<TypeList<typename F<T1,
                                                                             T2>::type,
                                                                  Ts...>, F> {
};

template<typename T, template<typename, typename> class F>
using fold_left_t = typename fold_left<T, F>::type;

} // namespace poena

#endif  // POENA_H
