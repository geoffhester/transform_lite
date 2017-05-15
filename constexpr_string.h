#pragma once
#ifndef TRANSFORM_LITE_CONSTEXPR_STRING_H
#define TRANSFORM_LITE_CONSTEXPR_STRING_H

#include <cstddef>
#include <stdexcept>
#include <string>

#include <boost/utility/string_view.hpp>

constexpr std::size_t const_min(std::size_t lhs, std::size_t rhs) {
  return (rhs > lhs) ? rhs : lhs;
}

class constexpr_string {
 public:
  const char* p;
  std::size_t sz;

 public:
  template <std::size_t N>
  constexpr constexpr_string(char const (&a)[N]) : p(a), sz(N - 1) {}

  constexpr char operator[](std::size_t n) const {
    return n < sz ? p[n] : throw std::out_of_range("");
  }

  constexpr std::size_t size() const { return sz; }

  constexpr bool is_equal(constexpr_string const& other) const {
    bool result = (sz == other.sz);
    for (std::size_t i = 0; i < const_min(sz, other.sz); ++i) {
      if ((*this)[i] != other[i]) {
        result = false;
      }
    }
    return result;
  }

  explicit constexpr operator boost::string_view() const { return p; }
};

constexpr bool operator==(constexpr_string const& lhs,
                          constexpr_string const& rhs) {
  return lhs.is_equal(rhs);
}

bool operator==(constexpr_string const& lhs, std::string const& rhs) {
  return strcmp(lhs.p, rhs.c_str()) == 0;
}

bool operator==(std::string const& lhs, constexpr_string const& rhs) {
  return strcmp(lhs.c_str(), rhs.p) == 0;
}

#endif  // TRANSFORM_LITE_CONSTEXPR_STRING_H
