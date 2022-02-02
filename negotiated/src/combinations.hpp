//  (C) Copyright Howard Hinnant 2005-2011.
//  Use, modification and distribution are subject to the Boost Software License,
//  Version 1.0. (See accompanying file LICENSE_1_0.txt or copy at
//  http://www.boost.org/LICENSE_1_0.txt).
//
//  See http://www.boost.org/libs/type_traits for most recent version including documentation.

//  Details are in namespace detail.  Every effort has been made to make
//  combine_discontinuous and permute as fast as possible.  They minimize the number
//  of swaps that are performed. Everything else builds on these two primitives.
//  The most complicated algorithm is for_each_reversible_permutation.  But it
//  builds on combine_discontinuous and permute and I believe represents a minimum
//  number of swaps.  Without care, algorithms such as for_each_reversible_permutation
//  will take longer than for_each_permutation instead of the intended half the time.

//  Speed is everything.  Lest you could just use std::next_permutation and manually
//  eliminate duplicate permutations.  If the implementation fails in being orders
//  of magnitude faster than that, then it has failed miserably.

// Taken from http://howardhinnant.github.io/combinations.html

#ifndef COMBINATIONS_HPP_
#define COMBINATIONS_HPP_

#include <algorithm>
#include <iterator>
#include <utility>

// Creates a functor with no arguments which calls f_(first_, last_).
//   Also has a variant that takes two It and ignores them.
template<class Function, class It>
class BoundRange
{
public:
  BoundRange(Function f, It first, It last)
  : f_(f), first_(first), last_(last) {}

  bool operator()()
  {
    return f_(first_, last_);
  }

  bool operator()(It, It)
  {
    return f_(first_, last_);
  }

private:
  Function f_;
  It first_;
  It last_;
};

// Rotates two discontinuous ranges to put *first2 where *first1 is.
//     If last1 == first2 this would be equivalent to rotate(first1, first2, last2),
//     but instead the rotate "jumps" over the discontinuity [last1, first2) -
//     which need not be a valid range.
//     In order to make it faster, the length of [first1, last1) is passed in as d1,
//     and d2 must be the length of [first2, last2).
//  In a perfect world the d1 > d2 case would have used swap_ranges and
//     reverse_iterator, but reverse_iterator is too inefficient.
template<class BidirIter>
void
rotate_discontinuous(
  BidirIter first1, BidirIter last1,
  typename std::iterator_traits<BidirIter>::difference_type d1,
  BidirIter first2, BidirIter last2,
  typename std::iterator_traits<BidirIter>::difference_type d2)
{
  if (d1 <= d2) {
    std::rotate(first2, std::swap_ranges(first1, last1, first2), last2);
  } else {
    BidirIter i1 = last1;
    while (first2 != last2) {
      std::swap(*--i1, *--last2);
    }
    std::rotate(first1, i1, last1);
  }
}

// Call f() for each combination of the elements [first1, last1) + [first2, last2)
//    swapped/rotated into the range [first1, last1).  As long as f() returns
//    false, continue for every combination and then return [first1, last1) and
//    [first2, last2) to their original state.  If f() returns true, return
//    immediately.
//  Does the absolute mininum amount of swapping to accomplish its task.
//  If f() always returns false it will be called (d1+d2)!/(d1!*d2!) times.
template<class BidirIter, class Function>
bool
combine_discontinuous(
  BidirIter first1, BidirIter last1,
  typename std::iterator_traits<BidirIter>::difference_type d1,
  BidirIter first2, BidirIter last2,
  typename std::iterator_traits<BidirIter>::difference_type d2,
  Function & f,
  typename std::iterator_traits<BidirIter>::difference_type d = 0)
{
  typedef typename std::iterator_traits<BidirIter>::difference_type D;
  if (d1 == 0 || d2 == 0) {
    return f();
  }
  if (d1 == 1) {
    for (BidirIter i2 = first2; i2 != last2; ++i2) {
      if (f()) {
        return true;
      }
      std::swap(*first1, *i2);
    }
  } else {
    BidirIter f1p = std::next(first1);
    BidirIter i2 = first2;
    for (D d22 = d2; i2 != last2; ++i2, --d22) {
      if (combine_discontinuous(f1p, last1, d1 - 1, i2, last2, d22, f, d + 1)) {
        return true;
      }
      std::swap(*first1, *i2);
    }
  }
  if (f()) {
    return true;
  }
  if (d != 0) {
    rotate_discontinuous(first1, last1, d1, std::next(first2), last2, d2 - 1);
  } else {
    rotate_discontinuous(first1, last1, d1, first2, last2, d2);
  }
  return false;
}

template<class BidirIter, class Function>
Function
for_each_combination(BidirIter first, BidirIter mid, BidirIter last, Function f)
{
  BoundRange<Function &, BidirIter> wfunc(f, first, mid);
  combine_discontinuous(
    first, mid, std::distance(first, mid),
    mid, last, std::distance(mid, last),
    wfunc);
  return f;
}

#endif  // COMBINATIONS_HPP_
