// Copyright 2021-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Generic algorithms.
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @ingroup aux_util
 */

#include <iterator>
#include <type_traits>
#include <algorithm>

/*!
 * @brief Copy the elements of a sequence whose indices do not appear in a sorted list.
 *
 * @param first Input iterator for start of sequence, considered to be index 0.
 * @param last Input iterator for past-the-end of sequence.
 * @param result Output iterator.
 * @param excluded_sorted_first Input iterator for start of a sorted list of indices to exclude from copying.
 * @param excluded_sorted_last Input iterator for past-the-end of a sorted list of indices to exclude from
 * copying.
 *
 * Copies elements in @p [first,last) whose zero-based index does not appear in
 * @p [excluded_sorted_first,excluded_sorted_last) .
 *
 * Performance is O(n + m) where n is the size of the input sequence and m is the size of the exclusion sequence: both
 * are iterated a single time.
 *
 * @return the end of the resulting sequence.
 */
template <typename InputIterator, typename OutputIterator, typename ExcludeInputIterator>
static inline OutputIterator
copy_excluding_indices(InputIterator first,
                       InputIterator last,
                       OutputIterator result,
                       ExcludeInputIterator excluded_sorted_first,
                       ExcludeInputIterator excluded_sorted_last)
{
	// Check InputIterator.
	static_assert(std::is_convertible<typename std::iterator_traits<InputIterator>::iterator_category,
	                                  std::input_iterator_tag>::value,
	              "first/last must be input iterators");
	// Check OutputIterator.
	static_assert(std::is_convertible<typename std::iterator_traits<OutputIterator>::iterator_category,
	                                  std::output_iterator_tag>::value,
	              "result must be an output iterator");
	// Check OutputIterator/InputIterator compatibility.
	// iterator_traits is useless for output iterators for everything except their category
	static_assert(
	    std::is_assignable<decltype(*result), typename std::iterator_traits<InputIterator>::reference>::value,
	    "result must be assignable from the input");
	// Check ExcludeInputIterator.
	static_assert(std::is_convertible<typename std::iterator_traits<ExcludeInputIterator>::iterator_category,
	                                  std::input_iterator_tag>::value,
	              "excluded_sorted_first/excluded_sorted_last must be input iterators");
	static_assert(
	    std::is_same<typename std::iterator_traits<ExcludeInputIterator>::value_type, size_t>::value,
	    "excluded_sorted_first/excluded_sorted_last must be iterators over a (sorted) sequence of size_t");

	auto exclude_b = excluded_sorted_first;
	const auto exclude_e = excluded_sorted_last;
	for (size_t i = 0; first != last; ++first, ++i) {
		// Move our exclude iterator forward, if applicable.
		exclude_b = std::find_if_not(exclude_b, exclude_e, [i](size_t val) { return val < i; });
		// b now points to the first element "not less than" i
		// we just step thru like this instead of std::lower_bound because we essentially just want to
		// walk both arrays, not do a binary search each time (that would be O(n log n))

		// if it's not past the end, see if it is i.
		bool should_copy = (exclude_b == exclude_e) || (*exclude_b != i);
		if (should_copy) {
			*result = *first;
			++result;
		}
	}
	return result;
}


/*!
 * @brief Apply the functor to each element in turn, and return the iterator with the largest result.
 *
 * Like std::max_result, except items are compared based on their functor application results, which are computed only
 * once per element.
 *
 * @param first Input iterator for start of sequence.
 * @param last Input iterator for past-the-end of sequence.
 * @param functor Unary functor
 *
 * @return the element that produced the largest result under the functor.
 */
template <typename FunctorResultType, typename InputIterator, typename Functor>
static inline InputIterator
max_result_element(InputIterator first, InputIterator last, Functor &&functor)
{
	// Check InputIterator.
	static_assert(std::is_convertible<typename std::iterator_traits<InputIterator>::iterator_category,
	                                  std::input_iterator_tag>::value,
	              "first/last must be input iterators");
	using partial_result = std::pair<FunctorResultType, InputIterator>;

	if (first == last) {
		// empty sequence
		return last;
	}
	// Make a list of the functor results and the associated iterator
	std::vector<partial_result> partials(std::distance(first, last));
	for (auto it = partials.begin(); first != last; ++it, ++first) {
		*it = std::make_pair(functor(*first), first);
	}

	auto partial_it =
	    std::max_element(partials.begin(), partials.end(),
	                     [](partial_result const &a, partial_result const &b) { return a.first < b.first; });
	return partial_it->second;
}
