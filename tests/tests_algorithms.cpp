// Copyright 2021-2024, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Generic util algorithm tests.
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 */

#include "catch/catch.hpp"

#include "util/u_algorithms.hpp"

#include <vector>
#include <initializer_list>

namespace Matchers = Catch::Matchers;

TEST_CASE("copy_excluding_indices")
{
	using std::back_inserter;
	using std::begin;
	using std::end;
	using std::initializer_list;
	using std::vector;
	const vector<char> input = {'a', 'b', 'c', 'd', 'e'};
	SECTION("effectively copy all")
	{
		vector<char> result;
		SECTION("empty exclude list")
		{
			initializer_list<size_t> exclude{};
			copy_excluding_indices(begin(input), end(input), back_inserter(result), begin(exclude),
			                       end(exclude));
			CHECK_THAT(result, Matchers::Equals(input));
		}
		SECTION("out-of-range exclude list")
		{
			initializer_list<size_t> exclude{5, 6, 7};
			copy_excluding_indices(begin(input), end(input), back_inserter(result), begin(exclude),
			                       end(exclude));
			CHECK_THAT(result, Matchers::Equals(input));
		}
	}

	SECTION("do not copy all")
	{
		vector<char> result;
		const vector<char> expected = {'a', 'c', 'e'};
		SECTION("excludes all in range")
		{
			initializer_list<size_t> exclude{1, 3};
			copy_excluding_indices(begin(input), end(input), back_inserter(result), begin(exclude),
			                       end(exclude));
			CHECK_THAT(result, Matchers::Equals(expected));
		}
		SECTION("some excludes out-of-range")
		{
			initializer_list<size_t> exclude{1, 3, 5, 6, 7};
			copy_excluding_indices(begin(input), end(input), back_inserter(result), begin(exclude),
			                       end(exclude));
			CHECK_THAT(result, Matchers::Equals(expected));
		}
		SECTION("some duplicate excludes, some out of range")
		{
			initializer_list<size_t> exclude{1, 3, 3, 5, 6, 6, 7};
			copy_excluding_indices(begin(input), end(input), back_inserter(result), begin(exclude),
			                       end(exclude));
			CHECK_THAT(result, Matchers::Equals(expected));
		}
	}
}

TEST_CASE("max_result_element")
{

	using std::begin;
	using std::end;
	using std::initializer_list;
	using std::vector;
	auto distance_from_c = [](char val) { return std::abs(int('c') - int(val)); };
	auto distance_from_d = [](char val) { return std::abs(int('d') - int(val)); };
	auto negative_distance_from_c = [](char val) { return -std::abs(int('c') - int(val)); };
	SECTION("empty sequence")
	{

		const vector<char> input = {};
		auto it = max_result_element<int>(input.begin(), input.end(), distance_from_c);
		CHECK(it == input.end());
	}
	SECTION("non-empty sequence")
	{
		const vector<char> input = {'a', 'b', 'c', 'd', 'e', 'f'};
		SECTION("result at end")
		{
			auto it = max_result_element<int>(input.begin(), input.end(), distance_from_c);
			CHECK(*it == 'f');
		}
		SECTION("result at beginning")
		{
			auto it = max_result_element<int>(input.begin(), input.end(), distance_from_d);
			CHECK(*it == 'a');
		}
		SECTION("result in middle")
		{
			auto it = max_result_element<int>(input.begin(), input.end(), negative_distance_from_c);
			CHECK(*it == 'c');
		}
	}
	SECTION("non-empty sequence with dupes")
	{
		const vector<char> input = {'a', 'a', 'b', 'c', 'c', 'd', 'e', 'f'};
		SECTION("result at beginning")
		{
			auto it = max_result_element<int>(input.begin(), input.end(), distance_from_d);
			CHECK(*it == 'a');
			CHECK(it == input.begin());
		}
		SECTION("result in middle")
		{
			auto it = max_result_element<int>(input.begin(), input.end(), negative_distance_from_c);
			CHECK(*it == 'c');
			CHECK(it == input.begin() + 3);
		}
	}
}
